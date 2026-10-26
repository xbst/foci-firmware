//! PositionTune: two-bound position-P sweep commissioning phase.

pub(crate) mod bounds;
pub(crate) mod ceiling;
pub(crate) mod metrics;
pub(crate) mod selection;

use crate::autotune::{
    CommissionError, CommissionPhase, CommissionProfile, CommissionStatus, PositionTuneOutcome,
    TRANSIENT_LEN, TransientBuffer,
};
use crate::commissioning::OuterGainRollbackSnapshot;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, fail_outer_position_tune,
};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::stimulus::trapezoid::{LegPlan, StrokeRequest, plan_leg, plan_stroke};
use crate::commissioning::types::{
    OuterResult, PositionTuneProvenance, StepResult, TuneProvenance,
};
use crate::tmc_register_values::{MOTION_POSITION_ADVANCED_PI, MOTION_STOPPED};
use ceiling::{ceiling_trips, compute_k};
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    MODE_RAMP_MODE_MOTION, PID_ERROR_PID_POSITION_ERROR, PID_ERROR_PID_POSITION_ERROR_SUM,
    PID_POSITION_ACTUAL, PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_TARGET,
    PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET, PidTorqueFluxTarget, Register, SubRegister,
};
use metrics::{LegMetrics, StrokeTiming};
use selection::{RungDecision, RungMeasurement, next_rung};

/// Standstill-dither observations per rung; odd so the majority vote cannot tie.
pub(crate) const STANDSTILL_CAPTURE_COUNT: u8 = 5;

pub(crate) const STANDSTILL_HOLD_US: u32 = 100_000;
pub(crate) const STANDSTILL_SAMPLE_PERIOD_US: u32 = 1_000;

/// Ramped-stroke repeats captured per rung, per leg kind; `selection::decide`
/// acts on their per-field medians.
pub(crate) const RAMPED_STROKE_REPEATS: u8 = 3;

pub(crate) const STROKE_LEAD_US: u32 = 20_000;

pub(crate) const MIN_CRUISE_US: u32 = 50_000;

pub(crate) const RESTORE_WAIT_US: u32 = 50_000;

pub(crate) const STROKE_POLL_US: u32 = 1_000;

/// Reserve kept between a planned stroke and the primitive duration cap, covering the
/// stroke poll period, the safety-check latency and clock jitter.
pub(crate) const STROKE_DURATION_MARGIN_US: u32 = 100_000;

const TAU_Q16: i64 = (core::f64::consts::TAU * 65_536.0 + 0.5) as i64;

/// Which directional leg of a rung's stroke is being driven.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum StrokeKind {
    Homing,
    Motion,
}

/// Which sub-step of one rung's evaluation is in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum PositionTuneStep {
    Predict,
    Stroke {
        rung: PidPGain,
        kind: StrokeKind,
        repeat: u8,
    },
    AwaitRestore {
        rung: PidPGain,
        since_us: u64,
    },
    CeilingStop {
        rung: PidPGain,
        since_us: u64,
    },
    StandstillHold {
        rung: PidPGain,
        capture_index: u8,
    },
    Done,
}

/// Phase-local state for the two-bound position-P ladder sweep.
#[derive(Debug, defmt::Format)]
pub(crate) struct PositionTuneState {
    pub(crate) step: PositionTuneStep,
    pub(crate) k_q16: i64,
    pub(crate) bound_units: u32,
    pub(crate) predicted_start: PidPGain,
    pub(crate) homing_leg: Option<LegPlan>,
    pub(crate) motion_leg: Option<LegPlan>,
    pub(crate) stroke_timing: Option<StrokeTiming>,
    pub(crate) ff_permille: u32,
    pub(crate) dither_captures: [i32; STANDSTILL_CAPTURE_COUNT as usize],
    pub(crate) homing_repeats: [Option<LegMetrics>; RAMPED_STROKE_REPEATS as usize],
    pub(crate) motion_repeats: [Option<LegMetrics>; RAMPED_STROKE_REPEATS as usize],
    /// Rung with the running-minimum motion overshoot, and that overshoot; `None` before the
    /// first `Advance`.
    pub(crate) min_overshoot_so_far: Option<(PidPGain, u32)>,
    pub(crate) rungs_measured: u8,
    pub(crate) last_measurement: Option<RungMeasurement>,
    /// Feedforward path flags of the last admitted motion stroke; zero until one is admitted.
    pub(crate) feedforward_paths_active: u8,
    pub(crate) current_rung: Option<PidPGain>,
    pub(crate) rung_peak_milli_counts_per_s: u64,
    pub(crate) candidate: Option<RungMeasurement>,
    pub(crate) counts_per_rev: u32,
    pub(crate) ceiling_reached: bool,
    pub(crate) provenance: PositionTuneProvenance,
    accepted: Option<RungMeasurement>,
    /// Outer gains at installed-tuning entry; the terminal `OuterResult` reports every other
    /// field from here.
    entry: OuterGainRollbackSnapshot,
    /// Velocity P read from the chip at phase entry.
    velocity_p: PidPGain,
    profile: CommissionProfile,
    sample_period_us: u32,
    /// False until the current sub-step's capture is armed.
    capture_running: bool,
    next_sample_us: u64,
    next_poll_us: u64,
    deadline_us: u64,
}

impl PositionTuneState {
    pub(crate) const fn new(entry: OuterGainRollbackSnapshot, profile: CommissionProfile) -> Self {
        Self {
            step: PositionTuneStep::Predict,
            k_q16: 0,
            bound_units: 0,
            predicted_start: PidPGain::ZERO,
            homing_leg: None,
            motion_leg: None,
            stroke_timing: None,
            ff_permille: 0,
            dither_captures: [0; STANDSTILL_CAPTURE_COUNT as usize],
            homing_repeats: [None; RAMPED_STROKE_REPEATS as usize],
            motion_repeats: [None; RAMPED_STROKE_REPEATS as usize],
            min_overshoot_so_far: None,
            rungs_measured: 0,
            last_measurement: None,
            feedforward_paths_active: 0,
            current_rung: None,
            rung_peak_milli_counts_per_s: 0,
            candidate: None,
            counts_per_rev: 0,
            ceiling_reached: false,
            provenance: PositionTuneProvenance {
                predicted_start_p: 0,
                last_rung_p: 0,
                min_overshoot_p: 0,
                min_overshoot_units: 0,
                bound_units: 0,
                homing_speed_mrev_s: 0,
                motion_speed_mrev_s: 0,
                homing_peak_abs_units: 0,
                motion_cruise_mean_abs_units: 0,
                motion_overshoot_units: 0,
                motion_cruise_rms_units: 0,
                nominal_bandwidth_hz: 0,
                dither_margin_milli: 0,
                rungs_measured: 0,
                stimulus_feedforward_paths: 0,
                velocity_ceiling_reached: 0,
                peak_velocity_mrev_s: 0,
                tune_step: 0,
            },
            accepted: None,
            entry,
            velocity_p: PidPGain::ZERO,
            profile,
            sample_period_us: 0,
            capture_running: false,
            next_sample_us: 0,
            next_poll_us: 0,
            deadline_us: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum PositionTuneFault {
    Engine(CommissionError),
    Outcome(PositionTuneOutcome),
}

impl From<CommissionError> for PositionTuneFault {
    fn from(error: CommissionError) -> Self {
        Self::Engine(error)
    }
}

impl From<PositionTuneOutcome> for PositionTuneFault {
    fn from(outcome: PositionTuneOutcome) -> Self {
        Self::Outcome(outcome)
    }
}

/// Halt the step-queue stimulus and clear the feedforward opt-in.
///
/// Must be the first effect at every call site, so no queued segment keeps pulsing a channel
/// whose driver is about to be cut.
pub(crate) fn abort_stimulus<B: CommissioningBackend>(backend: &mut B) {
    backend.stimulus_stop();
    backend.set_stimulus_feedforward(false);
}

fn merge_primitive_peak(state: &mut PositionTuneState, safety: &OuterSafetyLimits) {
    state.rung_peak_milli_counts_per_s = state
        .rung_peak_milli_counts_per_s
        .max(safety.primitive_peak_milli_counts_per_s());
}

pub(crate) fn note_safety_fault(phase: &mut OuterPhase) {
    if let OuterPhase::PositionTune { state, safety } = phase {
        merge_primitive_peak(state, safety);
        refresh_provenance(state);
    }
}

fn begin_ceiling_stop<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
    rung: PidPGain,
) -> StepResult {
    abort_stimulus(backend);
    merge_primitive_peak(state, safety);
    safety.end_primitive();
    state.capture_running = false;
    state.ceiling_reached = true;
    state.step = PositionTuneStep::CeilingStop {
        rung,
        since_us: backend.elapsed_us(),
    };
    refresh_provenance(state);
    defmt::warn!(
        "position tune ceiling: p={} peak={}mrev/s candidate={}",
        rung.raw(),
        state.provenance.peak_velocity_mrev_s,
        state.candidate.map(|m| m.position_p.raw()),
    );
    StepResult::ReadyNow
}

async fn fail_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
    fault: PositionTuneFault,
) -> StepResult {
    abort_stimulus(backend);
    let _ = backend.write_register(PID_VELOCITY_TARGET, 0).await;
    let _ = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
        .await;
    let provenance = match phase {
        OuterPhase::PositionTune { state, safety } => {
            merge_primitive_peak(state, safety);
            refresh_provenance(state);
            state.provenance
        }
        _ => PositionTuneProvenance::default(),
    };
    match fault {
        PositionTuneFault::Engine(error) => {
            fail_outer_position_tune(phase, error, None, provenance)
        }
        PositionTuneFault::Outcome(outcome) => {
            defmt::warn!(
                "position tune failed: outcome={} wire_code={}",
                outcome,
                outcome.to_wire_code(),
            );
            fail_outer_position_tune(
                phase,
                CommissionError::PositionTuneFailed,
                Some(outcome),
                provenance,
            )
        }
    }
}

async fn read_sub_register<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
) -> Result<u32, CommissionError> {
    backend
        .write_register(sub.addr, u32::from(sub.sub_addr))
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .read_register(sub.data)
        .await
        .map_err(|_| CommissionError::Spi)
}

async fn write_sub_register<B: CommissioningBackend>(
    backend: &mut B,
    sub: SubRegister,
    value: u32,
) -> Result<(), CommissionError> {
    backend
        .write_register(sub.addr, u32::from(sub.sub_addr))
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(sub.data, value)
        .await
        .map_err(|_| CommissionError::Spi)
}

async fn write_register<B: CommissioningBackend>(
    backend: &mut B,
    register: Register,
    value: u32,
) -> Result<(), CommissionError> {
    backend
        .write_register(register, value)
        .await
        .map_err(|_| CommissionError::Spi)
}

async fn read_capture<B: CommissioningBackend>(
    backend: &mut B,
    register: Register,
) -> Result<u32, CommissionError> {
    backend
        .read_capture_register(register)
        .await
        .map_err(|_| CommissionError::Spi)
}

fn start_capture(
    state: &mut PositionTuneState,
    now_us: u64,
    period_us: u32,
    window_us: u32,
) -> Result<(), CommissionError> {
    state.next_sample_us = now_us
        .checked_add(u64::from(period_us))
        .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
    state.deadline_us = now_us
        .checked_add(u64::from(window_us))
        .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
    state.capture_running = true;
    Ok(())
}

fn advance_sample(
    state: &mut PositionTuneState,
    period_us: u32,
    now_us: u64,
) -> Result<u64, CommissionError> {
    let next = next_grid_deadline(state.next_sample_us, u64::from(period_us), now_us)?;
    state.next_sample_us = next.min(state.deadline_us);
    Ok(state.next_sample_us)
}

async fn read_velocity_gains<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<PidGainPair, PositionTuneFault> {
    let raw = backend
        .read_register(PID_VELOCITY_P_VELOCITY_I)
        .await
        .map_err(|_| PositionTuneFault::Engine(CommissionError::Spi))?;
    PidGainPair::try_from_register(raw)
        .map_err(|_| PositionTuneFault::Engine(CommissionError::PidGainInvalid))
}

async fn predict<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    if backend.is_shutdown_requested() {
        return Err(CommissionError::Shutdown.into());
    }
    if backend.stimulus_step_timer_hz() == 0 || backend.stimulus_is_trigger_blocked() {
        return Err(PositionTuneOutcome::ConfigurationInvalid.into());
    }
    let counts_per_rev = backend.mechanical_encoder_quadrature_counts_per_rev();
    state.counts_per_rev = counts_per_rev;
    safety
        .configure_velocity_sweep_encoder_resolution(counts_per_rev)
        .map_err(|_| CommissionError::EncoderFault)?;
    let encoder_raw = backend
        .try_read_mechanical_encoder_position_counts()
        .await
        .map_err(|_| CommissionError::Spi)?;
    safety
        .set_encoder_baseline(encoder_raw)
        .map_err(|_| CommissionError::EncoderFault)?;
    safety.torque_cap = u16::from(backend.current_limit());
    let velocity_gains = read_velocity_gains(backend).await?;
    state.velocity_p = velocity_gains.p;
    state.k_q16 = compute_k(counts_per_rev / 4, velocity_gains.p)?;

    let budget = safety.budget;
    state.bound_units = bounds::bound_units(
        backend.stall_threshold_units(),
        bounds::margin_permille(state.profile),
    )?;
    state.ff_permille = backend.velocity_feedforward_gain_permille();
    let steps_per_rev = backend
        .configured_planner_steps_per_rev()
        .ok_or(PositionTuneOutcome::ConfigurationInvalid)?;
    let step_timer_hz = backend.stimulus_step_timer_hz();
    let request =
        |speed_mrev_s: u32| stroke_request(speed_mrev_s, &budget, steps_per_rev, step_timer_hz);
    let homing = plan_leg(&request(budget.homing_speed_mrev_s))
        .map_err(|_| PositionTuneOutcome::ConfigurationInvalid)?;
    let motion = plan_leg(&request(budget.machine_velocity_ceiling_mrev_s))
        .map_err(|_| PositionTuneOutcome::ConfigurationInvalid)?;
    state.predicted_start = bounds::start_rung(
        homing.speed_mrev_s,
        motion.speed_mrev_s,
        state.ff_permille,
        state.bound_units,
    )?;
    state.homing_leg = Some(homing);
    state.motion_leg = Some(motion);
    safety.arm_safety_observation(backend.elapsed_us())?;
    state.step = PositionTuneStep::Stroke {
        rung: state.predicted_start,
        kind: StrokeKind::Homing,
        repeat: 0,
    };
    refresh_provenance(state);
    defmt::info!(
        "position tune predict: bound={} start_p={} homing={}mrev/s motion={}mrev/s ff={} predicted_homing_error={} predicted_motion_error={}",
        state.bound_units,
        state.predicted_start.raw(),
        homing.speed_mrev_s,
        motion.speed_mrev_s,
        state.ff_permille,
        bounds::predicted_error_units(homing.speed_mrev_s, 0, state.predicted_start.raw()),
        bounds::predicted_error_units(
            motion.speed_mrev_s,
            state.ff_permille,
            state.predicted_start.raw()
        ),
    );
    Ok(())
}

/// Build the `StrokeRequest` for a leg at `speed_mrev_s` from `budget`, `steps_per_rev`, and
/// `step_timer_hz`, with the excursion fitted to the primitive duration cap.
pub(crate) fn stroke_request(
    speed_mrev_s: u32,
    budget: &OuterMotionBudget,
    steps_per_rev: u32,
    step_timer_hz: u32,
) -> StrokeRequest {
    StrokeRequest {
        speed_mrev_s,
        accel_mrev_s2: budget.max_accel_mrev_s2,
        excursion_mrev: duration_fit_excursion_mrev(
            speed_mrev_s,
            budget,
            steps_per_rev,
            step_timer_hz,
        ),
        steps_per_rev,
        step_timer_hz,
        min_cruise_us: MIN_CRUISE_US,
    }
}

/// Longest leg excursion, in mrev, whose full stroke (both legs with their ramps) stays
/// inside the host's per-primitive duration cap once the stroke lead-in and
/// [`STROKE_DURATION_MARGIN_US`] are reserved.
///
/// Starts from the budget's half stroke travel and shrinks it only when the leg speed is
/// too low to cover that distance in time. The sweep judges following error during
/// cruise, so a shorter stroke at the planned speed keeps the evidence meaningful where
/// the full distance would run into the outer safety duration fault: at a 40 mm/s
/// homing speed a 60 mm leg already takes the whole 3 s cap before the lead-in. When no
/// excursion plans at all the budget value is returned unchanged so the caller reports
/// the planner's own error.
pub(crate) fn duration_fit_excursion_mrev(
    speed_mrev_s: u32,
    budget: &OuterMotionBudget,
    steps_per_rev: u32,
    step_timer_hz: u32,
) -> u32 {
    let budget_excursion_mrev = budget.max_stroke_travel_mrev / 2;
    let allowed_stroke_us = (u64::from(budget.max_duration_ms) * 1_000)
        .saturating_sub(u64::from(STROKE_LEAD_US))
        .saturating_sub(u64::from(STROKE_DURATION_MARGIN_US));
    if budget_excursion_mrev == 0 || allowed_stroke_us == 0 {
        return budget_excursion_mrev;
    }
    // Judge the exact profile the stroke will run, not the leg estimate: the quantized
    // ramp runs take longer than the ideal ramp time.
    let stroke_fits = |excursion_mrev: u32| -> bool {
        plan_stroke(&StrokeRequest {
            speed_mrev_s,
            accel_mrev_s2: budget.max_accel_mrev_s2,
            excursion_mrev,
            steps_per_rev,
            step_timer_hz,
            min_cruise_us: MIN_CRUISE_US,
        })
        .is_ok_and(|profile| u64::from(profile.stroke_us()) <= allowed_stroke_us)
    };
    if stroke_fits(budget_excursion_mrev) {
        return budget_excursion_mrev;
    }
    let (mut fits, mut fails) = (0_u32, budget_excursion_mrev);
    while fails - fits > 1 {
        let mid = fits + (fails - fits) / 2;
        if stroke_fits(mid) {
            fits = mid;
        } else {
            fails = mid;
        }
    }
    if fits == 0 {
        budget_excursion_mrev
    } else {
        fits
    }
}

async fn arm_stroke<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
    rung: PidPGain,
    kind: StrokeKind,
) -> Result<(), PositionTuneFault> {
    let leg = match kind {
        StrokeKind::Homing => state.homing_leg,
        StrokeKind::Motion => state.motion_leg,
    }
    .expect("legs are planned before the first stroke");
    if state.current_rung != Some(rung) {
        state.current_rung = Some(rung);
        state.rung_peak_milli_counts_per_s = 0;
    }
    write_register(
        backend,
        PID_POSITION_P_POSITION_I,
        PidGainPair::new(rung, PidIGain::ZERO).register_value(),
    )
    .await?;
    write_sub_register(backend, PID_ERROR_PID_POSITION_ERROR_SUM, 0).await?;
    write_register(backend, MODE_RAMP_MODE_MOTION, MOTION_POSITION_ADVANCED_PI).await?;
    let envelope_speed_mrev_s = [state.homing_leg, state.motion_leg]
        .into_iter()
        .flatten()
        .map(|planned| planned.speed_mrev_s)
        .max()
        .unwrap_or(leg.speed_mrev_s);
    safety.configure_velocity_envelope(envelope_speed_mrev_s)?;
    let cap = safety.moving_velocity_cap_milli_counts_per_s();
    safety.set_velocity_ceiling(Some(
        cap * selection::VELOCITY_CEILING_NUM / selection::VELOCITY_CEILING_DEN,
    ));

    backend.stimulus_stop();
    if backend.stimulus_is_trigger_blocked() {
        return Err(PositionTuneOutcome::ConfigurationInvalid.into());
    }
    let lead_ticks = (u64::from(backend.stimulus_step_timer_hz()) * u64::from(STROKE_LEAD_US)
        / 1_000_000) as u32;
    let base_clock = backend.stimulus_step_clock_now().wrapping_add(lead_ticks);
    backend
        .stimulus_reset_step_clock(base_clock)
        .map_err(|_| PositionTuneOutcome::StimulusRejected)?;
    backend.set_stimulus_feedforward(matches!(kind, StrokeKind::Motion));
    if matches!(kind, StrokeKind::Motion) {
        state.feedforward_paths_active = backend.feedforward_path_flags();
    }

    let steps_per_rev = backend
        .configured_planner_steps_per_rev()
        .ok_or(PositionTuneOutcome::ConfigurationInvalid)?;
    let request = stroke_request(
        leg.speed_mrev_s,
        &safety.budget,
        steps_per_rev,
        backend.stimulus_step_timer_hz(),
    );
    let profile = plan_stroke(&request).map_err(|_| PositionTuneOutcome::StimulusRejected)?;
    if profile.segments.len() > backend.stimulus_queue_free() {
        defmt::warn!(
            "position tune stroke: {} segments exceed {} free queue slots",
            profile.segments.len(),
            backend.stimulus_queue_free()
        );
        abort_stimulus(backend);
        return Err(PositionTuneOutcome::StimulusRejected.into());
    }
    state.stroke_timing = Some(StrokeTiming::from_profile(&profile));
    let mut current_dir = None;
    for segment in profile.segments.iter() {
        if current_dir != Some(segment.forward) {
            backend.stimulus_set_direction(segment.forward);
            current_dir = Some(segment.forward);
        }
        if let Err(error) = backend.stimulus_queue_segment(segment.step_move()) {
            defmt::warn!("position tune stroke: enqueue refused {}", error);
            abort_stimulus(backend);
            return Err(PositionTuneOutcome::StimulusRejected.into());
        }
    }
    if backend.stimulus_arm().is_err() {
        abort_stimulus(backend);
        return Err(PositionTuneOutcome::StimulusRejected.into());
    }

    let started_us = backend.elapsed_us() + u64::from(STROKE_LEAD_US);
    state.sample_period_us = metrics::sample_period_us(profile.stroke_us());
    safety.begin_primitive(backend.elapsed_us())?;
    workspace.reset(state.sample_period_us as u16);
    start_capture(
        state,
        started_us,
        state.sample_period_us,
        profile.stroke_us(),
    )?;
    state.next_poll_us = backend.elapsed_us() + u64::from(STROKE_POLL_US);
    refresh_provenance(state);
    Ok(())
}

fn standstill_excursion(workspace: &TransientBuffer) -> Result<i32, PositionTuneOutcome> {
    let samples = workspace.samples();
    if samples.is_empty() {
        return Err(PositionTuneOutcome::CaptureEmpty);
    }
    let mut minimum = i64::MAX;
    let mut maximum = i64::MIN;
    for &raw in samples {
        let value = i64::from(raw as i32);
        minimum = minimum.min(value);
        maximum = maximum.max(value);
    }
    Ok(i32::try_from(maximum - minimum).unwrap_or(i32::MAX))
}

async fn arm_standstill_hold<B: CommissioningBackend>(
    state: &mut PositionTuneState,
    safety: &mut OuterSafetyLimits,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> Result<(), PositionTuneFault> {
    write_register(backend, PID_VELOCITY_TARGET, 0).await?;
    let position_actual = read_capture(backend, PID_POSITION_ACTUAL).await?;
    write_register(backend, PID_POSITION_TARGET, position_actual).await?;

    let started_us = backend.elapsed_us();
    safety.begin_primitive(started_us)?;
    workspace.reset(STANDSTILL_SAMPLE_PERIOD_US as u16);
    start_capture(
        state,
        started_us,
        STANDSTILL_SAMPLE_PERIOD_US,
        STANDSTILL_HOLD_US,
    )?;
    Ok(())
}

fn median_metrics(repeats: &[Option<LegMetrics>; RAMPED_STROKE_REPEATS as usize]) -> LegMetrics {
    let filled: [LegMetrics; RAMPED_STROKE_REPEATS as usize] =
        repeats.map(|entry| entry.expect("every repeat slot is filled before the median is read"));
    LegMetrics {
        cruise_mean_abs_units: metrics::median3([
            filled[0].cruise_mean_abs_units,
            filled[1].cruise_mean_abs_units,
            filled[2].cruise_mean_abs_units,
        ]),
        peak_abs_units: metrics::median3([
            filled[0].peak_abs_units,
            filled[1].peak_abs_units,
            filled[2].peak_abs_units,
        ]),
        overshoot_units: metrics::median3([
            filled[0].overshoot_units,
            filled[1].overshoot_units,
            filled[2].overshoot_units,
        ]),
        cruise_rms_units: metrics::median3([
            filled[0].cruise_rms_units,
            filled[1].cruise_rms_units,
            filled[2].cruise_rms_units,
        ]),
    }
}

fn resolve_completed_rung(
    state: &mut PositionTuneState,
    rung: PidPGain,
) -> Result<(), PositionTuneFault> {
    let homing = median_metrics(&state.homing_repeats);
    let motion = median_metrics(&state.motion_repeats);
    let (ceiling_tripped, ceiling_ratios_milli) =
        ceiling_trips(&state.dither_captures, state.k_q16, rung);
    let measurement = RungMeasurement {
        position_p: rung,
        homing_peak_abs_units: homing.peak_abs_units,
        motion_cruise_mean_abs_units: motion.cruise_mean_abs_units,
        motion_overshoot_units: motion.overshoot_units,
        motion_cruise_rms_units: motion.cruise_rms_units,
        ceiling_ratios_milli,
    };
    let decision = selection::decide(
        &measurement,
        state.bound_units,
        state.min_overshoot_so_far.map(|(_, units)| units),
        ceiling_tripped,
    );
    state.rungs_measured = state.rungs_measured.saturating_add(1);
    state.last_measurement = Some(measurement);
    defmt::info!(
        "position tune rung: p={} homing_peak={} motion_cruise_mean={} motion_overshoot={} motion_rms={} dither_margin_milli={} decision={}",
        measurement.position_p.raw(),
        measurement.homing_peak_abs_units,
        measurement.motion_cruise_mean_abs_units,
        measurement.motion_overshoot_units,
        measurement.motion_cruise_rms_units,
        dither_margin_milli(measurement.ceiling_ratios_milli),
        decision,
    );
    let result = match decision {
        RungDecision::Accept => {
            state.accepted = Some(measurement);
            state.step = PositionTuneStep::Done;
            state.capture_running = false;
            Ok(())
        }
        RungDecision::Conflict(_) => Err(PositionTuneOutcome::Conflict.into()),
        RungDecision::Advance => {
            if selection::qualifies_as_candidate(&measurement, state.bound_units) {
                state.candidate = Some(measurement);
            }
            state.min_overshoot_so_far = Some(match state.min_overshoot_so_far {
                Some((prev_rung, prev_units)) if prev_units <= motion.overshoot_units => {
                    (prev_rung, prev_units)
                }
                _ => (rung, motion.overshoot_units),
            });
            match next_rung(rung) {
                Some(next) => {
                    state.homing_repeats = [None; RAMPED_STROKE_REPEATS as usize];
                    state.motion_repeats = [None; RAMPED_STROKE_REPEATS as usize];
                    state.step = PositionTuneStep::Stroke {
                        rung: next,
                        kind: StrokeKind::Homing,
                        repeat: 0,
                    };
                    state.capture_running = false;
                    Ok(())
                }
                None => Err(PositionTuneOutcome::SweepExhausted.into()),
            }
        }
    };
    refresh_provenance(state);
    result
}

fn refresh_provenance(state: &mut PositionTuneState) {
    let measurement = state.accepted.or(state.last_measurement);
    let last_rung_p = state.current_rung.map_or(0, |rung| rung.raw());
    let homing_peak_abs_units = measurement.map_or(0, |m| m.homing_peak_abs_units);
    let motion_cruise_mean_abs_units = measurement.map_or(0, |m| m.motion_cruise_mean_abs_units);
    let motion_overshoot_units = measurement.map_or(0, |m| m.motion_overshoot_units);
    let motion_cruise_rms_units = measurement.map_or(0, |m| m.motion_cruise_rms_units);
    let dither_margin = measurement.map_or(0, |m| dither_margin_milli(m.ceiling_ratios_milli));
    let (min_overshoot_p, min_overshoot_units) = state
        .min_overshoot_so_far
        .map_or((0, 0), |(p, units)| (p.raw(), units));

    state.provenance = PositionTuneProvenance {
        predicted_start_p: state.predicted_start.raw(),
        last_rung_p,
        min_overshoot_p,
        min_overshoot_units,
        bound_units: state.bound_units,
        homing_speed_mrev_s: state.homing_leg.map_or(0, |leg| leg.speed_mrev_s),
        motion_speed_mrev_s: state.motion_leg.map_or(0, |leg| leg.speed_mrev_s),
        homing_peak_abs_units,
        motion_cruise_mean_abs_units,
        motion_overshoot_units,
        motion_cruise_rms_units,
        nominal_bandwidth_hz: nominal_bandwidth_hz(last_rung_p),
        dither_margin_milli: dither_margin,
        rungs_measured: state.rungs_measured,
        stimulus_feedforward_paths: state.feedforward_paths_active,
        velocity_ceiling_reached: u8::from(state.ceiling_reached),
        peak_velocity_mrev_s: peak_velocity_mrev_s(
            state.rung_peak_milli_counts_per_s,
            state.counts_per_rev,
        ),
        tune_step: tune_step_code(state.step),
    };
}

pub(crate) fn tune_step_code(step: PositionTuneStep) -> u8 {
    match step {
        PositionTuneStep::Predict | PositionTuneStep::Done => 0,
        PositionTuneStep::Stroke {
            kind: StrokeKind::Homing,
            ..
        } => 1,
        PositionTuneStep::Stroke {
            kind: StrokeKind::Motion,
            ..
        } => 2,
        PositionTuneStep::AwaitRestore { .. } | PositionTuneStep::CeilingStop { .. } => 3,
        PositionTuneStep::StandstillHold { .. } => 4,
    }
}

fn peak_velocity_mrev_s(milli_counts_per_s: u64, counts_per_rev: u32) -> u16 {
    if counts_per_rev == 0 {
        return 0;
    }
    u16::try_from(milli_counts_per_s / u64::from(counts_per_rev)).unwrap_or(u16::MAX)
}

fn nominal_bandwidth_hz(position_p_raw: u16) -> u32 {
    let numerator = i64::from(position_p_raw) * 16_777_216;
    let denominator = 60 * TAU_Q16;
    let rounded = (numerator + denominator / 2) / denominator;
    u32::try_from(rounded).unwrap_or(u32::MAX)
}

fn dither_margin_milli(ceiling_ratios_milli: [i64; 5]) -> i32 {
    let mut sorted = ceiling_ratios_milli;
    sorted.sort_unstable();
    sorted[2].clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

fn position_tune_terminal(state: &PositionTuneState, accepted: RungMeasurement) -> StepResult {
    defmt::info!(
        "position tune accepted: p={} bound_units={} rungs_measured={}",
        accepted.position_p.raw(),
        state.bound_units,
        state.rungs_measured,
    );
    let position_tune = state.provenance;
    StepResult::OuterDone(OuterResult {
        status: CommissionStatus::Accepted,
        velocity_p: state.entry.velocity_p.raw(),
        velocity_i: state.entry.velocity_i.raw(),
        velocity_limit: state.entry.velocity_limit,
        position_p: accepted.position_p.raw(),
        position_i: state.entry.position_i.raw(),
        velocity_filter_hz: state.entry.velocity_filter_hz,
        position_filter_hz: state.entry.position_filter_hz,
        flux_filter_hz: state.entry.flux_filter_hz,
        torque_filter_hz: state.entry.torque_filter_hz,
        warning_code: 0,
        provenance: TuneProvenance::default(),
        position_tune,
        position_tune_outcome_code: 0,
        motor_disarmed: false,
    })
}

/// One cooperative tick of the two-bound position-P ladder sweep.
///
/// Every failure path zeroes excitation and reports `OuterFailed`.
pub(crate) async fn step_position_tune<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    workspace: &mut TransientBuffer,
    backend: &mut B,
) -> StepResult {
    let (state, safety) = match phase {
        OuterPhase::PositionTune { state, safety } => (state, safety),
        _ => return StepResult::ReadyNow,
    };

    match state.step {
        PositionTuneStep::Predict => match predict(state, safety, backend).await {
            Ok(()) => StepResult::PhaseChanged(CommissionPhase::PositionTune),
            Err(fault) => fail_position_tune(phase, backend, fault).await,
        },
        PositionTuneStep::Stroke { rung, kind, repeat } => {
            if safety.ceiling_latched() {
                return begin_ceiling_stop(state, safety, backend, rung);
            }
            if !state.capture_running {
                return match arm_stroke(state, safety, workspace, backend, rung, kind).await {
                    Ok(()) => StepResult::At(state.next_poll_us),
                    Err(fault) => fail_position_tune(phase, backend, fault).await,
                };
            }
            let now_us = backend.elapsed_us();
            if now_us < state.next_poll_us {
                return StepResult::At(state.next_poll_us);
            }
            state.next_poll_us = now_us + u64::from(STROKE_POLL_US);
            if now_us >= state.next_sample_us && now_us < state.deadline_us {
                let position_error = match read_sub_register(backend, PID_ERROR_PID_POSITION_ERROR)
                    .await
                {
                    Ok(value) => value,
                    Err(error) => return fail_position_tune(phase, backend, error.into()).await,
                };
                workspace.push(position_error);
                if let Err(error) = advance_sample(state, state.sample_period_us, now_us) {
                    return fail_position_tune(phase, backend, error.into()).await;
                }
            }
            if now_us < state.deadline_us {
                return StepResult::At(state.next_poll_us);
            }
            merge_primitive_peak(state, safety);
            safety.end_primitive();
            backend.stimulus_stop();
            let timing = state
                .stroke_timing
                .expect("timing is captured at admission");
            let samples = workspace.samples();
            let mut signed = [0_i32; TRANSIENT_LEN];
            for (slot, &raw) in signed.iter_mut().zip(samples) {
                *slot = raw as i32;
            }
            let [forward, reverse] =
                metrics::leg_windows(&timing, state.sample_period_us, samples.len());
            let measured = match (
                metrics::leg_metrics(&signed[..samples.len()], &forward),
                metrics::leg_metrics(&signed[..samples.len()], &reverse),
            ) {
                (Ok(a), Ok(b)) => metrics::worst_of(&a, &b),
                (Err(outcome), _) | (_, Err(outcome)) => {
                    return fail_position_tune(phase, backend, outcome.into()).await;
                }
            };
            let repeats = match kind {
                StrokeKind::Homing => &mut state.homing_repeats,
                StrokeKind::Motion => &mut state.motion_repeats,
            };
            repeats[usize::from(repeat)] = Some(measured);
            state.capture_running = false;
            state.step = match (kind, repeat + 1 < RAMPED_STROKE_REPEATS) {
                (kind, true) => PositionTuneStep::Stroke {
                    rung,
                    kind,
                    repeat: repeat + 1,
                },
                (StrokeKind::Homing, false) => PositionTuneStep::Stroke {
                    rung,
                    kind: StrokeKind::Motion,
                    repeat: 0,
                },
                (StrokeKind::Motion, false) => {
                    backend.set_stimulus_feedforward(false);
                    PositionTuneStep::AwaitRestore {
                        rung,
                        since_us: now_us,
                    }
                }
            };
            StepResult::ReadyNow
        }
        PositionTuneStep::AwaitRestore { rung, since_us } => {
            if safety.ceiling_latched() {
                return begin_ceiling_stop(state, safety, backend, rung);
            }
            let now_us = backend.elapsed_us();
            if backend.stimulus_feedforward_idle() {
                state.step = PositionTuneStep::StandstillHold {
                    rung,
                    capture_index: 0,
                };
                return StepResult::ReadyNow;
            }
            if now_us.saturating_sub(since_us) >= u64::from(RESTORE_WAIT_US) {
                return fail_position_tune(
                    phase,
                    backend,
                    PositionTuneOutcome::RestorationTimeout.into(),
                )
                .await;
            }
            StepResult::At(now_us + u64::from(STROKE_POLL_US))
        }
        PositionTuneStep::StandstillHold {
            rung,
            capture_index,
        } => {
            if safety.ceiling_latched() {
                return begin_ceiling_stop(state, safety, backend, rung);
            }
            if !state.capture_running {
                return match arm_standstill_hold(state, safety, workspace, backend).await {
                    Ok(()) => StepResult::At(state.next_sample_us),
                    Err(fault) => fail_position_tune(phase, backend, fault).await,
                };
            }
            let now_us = backend.elapsed_us();
            if now_us < state.next_sample_us {
                return StepResult::At(state.next_sample_us);
            }
            let raw_target = match read_capture(backend, PID_TORQUE_FLUX_TARGET).await {
                Ok(value) => value,
                Err(error) => return fail_position_tune(phase, backend, error.into()).await,
            };
            let torque_target = PidTorqueFluxTarget::from(raw_target).pid_torque_target();
            workspace.push(i32::from(torque_target) as u32);
            let now_us = backend.elapsed_us();
            if now_us < state.deadline_us {
                return match advance_sample(state, STANDSTILL_SAMPLE_PERIOD_US, now_us) {
                    Ok(next_sample_us) => StepResult::At(next_sample_us),
                    Err(error) => fail_position_tune(phase, backend, error.into()).await,
                };
            }
            merge_primitive_peak(state, safety);
            safety.end_primitive();
            let excursion = match standstill_excursion(workspace) {
                Ok(value) => value,
                Err(outcome) => return fail_position_tune(phase, backend, outcome.into()).await,
            };
            let Some(slot) = state.dither_captures.get_mut(usize::from(capture_index)) else {
                return fail_position_tune(
                    phase,
                    backend,
                    CommissionError::RungIndexOutOfRange.into(),
                )
                .await;
            };
            *slot = excursion;
            if capture_index + 1 < STANDSTILL_CAPTURE_COUNT {
                state.step = PositionTuneStep::StandstillHold {
                    rung,
                    capture_index: capture_index + 1,
                };
                state.capture_running = false;
                return StepResult::ReadyNow;
            }
            match resolve_completed_rung(state, rung) {
                Ok(()) => match state.accepted {
                    Some(accepted) => position_tune_terminal(state, accepted),
                    None => StepResult::ReadyNow,
                },
                Err(fault) => fail_position_tune(phase, backend, fault).await,
            }
        }
        PositionTuneStep::CeilingStop { rung, since_us } => {
            let now_us = backend.elapsed_us();
            if !backend.stimulus_feedforward_idle() {
                if now_us.saturating_sub(since_us) >= u64::from(RESTORE_WAIT_US) {
                    return fail_position_tune(
                        phase,
                        backend,
                        PositionTuneOutcome::RestorationTimeout.into(),
                    )
                    .await;
                }
                return StepResult::At(now_us + u64::from(STROKE_POLL_US));
            }
            let Some(candidate) = state.candidate else {
                defmt::warn!(
                    "position tune rung: p={} decision={}",
                    rung.raw(),
                    RungDecision::Conflict(selection::ConflictKind::VelocityCeiling),
                );
                return fail_position_tune(phase, backend, PositionTuneOutcome::Conflict.into())
                    .await;
            };
            if let Err(error) = write_register(
                backend,
                PID_POSITION_P_POSITION_I,
                PidGainPair::new(candidate.position_p, PidIGain::ZERO).register_value(),
            )
            .await
            {
                return fail_position_tune(phase, backend, error.into()).await;
            }
            if let Err(error) =
                write_sub_register(backend, PID_ERROR_PID_POSITION_ERROR_SUM, 0).await
            {
                return fail_position_tune(phase, backend, error.into()).await;
            }
            state.accepted = Some(candidate);
            state.step = PositionTuneStep::Done;
            refresh_provenance(state);
            position_tune_terminal(state, candidate)
        }
        PositionTuneStep::Done => match state.accepted {
            Some(accepted) => position_tune_terminal(state, accepted),
            None => {
                fail_position_tune(phase, backend, CommissionError::MissingCaptureState.into())
                    .await
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{
        MockBackend, MockEvent, SpiOp, StimulusOp, current_limit, sample_entry_snapshot,
        sample_outer_params,
    };
    use crate::commissioning::outer::OuterMotionBudget;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::pid::PidPGain;

    fn position_tune_budget() -> OuterMotionBudget {
        OuterMotionBudget::test_budget(200, 2_000, 3_000, 1_000, 200_000)
    }

    fn position_tune_entry_snapshot() -> OuterGainRollbackSnapshot {
        OuterGainRollbackSnapshot {
            velocity_p: PidPGain::new(350).unwrap(),
            velocity_i: PidIGain::new(1_280).unwrap(),
            position_p: PidPGain::new(640).unwrap(),
            position_i: PidIGain::new(7).unwrap(),
            velocity_limit: 500_000,
            voltage_limit: 16_000,
            velocity_filter_hz: 1_100,
            torque_filter_hz: 1_200,
            position_filter_hz: 1_300,
            flux_filter_hz: 1_400,
        }
    }

    fn position_tune_phase() -> OuterPhase {
        OuterPhase::PositionTune {
            state: PositionTuneState::new(
                position_tune_entry_snapshot(),
                CommissionProfile::Balanced,
            ),
            safety: OuterSafetyLimits::new(0, position_tune_budget()),
        }
    }

    fn position_tune_backend() -> MockBackend {
        let mut backend = MockBackend::new();
        backend.spi_elapsed_step_us = 0;
        backend.elapsed_us_value = 1_000_000;
        backend.current_limit_value = current_limit(5_000);
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
        backend.configured_physical_width = Some(8);
        backend.configured_planner_steps = Some(8_192);
        let _ = backend.registers.insert(
            PID_VELOCITY_P_VELOCITY_I.addr(),
            PidGainPair::new(PidPGain::new(350).unwrap(), PidIGain::ZERO).register_value(),
        );
        backend
    }

    fn torque_target_register(torque: i32) -> u32 {
        PidTorqueFluxTarget::new()
            .with_pid_torque_target(torque as i16)
            .into()
    }

    fn quantization_only(rung_raw: u16, k_q16: i64) -> i32 {
        i32::try_from(k_q16.saturating_mul(i64::from(rung_raw)) / 65_536).unwrap_or(i32::MAX)
    }

    fn drive_sweep(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        error_for: impl Fn(&PositionTuneState, u16) -> i32,
        dither_for: impl Fn(u16, i64) -> i32,
    ) -> StepResult {
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;

        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &*phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Predict => {}
                PositionTuneStep::Stroke { .. } => {
                    let sample_index = workspace.len();
                    let error = error_for(state, sample_index);
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                }
                PositionTuneStep::AwaitRestore { .. } => {
                    backend
                        .tmc_state
                        .feedforward_surfaces_idle
                        .store(true, Relaxed);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let sample_index = workspace.len();
                    let predicted = dither_for(rung.raw(), state.k_q16);
                    let torque = if sample_index.is_multiple_of(2) {
                        0
                    } else {
                        predicted
                    };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::CeilingStop { .. } | PositionTuneStep::Done => {}
            }

            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(step_position_tune(phase, &mut workspace, backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => return terminal,
            }
        }
        panic!("the sweep did not reach a terminal within the tick budget");
    }

    fn predicted_start_of(phase: &mut OuterPhase, backend: &mut MockBackend) -> u16 {
        let mut scratch = TransientBuffer::new();
        let result = pollster::block_on(step_position_tune(phase, &mut scratch, backend));
        assert!(
            matches!(result, StepResult::PhaseChanged(_)),
            "predict must report a phase change, got {result:?}"
        );
        let OuterPhase::PositionTune { state, .. } = &*phase else {
            panic!("phase stays PositionTune after predict");
        };
        state.predicted_start.raw()
    }

    fn assert_ends_with_stop_and_feedforward_false(log: &[StimulusOp]) {
        let last_non_feedforward = log
            .iter()
            .rev()
            .find(|op| !matches!(op, StimulusOp::Feedforward(_)));
        assert_eq!(
            last_non_feedforward,
            Some(&StimulusOp::Stop),
            "expected Stop as the last non-feedforward stimulus op, log: {log:?}"
        );
        let last_feedforward = log
            .iter()
            .rev()
            .find(|op| matches!(op, StimulusOp::Feedforward(_)));
        assert_eq!(
            last_feedforward,
            Some(&StimulusOp::Feedforward(false)),
            "expected the final Feedforward op to be false, log: {log:?}"
        );
    }

    #[fixture_test]
    fn homing_strokes_share_the_motion_legs_velocity_envelope() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;
        let mut homing_cap = None;
        let mut motion_cap = None;

        for _ in 0..20_000 {
            backend.elapsed_us_value = clock_us;
            let _ = backend
                .registers
                .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), 50);
            match pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend)) {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => panic!("unexpected terminal {terminal:?}"),
            }
            let OuterPhase::PositionTune { state, safety } = &phase else {
                panic!("the phase stays resident");
            };
            if let (PositionTuneStep::Stroke { kind, .. }, true) =
                (state.step, state.capture_running)
            {
                let cap = safety.moving_velocity_cap_milli_counts_per_s();
                match kind {
                    StrokeKind::Homing => homing_cap = homing_cap.or(Some(cap)),
                    StrokeKind::Motion => motion_cap = motion_cap.or(Some(cap)),
                }
            }
            if motion_cap.is_some() {
                break;
            }
        }

        let OuterPhase::PositionTune { state, .. } = &phase else {
            panic!("the phase stays resident");
        };
        assert!(
            state.homing_leg.expect("planned").speed_mrev_s
                < state.motion_leg.expect("planned").speed_mrev_s,
            "fixture must plan a slower homing leg"
        );
        assert_eq!(
            homing_cap.expect("a homing stroke armed"),
            motion_cap.expect("a motion stroke armed")
        );
    }

    #[fixture_test]
    fn strokes_are_planned_in_logical_planner_steps() {
        let mut backend = position_tune_backend();
        backend.configured_planner_steps = Some(6_400);
        let mut phase = position_tune_phase();
        let mut workspace = TransientBuffer::new();

        predicted_start_of(&mut phase, &mut backend);
        let _ = pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend));

        let mut forward_steps = 0_u32;
        let mut reverse_steps = 0_u32;
        let mut forward = None;
        for op in backend.stimulus_log.iter() {
            match *op {
                StimulusOp::SetDirection(direction) => forward = Some(direction),
                StimulusOp::Queue(segment) if forward == Some(true) => {
                    forward_steps += u32::from(segment.count);
                }
                StimulusOp::Queue(segment) if forward == Some(false) => {
                    reverse_steps += u32::from(segment.count);
                }
                _ => {}
            }
        }
        let excursion_mrev = position_tune_budget().max_stroke_travel_mrev / 2;
        let expected_steps = excursion_mrev * 6_400 / 1_000;
        assert_eq!(forward_steps, expected_steps);
        assert_eq!(reverse_steps, expected_steps);
    }

    #[fixture_test]
    fn a_rung_inside_the_bound_is_accepted_on_the_start_rung() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            quantization_only,
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        let OuterPhase::PositionTune { state, .. } = &phase else {
            panic!("the phase stays resident after a successful terminal");
        };
        assert_eq!(result.position_p, state.predicted_start.raw());

        let filtered: std::vec::Vec<StimulusOp> = backend
            .stimulus_log
            .iter()
            .copied()
            .filter(|op| !matches!(op, StimulusOp::Feedforward(_)))
            .collect();
        let mut idx = 0;
        let mut stroke_count = 0;
        while idx < filtered.len() {
            assert_eq!(
                filtered[idx],
                StimulusOp::Stop,
                "stroke must open with Stop"
            );
            idx += 1;
            assert!(
                matches!(filtered[idx], StimulusOp::ResetClock(_)),
                "expected ResetClock, got {:?}",
                filtered[idx]
            );
            idx += 1;
            assert_eq!(filtered[idx], StimulusOp::SetDirection(true));
            idx += 1;
            let mut queued_forward = 0;
            while matches!(filtered[idx], StimulusOp::Queue(_)) {
                idx += 1;
                queued_forward += 1;
            }
            assert!(
                queued_forward > 0,
                "forward leg must queue at least one segment"
            );
            assert_eq!(filtered[idx], StimulusOp::SetDirection(false));
            idx += 1;
            let mut queued_reverse = 0;
            while matches!(filtered[idx], StimulusOp::Queue(_)) {
                idx += 1;
                queued_reverse += 1;
            }
            assert!(
                queued_reverse > 0,
                "reverse leg must queue at least one segment"
            );
            assert_eq!(filtered[idx], StimulusOp::Arm);
            idx += 1;
            assert_eq!(
                filtered[idx],
                StimulusOp::Stop,
                "stroke must close with Stop"
            );
            idx += 1;
            stroke_count += 1;
        }
        assert_eq!(
            stroke_count, 6,
            "expected 3 homing + 3 motion repeats for one accepted rung"
        );

        let first_feedforward_true = backend
            .stimulus_log
            .iter()
            .position(|op| *op == StimulusOp::Feedforward(true));
        assert!(
            first_feedforward_true.is_some(),
            "motion strokes must opt in feedforward"
        );
        let last_feedforward_false = backend
            .stimulus_log
            .iter()
            .rposition(|op| *op == StimulusOp::Feedforward(false));
        assert!(
            first_feedforward_true.unwrap()
                < last_feedforward_false.expect("feedforward is cleared"),
            "Feedforward(true) must precede the trailing Feedforward(false)"
        );
    }

    #[fixture_test]
    fn the_walk_advances_while_the_homing_peak_is_outside_the_bound() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let expected = next_rung(PidPGain::new(start_raw).unwrap())
            .expect("a representable next rung")
            .raw();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            move |state, _idx| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                if rung.raw() == start_raw {
                    match kind {
                        StrokeKind::Homing => 10_000,
                        StrokeKind::Motion => 50,
                    }
                } else {
                    50
                }
            },
            quantization_only,
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, expected);
    }

    #[fixture_test]
    fn an_overshoot_ratio_trip_reports_conflict_with_the_last_rung() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            move |state, sample_index| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                match kind {
                    StrokeKind::Homing => {
                        if rung.raw() == start_raw {
                            10_000
                        } else {
                            50
                        }
                    }
                    StrokeKind::Motion => {
                        let spike_over_steady = if rung.raw() == start_raw { 40 } else { 200 };
                        let ramp_samples = state
                            .stroke_timing
                            .map(|timing| timing.ramp_end_us / state.sample_period_us.max(1))
                            .unwrap_or(0);
                        let spike_index = (ramp_samples / 2).max(1);
                        if sample_index == 0 {
                            0
                        } else if u32::from(sample_index) == spike_index {
                            50 + spike_over_steady
                        } else {
                            50
                        }
                    }
                }
            },
            quantization_only,
        );

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::Conflict),
                ..
            }
        ));
    }

    #[fixture_test]
    fn a_conflict_terminal_carries_both_rungs() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_rung_raw = next_rung(PidPGain::new(start_raw).unwrap())
            .expect("a representable next rung")
            .raw();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            move |state, sample_index| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                match kind {
                    StrokeKind::Homing => {
                        if rung.raw() == start_raw {
                            10_000
                        } else {
                            50
                        }
                    }
                    StrokeKind::Motion => {
                        let spike_over_steady = if rung.raw() == start_raw { 40 } else { 200 };
                        let ramp_samples = state
                            .stroke_timing
                            .map(|timing| timing.ramp_end_us / state.sample_period_us.max(1))
                            .unwrap_or(0);
                        let spike_index = (ramp_samples / 2).max(1);
                        if sample_index == 0 {
                            0
                        } else if u32::from(sample_index) == spike_index {
                            50 + spike_over_steady
                        } else {
                            50
                        }
                    }
                }
            },
            quantization_only,
        );

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        let OuterPhase::PositionTuneFailed { provenance, .. } = &phase else {
            panic!("expected PositionTuneFailed, got {phase:?}");
        };
        assert_eq!(provenance.rungs_measured, 2);
        assert_eq!(provenance.last_rung_p, second_rung_raw);
        assert_eq!(provenance.min_overshoot_p, start_raw);
        assert!(provenance.motion_overshoot_units > provenance.min_overshoot_units);
    }

    #[fixture_test]
    fn a_refused_enqueue_stops_and_reports_stimulus_rejected() {
        let mut backend = position_tune_backend();
        backend.stimulus_refuse_after = Some(0);
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::StimulusRejected),
                ..
            }
        ));
        assert!(!backend.stimulus_log.contains(&StimulusOp::Arm));
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn insufficient_queue_capacity_is_rejected_before_any_enqueue() {
        let mut backend = position_tune_backend();
        backend.stimulus_queue_free = 1;
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::StimulusRejected),
                ..
            }
        ));
        assert!(
            !backend
                .stimulus_log
                .iter()
                .any(|op| matches!(op, StimulusOp::Queue(_))),
            "no segment may be queued once admission is refused"
        );
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn an_outer_safety_fault_mid_stroke_stops_the_stimulus_before_the_generic_path() {
        let mut backend = position_tune_backend();
        let params = sample_outer_params();
        let entry = sample_entry_snapshot();
        let mut engine =
            crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_position_tune(
                7,
                crate::state::tmc::tmc_channel0(),
                params,
                entry,
                0,
            );

        let mut clock_us = backend.elapsed_us_value;
        for _ in 0..6 {
            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(engine.step(&mut backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                other => panic!("unexpected early terminal: {other:?}"),
            }
        }

        backend.mechanical_encoder_position_counts_value = 5_000;
        backend.elapsed_us_value = clock_us;

        let terminal = pollster::block_on(engine.step(&mut backend));
        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
        ));

        let write_zero_index = backend
            .event_log
            .iter()
            .position(|event| {
                matches!(event, MockEvent::Spi(SpiOp::Write(addr, 0)) if *addr == PID_TORQUE_FLUX_TARGET.addr())
            })
            .expect("the safety-fault exit's PID_TORQUE_FLUX_TARGET zero write must be recorded");
        let preceding = &backend.event_log[..write_zero_index];
        assert!(
            preceding.len() >= 2,
            "expected abort_stimulus's pair before the safety exit's write, got {preceding:?}"
        );
        assert_eq!(
            &preceding[preceding.len() - 2..],
            [
                MockEvent::Stimulus(StimulusOp::Stop),
                MockEvent::Stimulus(StimulusOp::Feedforward(false)),
            ],
            "abort_stimulus's Stop then Feedforward(false) must be the last events before the \
             safety exit's excitation zeroing, with nothing else between them and the write"
        );
    }

    #[test]
    fn tune_step_codes_cover_every_sub_state() {
        let rung = PidPGain::new(100).unwrap();
        assert_eq!(tune_step_code(PositionTuneStep::Predict), 0);
        assert_eq!(
            tune_step_code(PositionTuneStep::Stroke {
                rung,
                kind: StrokeKind::Homing,
                repeat: 0
            }),
            1
        );
        assert_eq!(
            tune_step_code(PositionTuneStep::Stroke {
                rung,
                kind: StrokeKind::Motion,
                repeat: 2
            }),
            2
        );
        assert_eq!(
            tune_step_code(PositionTuneStep::AwaitRestore { rung, since_us: 0 }),
            3
        );
        assert_eq!(
            tune_step_code(PositionTuneStep::CeilingStop { rung, since_us: 0 }),
            3
        );
        assert_eq!(
            tune_step_code(PositionTuneStep::StandstillHold {
                rung,
                capture_index: 0
            }),
            4
        );
        assert_eq!(tune_step_code(PositionTuneStep::Done), 0);
    }

    #[test]
    fn peak_velocity_converts_to_mrev_s_and_saturates() {
        assert_eq!(peak_velocity_mrev_s(80_000_000, 4_000), 20_000);
        assert_eq!(peak_velocity_mrev_s(0, 4_000), 0);
        assert_eq!(peak_velocity_mrev_s(u64::MAX, 4_000), u16::MAX);
        assert_eq!(peak_velocity_mrev_s(1_000, 0), 0);
    }

    #[fixture_test]
    fn last_rung_p_names_the_in_flight_rung_and_the_metrics_the_completed_one() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_rung_raw = next_rung(PidPGain::new(start_raw).unwrap())
            .expect("a representable next rung")
            .raw();

        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;
        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &phase else {
                panic!("the phase stays resident");
            };
            if let PositionTuneStep::Stroke { rung, .. } = state.step
                && rung.raw() == second_rung_raw
                && state.capture_running
            {
                assert_eq!(state.provenance.last_rung_p, second_rung_raw);
                assert_eq!(state.provenance.rungs_measured, 1);
                assert_eq!(
                    state.provenance.homing_peak_abs_units, 10_000,
                    "metrics still describe the completed first rung"
                );
                assert_eq!(state.provenance.tune_step, 1);
                return;
            }
            match state.step {
                PositionTuneStep::Stroke { rung, kind, .. } => {
                    let error = if rung.raw() == start_raw && kind == StrokeKind::Homing {
                        10_000
                    } else {
                        50
                    };
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                }
                PositionTuneStep::AwaitRestore { .. } => {
                    backend
                        .tmc_state
                        .feedforward_surfaces_idle
                        .store(true, Relaxed);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let predicted = quantization_only(rung.raw(), state.k_q16);
                    let torque = if workspace.len().is_multiple_of(2) {
                        0
                    } else {
                        predicted
                    };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::Predict
                | PositionTuneStep::CeilingStop { .. }
                | PositionTuneStep::Done => {}
            }
            backend.elapsed_us_value = clock_us;
            match pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend)) {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => panic!("unexpected terminal {terminal:?}"),
            }
        }
        panic!("the second rung never armed a stroke");
    }

    #[fixture_test]
    fn a_safety_fault_after_a_completed_rung_keeps_its_evidence() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let mut workspace = TransientBuffer::new();
        let mut trace_summary_seqs = crate::commissioning::outer::TraceSummarySeqs::new();
        let mut clock_us = backend.elapsed_us_value;
        let mut second_rung_entered = false;

        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Predict => {}
                PositionTuneStep::Stroke { rung, kind, .. } => {
                    let error = if rung.raw() == start_raw {
                        match kind {
                            StrokeKind::Homing => 10_000,
                            StrokeKind::Motion => 50,
                        }
                    } else {
                        50
                    };
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                    if rung.raw() != start_raw && !second_rung_entered {
                        backend.mechanical_encoder_position_counts_value = 5_000;
                        second_rung_entered = true;
                    }
                }
                PositionTuneStep::AwaitRestore { .. } => {
                    backend
                        .tmc_state
                        .feedforward_surfaces_idle
                        .store(true, Relaxed);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let sample_index = workspace.len();
                    let predicted = quantization_only(rung.raw(), state.k_q16);
                    let torque = if sample_index.is_multiple_of(2) {
                        0
                    } else {
                        predicted
                    };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::CeilingStop { .. } | PositionTuneStep::Done => {}
            }

            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(crate::commissioning::outer::step_outer_with_trace(
                &mut phase,
                &mut workspace,
                &mut trace_summary_seqs,
                &mut backend,
            ));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => {
                    assert!(matches!(
                        terminal,
                        StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
                    ));
                    let OuterPhase::FailedWithSafety { position_tune, .. } = &phase else {
                        panic!("expected FailedWithSafety, got {phase:?}");
                    };
                    assert_eq!(position_tune.rungs_measured, 1);
                    let second_rung_raw = next_rung(PidPGain::new(start_raw).unwrap())
                        .expect("a representable next rung")
                        .raw();
                    assert_eq!(position_tune.last_rung_p, second_rung_raw);
                    assert_eq!(position_tune.homing_peak_abs_units, 10_000);
                    assert_eq!(position_tune.tune_step, 1);
                    return;
                }
            }
        }
        panic!("expected a terminal within the tick budget");
    }

    #[fixture_test]
    fn a_shutdown_mid_stroke_stops_the_stimulus_before_the_engines_safe_state_restore() {
        let mut backend = position_tune_backend();
        let params = sample_outer_params();
        let entry = sample_entry_snapshot();
        let mut engine =
            crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_position_tune(
                7,
                crate::state::tmc::tmc_channel0(),
                params,
                entry,
                0,
            );

        let mut clock_us = backend.elapsed_us_value;
        for _ in 0..6 {
            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(engine.step(&mut backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                other => panic!("unexpected early terminal: {other:?}"),
            }
        }

        backend.shutdown_requested = true;
        backend.elapsed_us_value = clock_us;
        let terminal = pollster::block_on(engine.step(&mut backend));
        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::Shutdown)
        ));

        let restore_write_index = backend
            .event_log
            .iter()
            .position(|event| {
                matches!(event, MockEvent::Spi(SpiOp::Write(addr, 0)) if *addr == MODE_RAMP_MODE_MOTION.addr())
            })
            .expect("the safe-state restore's MODE_RAMP_MODE_MOTION zero write must be recorded");
        let preceding = &backend.event_log[..restore_write_index];
        assert!(
            preceding.len() >= 2,
            "expected abort_stimulus's pair before the safe-state restore, got {preceding:?}"
        );
        assert_eq!(
            &preceding[preceding.len() - 2..],
            [
                MockEvent::Stimulus(StimulusOp::Stop),
                MockEvent::Stimulus(StimulusOp::Feedforward(false)),
            ],
            "the shutdown branch's abort_stimulus call must stop the stimulus and clear \
             feedforward before the safe-state restore begins"
        );
    }

    #[fixture_test]
    fn a_restoration_timeout_fails_the_phase() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;

        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Stroke { .. } => {
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), 50);
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let sample_index = workspace.len();
                    let predicted = quantization_only(rung.raw(), state.k_q16);
                    let torque = if sample_index.is_multiple_of(2) {
                        0
                    } else {
                        predicted
                    };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::AwaitRestore { .. }
                | PositionTuneStep::CeilingStop { .. }
                | PositionTuneStep::Predict
                | PositionTuneStep::Done => {}
            }

            backend.elapsed_us_value = clock_us;
            let result =
                pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => {
                    assert!(matches!(
                        terminal,
                        StepResult::OuterFailed(CommissionError::PositionTuneFailed)
                    ));
                    assert!(matches!(
                        phase,
                        OuterPhase::PositionTuneFailed {
                            error: CommissionError::PositionTuneFailed,
                            outcome: Some(PositionTuneOutcome::RestorationTimeout),
                            ..
                        }
                    ));
                    assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
                    return;
                }
            }
        }
        panic!("expected a terminal within the tick budget");
    }

    #[fixture_test]
    fn a_trigger_blocked_stepper_fails_at_admission() {
        let mut backend = position_tune_backend();
        backend.stimulus_trigger_blocked = true;
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        assert!(matches!(
            phase,
            OuterPhase::PositionTuneFailed {
                error: CommissionError::PositionTuneFailed,
                outcome: Some(PositionTuneOutcome::ConfigurationInvalid),
                ..
            }
        ));
        assert!(
            !backend
                .stimulus_log
                .iter()
                .any(|op| matches!(op, StimulusOp::Queue(_))),
            "no segment may be queued once the stepper is trigger-blocked"
        );
    }

    #[fixture_test]
    fn a_failure_before_the_first_motion_stroke_reports_no_feedforward_paths() {
        let mut backend = position_tune_backend();
        backend.stimulus_refuse_after = Some(0);
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        let OuterPhase::PositionTuneFailed { provenance, .. } = &phase else {
            panic!("expected PositionTuneFailed, got {phase:?}");
        };
        assert_eq!(provenance.stimulus_feedforward_paths, 0);

        let mut backend = position_tune_backend();
        backend.feedforward_path_flags_value = 0b0_0101;
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            quantization_only,
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_tune.stimulus_feedforward_paths, 0b0_0101);
    }

    #[fixture_test]
    fn every_failure_path_stops_the_stimulus_and_clears_feedforward() {
        {
            let mut backend = position_tune_backend();
            let mut phase = position_tune_phase();
            let start_raw = predicted_start_of(&mut phase, &mut backend);
            let _ = drive_sweep(
                &mut phase,
                &mut backend,
                move |state, sample_index| {
                    let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                        return 0;
                    };
                    match kind {
                        StrokeKind::Homing => {
                            if rung.raw() == start_raw {
                                10_000
                            } else {
                                50
                            }
                        }
                        StrokeKind::Motion => {
                            let spike_over_steady = if rung.raw() == start_raw { 40 } else { 200 };
                            let ramp_samples = state
                                .stroke_timing
                                .map(|timing| timing.ramp_end_us / state.sample_period_us.max(1))
                                .unwrap_or(0);
                            let spike_index = (ramp_samples / 2).max(1);
                            if sample_index == 0 {
                                0
                            } else if u32::from(sample_index) == spike_index {
                                50 + spike_over_steady
                            } else {
                                50
                            }
                        }
                    }
                },
                quantization_only,
            );
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
        {
            let mut backend = position_tune_backend();
            backend.stimulus_refuse_after = Some(0);
            let mut phase = position_tune_phase();
            let _ = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
        {
            let mut backend = position_tune_backend();
            backend.stimulus_queue_free = 1;
            let mut phase = position_tune_phase();
            let _ = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
        {
            let mut backend = position_tune_backend();
            backend.stimulus_trigger_blocked = true;
            let mut phase = position_tune_phase();
            let _ = drive_sweep(&mut phase, &mut backend, |_state, _idx| 50, |_rung, _k| 0);
            assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
        }
    }

    #[fixture_test]
    fn the_terminal_carries_every_untuned_entry_gain() {
        let entry = position_tune_entry_snapshot();
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();

        let terminal = drive_sweep(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            quantization_only,
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("a completed sweep reports OuterDone, got {terminal:?}");
        };
        assert_eq!(result.velocity_limit, entry.velocity_limit);
        assert_eq!(result.velocity_p, entry.velocity_p.raw());
        assert_eq!(result.velocity_i, entry.velocity_i.raw());
        assert_eq!(result.position_i, entry.position_i.raw());
        assert_eq!(result.velocity_filter_hz, entry.velocity_filter_hz);
        assert_eq!(result.position_filter_hz, entry.position_filter_hz);
        assert_eq!(result.flux_filter_hz, entry.flux_filter_hz);
        assert_eq!(result.torque_filter_hz, entry.torque_filter_hz);
        assert_ne!(result.position_p, entry.position_p.raw());
    }

    #[test]
    fn stroke_request_keeps_every_stroke_inside_the_primitive_duration_cap() {
        use crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource;

        // The Ouroboros bench: 120 mm stroke budget (60 mm per leg at 40 mm/rev),
        // 3 s primitive cap, 10 000 mm/s^2, 1/16 physical step width.
        let budget = OuterMotionBudget {
            requested_velocity_mrev_s: 8_333,
            machine_velocity_ceiling_mrev_s: 12_500,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 3_000,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 2_750,
            positive_position_headroom_mrev: 2_750,
            max_duration_ms: 3_000,
            homing_speed_mrev_s: 1_000,
            max_accel_mrev_s2: 250_000,
        };
        let steps_per_rev = 65_536 / 16;
        let step_timer_hz = 260_000_000;
        let cap_us = u64::from(budget.max_duration_ms) * 1_000;
        let overhead_us = u64::from(STROKE_LEAD_US) + u64::from(STROKE_DURATION_MARGIN_US);

        // A fast motion leg covers the full budget excursion well inside the cap.
        let fast = stroke_request(12_500, &budget, steps_per_rev, step_timer_hz);
        assert_eq!(fast.excursion_mrev, 1_500);

        // 40 mm/s over 60 mm is 1.5 s per leg: the bare stroke alone fills the cap, which
        // is what tripped the outer safety duration fault on hardware. The fitted leg
        // must leave room for the lead-in and the margin.
        let slow = stroke_request(1_000, &budget, steps_per_rev, step_timer_hz);
        assert!(slow.excursion_mrev < 1_500, "{}", slow.excursion_mrev);
        let slow_profile = plan_stroke(&slow).expect("the fitted homing stroke plans");
        assert!(u64::from(slow_profile.stroke_us()) + overhead_us <= cap_us);
        assert!(slow_profile.leg.cruise_us >= MIN_CRUISE_US);
        assert_eq!(
            slow_profile.leg.speed_mrev_s, 1_000,
            "fitting trims distance, not speed"
        );

        // Klipper's default 5 mm/s homing speed must plan too.
        let crawl = stroke_request(125, &budget, steps_per_rev, step_timer_hz);
        let crawl_profile = plan_stroke(&crawl).expect("the fitted crawl stroke plans");
        assert!(u64::from(crawl_profile.stroke_us()) + overhead_us <= cap_us);
        assert!(crawl_profile.leg.cruise_us >= MIN_CRUISE_US);
    }

    #[test]
    fn nominal_bandwidth_hz_matches_the_host_formula() {
        let cases: [(u16, u32); 7] = [
            (0, 0),
            (1, 1),
            (256, 174),
            (640, 435),
            (1_000, 679),
            (32_768, 22_251),
            (u16::MAX, 44_502),
        ];
        for (position_p_raw, expected) in cases {
            assert_eq!(
                nominal_bandwidth_hz(position_p_raw),
                expected,
                "nominal_bandwidth_hz({position_p_raw}) drifted from the pinned value"
            );
        }
    }

    fn drive_with_safety(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        error_for: impl Fn(&PositionTuneState, u16) -> i32,
        encoder_counts_per_ms: impl Fn(&PositionTuneState, &OuterSafetyLimits) -> Option<u32>,
    ) -> StepResult {
        drive_with_safety_and_restore(phase, backend, error_for, encoder_counts_per_ms, true)
    }

    fn drive_with_safety_and_restore(
        phase: &mut OuterPhase,
        backend: &mut MockBackend,
        error_for: impl Fn(&PositionTuneState, u16) -> i32,
        encoder_counts_per_ms: impl Fn(&PositionTuneState, &OuterSafetyLimits) -> Option<u32>,
        restore_completes: bool,
    ) -> StepResult {
        let mut workspace = TransientBuffer::new();
        let mut trace_summary_seqs = crate::commissioning::outer::TraceSummarySeqs::new();
        let mut clock_us = backend.elapsed_us_value;
        let mut motion_origin: Option<(u64, u32)> = None;

        for _ in 0..40_000 {
            let OuterPhase::PositionTune { state, safety } = &*phase else {
                panic!("the phase must stay resident until it reports a terminal");
            };
            match state.step {
                PositionTuneStep::Predict | PositionTuneStep::CeilingStop { .. } => {}
                PositionTuneStep::Stroke { .. } => {
                    let sample_index = workspace.len();
                    let error = error_for(state, sample_index);
                    let _ = backend
                        .registers
                        .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), error as u32);
                }
                PositionTuneStep::AwaitRestore { .. } => {
                    if restore_completes {
                        backend
                            .tmc_state
                            .feedforward_surfaces_idle
                            .store(true, Relaxed);
                    }
                }
                PositionTuneStep::StandstillHold { rung, .. } => {
                    let predicted = quantization_only(rung.raw(), state.k_q16);
                    let torque = if workspace.len().is_multiple_of(2) {
                        0
                    } else {
                        predicted
                    };
                    let _ = backend.registers.insert(
                        PID_TORQUE_FLUX_TARGET.addr(),
                        torque_target_register(torque),
                    );
                }
                PositionTuneStep::Done => {}
            }
            if matches!(state.step, PositionTuneStep::CeilingStop { .. }) {
                backend
                    .tmc_state
                    .feedforward_surfaces_idle
                    .store(true, Relaxed);
            }
            if let Some(d) = encoder_counts_per_ms(state, safety) {
                let (origin_us, origin_counts) = *motion_origin
                    .get_or_insert((clock_us, backend.mechanical_encoder_position_counts_value));
                let elapsed_ms = u32::try_from((clock_us - origin_us) / 1_000).unwrap();
                backend.mechanical_encoder_position_counts_value =
                    origin_counts.wrapping_add(elapsed_ms.wrapping_mul(d));
            }

            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(crate::commissioning::outer::step_outer_with_trace(
                phase,
                &mut workspace,
                &mut trace_summary_seqs,
                backend,
            ));
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => return terminal,
            }
        }
        panic!("the sweep did not reach a terminal within the tick budget");
    }

    fn between_ceiling_and_cap(safety: &OuterSafetyLimits) -> u32 {
        let cap = safety.moving_velocity_cap_milli_counts_per_s();
        let ceiling = cap * selection::VELOCITY_CEILING_NUM / selection::VELOCITY_CEILING_DEN;
        let d = u32::try_from(ceiling / 1_000_000 + 1).unwrap();
        assert!(
            u64::from(d) * 1_000_000 < cap,
            "the fixture must leave room between the ceiling and the cap"
        );
        d
    }

    fn over_the_cap(safety: &OuterSafetyLimits) -> u32 {
        let cap = safety.moving_velocity_cap_milli_counts_per_s();
        let d = u32::try_from(cap / 1_000_000 + 2).unwrap();
        assert!(d < 1_000, "stay under the quarter-turn guard");
        d
    }

    fn envelope_mrev_s_of(provenance: &PositionTuneProvenance) -> u64 {
        let v = u64::from(
            provenance
                .homing_speed_mrev_s
                .max(provenance.motion_speed_mrev_s),
        );
        ((v * 3).div_ceil(2)).max(v + 2_000)
    }

    fn ceiling_mrev_s_of(provenance: &PositionTuneProvenance) -> u64 {
        envelope_mrev_s_of(provenance) * selection::VELOCITY_CEILING_NUM
            / selection::VELOCITY_CEILING_DEN
    }

    fn ceiling_trip_error(start_raw: u16) -> impl Fn(&PositionTuneState, u16) -> i32 {
        move |state, _idx| {
            let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                return 0;
            };
            match (rung.raw() == start_raw, kind) {
                (true, StrokeKind::Homing) => 50,
                (true, StrokeKind::Motion) => 10_000,
                (false, _) => 50,
            }
        }
    }

    fn in_motion_stroke_of(state: &PositionTuneState, rung_raw: u16) -> bool {
        matches!(
            state.step,
            PositionTuneStep::Stroke { rung, kind: StrokeKind::Motion, .. }
                if rung.raw() == rung_raw && state.capture_running
        )
    }

    #[fixture_test]
    fn a_ceiling_trip_accepts_the_candidate_and_names_the_interrupted_rung() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_raw = next_rung(PidPGain::new(start_raw).unwrap()).unwrap().raw();

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            ceiling_trip_error(start_raw),
            move |state, safety| {
                in_motion_stroke_of(state, second_raw).then(|| between_ceiling_and_cap(safety))
            },
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, start_raw, "the candidate is accepted");
        assert_eq!(result.position_tune.velocity_ceiling_reached, 1);
        assert_eq!(result.position_tune.last_rung_p, second_raw);
        assert_eq!(result.position_tune.rungs_measured, 1);
        assert_eq!(result.position_tune.motion_cruise_mean_abs_units, 10_000);
        assert_eq!(result.position_tune.tune_step, 0);
        let peak = u64::from(result.position_tune.peak_velocity_mrev_s);
        assert!(
            peak + 1 >= ceiling_mrev_s_of(&result.position_tune)
                && peak < envelope_mrev_s_of(&result.position_tune),
            "the peak is the window that crossed the ceiling, below the cap: {peak}"
        );

        let candidate_write = backend
            .event_log
            .iter()
            .rposition(|event| {
                matches!(
                    event,
                    MockEvent::Spi(SpiOp::Write(addr, value))
                        if *addr == PID_POSITION_P_POSITION_I.addr()
                            && *value == PidGainPair::new(
                                PidPGain::new(start_raw).unwrap(),
                                PidIGain::ZERO
                            ).register_value()
                )
            })
            .expect("the candidate P is written back before the terminal");
        let last_stop = backend
            .event_log
            .iter()
            .rposition(|event| matches!(event, MockEvent::Stimulus(StimulusOp::Stop)))
            .expect("the ceiling stop halts the stimulus");
        assert!(
            last_stop < candidate_write,
            "stop first, then the candidate rewrite"
        );
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn a_ceiling_trip_stops_the_stimulus_in_the_tick_that_latched() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let mut workspace = TransientBuffer::new();
        let mut trace_summary_seqs = crate::commissioning::outer::TraceSummarySeqs::new();
        let mut clock_us = backend.elapsed_us_value;
        let mut origin: Option<(u64, u32, u32)> = None;
        for _ in 0..40_000 {
            let OuterPhase::PositionTune { state, safety } = &phase else {
                panic!("the phase stays resident");
            };
            let _ = backend
                .registers
                .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), 50);
            if in_motion_stroke_of(state, start_raw) {
                let (o_us, o_counts, d) = *origin.get_or_insert((
                    clock_us,
                    backend.mechanical_encoder_position_counts_value,
                    between_ceiling_and_cap(safety),
                ));
                let ms = u32::try_from((clock_us - o_us) / 1_000).unwrap();
                backend.mechanical_encoder_position_counts_value = o_counts.wrapping_add(ms * d);
            }
            let latched_before = safety.ceiling_latched();
            let stops_before = backend
                .stimulus_log
                .iter()
                .filter(|op| **op == StimulusOp::Stop)
                .count();
            backend.elapsed_us_value = clock_us;
            let result = pollster::block_on(crate::commissioning::outer::step_outer_with_trace(
                &mut phase,
                &mut workspace,
                &mut trace_summary_seqs,
                &mut backend,
            ));
            let OuterPhase::PositionTune { state, safety } = &phase else {
                panic!("the phase stays resident");
            };
            if !latched_before && safety.ceiling_latched() {
                let stops_after = backend
                    .stimulus_log
                    .iter()
                    .filter(|op| **op == StimulusOp::Stop)
                    .count();
                assert!(
                    stops_after > stops_before,
                    "the latching tick must stop the stimulus"
                );
                assert!(matches!(state.step, PositionTuneStep::CeilingStop { .. }));
                assert!(!state.capture_running);
                return;
            }
            match result {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => panic!("unexpected terminal before the latch: {terminal:?}"),
            }
        }
        panic!("the ceiling never latched");
    }

    #[fixture_test]
    fn a_ceiling_trip_with_no_candidate_is_a_conflict_with_the_flag_set() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            move |state, safety| {
                in_motion_stroke_of(state, start_raw).then(|| between_ceiling_and_cap(safety))
            },
        );

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        let OuterPhase::PositionTuneFailed {
            outcome,
            provenance,
            ..
        } = &phase
        else {
            panic!("expected PositionTuneFailed, got {phase:?}");
        };
        assert_eq!(*outcome, Some(PositionTuneOutcome::Conflict));
        assert_eq!(provenance.velocity_ceiling_reached, 1);
        assert_eq!(provenance.rungs_measured, 0);
        assert_eq!(provenance.last_rung_p, start_raw);
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn a_rung_that_failed_the_homing_bound_is_not_accepted_at_the_ceiling() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_raw = next_rung(PidPGain::new(start_raw).unwrap()).unwrap().raw();

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            move |state, _idx| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                if rung.raw() == start_raw && kind == StrokeKind::Homing {
                    10_000
                } else {
                    50
                }
            },
            move |state, safety| {
                in_motion_stroke_of(state, second_raw).then(|| between_ceiling_and_cap(safety))
            },
        );

        assert!(matches!(
            terminal,
            StepResult::OuterFailed(CommissionError::PositionTuneFailed)
        ));
        let OuterPhase::PositionTuneFailed {
            outcome,
            provenance,
            ..
        } = &phase
        else {
            panic!("expected PositionTuneFailed, got {phase:?}");
        };
        assert_eq!(*outcome, Some(PositionTuneOutcome::Conflict));
        assert_eq!(provenance.velocity_ceiling_reached, 1);
        assert_eq!(provenance.rungs_measured, 1);
        assert_eq!(provenance.last_rung_p, second_raw);
    }

    #[fixture_test]
    fn a_ceiling_latch_during_a_standstill_hold_resolves_without_arming_another_primitive() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_raw = next_rung(PidPGain::new(start_raw).unwrap()).unwrap().raw();

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            ceiling_trip_error(start_raw),
            move |state, safety| match state.step {
                PositionTuneStep::StandstillHold { rung, .. }
                    if rung.raw() == second_raw && state.capture_running =>
                {
                    Some(between_ceiling_and_cap(safety))
                }
                _ => None,
            },
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, start_raw);
        assert_eq!(result.position_tune.velocity_ceiling_reached, 1);
        assert_eq!(result.position_tune.last_rung_p, second_raw);
        let arms_after_latch = backend
            .stimulus_log
            .iter()
            .rposition(|op| *op == StimulusOp::Stop)
            .map(|stop| {
                backend.stimulus_log[stop..]
                    .iter()
                    .filter(|op| **op == StimulusOp::Arm)
                    .count()
            })
            .unwrap_or(0);
        assert_eq!(
            arms_after_latch, 0,
            "nothing is armed after the ceiling stop"
        );
    }

    #[fixture_test]
    fn a_latch_set_between_primitives_is_handled_before_the_next_stroke_arms() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let mut workspace = TransientBuffer::new();
        let mut clock_us = backend.elapsed_us_value;
        for _ in 0..20_000 {
            let OuterPhase::PositionTune { state, .. } = &phase else {
                panic!("the phase stays resident");
            };
            if matches!(state.step, PositionTuneStep::Stroke { repeat: 1, .. })
                && !state.capture_running
            {
                break;
            }
            let _ = backend
                .registers
                .insert(PID_ERROR_PID_POSITION_ERROR.data.addr(), 50);
            backend.elapsed_us_value = clock_us;
            match pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend)) {
                StepResult::At(next_us) => clock_us = next_us,
                StepResult::ReadyNow | StepResult::PhaseChanged(_) => {}
                terminal => panic!("unexpected terminal {terminal:?}"),
            }
        }
        let OuterPhase::PositionTune { state, safety } = &mut phase else {
            panic!("the phase stays resident");
        };
        assert!(matches!(
            state.step,
            PositionTuneStep::Stroke { repeat: 1, .. }
        ));
        safety.force_ceiling_latch_for_test();
        let arms_before = backend
            .stimulus_log
            .iter()
            .filter(|op| **op == StimulusOp::Arm)
            .count();

        backend.elapsed_us_value = clock_us;
        let result =
            pollster::block_on(step_position_tune(&mut phase, &mut workspace, &mut backend));

        assert!(matches!(result, StepResult::ReadyNow));
        let OuterPhase::PositionTune { state, .. } = &phase else {
            panic!("the phase stays resident");
        };
        assert!(
            matches!(state.step, PositionTuneStep::CeilingStop { rung, .. } if rung.raw() == start_raw),
            "the latch is handled before arm_stroke could clear it, got {:?}",
            state.step
        );
        let arms_after = backend
            .stimulus_log
            .iter()
            .filter(|op| **op == StimulusOp::Arm)
            .count();
        assert_eq!(
            arms_before, arms_after,
            "no new stroke is armed over a latched ceiling"
        );
        assert_ends_with_stop_and_feedforward_false(&backend.stimulus_log);
    }

    #[fixture_test]
    fn a_safety_fault_merges_the_faulting_primitives_peak() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            move |state, safety| match state.step {
                PositionTuneStep::Stroke {
                    rung,
                    kind: StrokeKind::Homing,
                    ..
                } if rung.raw() == start_raw && state.capture_running => Some(over_the_cap(safety)),
                _ => None,
            },
        );

        assert!(
            matches!(
                terminal,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ),
            "terminal was {terminal:?}"
        );
        let OuterPhase::FailedWithSafety {
            position_tune,
            safety_fault,
            ..
        } = &phase
        else {
            panic!("expected FailedWithSafety, got {phase:?}");
        };
        assert_eq!(
            safety_fault.reason,
            crate::commissioning::types::OUTER_SAFETY_FAULT_VELOCITY
        );
        assert_eq!(position_tune.tune_step, 1);
        assert_eq!(position_tune.last_rung_p, start_raw);
        assert!(
            u64::from(position_tune.peak_velocity_mrev_s) > envelope_mrev_s_of(position_tune),
            "the faulting window, over the cap, is the reported peak"
        );
        assert_eq!(position_tune.velocity_ceiling_reached, 0);
    }

    fn reproduction_case_budget() -> OuterMotionBudget {
        OuterMotionBudget {
            requested_velocity_mrev_s: 12_500,
            machine_velocity_ceiling_mrev_s: 12_500,
            requested_velocity_source:
                crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 3_000,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: 4_000,
            positive_position_headroom_mrev: 4_000,
            max_duration_ms: 60_000,
            homing_speed_mrev_s: 4_375,
            max_accel_mrev_s2: 200_000,
        }
    }

    #[fixture_test]
    fn the_reproduction_case_accepts_p176_and_names_p248() {
        let mut backend = position_tune_backend();
        backend.velocity_feedforward_gain_value = 1_750;
        backend.stall_threshold_units_value = 1_638;
        let mut phase = OuterPhase::PositionTune {
            state: PositionTuneState::new(
                position_tune_entry_snapshot(),
                CommissionProfile::Balanced,
            ),
            safety: OuterSafetyLimits::new(0, reproduction_case_budget()),
        };
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        assert_eq!(start_raw, 176);
        assert_eq!(next_rung(PidPGain::new(176).unwrap()).unwrap().raw(), 248);

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            ceiling_trip_error(176),
            |state, _safety| in_motion_stroke_of(state, 248).then_some(70),
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        let provenance = &result.position_tune;
        assert_eq!(result.position_p, 176);
        assert_eq!(provenance.last_rung_p, 248);
        assert_eq!(provenance.velocity_ceiling_reached, 1);
        assert_eq!(provenance.peak_velocity_mrev_s, 17_500);
        assert_eq!(provenance.motion_speed_mrev_s, 12_500);
        assert_eq!(provenance.homing_speed_mrev_s, 4_375);
        assert_eq!(provenance.rungs_measured, 1);
        assert_eq!(provenance.motion_cruise_mean_abs_units, 10_000);
        assert_eq!(envelope_mrev_s_of(provenance), 18_750);
        assert_eq!(ceiling_mrev_s_of(provenance), 15_000);
    }

    #[fixture_test]
    fn a_later_qualifying_rung_replaces_the_candidate() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let second_raw = next_rung(PidPGain::new(start_raw).unwrap()).unwrap().raw();
        let third_raw = next_rung(PidPGain::new(second_raw).unwrap()).unwrap().raw();

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            move |state, _idx| {
                let PositionTuneStep::Stroke { rung, kind, .. } = state.step else {
                    return 0;
                };
                match (rung.raw() == third_raw, kind) {
                    (false, StrokeKind::Motion) => 10_000,
                    (false, StrokeKind::Homing) | (true, _) => 50,
                }
            },
            move |state, safety| {
                in_motion_stroke_of(state, third_raw).then(|| between_ceiling_and_cap(safety))
            },
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        assert_eq!(result.position_p, second_raw);
        assert_eq!(result.position_tune.last_rung_p, third_raw);
        assert_eq!(result.position_tune.velocity_ceiling_reached, 1);
    }

    #[fixture_test]
    fn the_rung_peak_keeps_the_first_primitives_window() {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);
        let first_homing_speed = core::cell::Cell::new(None);
        let burst_ticks_left = core::cell::Cell::new(20_u32);

        let terminal = drive_with_safety(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            |state, safety| match state.step {
                PositionTuneStep::Stroke {
                    rung,
                    kind: StrokeKind::Homing,
                    repeat: 0,
                } if rung.raw() == start_raw
                    && state.capture_running
                    && burst_ticks_left.get() > 0 =>
                {
                    burst_ticks_left.set(burst_ticks_left.get() - 1);
                    let d = (between_ceiling_and_cap(safety) / 2).max(1);
                    first_homing_speed.set(Some(d));
                    Some(d)
                }
                _ => None,
            },
        );

        let StepResult::OuterDone(result) = terminal else {
            panic!("expected OuterDone, got {terminal:?}");
        };
        let d = first_homing_speed
            .get()
            .expect("the first homing stroke moved");
        assert_eq!(result.position_p, start_raw);
        assert_eq!(result.position_tune.velocity_ceiling_reached, 0);
        assert_eq!(
            u64::from(result.position_tune.peak_velocity_mrev_s),
            u64::from(d) * 1_000_000 / 4_000
        );
    }

    fn safety_fault_tune_step_while(
        moving: impl Fn(&PositionTuneState, u16) -> bool,
        restore_completes: bool,
    ) -> u8 {
        let mut backend = position_tune_backend();
        let mut phase = position_tune_phase();
        let start_raw = predicted_start_of(&mut phase, &mut backend);

        let terminal = drive_with_safety_and_restore(
            &mut phase,
            &mut backend,
            |_state, _idx| 50,
            |state, safety| moving(state, start_raw).then(|| over_the_cap(safety)),
            restore_completes,
        );

        assert!(
            matches!(
                terminal,
                StepResult::OuterFailed(CommissionError::SafetyEnvelopeViolation)
            ),
            "expected a safety fault, got {terminal:?}"
        );
        let OuterPhase::FailedWithSafety { position_tune, .. } = &phase else {
            panic!("expected FailedWithSafety, got {phase:?}");
        };
        assert_eq!(position_tune.last_rung_p, start_raw);
        assert_eq!(position_tune.velocity_ceiling_reached, 0);
        position_tune.tune_step
    }

    #[fixture_test]
    fn a_safety_fault_in_a_motion_stroke_reports_the_motion_sub_state() {
        let tune_step = safety_fault_tune_step_while(in_motion_stroke_of, true);
        assert_eq!(tune_step, 2);
    }

    #[fixture_test]
    fn a_safety_fault_in_a_restore_wait_reports_the_restore_sub_state() {
        let tune_step = safety_fault_tune_step_while(
            |state, start_raw| {
                matches!(
                    state.step,
                    PositionTuneStep::AwaitRestore { rung, .. } if rung.raw() == start_raw
                )
            },
            false,
        );
        assert_eq!(tune_step, 3);
    }

    #[fixture_test]
    fn a_safety_fault_in_a_standstill_hold_reports_the_hold_sub_state() {
        let tune_step = safety_fault_tune_step_while(
            |state, start_raw| {
                matches!(
                    state.step,
                    PositionTuneStep::StandstillHold { rung, .. }
                        if rung.raw() == start_raw && state.capture_running
                )
            },
            true,
        );
        assert_eq!(tune_step, 4);
    }
}
