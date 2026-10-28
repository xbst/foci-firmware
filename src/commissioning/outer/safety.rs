//! Outer commissioning safety-envelope checks.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain, EncoderMathError};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::{
    OUTER_SAFETY_FAULT_DURATION, OUTER_SAFETY_FAULT_INVALID_BUDGET,
    OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_POSITION,
    OUTER_SAFETY_FAULT_POSITION_LOCAL, OUTER_SAFETY_FAULT_QUARTER_TURN,
    OUTER_SAFETY_FAULT_UNUSABLE_BUDGET, OUTER_SAFETY_FAULT_VELOCITY, OuterSafetyFault,
};

use super::limits::{
    MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS, MECHANICAL_ID_MIN_TRAVEL_COUNTS,
    MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS, POSITION_WINDOW_COUNTS,
};
use super::velocity::sweep_plan::{
    ENVELOPE_ABSOLUTE_MARGIN_MREV_S, ENVELOPE_PROPORTIONAL_DEN, ENVELOPE_PROPORTIONAL_NUM,
    MAX_SAFETY_OBSERVATION_GAP_US, RequestedVelocitySource, TRANSIENT_ABSOLUTE_MARGIN_MREV_S,
    TRANSIENT_ALLOWANCE_US, TRANSIENT_PROPORTIONAL_DEN, TRANSIENT_PROPORTIONAL_NUM,
};
use super::{mechanical_id_velocity_cap_counts_per_ms, position_window_counts, state::OuterPhase};

const MIN_USEFUL_DURATION_MS: u16 = 4;
pub(super) const OUTER_SAFETY_SAMPLE_PERIOD_US: u64 = 1_000;

const INTEGRAL_POSITION_RESERVE_COUNTS: Option<u32> = None;

/// Host-provided motor-local motion budget for outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterMotionBudget {
    pub(crate) requested_velocity_mrev_s: u32,
    pub(crate) machine_velocity_ceiling_mrev_s: u32,
    pub(crate) requested_velocity_source: RequestedVelocitySource,
    pub(crate) max_stroke_travel_mrev: u32,
    pub(crate) settle_travel_reserve_mrev: u32,
    pub(crate) negative_position_headroom_mrev: u32,
    pub(crate) positive_position_headroom_mrev: u32,
    pub(crate) max_duration_ms: u16,
    /// Axis homing speed, mrev/s; the homing bound's speed.
    pub(crate) homing_speed_mrev_s: u32,
    /// Machine acceleration, mrev/s^2; the stroke ramp rate.
    pub(crate) max_accel_mrev_s2: u32,
}

impl OuterMotionBudget {
    #[cfg(test)]
    pub(crate) const fn firmware_caps() -> Self {
        const FIRMWARE_MAX_TRAVEL_MREV: u32 = 2_000;
        const FIRMWARE_MAX_VELOCITY_MREV_S: u32 = 6_000;
        Self {
            requested_velocity_mrev_s: FIRMWARE_MAX_VELOCITY_MREV_S,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            positive_position_headroom_mrev: FIRMWARE_MAX_TRAVEL_MREV,
            max_duration_ms: 3_000,
            homing_speed_mrev_s: 4_375,
            max_accel_mrev_s2: 200_000,
        }
    }

    #[cfg(test)]
    pub(crate) const fn test_budget(
        max_stroke_travel_mrev: u32,
        requested_velocity_mrev_s: u32,
        max_duration_ms: u16,
        homing_speed_mrev_s: u32,
        max_accel_mrev_s2: u32,
    ) -> Self {
        Self {
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source: RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev: if max_stroke_travel_mrev > 100 { 100 } else { 0 },
            negative_position_headroom_mrev: 2_000,
            positive_position_headroom_mrev: 2_000,
            max_duration_ms,
            homing_speed_mrev_s,
            max_accel_mrev_s2,
        }
    }

    fn travel_counts(self, counts_per_rev: u32) -> u32 {
        mrev_to_counts(self.max_stroke_travel_mrev, counts_per_rev)
    }

    fn velocity_counts_per_ms(self, counts_per_rev: u32) -> u32 {
        let counts_per_rev = u64::from(counts_per_rev);
        let scaled = counts_per_rev.saturating_mul(self.requested_velocity_mrev_s as u64);
        let rounded = (scaled + 500_000) / 1_000_000;
        clamp_positive_u32(rounded)
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct OuterVelocityObservation {
    signed_delta_counts: i32,
    absolute_delta_counts: u32,
    elapsed_us: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct OuterSafetyTimestamp(u64);

impl OuterSafetyTimestamp {
    const NONE: u64 = u64::MAX;

    const fn none() -> Self {
        Self(Self::NONE)
    }

    pub(crate) fn some(value: u64) -> Result<Self, CommissionError> {
        if value == Self::NONE {
            Err(CommissionError::InvalidSchedule)
        } else {
            Ok(Self(value))
        }
    }

    pub(crate) const fn get(self) -> Option<u64> {
        if self.0 == Self::NONE {
            None
        } else {
            Some(self.0)
        }
    }

    pub(crate) const fn is_some(self) -> bool {
        self.0 != Self::NONE
    }

    fn map<T>(self, map: impl FnOnce(u64) -> T) -> Option<T> {
        self.get().map(map)
    }

    fn unwrap_or(self, default: u64) -> u64 {
        self.get().unwrap_or(default)
    }
}

impl PartialEq<Option<u64>> for OuterSafetyTimestamp {
    fn eq(&self, other: &Option<u64>) -> bool {
        self.get() == *other
    }
}

/// Outer safety limits, computed once at the start of outer commissioning.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct OuterSafetyLimits {
    /// Velocity cap while a commanded amplitude is active, in milli-counts per second.
    velocity_cap_milli_counts_per_s: u64,
    transient_cap_milli_counts_per_s: u64,
    /// Velocity bound after the target is zeroed, mrev/s; zero means no separate settle allowance.
    settle_ceiling_mrev_s: u32,
    in_settle_phase: bool,
    velocity_step_primitive: bool,
    pub(crate) position_window: u32,
    pub(crate) negative_position_window: u32,
    pub(crate) positive_position_window: u32,
    pub(crate) quarter_turn_fault_counts: u32,
    pub(crate) unwrapped_position: i32,
    pub(crate) torque_cap: u16,
    pub(crate) prev_encoder_count: Option<EncoderCount>,
    pub(crate) encoder_domain: Option<EncoderDomain>,
    pub(crate) last_sample_us: OuterSafetyTimestamp,
    pub(crate) primitive_start_us: Option<u64>,
    pub(crate) primitive_start_position: Option<i32>,
    pub(crate) next_safety_sample_us: OuterSafetyTimestamp,
    previous_velocity_observation: Option<OuterVelocityObservation>,
    max_local_position_excursion: u32,
    last_observation_scoring_eligible: bool,
    velocity_ceiling_milli_counts_per_s: Option<u64>,
    ceiling_latched: bool,
    primitive_peak_milli_counts_per_s: u64,
    skip_next_peak_window: bool,
    pub(crate) budget: OuterMotionBudget,
}

impl OuterSafetyLimits {
    pub(crate) fn new(torque_cap: u16, budget: OuterMotionBudget) -> Self {
        let velocity_cap_milli_counts_per_s =
            counts_per_ms_to_milli_counts_per_s(MECHANICAL_ID_VELOCITY_CAP_COUNTS_PER_MS);
        Self {
            velocity_cap_milli_counts_per_s,
            transient_cap_milli_counts_per_s: velocity_cap_milli_counts_per_s,
            settle_ceiling_mrev_s: 0,
            in_settle_phase: false,
            velocity_step_primitive: false,
            position_window: POSITION_WINDOW_COUNTS,
            negative_position_window: POSITION_WINDOW_COUNTS,
            positive_position_window: POSITION_WINDOW_COUNTS,
            quarter_turn_fault_counts: 0,
            unwrapped_position: 0,
            torque_cap,
            prev_encoder_count: None,
            encoder_domain: None,
            last_sample_us: OuterSafetyTimestamp::none(),
            primitive_start_us: None,
            primitive_start_position: None,
            next_safety_sample_us: OuterSafetyTimestamp::none(),
            previous_velocity_observation: None,
            max_local_position_excursion: 0,
            last_observation_scoring_eligible: false,
            velocity_ceiling_milli_counts_per_s: None,
            ceiling_latched: false,
            primitive_peak_milli_counts_per_s: 0,
            skip_next_peak_window: false,
            budget,
        }
    }

    pub(crate) fn budget_request_is_valid(&self) -> bool {
        self.budget.max_stroke_travel_mrev > 0
            && self.budget.requested_velocity_mrev_s > 0
            && self.budget.machine_velocity_ceiling_mrev_s > 0
            && self.budget.settle_travel_reserve_mrev < self.budget.max_stroke_travel_mrev
            && self.budget.negative_position_headroom_mrev > 0
            && self.budget.positive_position_headroom_mrev > 0
            && self.budget.max_duration_ms > 0
    }

    pub(crate) fn configure_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        let encoder_domain = EncoderDomain::new(counts_per_rev)?;
        self.encoder_domain = Some(encoder_domain);
        self.prev_encoder_count = None;
        self.previous_velocity_observation = None;
        self.quarter_turn_fault_counts = counts_per_rev.div_ceil(4);
        self.velocity_cap_milli_counts_per_s = counts_per_ms_to_milli_counts_per_s(
            mechanical_id_velocity_cap_counts_per_ms(counts_per_rev)
                .min(self.budget.velocity_counts_per_ms(counts_per_rev)),
        );
        self.transient_cap_milli_counts_per_s = self.velocity_cap_milli_counts_per_s;
        self.settle_ceiling_mrev_s = 0;
        self.position_window =
            position_window_counts(counts_per_rev).min(self.budget.travel_counts(counts_per_rev));
        self.negative_position_window = position_window_counts(counts_per_rev).min(mrev_to_counts(
            self.budget.negative_position_headroom_mrev,
            counts_per_rev,
        ));
        self.positive_position_window = position_window_counts(counts_per_rev).min(mrev_to_counts(
            self.budget.positive_position_headroom_mrev,
            counts_per_rev,
        ));
        Ok(())
    }

    /// Configure encoder-domain safety using the host-declared sweep windows.
    pub(crate) fn configure_velocity_sweep_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        self.configure_encoder_resolution(counts_per_rev)?;
        self.position_window = self.budget.travel_counts(counts_per_rev);
        self.negative_position_window =
            mrev_to_counts(self.budget.negative_position_headroom_mrev, counts_per_rev);
        self.positive_position_window =
            mrev_to_counts(self.budget.positive_position_headroom_mrev, counts_per_rev);
        Ok(())
    }

    /// Configure encoder-domain safety for the reversal-standstill robustness phase.
    ///
    /// Like the velocity-sweep configuration, but the local per-stroke window is
    /// `min(negative_position_window, positive_position_window)`.
    pub(crate) fn configure_robustness_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        self.configure_velocity_sweep_encoder_resolution(counts_per_rev)?;
        self.reserve_absolute_window_settle_travel(counts_per_rev);
        self.open_position_window_to_headroom();
        Ok(())
    }

    fn reserve_absolute_window_settle_travel(&mut self, counts_per_rev: u32) {
        let reserve_counts = mrev_to_counts(self.budget.settle_travel_reserve_mrev, counts_per_rev);
        self.negative_position_window =
            self.negative_position_window.saturating_add(reserve_counts);
        self.positive_position_window =
            self.positive_position_window.saturating_add(reserve_counts);
    }

    /// Configure encoder-domain safety for the Integral `VelocityIntegral` phase.
    ///
    /// Like the velocity-sweep configuration, but the local window is tightened by
    /// [`Self::tighten_position_window`].
    pub(crate) fn configure_integral_encoder_resolution(
        &mut self,
        counts_per_rev: u32,
    ) -> Result<(), EncoderMathError> {
        self.configure_velocity_sweep_encoder_resolution(counts_per_rev)?;
        self.tighten_position_window(INTEGRAL_POSITION_RESERVE_COUNTS);
        Ok(())
    }

    fn tighten_position_window(&mut self, reserve_counts: Option<u32>) {
        self.position_window = match reserve_counts {
            Some(reserve) => self
                .position_window
                .saturating_add(reserve)
                .min(self.headroom_clamp()),
            None => self.headroom_clamp(),
        };
    }

    fn open_position_window_to_headroom(&mut self) {
        self.position_window = self.headroom_clamp();
    }

    fn headroom_clamp(&self) -> u32 {
        self.negative_position_window
            .min(self.positive_position_window)
    }

    pub(crate) fn envelope_mrev_s(planned_velocity_mrev_s: u32) -> Result<u32, CommissionError> {
        if planned_velocity_mrev_s == 0 {
            return Err(CommissionError::InvalidSchedule);
        }
        let proportional = planned_velocity_mrev_s
            .checked_mul(ENVELOPE_PROPORTIONAL_NUM)
            .and_then(|value| value.checked_add(ENVELOPE_PROPORTIONAL_DEN - 1))
            .map(|value| value / ENVELOPE_PROPORTIONAL_DEN)
            .ok_or(CommissionError::TargetRateInvalid)?;
        let absolute = planned_velocity_mrev_s
            .checked_add(ENVELOPE_ABSOLUTE_MARGIN_MREV_S)
            .ok_or(CommissionError::InvalidSchedule)?;
        Ok(proportional.max(absolute))
    }

    fn apply_velocity_envelope(
        &mut self,
        planned_velocity_mrev_s: u32,
        moving_mrev_s: u32,
        settle_ceiling_mrev_s: u32,
    ) {
        let counts_per_rev = self.counts_per_rev();
        self.velocity_cap_milli_counts_per_s =
            velocity_mrev_s_to_milli_counts_per_s(moving_mrev_s, counts_per_rev);
        let transient_mrev_s = Self::transient_mrev_s(planned_velocity_mrev_s)
            .min(self.budget.machine_velocity_ceiling_mrev_s)
            .max(moving_mrev_s);
        self.transient_cap_milli_counts_per_s =
            velocity_mrev_s_to_milli_counts_per_s(transient_mrev_s, counts_per_rev);
        self.settle_ceiling_mrev_s = settle_ceiling_mrev_s;
    }

    fn transient_mrev_s(planned_velocity_mrev_s: u32) -> u32 {
        let proportional = u64::from(planned_velocity_mrev_s)
            .saturating_mul(u64::from(TRANSIENT_PROPORTIONAL_NUM))
            .div_ceil(u64::from(TRANSIENT_PROPORTIONAL_DEN));
        let proportional = u32::try_from(proportional).unwrap_or(u32::MAX);
        let absolute = planned_velocity_mrev_s.saturating_add(TRANSIENT_ABSOLUTE_MARGIN_MREV_S);
        proportional.max(absolute)
    }

    /// Configure the runtime envelope around the frozen sweep amplitude.
    ///
    /// # Errors
    ///
    /// Returns [`CommissionError::InvalidSchedule`] when the amplitude is
    /// zero, and [`CommissionError::TargetRateInvalid`] when the envelope's
    /// proportional-margin arithmetic overflows.
    pub(crate) fn configure_velocity_envelope(
        &mut self,
        planned_velocity_mrev_s: u32,
    ) -> Result<(), CommissionError> {
        let envelope_mrev_s = Self::envelope_mrev_s(planned_velocity_mrev_s)?;
        self.apply_velocity_envelope(planned_velocity_mrev_s, envelope_mrev_s, 0);
        Ok(())
    }

    /// Configure the runtime envelope from the plan's effective ceiling.
    ///
    /// Both the stroke and its zero-target settle are bounded by `effective_ceiling_mrev_s`,
    /// i.e. `min(host machine ceiling, encoder-observable)`.
    ///
    /// # Errors
    ///
    /// Returns [`CommissionError::VelocityEnvelopeCeilingZero`] when
    /// `effective_ceiling_mrev_s` is zero,
    /// [`CommissionError::VelocityEnvelopeExceedsCeiling`] when the derived
    /// envelope exceeds that ceiling, [`CommissionError::InvalidSchedule`]
    /// when `planned_velocity_mrev_s` is zero, and
    /// [`CommissionError::TargetRateInvalid`] when the envelope's
    /// proportional-margin arithmetic overflows.
    pub(crate) fn configure_velocity_envelope_with_settle_ceiling(
        &mut self,
        planned_velocity_mrev_s: u32,
        effective_ceiling_mrev_s: u32,
    ) -> Result<(), CommissionError> {
        if effective_ceiling_mrev_s == 0 {
            return Err(CommissionError::VelocityEnvelopeCeilingZero);
        }
        let envelope_mrev_s = Self::envelope_mrev_s(planned_velocity_mrev_s)?;
        if envelope_mrev_s > effective_ceiling_mrev_s {
            return Err(CommissionError::VelocityEnvelopeExceedsCeiling);
        }
        self.apply_velocity_envelope(
            planned_velocity_mrev_s,
            effective_ceiling_mrev_s,
            effective_ceiling_mrev_s,
        );
        Ok(())
    }

    /// Configure the runtime envelope for a verification primitive whose legs
    /// may legitimately overshoot: the motor is bounded by the plan's effective
    /// ceiling, `min(host machine ceiling, encoder-observable)`, whenever that
    /// ceiling is wider than the proportional envelope of the planned velocity.
    ///
    /// The bound never drops below [`Self::configure_velocity_envelope`]'s, so
    /// a ceiling of zero (or one below the proportional envelope) keeps that
    /// envelope; the stroke and its zero-target settle share the bound.
    ///
    /// # Errors
    ///
    /// Returns [`CommissionError::InvalidSchedule`] when
    /// `planned_velocity_mrev_s` is zero and
    /// [`CommissionError::TargetRateInvalid`] when the envelope's
    /// proportional-margin arithmetic overflows.
    pub(crate) fn configure_velocity_envelope_up_to_ceiling(
        &mut self,
        planned_velocity_mrev_s: u32,
        effective_ceiling_mrev_s: u32,
    ) -> Result<(), CommissionError> {
        let envelope_mrev_s =
            Self::envelope_mrev_s(planned_velocity_mrev_s)?.max(effective_ceiling_mrev_s);
        self.apply_velocity_envelope(planned_velocity_mrev_s, envelope_mrev_s, envelope_mrev_s);
        Ok(())
    }

    /// Mark the active primitive as past its zero-target boundary.
    pub(crate) const fn enter_settle_phase(&mut self) {
        self.in_settle_phase = true;
    }

    /// The steady velocity cap while a commanded amplitude is active, ignoring the transient
    /// allowance and the settle ceiling.
    pub(crate) fn moving_velocity_cap_milli_counts_per_s(&self) -> u64 {
        self.velocity_cap_milli_counts_per_s
    }

    /// The velocity bound in force for the active primitive phase at `now_us`.
    ///
    /// For the first [`TRANSIENT_ALLOWANCE_US`] of a primitive opened with
    /// [`Self::begin_velocity_step_primitive`] the transient cap applies, so a velocity step may
    /// overshoot its target without ending the campaign; afterwards, and for every other
    /// primitive, the moving cap, or the settle ceiling once the target is zeroed.
    pub(crate) fn active_velocity_cap_milli_counts_per_s(&self, now_us: u64) -> u64 {
        let moving = self.velocity_cap_milli_counts_per_s;
        let in_transient_window = self.velocity_step_primitive
            && self
                .primitive_start_us
                .is_some_and(|start_us| now_us.saturating_sub(start_us) < TRANSIENT_ALLOWANCE_US);
        if in_transient_window && !self.in_settle_phase {
            return self.transient_cap_milli_counts_per_s;
        }
        if !self.in_settle_phase || self.settle_ceiling_mrev_s == 0 {
            return moving;
        }
        match self.encoder_domain {
            Some(domain) => velocity_mrev_s_to_milli_counts_per_s(
                self.settle_ceiling_mrev_s,
                domain.counts_per_rev(),
            )
            .max(moving),
            None => moving,
        }
    }

    pub(crate) fn set_encoder_baseline(&mut self, raw_counts: u32) -> Result<(), EncoderMathError> {
        let domain = self
            .encoder_domain
            .ok_or(EncoderMathError::InvalidCountsPerRev)?;
        self.prev_encoder_count = Some(domain.count(raw_counts));
        self.previous_velocity_observation = None;
        Ok(())
    }

    pub(crate) fn arm_safety_observation(&mut self, now_us: u64) -> Result<(), CommissionError> {
        let next_safety_sample_us = now_us
            .checked_add(OUTER_SAFETY_SAMPLE_PERIOD_US)
            .ok_or(CommissionError::ScheduleDeadlineOverflow)?;
        self.last_sample_us = OuterSafetyTimestamp::some(now_us)?;
        self.next_safety_sample_us = OuterSafetyTimestamp::some(next_safety_sample_us)?;
        self.previous_velocity_observation = None;
        self.last_observation_scoring_eligible = false;
        Ok(())
    }

    pub(crate) fn begin_primitive(&mut self, now_us: u64) -> Result<(), CommissionError> {
        if self.primitive_start_us.is_some() {
            return Err(CommissionError::PrimitiveAlreadyOpen);
        }
        self.primitive_start_us = Some(now_us);
        self.primitive_start_position = Some(self.unwrapped_position);
        self.previous_velocity_observation = None;
        self.in_settle_phase = false;
        self.velocity_step_primitive = false;
        self.max_local_position_excursion = 0;
        self.ceiling_latched = false;
        self.primitive_peak_milli_counts_per_s = 0;
        self.skip_next_peak_window = false;
        Ok(())
    }

    /// Open a primitive that commands a velocity step, which earns the transient allowance.
    pub(crate) fn begin_velocity_step_primitive(
        &mut self,
        now_us: u64,
    ) -> Result<(), CommissionError> {
        self.begin_primitive(now_us)?;
        self.velocity_step_primitive = true;
        Ok(())
    }

    pub(crate) fn end_primitive(&mut self) {
        if self.primitive_start_us.is_some() {
            defmt::info!(
                "outer safety: primitive local position peak {} of window {}",
                self.max_local_position_excursion,
                self.position_window,
            );
        }
        self.primitive_start_us = None;
        self.primitive_start_position = None;
        self.previous_velocity_observation = None;
        self.in_settle_phase = false;
        self.velocity_step_primitive = false;
    }

    pub(crate) fn set_velocity_ceiling(&mut self, ceiling_milli_counts_per_s: Option<u64>) {
        self.velocity_ceiling_milli_counts_per_s = ceiling_milli_counts_per_s;
    }

    pub(crate) fn ceiling_latched(&self) -> bool {
        self.ceiling_latched
    }

    pub(crate) fn primitive_peak_milli_counts_per_s(&self) -> u64 {
        self.primitive_peak_milli_counts_per_s
    }

    /// Zero the primitive peak without opening a new primitive, so a caller can isolate the peak
    /// of one leg. The next two-sample window spans the reset and is kept out of the peak; its
    /// cap and ceiling evaluation is unchanged.
    pub(crate) fn reset_primitive_peak(&mut self) {
        self.primitive_peak_milli_counts_per_s = 0;
        self.skip_next_peak_window = true;
    }

    #[cfg(test)]
    pub(crate) fn force_ceiling_latch_for_test(&mut self) {
        self.ceiling_latched = true;
    }

    fn observe_window_velocity(&mut self, window_counts: u128, window_elapsed_us: u128) {
        if window_elapsed_us == 0 {
            return;
        }
        let milli_counts_per_s =
            clamp_u128_to_u64(window_counts * 1_000_000_000 / window_elapsed_us);
        if self.skip_next_peak_window {
            self.skip_next_peak_window = false;
        } else {
            self.primitive_peak_milli_counts_per_s = self
                .primitive_peak_milli_counts_per_s
                .max(milli_counts_per_s);
        }
        if let Some(ceiling) = self.velocity_ceiling_milli_counts_per_s
            && window_counts * 1_000_000_000 >= u128::from(ceiling) * window_elapsed_us
        {
            self.ceiling_latched = true;
        }
    }

    #[cfg(test)]
    pub(crate) fn max_local_position_excursion(&self) -> u32 {
        self.max_local_position_excursion
    }

    #[cfg(test)]
    pub(crate) fn last_observation_scoring_eligible(&self) -> bool {
        self.last_observation_scoring_eligible
    }

    pub(crate) fn last_accepted_observation(&self) -> Option<(i32, u64)> {
        self.previous_velocity_observation
            .map(|observation| (observation.signed_delta_counts, observation.elapsed_us))
    }

    #[cfg(test)]
    pub(crate) fn force_accepted_observation_for_test(
        &mut self,
        delta_counts: i32,
        elapsed_us: u64,
    ) {
        self.previous_velocity_observation = Some(OuterVelocityObservation {
            signed_delta_counts: delta_counts,
            absolute_delta_counts: delta_counts.unsigned_abs(),
            elapsed_us,
        });
    }

    pub(crate) fn next_safety_deadline(&self) -> Option<u64> {
        if self.torque_cap == 0 {
            None
        } else {
            self.next_safety_sample_us.get()
        }
    }

    pub(crate) fn counts_per_rev(&self) -> u32 {
        self.encoder_domain
            .expect("outer encoder domain configured before motion")
            .counts_per_rev()
    }

    pub(crate) fn budget_is_usable(&self) -> bool {
        self.budget_request_is_valid()
            && self.budget.max_duration_ms >= MIN_USEFUL_DURATION_MS
            && self.position_window >= MECHANICAL_ID_MIN_TRAVEL_COUNTS
            && self.velocity_cap_milli_counts_per_s
                >= counts_per_ms_to_milli_counts_per_s(
                    MECHANICAL_ID_MIN_PEAK_VELOCITY_COUNTS_PER_MS,
                )
    }

    pub(crate) fn invalid_budget_fault(&self) -> OuterSafetyFault {
        self.fault(OUTER_SAFETY_FAULT_INVALID_BUDGET, 0, 0, 0, 0)
    }

    pub(crate) fn fault(
        &self,
        reason: u8,
        delta_counts: i32,
        dt_us: u64,
        velocity_counts_per_ms: i32,
        elapsed_us: u64,
    ) -> OuterSafetyFault {
        let duration_cap_us = u64::from(self.budget.max_duration_ms) * 1_000;
        OuterSafetyFault {
            reason,
            max_travel_mrev: clamp_u32_to_u16(self.budget.max_stroke_travel_mrev),
            max_velocity_mrev_s: clamp_u32_to_u16(self.budget.requested_velocity_mrev_s),
            max_duration_ms: self.budget.max_duration_ms,
            direction_mask: 0x03,
            delta_counts,
            dt_us: clamp_u64_to_u32(dt_us),
            velocity_counts_per_ms,
            velocity_cap_counts_per_ms: clamp_u32_to_i32(milli_counts_per_s_to_counts_per_ms(
                self.active_velocity_cap_milli_counts_per_s(
                    self.primitive_start_us
                        .unwrap_or(0)
                        .saturating_add(elapsed_us),
                ),
            )),
            position_counts: self.unwrapped_position,
            position_window_counts: clamp_u32_to_i32(self.position_window),
            elapsed_us: clamp_u64_to_u32(elapsed_us),
            duration_cap_us: clamp_u64_to_u32(duration_cap_us),
        }
    }

    pub(crate) fn recovery_wrong_way_fault(
        &self,
        start_offset_counts: i32,
        elapsed_us: u64,
    ) -> OuterSafetyFault {
        self.fault(
            crate::commissioning::types::OUTER_SAFETY_FAULT_RECOVERY_WRONG_WAY,
            self.unwrapped_position.saturating_sub(start_offset_counts),
            0,
            0,
            elapsed_us,
        )
    }
}

fn mrev_to_counts(mrev: u32, counts_per_rev: u32) -> u32 {
    let counts_per_rev = u64::from(counts_per_rev);
    let scaled = counts_per_rev.saturating_mul(mrev as u64);
    clamp_positive_u32(scaled / 1_000)
}

const MILLI_COUNTS_PER_S_PER_COUNT_PER_MS: u64 = 1_000_000;

fn velocity_mrev_s_to_milli_counts_per_s(velocity_mrev_s: u32, counts_per_rev: u32) -> u64 {
    u64::from(velocity_mrev_s).saturating_mul(u64::from(counts_per_rev))
}

const fn counts_per_ms_to_milli_counts_per_s(counts_per_ms: u32) -> u64 {
    (counts_per_ms as u64).saturating_mul(MILLI_COUNTS_PER_S_PER_COUNT_PER_MS)
}

const fn milli_counts_per_s_to_counts_per_ms(milli_counts_per_s: u64) -> u32 {
    let counts_per_ms = milli_counts_per_s / MILLI_COUNTS_PER_S_PER_COUNT_PER_MS;
    if counts_per_ms > u32::MAX as u64 {
        u32::MAX
    } else {
        counts_per_ms as u32
    }
}

fn clamp_positive_u32(value: u64) -> u32 {
    if value == 0 {
        1
    } else if value > u32::MAX as u64 {
        u32::MAX
    } else {
        value as u32
    }
}

fn clamp_u32_to_i32(value: u32) -> i32 {
    value.min(i32::MAX as u32) as i32
}

fn clamp_u32_to_u16(value: u32) -> u16 {
    value.min(u16::MAX as u32) as u16
}

fn clamp_u64_to_u32(value: u64) -> u32 {
    value.min(u32::MAX as u64) as u32
}

fn clamp_u128_to_u64(value: u128) -> u64 {
    value.min(u64::MAX as u128) as u64
}

fn signed_report_value(delta_counts: i32, magnitude: u64) -> i32 {
    if delta_counts < 0 {
        let magnitude = magnitude.min(i32::MAX as u64 + 1);
        -(magnitude as i64) as i32
    } else {
        magnitude.min(i32::MAX as u64) as i32
    }
}

/// Map a safety-envelope violation to the dedicated error variant.
pub(crate) fn safety_error_for_phase(_phase: &OuterPhase) -> CommissionError {
    CommissionError::SafetyEnvelopeViolation
}

/// Per-tick safety envelope check for all outer phases.
///
/// Returns `Some(fault)` if the velocity or position-window limit is violated.
pub(crate) async fn check_outer_safety<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Result<Option<OuterSafetyFault>, CommissionError> {
    check_outer_safety_inner(safety, None, backend).await
}

pub(crate) async fn check_outer_safety_with_timing<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    timing: &mut super::OuterSafetyTimingAccumulator,
    context: super::OuterSafetyTimingContext,
    backend: &mut B,
) -> Result<Option<OuterSafetyFault>, CommissionError> {
    check_outer_safety_inner(safety, Some((timing, context)), backend).await
}

async fn check_outer_safety_inner<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    timing: Option<(
        &mut super::OuterSafetyTimingAccumulator,
        super::safety_timing::OuterSafetyTimingContext,
    )>,
    backend: &mut B,
) -> Result<Option<OuterSafetyFault>, CommissionError> {
    if safety.torque_cap == 0 {
        return Ok(None);
    }

    let entry_us = backend.elapsed_us();
    let sample_deadline_us = safety.next_safety_sample_us.unwrap_or(entry_us);
    if entry_us < sample_deadline_us {
        return Ok(None);
    }

    if !safety.budget_is_usable() {
        defmt::warn!(
            "outer safety: unusable motion budget travel={} velocity={} duration={} dir={}",
            safety.budget.max_stroke_travel_mrev,
            safety.budget.requested_velocity_mrev_s,
            safety.budget.max_duration_ms,
            0x03_u8,
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_UNUSABLE_BUDGET,
            0,
            0,
            0,
            0,
        )));
    }

    let encoder_raw = backend
        .try_read_mechanical_encoder_position_counts()
        .await
        .map_err(|_| CommissionError::Spi)?;
    let encoder_count = safety
        .encoder_domain
        .expect("outer encoder domain configured before motion")
        .count(encoder_raw);
    let now_us = backend.elapsed_us();

    let elapsed_us = safety
        .primitive_start_us
        .map(|primitive_start_us| {
            now_us
                .checked_sub(primitive_start_us)
                .ok_or(CommissionError::InvalidSchedule)
        })
        .transpose()?
        .unwrap_or(0);
    let max_duration_us = u64::from(safety.budget.max_duration_ms) * 1_000;
    if safety.primitive_start_us.is_some() && elapsed_us > max_duration_us {
        defmt::warn!(
            "outer safety: duration {}us exceeds cap {}us",
            elapsed_us,
            max_duration_us
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_DURATION,
            0,
            0,
            0,
            elapsed_us,
        )));
    }

    let dt_us = safety
        .last_sample_us
        .map(|last_sample_us| {
            now_us
                .checked_sub(last_sample_us)
                .ok_or(CommissionError::InvalidSchedule)
        })
        .transpose()?
        .unwrap_or(0);
    if safety.last_sample_us.is_some()
        && let Some((timing, context)) = timing
    {
        timing.observe(dt_us, context)?;
    }
    if safety.last_sample_us.is_some() && (dt_us == 0 || dt_us > MAX_SAFETY_OBSERVATION_GAP_US) {
        defmt::warn!(
            "outer safety: observation gap {}us exceeds wrap-safe cap {}us",
            dt_us,
            MAX_SAFETY_OBSERVATION_GAP_US
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_OBSERVATION_GAP,
            0,
            dt_us,
            0,
            elapsed_us,
        )));
    }
    let encoder_domain = safety
        .encoder_domain
        .expect("outer encoder domain configured before motion");
    let previous_count = safety
        .prev_encoder_count
        .expect("outer encoder baseline configured before motion");
    let delta = match encoder_domain.delta(previous_count, encoder_count) {
        Ok(delta) => delta,
        Err(EncoderMathError::AmbiguousHalfTurn) => {
            return Ok(Some(safety.fault(
                OUTER_SAFETY_FAULT_QUARTER_TURN,
                0,
                dt_us,
                0,
                elapsed_us,
            )));
        }
        Err(EncoderMathError::InvalidCountsPerRev) => {
            unreachable!("validated encoder domain cannot become invalid")
        }
    };
    let delta_counts = delta.signed_counts();
    if delta.absolute_counts() >= safety.quarter_turn_fault_counts {
        defmt::warn!(
            "outer safety: delta {} reaches quarter-turn guard {}",
            delta.absolute_counts(),
            safety.quarter_turn_fault_counts
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_QUARTER_TURN,
            delta_counts,
            dt_us,
            0,
            elapsed_us,
        )));
    }
    let velocity_magnitude_cpms = (u64::from(delta.absolute_counts()) * 1_000)
        .checked_div(dt_us)
        .unwrap_or(0);

    safety.prev_encoder_count = Some(encoder_count);
    safety.last_sample_us = OuterSafetyTimestamp::some(now_us)?;
    safety.next_safety_sample_us = OuterSafetyTimestamp::some(next_grid_deadline(
        sample_deadline_us,
        OUTER_SAFETY_SAMPLE_PERIOD_US,
        now_us,
    )?)?;
    safety.unwrapped_position = safety.unwrapped_position.saturating_add(delta_counts);
    safety.last_observation_scoring_eligible = dt_us <= OUTER_SAFETY_SAMPLE_PERIOD_US;

    if let Some(start) = safety.primitive_start_position {
        let position = safety.unwrapped_position.saturating_sub(start);
        safety.max_local_position_excursion = safety
            .max_local_position_excursion
            .max(position.unsigned_abs());
    }

    let velocity_observation = OuterVelocityObservation {
        signed_delta_counts: delta_counts,
        absolute_delta_counts: delta.absolute_counts(),
        elapsed_us: dt_us,
    };
    let previous_velocity_observation = safety
        .previous_velocity_observation
        .replace(velocity_observation);

    let active_cap_milli_counts_per_s = safety.active_velocity_cap_milli_counts_per_s(now_us);
    let window = previous_velocity_observation.map(|previous| {
        let window_counts = u128::from(previous.absolute_delta_counts)
            + u128::from(velocity_observation.absolute_delta_counts);
        let window_elapsed_us =
            u128::from(previous.elapsed_us) + u128::from(velocity_observation.elapsed_us);
        (window_counts, window_elapsed_us)
    });
    if let Some((window_counts, window_elapsed_us)) = window {
        safety.observe_window_velocity(window_counts, window_elapsed_us);
    }
    let violating_window = window.filter(|(window_counts, window_elapsed_us)| {
        window_counts * 1_000_000_000
            > u128::from(active_cap_milli_counts_per_s) * window_elapsed_us
    });
    if let Some((window_counts, window_elapsed_us)) = violating_window {
        let window_magnitude_cpms = clamp_u128_to_u64(
            (window_counts * 1_000)
                .checked_div(window_elapsed_us)
                .unwrap_or(0),
        );
        let window_delta_counts =
            signed_report_value(delta_counts, clamp_u128_to_u64(window_counts));
        let window_velocity_cpms = signed_report_value(delta_counts, window_magnitude_cpms);
        defmt::warn!(
            "outer safety: window velocity {} exceeds cap {} milli-counts/s \
             (window delta={} dt_us={}, latest delta={} dt_us={}, settle={})",
            window_magnitude_cpms,
            active_cap_milli_counts_per_s,
            window_delta_counts,
            clamp_u128_to_u64(window_elapsed_us),
            delta_counts,
            dt_us,
            safety.in_settle_phase,
        );
        return Ok(Some(safety.fault(
            OUTER_SAFETY_FAULT_VELOCITY,
            window_delta_counts,
            clamp_u128_to_u64(window_elapsed_us),
            window_velocity_cpms,
            elapsed_us,
        )));
    }

    let local_position = safety
        .primitive_start_position
        .map(|start| safety.unwrapped_position.saturating_sub(start));
    let local_position_exceeds_window =
        local_position.is_some_and(|position| position.unsigned_abs() > safety.position_window);
    let absolute_position_exceeds_window = safety.unwrapped_position
        < -(clamp_u32_to_i32(safety.negative_position_window))
        || safety.unwrapped_position > clamp_u32_to_i32(safety.positive_position_window);
    if local_position_exceeds_window || absolute_position_exceeds_window {
        defmt::warn!(
            "outer safety: position {} exceeds local {} or absolute -{}/+{}",
            safety.unwrapped_position,
            safety.position_window,
            safety.negative_position_window,
            safety.positive_position_window,
        );
        let velocity_cpms = signed_report_value(delta_counts, velocity_magnitude_cpms);
        let (reason, excursion, signed_window) = if absolute_position_exceeds_window {
            let signed_window =
                if safety.unwrapped_position > clamp_u32_to_i32(safety.positive_position_window) {
                    clamp_u32_to_i32(safety.positive_position_window)
                } else {
                    -clamp_u32_to_i32(safety.negative_position_window)
                };
            (
                OUTER_SAFETY_FAULT_POSITION,
                safety.unwrapped_position,
                signed_window,
            )
        } else {
            let excursion = local_position.unwrap_or(safety.unwrapped_position);
            let magnitude = clamp_u32_to_i32(safety.position_window);
            let signed_window = if excursion < 0 { -magnitude } else { magnitude };
            (OUTER_SAFETY_FAULT_POSITION_LOCAL, excursion, signed_window)
        };
        let mut fault = safety.fault(reason, delta_counts, dt_us, velocity_cpms, elapsed_us);
        fault.position_counts = excursion;
        fault.position_window_counts = signed_window;
        return Ok(Some(fault));
    }

    Ok(None)
}

/// Get a mutable reference to the safety limits for the current phase.
///
/// Returns `None` for terminal phases.
pub(super) fn outer_safety_mut(phase: &mut OuterPhase) -> Option<&mut OuterSafetyLimits> {
    match phase {
        OuterPhase::VelocityIntegral { safety, .. } => Some(safety),
        OuterPhase::BreakawayCampaign { safety, .. } => Some(safety),
        OuterPhase::RobustnessReversal { safety, .. } => Some(safety),
        OuterPhase::PositionTune { safety, .. } => Some(safety),
        OuterPhase::Failed(_)
        | OuterPhase::PositionTuneFailed { .. }
        | OuterPhase::FailedWithSafety { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use super::check_outer_safety_with_timing;
    use super::{
        MAX_SAFETY_OBSERVATION_GAP_US, OuterMotionBudget, OuterSafetyLimits, OuterSafetyTimestamp,
        check_outer_safety,
    };

    const OUTER_SAFETY_SAMPLE_MAX_REGISTER_OPS: usize = 1;
    use crate::autotune::{CommissionError, TransientBuffer};
    use crate::commission_dispatch::test_support::MockBackend;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptancePoint, IntegralPlanIdentity,
    };
    use crate::commissioning::outer::velocity::robustness_reversal::{
        RobustnessReversalSession, RobustnessReversalState, RobustnessReversalWorkspace,
    };
    use crate::commissioning::outer::{
        OuterPhase, OuterSafetyTimingAccumulator, OuterSafetyTimingContext, OuterSafetyTimingPhase,
        OuterSafetyTimingStage, TraceSummarySeqs, step_outer_with_trace,
    };
    use crate::commissioning::types::{
        OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_POSITION,
        OUTER_SAFETY_FAULT_POSITION_LOCAL, OUTER_SAFETY_FAULT_QUARTER_TURN,
        OUTER_SAFETY_FAULT_VELOCITY, StepResult,
    };
    use foci_tmc4671::registers::ABN_DECODER_COUNT;

    fn armed_safety(last_sample_us: Option<u64>, next_sample_us: u64) -> OuterSafetyLimits {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.configure_encoder_resolution(4_000).unwrap();
        safety.set_encoder_baseline(0).unwrap();
        safety.last_sample_us = match last_sample_us {
            Some(value) => OuterSafetyTimestamp::some(value).unwrap(),
            None => OuterSafetyTimestamp::none(),
        };
        safety.next_safety_sample_us = OuterSafetyTimestamp::some(next_sample_us).unwrap();
        safety
    }

    fn robustness_test_point() -> AcceptancePoint {
        AcceptancePoint {
            identity: IntegralPlanIdentity {
                channel: 0,
                build_features: 0x1234,
                schema_revision: 1,
                plan_digest: 0xfeed_beef,
            },
            selected_p: 1_024,
            selected_i: 512,
            target_velocity_rpm: 176,
            selected_response_intervals_q: [[10, 20], [-20, -10]],
            confidence_q: 0,
            band_position_q: 0,
            qualifier_bits: 0,
            token: None,
        }
    }

    fn robustness_reversal_session() -> RobustnessReversalSession {
        RobustnessReversalSession::new(robustness_test_point(), 1, 4_000)
    }

    #[test]
    fn outer_safety_observes_zero_target_settle() {
        pollster::block_on(async {
            let safety = armed_safety(Some(0), 1_000);
            let mut phase = OuterPhase::RobustnessReversal {
                state: RobustnessReversalState::ZeroCapture {
                    next_sample_us: 300_000,
                    deadline_us: 500_000,
                    retry_available: true,
                },
                workspace: RobustnessReversalWorkspace::new(),
                session: robustness_reversal_session(),
                safety,
            };
            let mut buf = TransientBuffer::new();
            let mut trace_summary_seqs = TraceSummarySeqs::new();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 4;

            let result =
                step_outer_with_trace(&mut phase, &mut buf, &mut trace_summary_seqs, &mut backend)
                    .await;

            assert!(matches!(result, StepResult::At(2_000)));
            assert!(matches!(
                phase,
                OuterPhase::RobustnessReversal { safety, .. }
                    if safety.unwrapped_position == 4
                        && safety.next_safety_sample_us == Some(2_000)
            ));
        });
    }

    #[test]
    fn primitive_timer_restarts_for_a_retry_after_the_previous_primitive_ends() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.unwrapped_position = 37;

        assert_eq!(safety.begin_primitive(0), Ok(()));
        assert_eq!(safety.primitive_start_us, Some(0));
        assert_eq!(safety.primitive_start_position, Some(37));
        safety.end_primitive();
        assert_eq!(safety.primitive_start_us, None);
        assert_eq!(safety.primitive_start_position, None);
        assert_eq!(safety.begin_primitive(5_000), Ok(()));
        assert_eq!(safety.primitive_start_us, Some(5_000));
        assert_eq!(safety.primitive_start_position, Some(37));
    }

    #[test]
    fn primitive_timer_rejects_an_overlapping_start() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        safety.begin_primitive(1_000).unwrap();

        assert!(matches!(
            safety.begin_primitive(2_000),
            Err(CommissionError::PrimitiveAlreadyOpen)
        ));
        assert_eq!(safety.primitive_start_us, Some(1_000));
    }

    #[test]
    fn max_local_position_excursion_tracks_the_peak_not_the_latest_reading() {
        pollster::block_on(async {
            let mut safety = armed_safety(None, 0);
            safety.begin_primitive(0).unwrap();

            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 50;
            check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first observation should succeed");

            backend.elapsed_us_value = 2_000;
            backend.mechanical_encoder_position_counts_value = 20;
            check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second observation should succeed");

            assert_eq!(
                safety.max_local_position_excursion(),
                50,
                "the peak from the first sample must survive a smaller later reading"
            );
        });
    }

    #[test]
    fn max_local_position_excursion_updates_even_on_a_faulting_sample() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            safety.begin_primitive(100_000).unwrap();

            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 102_500;
            backend.mechanical_encoder_position_counts_value = 35;
            check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");

            backend.elapsed_us_value = 105_000;
            backend.mechanical_encoder_position_counts_value = 70;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed")
                .expect("14.0 counts/ms must exceed the 13.8 counts/ms envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(
                safety.max_local_position_excursion(),
                70,
                "the peak must include the sample that tripped the velocity fault"
            );
        });
    }

    #[test]
    fn inactive_settle_does_not_consume_a_primitive_duration_budget() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(3_000_000), 3_000_001);
            safety.primitive_start_us = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 3_000_001;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("inactive safety observation should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.primitive_start_us, None);
        });
    }

    #[test]
    fn outer_safety_does_not_read_encoder_before_deadline() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(1_000), 2_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_999;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("an early check should not access the encoder");

            assert!(fault.is_none());
            assert!(backend.spi_log.is_empty());
            assert_eq!(safety.last_sample_us, Some(1_000));
            assert_eq!(safety.next_safety_sample_us, Some(2_000));
        });
    }

    #[test]
    fn late_outer_safety_sample_skips_to_first_future_grid_point() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(1_000), 2_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_500;
            backend.mechanical_encoder_position_counts_value = 4;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a due encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.last_sample_us, Some(2_500));
            assert_eq!(safety.next_safety_sample_us, Some(3_000));
            assert_eq!(backend.spi_log.len(), OUTER_SAFETY_SAMPLE_MAX_REGISTER_OPS);
        });
    }

    #[test]
    fn outer_safety_accepts_a_legitimate_time_zero_sample() {
        pollster::block_on(async {
            let mut safety = armed_safety(None, 0);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 0;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a due time-zero encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.last_sample_us, Some(0));
            assert_eq!(safety.primitive_start_us, None);
            assert_eq!(safety.next_safety_sample_us, Some(1_000));
        });
    }

    #[test]
    fn outer_safety_rejects_zero_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());

        let result = safety.configure_encoder_resolution(0);

        assert!(result.is_err());
        assert!(safety.encoder_domain.is_none());
    }

    #[test]
    fn outer_safety_retains_a_normalized_typed_baseline() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());

        safety.configure_encoder_resolution(4_000).unwrap();
        safety.set_encoder_baseline(8_003).unwrap();

        assert_eq!(
            safety.prev_encoder_count.map(|count| count.raw_counts()),
            Some(3)
        );
    }

    #[test]
    fn outer_safety_limits_scale_with_encoder_resolution() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());

        safety.configure_encoder_resolution(8_000).unwrap();

        assert_eq!(safety.counts_per_rev(), 8_000);
        assert_eq!(safety.position_window, 16_000);
    }

    #[test]
    fn outer_safety_clamps_position_window_to_host_budget() {
        let mut safety = OuterSafetyLimits::new(
            100,
            OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000),
        );

        safety.configure_encoder_resolution(4_000).unwrap();

        assert_eq!(safety.position_window, 2_000);
        assert_eq!(safety.negative_position_window, 8_000);
        assert_eq!(safety.positive_position_window, 8_000);
    }

    #[test]
    fn outer_safety_legacy_setup_retains_the_two_revolution_backstop() {
        let mut budget = OuterMotionBudget::test_budget(2_500, 6_000, 3_000, 4_375, 200_000);
        budget.negative_position_headroom_mrev = 3_000;
        budget.positive_position_headroom_mrev = 4_000;
        let mut safety = OuterSafetyLimits::new(100, budget);

        safety.configure_encoder_resolution(4_000).unwrap();

        assert_eq!(safety.position_window, 8_000);
        assert_eq!(safety.negative_position_window, 8_000);
        assert_eq!(safety.positive_position_window, 8_000);
    }

    #[test]
    fn robustness_window_opens_to_the_absolute_clamp() {
        let mut budget = OuterMotionBudget::test_budget(3_000, 6_000, 3_000, 4_375, 200_000);
        budget.negative_position_headroom_mrev = 4_000;
        budget.positive_position_headroom_mrev = 5_000;

        let mut sweep_safety = OuterSafetyLimits::new(100, budget);
        sweep_safety
            .configure_velocity_sweep_encoder_resolution(4_000)
            .unwrap();
        assert_eq!(sweep_safety.position_window, 12_000);
        assert_eq!(sweep_safety.negative_position_window, 16_000);
        assert_eq!(sweep_safety.positive_position_window, 20_000);

        let mut robustness_safety = OuterSafetyLimits::new(100, budget);
        robustness_safety
            .configure_robustness_encoder_resolution(4_000)
            .unwrap();
        assert_eq!(robustness_safety.position_window, 16_400);
        assert_eq!(robustness_safety.negative_position_window, 16_400);
        assert_eq!(robustness_safety.positive_position_window, 20_400);
    }

    #[test]
    fn robustness_absolute_window_tolerates_a_transient_within_the_reserve() {
        pollster::block_on(async {
            let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000);
            budget.negative_position_headroom_mrev = 100;
            budget.positive_position_headroom_mrev = 200;
            let mut safety = OuterSafetyLimits::new(100, budget);
            safety
                .configure_robustness_encoder_resolution(4_000)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(0).unwrap();
            safety.next_safety_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            safety.unwrapped_position = 799;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            assert_eq!(
                check_outer_safety(&mut safety, &mut backend).await,
                Ok(None),
                "801 counts is past raw headroom (800) but within the 400-count reserve"
            );
        });
    }

    #[test]
    fn robustness_absolute_window_still_faults_beyond_the_reserve() {
        pollster::block_on(async {
            let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000);
            budget.negative_position_headroom_mrev = 100;
            budget.positive_position_headroom_mrev = 200;
            let mut safety = OuterSafetyLimits::new(100, budget);
            safety
                .configure_robustness_encoder_resolution(4_000)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(0).unwrap();
            safety.next_safety_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            safety.unwrapped_position = 1_199;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should succeed")
                .expect("drift beyond the reserved window must still fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_POSITION);
        });
    }

    #[test]
    fn integral_window_opens_to_the_absolute_clamp() {
        let mut budget = OuterMotionBudget::test_budget(3_000, 6_000, 3_000, 4_375, 200_000);
        budget.negative_position_headroom_mrev = 4_000;
        budget.positive_position_headroom_mrev = 5_000;

        let mut integral_safety = OuterSafetyLimits::new(100, budget);
        integral_safety
            .configure_integral_encoder_resolution(4_000)
            .unwrap();
        assert_eq!(integral_safety.position_window, 16_000);
        assert_eq!(integral_safety.negative_position_window, 16_000);
        assert_eq!(integral_safety.positive_position_window, 20_000);
    }

    #[test]
    fn integral_window_tightens_with_a_finite_reserve() {
        let mut budget = OuterMotionBudget::test_budget(3_000, 6_000, 3_000, 4_375, 200_000);
        budget.negative_position_headroom_mrev = 4_000;
        budget.positive_position_headroom_mrev = 5_000;

        let mut integral_safety = OuterSafetyLimits::new(100, budget);
        integral_safety
            .configure_velocity_sweep_encoder_resolution(4_000)
            .unwrap();
        assert_eq!(integral_safety.position_window, 12_000);

        integral_safety.tighten_position_window(Some(500));
        assert_eq!(integral_safety.position_window, 12_500);
    }

    #[test]
    fn integral_window_finite_reserve_clamps_to_the_headroom_ceiling() {
        let mut budget = OuterMotionBudget::test_budget(3_000, 6_000, 3_000, 4_375, 200_000);
        budget.negative_position_headroom_mrev = 4_000;
        budget.positive_position_headroom_mrev = 5_000;

        let mut integral_safety = OuterSafetyLimits::new(100, budget);
        integral_safety
            .configure_velocity_sweep_encoder_resolution(4_000)
            .unwrap();

        integral_safety.tighten_position_window(Some(u32::MAX));
        assert_eq!(integral_safety.position_window, 16_000);
    }

    #[test]
    fn integral_window_no_reserve_opens_to_the_headroom_ceiling() {
        let mut budget = OuterMotionBudget::test_budget(3_000, 6_000, 3_000, 4_375, 200_000);
        budget.negative_position_headroom_mrev = 4_000;
        budget.positive_position_headroom_mrev = 5_000;

        let mut integral_safety = OuterSafetyLimits::new(100, budget);
        integral_safety
            .configure_velocity_sweep_encoder_resolution(4_000)
            .unwrap();

        integral_safety.tighten_position_window(None);
        assert_eq!(integral_safety.position_window, 16_000);
    }

    fn envelope_armed_safety(
        planned_velocity_mrev_s: u32,
        effective_ceiling_mrev_s: u32,
    ) -> OuterSafetyLimits {
        let mut safety = armed_safety(Some(100_000), 101_000);
        safety
            .configure_velocity_envelope_with_settle_ceiling(
                planned_velocity_mrev_s,
                effective_ceiling_mrev_s,
            )
            .expect("the ceiling admits the envelope");
        safety
    }

    #[test]
    fn outer_safety_velocity_envelope_is_not_truncated_to_whole_counts_per_ms() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 13;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 27;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed");

            assert!(
                fault.is_none(),
                "27 counts over 2 ms is 13.5 counts/ms, inside the 13.8 counts/ms envelope"
            );
        });
    }

    #[test]
    fn outer_safety_velocity_envelope_admits_the_exact_cap() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 102_500;
            backend.mechanical_encoder_position_counts_value = 34;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 105_000;
            backend.mechanical_encoder_position_counts_value = 69;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed");

            assert!(fault.is_none(), "the cap itself is not a violation");
        });
    }

    #[test]
    fn outer_safety_velocity_envelope_rejects_one_count_above_the_exact_cap() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 102_500;
            backend.mechanical_encoder_position_counts_value = 35;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 105_000;
            backend.mechanical_encoder_position_counts_value = 70;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed")
                .expect("14.0 counts/ms must exceed the 13.8 counts/ms envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
        });
    }

    #[test]
    fn primitive_peak_tracks_the_two_sample_window_and_resets_per_primitive() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 10;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert_eq!(
                safety.primitive_peak_milli_counts_per_s(),
                0,
                "one sample is no window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 14;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert_eq!(
                safety.primitive_peak_milli_counts_per_s(),
                7_000_000,
                "14 counts over 2 ms is 7 counts/ms"
            );

            backend.elapsed_us_value = 103_000;
            backend.mechanical_encoder_position_counts_value = 15;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert_eq!(
                safety.primitive_peak_milli_counts_per_s(),
                7_000_000,
                "a slower window does not lower the peak"
            );

            safety.end_primitive();
            safety.begin_primitive(103_000).unwrap();
            assert_eq!(safety.primitive_peak_milli_counts_per_s(), 0);
            assert!(!safety.ceiling_latched());
        });
    }

    #[test]
    fn reset_primitive_peak_excludes_the_straddling_window_from_the_peak_only() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 10;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 14;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert_eq!(safety.primitive_peak_milli_counts_per_s(), 7_000_000);

            safety.reset_primitive_peak();
            assert_eq!(safety.primitive_peak_milli_counts_per_s(), 0);

            backend.elapsed_us_value = 103_000;
            backend.mechanical_encoder_position_counts_value = 30;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert_eq!(
                safety.primitive_peak_milli_counts_per_s(),
                0,
                "the window straddling the reset stays out of the peak"
            );

            backend.elapsed_us_value = 104_000;
            backend.mechanical_encoder_position_counts_value = 40;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert_eq!(
                safety.primitive_peak_milli_counts_per_s(),
                13_000_000,
                "26 counts over 2 ms is 13 counts/ms"
            );
        });
    }

    #[test]
    fn reset_primitive_peak_still_evaluates_the_straddling_window_against_the_cap() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 102_500;
            backend.mechanical_encoder_position_counts_value = 35;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();

            safety.reset_primitive_peak();

            backend.elapsed_us_value = 105_000;
            backend.mechanical_encoder_position_counts_value = 70;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("velocity observation should succeed")
                .expect("14.0 counts/ms must still exceed the 13.8 counts/ms envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(safety.primitive_peak_milli_counts_per_s(), 0);
        });
    }

    #[test]
    fn reset_primitive_peak_leaves_primitive_state_alone() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(5_000, 7_500);
            let cap = safety.active_velocity_cap_milli_counts_per_s(100_000);
            safety.set_velocity_ceiling(Some(cap * 4 / 5));
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            for (t, pos) in [(101_000, 24), (102_000, 48)] {
                backend.elapsed_us_value = t;
                backend.mechanical_encoder_position_counts_value = pos;
                check_outer_safety(&mut safety, &mut backend).await.unwrap();
            }
            assert!(safety.ceiling_latched());

            let start_position = safety.primitive_start_position;
            let start_us = safety.primitive_start_us;
            let excursion = safety.max_local_position_excursion();
            let observation = safety.last_accepted_observation();

            safety.reset_primitive_peak();

            assert_eq!(safety.primitive_start_position, start_position);
            assert_eq!(safety.primitive_start_us, start_us);
            assert_eq!(safety.max_local_position_excursion(), excursion);
            assert_eq!(safety.last_accepted_observation(), observation);
            assert!(safety.ceiling_latched());
        });
    }

    #[test]
    fn ceiling_latches_at_the_ceiling_and_not_one_count_below() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(5_000, 7_500);
            let cap = safety.active_velocity_cap_milli_counts_per_s(100_000);
            assert_eq!(cap, 30_000_000);
            safety.set_velocity_ceiling(Some(cap * 4 / 5));
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();

            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 24;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 47;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert!(
                !safety.ceiling_latched(),
                "47 counts over 2 ms is one count below the ceiling"
            );

            safety.end_primitive();
            safety.begin_primitive(102_000).unwrap();
            backend.elapsed_us_value = 103_000;
            backend.mechanical_encoder_position_counts_value = 71;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            backend.elapsed_us_value = 104_000;
            backend.mechanical_encoder_position_counts_value = 95;
            let fault = check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert!(fault.is_none(), "the ceiling is below the cap, so no fault");
            assert!(
                safety.ceiling_latched(),
                "48 counts over 2 ms is exactly the ceiling"
            );
        });
    }

    #[test]
    fn a_forced_latch_reads_back_until_the_next_primitive() {
        let mut safety = envelope_armed_safety(1_450, 3_450);
        safety.begin_primitive(100_000).unwrap();
        safety.force_ceiling_latch_for_test();
        assert!(safety.ceiling_latched());
        safety.end_primitive();
        assert!(
            safety.ceiling_latched(),
            "end_primitive leaves the latch for the reader"
        );
        safety.begin_primitive(101_000).unwrap();
        assert!(!safety.ceiling_latched());
    }

    #[test]
    fn a_faulting_window_also_sets_the_latch_and_the_peak() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            let cap = safety.active_velocity_cap_milli_counts_per_s(100_000);
            safety.set_velocity_ceiling(Some(cap * 4 / 5));
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 102_500;
            backend.mechanical_encoder_position_counts_value = 35;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            backend.elapsed_us_value = 105_000;
            backend.mechanical_encoder_position_counts_value = 70;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .unwrap()
                .expect("14.0 counts/ms exceeds the 13.8 counts/ms envelope");
            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert!(safety.ceiling_latched());
            assert_eq!(safety.primitive_peak_milli_counts_per_s(), 14_000_000);
        });
    }

    #[test]
    fn no_ceiling_means_no_latch() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 3_450);
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 102_500;
            backend.mechanical_encoder_position_counts_value = 34;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            backend.elapsed_us_value = 105_000;
            backend.mechanical_encoder_position_counts_value = 69;
            check_outer_safety(&mut safety, &mut backend).await.unwrap();
            assert!(!safety.ceiling_latched());
        });
    }

    #[test]
    fn outer_safety_settle_phase_admits_braking_up_to_the_effective_ceiling() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 10_000);
            safety.begin_primitive(100_000).unwrap();
            safety.enter_settle_phase();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 20;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 40;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed");

            assert!(
                fault.is_none(),
                "20 counts/ms is above the moving envelope and inside the settle ceiling"
            );
        });
    }

    #[test]
    fn outer_safety_moving_phase_admits_machine_scale_velocity() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 10_000);
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 20;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 40;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed");

            assert!(
                fault.is_none(),
                "20 counts/ms is inside the 40 counts/ms machine ceiling"
            );
        });
    }

    #[test]
    fn outer_safety_settle_phase_still_faults_above_the_effective_ceiling() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 10_000);
            safety.begin_primitive(100_000).unwrap();
            safety.enter_settle_phase();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 45;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 90;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed")
                .expect("45 counts/ms must exceed the 40 counts/ms settle ceiling");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
        });
    }

    #[test]
    fn outer_safety_both_phases_share_the_effective_ceiling_bound() {
        let mut safety = envelope_armed_safety(1_450, 10_000);
        safety.begin_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            40_000_000
        );

        safety.enter_settle_phase();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            40_000_000
        );

        safety.end_primitive();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            40_000_000
        );
    }

    #[test]
    fn outer_safety_settle_ceiling_is_never_below_the_moving_envelope() {
        let mut safety = armed_safety(Some(100_000), 101_000);

        assert_eq!(
            safety.configure_velocity_envelope_with_settle_ceiling(1_450, 3_000),
            Err(CommissionError::VelocityEnvelopeExceedsCeiling)
        );
    }

    #[test]
    fn transient_allowance_widens_the_cap_for_the_first_24_ms_of_a_velocity_step() {
        let mut safety = armed_safety(Some(100_000), 101_000);
        safety.configure_velocity_envelope(3_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            20_000_000,
            "no primitive open: the steady cap applies"
        );

        safety.begin_velocity_step_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            30_000_000
        );
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(123_999),
            30_000_000
        );
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(124_000),
            20_000_000
        );

        safety.end_primitive();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            20_000_000
        );
    }

    #[test]
    fn transient_allowance_is_clamped_to_the_machine_ceiling() {
        let mut budget = OuterMotionBudget::firmware_caps();
        budget.machine_velocity_ceiling_mrev_s = 6_000;
        let mut safety = OuterSafetyLimits::new(100, budget);
        safety.configure_encoder_resolution(4_000).unwrap();
        safety.configure_velocity_envelope(3_000).unwrap();
        safety.begin_velocity_step_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            24_000_000,
            "max(7_500, 7_000) mrev/s clamps to the 6_000 mrev/s machine ceiling"
        );
    }

    #[test]
    fn transient_allowance_never_drops_below_the_moving_cap() {
        let mut budget = OuterMotionBudget::firmware_caps();
        budget.machine_velocity_ceiling_mrev_s = 4_000;
        let mut safety = OuterSafetyLimits::new(100, budget);
        safety.configure_encoder_resolution(4_000).unwrap();
        safety.configure_velocity_envelope(3_000).unwrap();
        safety.begin_velocity_step_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            20_000_000,
            "a ceiling below the moving cap leaves the moving cap in force"
        );
    }

    #[test]
    fn transient_allowance_applies_above_an_effective_ceiling_moving_cap() {
        let mut safety = envelope_armed_safety(5_000, 7_500);
        safety.begin_velocity_step_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            40_000_000,
            "max(12_500, 9_000) mrev/s clamps to the 10_000 mrev/s machine ceiling"
        );
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(124_000),
            30_000_000
        );
    }

    #[test]
    fn a_light_axis_step_overshoot_inside_the_transient_window_does_not_fault() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            safety.configure_velocity_envelope(3_000).unwrap();
            safety.begin_velocity_step_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            let deltas: [u32; 9] = [1, 4, 6, 9, 11, 14, 16, 19, 22];
            let mut position = 0;
            for (index, delta) in deltas.iter().enumerate() {
                position += delta;
                backend.elapsed_us_value = 101_000 + index as u64 * 1_000;
                backend.mechanical_encoder_position_counts_value = position;
                let fault = check_outer_safety(&mut safety, &mut backend).await.unwrap();
                assert!(
                    fault.is_none(),
                    "41 counts over the 2 ms window ending at {} ms is 20.5 counts/ms, \
                     above the 20 counts/ms steady cap and below the 30 counts/ms transient cap",
                    index + 1
                );
            }
        });
    }

    #[test]
    fn velocity_still_above_the_steady_cap_after_the_transient_window_faults() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            safety.configure_velocity_envelope(3_000).unwrap();
            safety.begin_velocity_step_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            let mut position = 0;
            let mut faulted = None;
            for index in 0..30u64 {
                position += 22;
                backend.elapsed_us_value = 101_000 + index * 1_000;
                backend.mechanical_encoder_position_counts_value = position;
                if let Some(fault) = check_outer_safety(&mut safety, &mut backend).await.unwrap() {
                    faulted = Some((index + 1, fault));
                    break;
                }
            }
            let (ms, fault) =
                faulted.expect("a sustained 22 counts/ms faults once the window closes");
            assert_eq!(
                ms, 24,
                "the first sample at 24 ms is evaluated against the steady cap"
            );
            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.velocity_cap_counts_per_ms, 20);
        });
    }

    #[test]
    fn a_fault_inside_the_transient_window_reports_the_transient_cap() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            safety.configure_velocity_envelope(3_000).unwrap();
            safety.begin_velocity_step_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 35;
            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .unwrap()
                    .is_none()
            );
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 70;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .unwrap()
                .expect("35 counts/ms exceeds the 30 counts/ms transient cap");
            assert_eq!(fault.velocity_cap_counts_per_ms, 30);
            assert_eq!(fault.velocity_counts_per_ms, 35);
        });
    }

    #[test]
    fn a_profiled_primitive_keeps_the_steady_cap_inside_the_window() {
        let mut safety = armed_safety(Some(100_000), 101_000);
        safety.configure_velocity_envelope(3_000).unwrap();
        safety.begin_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            20_000_000
        );
    }

    #[test]
    fn ending_a_velocity_step_clears_the_allowance_for_the_next_primitive() {
        let mut safety = armed_safety(Some(100_000), 101_000);
        safety.configure_velocity_envelope(3_000).unwrap();
        safety.begin_velocity_step_primitive(100_000).unwrap();
        safety.end_primitive();
        safety.begin_primitive(100_000).unwrap();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            20_000_000
        );
    }

    #[test]
    fn configure_velocity_envelope_with_settle_ceiling_rejects_zero_ceiling() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        let error = safety
            .configure_velocity_envelope_with_settle_ceiling(1_000, 0)
            .unwrap_err();
        assert_eq!(error, CommissionError::VelocityEnvelopeCeilingZero);
    }

    #[test]
    fn configure_velocity_envelope_with_settle_ceiling_rejects_envelope_above_ceiling() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        let error = safety
            .configure_velocity_envelope_with_settle_ceiling(1_000_000, 1)
            .unwrap_err();
        assert_eq!(error, CommissionError::VelocityEnvelopeExceedsCeiling);
    }

    #[test]
    fn configure_velocity_envelope_up_to_ceiling_widens_to_the_effective_ceiling() {
        let mut safety = armed_safety(Some(100_000), 101_000);
        safety
            .configure_velocity_envelope_up_to_ceiling(5_366, 12_500)
            .expect("a ceiling above the proportional envelope is accepted");
        // 12_500 mrev/s at 4_000 counts/rev is 50_000_000 milli-counts/s, for the
        // stroke and for its zero-target settle alike.
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            50_000_000
        );
        safety.enter_settle_phase();
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            50_000_000
        );
    }

    #[test]
    fn configure_velocity_envelope_up_to_ceiling_never_drops_below_the_proportional_envelope() {
        let mut safety = armed_safety(Some(100_000), 101_000);
        // max(ceil(5_366 * 3 / 2), 5_366 + 2_000) = 8_049 mrev/s at 4_000 counts/rev.
        let proportional_envelope_milli_counts_per_s = 8_049 * 4_000;
        safety
            .configure_velocity_envelope_up_to_ceiling(5_366, 0)
            .expect("a zero ceiling keeps the proportional envelope");
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            proportional_envelope_milli_counts_per_s
        );
        safety
            .configure_velocity_envelope_up_to_ceiling(5_366, 8_000)
            .expect("a ceiling below the proportional envelope keeps it");
        assert_eq!(
            safety.active_velocity_cap_milli_counts_per_s(100_000),
            proportional_envelope_milli_counts_per_s
        );
    }

    #[test]
    fn configure_velocity_envelope_up_to_ceiling_rejects_zero_planned_velocity() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        assert_eq!(
            safety.configure_velocity_envelope_up_to_ceiling(0, 10_000),
            Err(CommissionError::InvalidSchedule)
        );
    }

    #[test]
    fn envelope_mrev_s_rejects_overflowing_proportional_margin() {
        let error = OuterSafetyLimits::envelope_mrev_s(u32::MAX).unwrap_err();
        assert_eq!(error, CommissionError::TargetRateInvalid);
    }

    #[test]
    fn outer_safety_settle_phase_keeps_the_quarter_turn_fault() {
        pollster::block_on(async {
            let mut safety = envelope_armed_safety(1_450, 10_000);
            safety.begin_primitive(100_000).unwrap();
            safety.enter_settle_phase();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 1_000;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("a quarter turn in one sample must fault regardless of phase");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
        });
    }

    #[test]
    fn outer_safety_marks_soft_lateness_unscored_without_faulting() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_500;
            backend.mechanical_encoder_position_counts_value = 1;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("a soft-late observation should remain safe");

            assert!(fault.is_none());
            assert!(!safety.last_observation_scoring_eligible());
        });
    }

    #[test]
    fn outer_safety_marks_on_time_observations_scoring_eligible() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 1;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("an on-time observation should remain safe");

            assert!(fault.is_none());
            assert!(safety.last_observation_scoring_eligible());
        });
    }

    #[test]
    fn outer_safety_faults_on_large_gap_before_interpreting_delta() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            safety.unwrapped_position = 17;
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = MAX_SAFETY_OBSERVATION_GAP_US + 1;
            backend.mechanical_encoder_position_counts_value = 1_999;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should produce a safety fault")
                .expect("a read-to-read gap over the configured maximum must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_OBSERVATION_GAP);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn outer_safety_allows_only_the_nonfaulting_quarter_turn_at_the_maximum_gap() {
        pollster::block_on(async {
            let mut legal = armed_safety(Some(0), 1_000);
            let mut legal_backend = MockBackend::new();
            legal_backend.elapsed_us_value = MAX_SAFETY_OBSERVATION_GAP_US;
            legal_backend.mechanical_encoder_position_counts_value = 999;

            assert_eq!(
                check_outer_safety(&mut legal, &mut legal_backend).await,
                Ok(None)
            );
            assert_eq!(legal.unwrapped_position, 999);

            let mut quarter_turn = armed_safety(Some(0), 1_000);
            let before = quarter_turn;
            let mut quarter_turn_backend = MockBackend::new();
            quarter_turn_backend.elapsed_us_value = MAX_SAFETY_OBSERVATION_GAP_US;
            quarter_turn_backend.mechanical_encoder_position_counts_value = 1_000;

            let fault = check_outer_safety(&mut quarter_turn, &mut quarter_turn_backend)
                .await
                .expect("the observation should produce a safety fault")
                .expect("an exact quarter turn must fault at the maximum gap");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
            assert_eq!(quarter_turn.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(quarter_turn.last_sample_us, before.last_sample_us);
            assert_eq!(quarter_turn.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn observation_gap_contract_records_the_faulting_gap_before_delta_interpretation() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1);
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = MAX_SAFETY_OBSERVATION_GAP_US + 1;
            backend.mechanical_encoder_position_counts_value = 1_000;
            let mut timing = OuterSafetyTimingAccumulator::new();
            let context = OuterSafetyTimingContext::new(
                OuterSafetyTimingStage::Proportional,
                OuterSafetyTimingPhase::RecoverySetup,
                21,
                7,
                true,
            )
            .unwrap();

            let fault =
                check_outer_safety_with_timing(&mut safety, &mut timing, context, &mut backend)
                    .await
                    .expect("the observation should be interpreted")
                    .expect("the over-bound gap must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_OBSERVATION_GAP);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
            let summary = timing.summary();
            assert_eq!(summary.sample_count, 1);
            assert_eq!(
                u64::from(summary.max_gap_us),
                MAX_SAFETY_OBSERVATION_GAP_US + 1
            );
            assert_eq!(summary.max_context, Some(context));
            assert_eq!(summary.over_bound_count, 1);
        });
    }

    #[test]
    fn outer_safety_faults_inclusively_at_one_quarter_turn() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(0), 1_000);
            safety.unwrapped_position = 17;
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 1_000;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should produce a safety fault")
                .expect("an exact quarter-turn delta must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn outer_safety_local_window_spans_motion_and_zero_settle() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(0).unwrap();
            safety.next_safety_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            safety.begin_primitive(0).unwrap();
            safety.unwrapped_position = 1_999;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the settle observation should succeed")
                .expect("settle drift beyond the full primitive window must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_POSITION_LOCAL);
            assert_eq!(fault.position_counts, 2_001);
            assert!(fault.position_window_counts > 0);
            assert!(
                fault.position_counts.unsigned_abs() > fault.position_window_counts.unsigned_abs()
            );
        });
    }

    #[test]
    fn outer_safety_allows_healthy_deceleration_to_use_settle_reserve() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(0).unwrap();
            safety.next_safety_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            safety.begin_primitive(0).unwrap();
            safety.unwrapped_position = 1_600;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 200;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the settle observation should succeed");

            assert!(fault.is_none());
            assert_eq!(safety.unwrapped_position, 1_800);
        });
    }

    #[test]
    fn outer_safety_asymmetric_window_catches_cumulative_drift() {
        pollster::block_on(async {
            let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000);
            budget.negative_position_headroom_mrev = 100;
            budget.positive_position_headroom_mrev = 200;
            let mut safety = OuterSafetyLimits::new(100, budget);
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(0).unwrap();
            safety.next_safety_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            safety.unwrapped_position = 799;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should succeed")
                .expect("positive cumulative drift beyond headroom must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_POSITION);
            assert_eq!(fault.position_counts, 801);
            assert!(fault.position_window_counts > 0);
            assert!(
                fault.position_counts.unsigned_abs() > fault.position_window_counts.unsigned_abs()
            );
        });
    }

    #[test]
    fn outer_safety_position_tie_break_reports_absolute_branch() {
        pollster::block_on(async {
            let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000);
            budget.positive_position_headroom_mrev = 200;
            let mut safety = OuterSafetyLimits::new(100, budget);
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(0).unwrap();
            safety.next_safety_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            safety.begin_primitive(0).unwrap();
            safety.unwrapped_position = 2_499;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("the observation should succeed")
                .expect("position beyond both windows must fault");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_POSITION);
            assert_eq!(fault.position_counts, 2_501);
            assert!(fault.position_window_counts > 0);
            assert!(
                fault.position_counts.unsigned_abs() > fault.position_window_counts.unsigned_abs()
            );
        });
    }

    #[test]
    fn outer_safety_refuses_zero_budget() {
        let safety = OuterSafetyLimits::new(
            100,
            OuterMotionBudget::test_budget(0, 6_000, 3_000, 4_375, 200_000),
        );

        assert!(!safety.budget_is_usable());
    }

    #[test]
    fn outer_safety_refuses_reserve_that_consumes_the_window() {
        let mut budget = OuterMotionBudget::test_budget(500, 6_000, 3_000, 4_375, 200_000);
        budget.settle_travel_reserve_mrev = 500;
        let safety = OuterSafetyLimits::new(100, budget);

        assert!(!safety.budget_is_usable());
    }

    #[test]
    fn outer_safety_violates_on_unusable_budget() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget::test_budget(0, 6_000, 3_000, 4_375, 200_000),
            );
            let mut backend = MockBackend::new();

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_some());
        });
    }

    #[test]
    fn outer_safety_violates_on_duration_budget() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget::test_budget(500, 6_000, 1, 4_375, 200_000),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.primitive_start_us = Some(1_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_001;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_some());
        });
    }

    #[test]
    fn outer_safety_normalizes_velocity_by_elapsed_time() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(2328).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_500;
            backend.mechanical_encoder_position_counts_value = 2339;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(
                fault.is_none(),
                "time-normalized velocity should stay below the cap"
            );
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(2339)
            );
            assert_eq!(safety.last_sample_us, Some(101_500));
            assert_eq!(safety.unwrapped_position, 11);
        });
    }

    #[test]
    fn outer_safety_velocity_window_tolerates_isolated_one_count_spike() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            let spike_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");

            assert!(
                spike_fault.is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 48;
            let recovery_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed");

            assert!(
                recovery_fault.is_none(),
                "25 + 23 counts over 2 ms equals the 24 counts/ms cap"
            );
        });
    }

    #[test]
    fn outer_safety_velocity_window_rejects_sustained_excess_on_second_interval() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");

            assert!(
                first_fault.is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;
            let second_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed")
                .expect("two 25-count intervals must exceed the 24 counts/ms cap");

            assert_eq!(second_fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
        });
    }

    #[test]
    fn outer_safety_velocity_fault_reports_the_window_that_tripped() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 30;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none(),
                "the first interval only primes the window"
            );

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("second velocity observation should succeed")
                .expect("50 counts over 2 ms is 25 counts/ms against the 24 counts/ms cap");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, 50, "summed counts across the window");
            assert_eq!(fault.dt_us, 2_000, "summed elapsed across the window");
            assert_eq!(fault.velocity_counts_per_ms, 25);
            assert!(
                fault.velocity_counts_per_ms >= fault.velocity_cap_counts_per_ms,
                "a reported velocity below the reported cap reads as a spurious fault"
            );
        });
    }

    #[test]
    fn outer_safety_velocity_window_resets_with_encoder_baseline() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none()
            );

            safety.set_encoder_baseline(25).unwrap();
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("post-baseline observation should succeed")
                    .is_none(),
                "a new encoder baseline must prime a new velocity window"
            );
        });
    }

    #[test]
    fn outer_safety_velocity_window_resets_with_motion_primitive() {
        pollster::block_on(async {
            let mut safety = armed_safety(Some(100_000), 101_000);
            safety.begin_primitive(100_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("first velocity observation should succeed")
                    .is_none()
            );

            safety.end_primitive();
            safety.begin_primitive(101_000).unwrap();
            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;

            assert!(
                check_outer_safety(&mut safety, &mut backend)
                    .await
                    .expect("new-primitive observation should succeed")
                    .is_none(),
                "a new motion primitive must prime a new velocity window"
            );
        });
    }

    #[test]
    fn outer_safety_tracks_forward_origin_crossing() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(3_998).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100_000).unwrap();
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 2;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(2)
            );
            assert_eq!(safety.unwrapped_position, 4);
        });
    }

    #[test]
    fn outer_safety_tracks_reverse_origin_crossing() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(2).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100_000).unwrap();
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 3_998;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed");

            assert!(fault.is_none());
            assert_eq!(
                safety.prev_encoder_count.map(|count| count.raw_counts()),
                Some(3_998)
            );
            assert_eq!(safety.unwrapped_position, -4);
        });
    }

    #[test]
    fn outer_safety_half_turn_fault_preserves_accumulation_state() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(1_000).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100_000).unwrap();
            safety.primitive_start_us = Some(90_000);
            safety.unwrapped_position = 17;
            let before = safety;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 3_000;

            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("ambiguous half-turn should violate safety");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_QUARTER_TURN);
            assert_eq!(safety.prev_encoder_count, before.prev_encoder_count);
            assert_eq!(safety.last_sample_us, before.last_sample_us);
            assert_eq!(safety.primitive_start_us, before.primitive_start_us);
            assert_eq!(safety.unwrapped_position, before.unwrapped_position);
        });
    }

    #[test]
    fn outer_safety_reports_velocity_fault_details() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(
                100,
                OuterMotionBudget::test_budget(750, 6_000, 3_000, 4_375, 200_000),
            );
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100_000).unwrap();
            safety.primitive_start_us = Some(90_000);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101_000;
            backend.mechanical_encoder_position_counts_value = 25;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");
            assert!(first_fault.is_none());

            backend.elapsed_us_value = 102_000;
            backend.mechanical_encoder_position_counts_value = 50;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.max_travel_mrev, 750);
            assert_eq!(fault.max_velocity_mrev_s, 6000);
            assert_eq!(fault.max_duration_ms, 3000);
            assert_eq!(fault.direction_mask, 3);
            assert_eq!(fault.delta_counts, 50);
            assert_eq!(fault.dt_us, 2_000);
            assert_eq!(fault.velocity_counts_per_ms, 25);
            assert_eq!(fault.velocity_cap_counts_per_ms, 24);
            assert_eq!(fault.position_counts, 50);
            assert_eq!(fault.position_window_counts, 3000);
            assert_eq!(fault.elapsed_us, 12_000);
            assert_eq!(fault.duration_cap_us, 3_000_000);
        });
    }

    #[test]
    fn outer_safety_reports_large_positive_velocity_fault_evidence() {
        pollster::block_on(async {
            const LARGE_DELTA: u32 = 536_870_910;
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety
                .configure_encoder_resolution(i32::MAX as u32)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100).unwrap();
            safety.primitive_start_us = Some(90);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101;
            backend.mechanical_encoder_position_counts_value = LARGE_DELTA;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");
            assert!(first_fault.is_none());

            backend.elapsed_us_value = 1_101;
            backend.mechanical_encoder_position_counts_value = LARGE_DELTA * 2;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, (LARGE_DELTA * 2) as i32);
            assert_eq!(fault.dt_us, 1_001);
            assert_eq!(fault.velocity_counts_per_ms, 1_072_669_150);
        });
    }

    #[test]
    fn outer_safety_reports_large_negative_velocity_fault_evidence() {
        pollster::block_on(async {
            const LARGE_DELTA: u32 = 536_870_910;
            const ENCODER_DOMAIN: u32 = i32::MAX as u32;
            let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
            safety
                .configure_encoder_resolution(i32::MAX as u32)
                .unwrap();
            safety.set_encoder_baseline(0).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(100).unwrap();
            safety.primitive_start_us = Some(90);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 101;
            backend.mechanical_encoder_position_counts_value = ENCODER_DOMAIN - LARGE_DELTA;

            let first_fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("first velocity observation should succeed");
            assert!(first_fault.is_none());

            backend.elapsed_us_value = 1_101;
            backend.mechanical_encoder_position_counts_value = ENCODER_DOMAIN - LARGE_DELTA * 2;
            let fault = check_outer_safety(&mut safety, &mut backend)
                .await
                .expect("encoder read should succeed")
                .expect("velocity should violate the safety envelope");

            assert_eq!(fault.reason, OUTER_SAFETY_FAULT_VELOCITY);
            assert_eq!(fault.delta_counts, -((LARGE_DELTA * 2) as i32));
            assert_eq!(fault.dt_us, 1_001);
            assert_eq!(fault.velocity_counts_per_ms, -1_072_669_150);
        });
    }

    #[test]
    fn outer_safety_read_error_fails_with_spi() {
        pollster::block_on(async {
            let mut safety = OuterSafetyLimits::new(2448, OuterMotionBudget::firmware_caps());
            safety.configure_encoder_resolution(4_000).unwrap();
            safety.last_sample_us = OuterSafetyTimestamp::some(1_000).unwrap();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 2_000;
            backend.fail_on_read = Some(ABN_DECODER_COUNT.addr());

            let result = check_outer_safety(&mut safety, &mut backend).await;

            assert!(matches!(result, Err(CommissionError::Spi)));
        });
    }

    #[test]
    fn arm_safety_observation_rejects_overflowing_deadline() {
        let mut safety = OuterSafetyLimits::new(100, OuterMotionBudget::firmware_caps());
        let error = safety.arm_safety_observation(u64::MAX).unwrap_err();
        assert_eq!(error, CommissionError::ScheduleDeadlineOverflow);
    }
}
