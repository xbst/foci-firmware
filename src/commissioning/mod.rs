//! Cooperative commissioning state machine.

pub mod backend;
pub mod capture_timing;
pub(crate) mod encoder_domain;
pub mod helpers;
pub mod inner;
pub mod outer;
pub mod schedule;
pub(crate) mod stimulus;
pub mod types;

#[cfg(test)]
#[path = "phase_emission_tests.rs"]
mod phase_emission_tests;

pub(crate) mod vocabulary_generated;

use crate::autotune::{
    CommissionError, CommissionPhase, CommissionProfile, CommissionStatus, InnerSequence,
    OperatingMode, TransientBuffer,
};
use crate::commissioning::inner::InnerPhase;
use crate::commissioning::outer::{
    OuterMotionBudget, OuterPhase, OuterSafetyLimits, TraceSummarySeqs,
};
use crate::commissioning::outer::{VelocityIntegralRuntime, VelocityIntegralState};
use crate::commissioning::types::{AdcCalResult, EngineKind, StepResult};
use crate::pid::InnerLoopGains;
use crate::state::tmc::TmcChannelState;
use crate::trace::events::{
    CommissionTraceEvent, EVENT_COMMISSION_ENGINE_START, EVENT_COMMISSION_ENGINE_TERMINAL,
    EVENT_COMMISSION_PHASE_END, EVENT_COMMISSION_PHASE_START,
    EVENT_COMMISSION_REST_SELECTION_DIAGNOSTIC, EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY,
    EVENT_COMMISSION_SELECTED_RECOVERY_REST, EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED,
    EVENT_COMMISSION_VELOCITY_INTEGRAL_OBSERVATION, EVENT_COMMISSION_VELOCITY_INTEGRAL_RUNG,
    TraceEventLevel, u32_arg,
};
#[cfg(feature = "trace")]
use crate::trace::events::{
    EVENT_COMMISSION_BREAKAWAY_CAMPAIGN_TERMINAL, EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_PLAN,
    EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_TERMINAL, EVENT_COMMISSION_BREAKAWAY_DISCOVERY_MARGIN,
    EVENT_COMMISSION_BREAKAWAY_DISCOVERY_PLAN, EVENT_COMMISSION_BREAKAWAY_DISCOVERY_TERMINAL,
    EVENT_COMMISSION_BREAKAWAY_PROBE_DECISION, EVENT_COMMISSION_BREAKAWAY_PROBE_PLAN,
    EVENT_COMMISSION_BREAKAWAY_PROBE_RESULT, EVENT_COMMISSION_BREAKAWAY_PROBE_TERMINAL,
    EVENT_COMMISSION_BREAKAWAY_RAW_OBSERVATION,
    EVENT_COMMISSION_BREAKAWAY_RECOVERY_ORIGIN_MISMATCH,
    EVENT_COMMISSION_BREAKAWAY_RECOVERY_REST_REJECTED, EVENT_COMMISSION_OUTER_SAFETY_TIMING,
    EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE, EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL,
};
use foci_tmc4671::pid::{PidIGain, PidPGain};

pub use crate::commissioning::backend::{
    CommissionBoardConfig, CommissionEncoderRead, CommissionMotorControl, CommissionRegisterIo,
    CommissionTiming, CommissionTrace, CommissioningBackend, CommissioningBackend as Backend,
};
pub use crate::commissioning::types::{
    AdcCalResult as AdcCalResultExport, CommissionResult, InnerResult, OuterResult,
    StepResult as Step,
};

const PHASE_END_OK: u8 = 0;
/// Promotion-diagnostic acceptance-source code for a breakaway-confirmed Integral plan.
/// Code 2, the non-breakaway acceptance source, is reserved and must never be reused.
#[cfg(feature = "trace")]
const PROMOTION_SOURCE_BREAKAWAY: u8 = 1;

/// Parameters for outer (mechanical) commissioning engine construction.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct OuterAutotuneInputs {
    pub profile: CommissionProfile,
    pub mode: OperatingMode,
    pub inner_lambda_us: u32,
    pub theta_e_us: u32,
    pub current_bandwidth_hz: u32,
    pub inner_warning_flags: u8,
    pub requested_velocity_mrev_s: u32,
    pub machine_velocity_ceiling_mrev_s: u32,
    pub requested_velocity_source:
        crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource,
    pub max_stroke_travel_mrev: u32,
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom from the installed-tuning entry anchor.
    pub negative_position_headroom_mrev: u32,
    pub positive_position_headroom_mrev: u32,
    pub max_duration_ms: u16,
    /// Axis homing speed, mrev/s; the homing bound's speed.
    pub homing_speed_mrev_s: u32,
    /// Machine acceleration, mrev/s^2; the stroke ramp rate.
    pub max_accel_mrev_s2: u32,
    pub slot_order: crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder,
}

/// Snapshot of outer gains at installed-tuning entry, for soft-failure restoration.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct OuterGainRollbackSnapshot {
    pub velocity_p: PidPGain,
    pub velocity_i: PidIGain,
    pub position_p: PidPGain,
    pub position_i: PidIGain,
    pub velocity_limit: u32,
    pub voltage_limit: u32,
    /// Velocity biquad filter cutoff (Hz), 0 = disabled.
    pub velocity_filter_hz: u16,
    /// Torque biquad filter cutoff (Hz), 0 = disabled.
    pub torque_filter_hz: u16,
    /// Position biquad filter cutoff (Hz), 0 = disabled.
    pub position_filter_hz: u16,
    /// Flux biquad filter cutoff (Hz), 0 = disabled.
    pub flux_filter_hz: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CalibrateEntryGains {
    pub inner: InnerLoopGains,
    /// Validated outer-loop gains, or `None` when retained storage was corrupt.
    pub outer: Option<crate::pid::OuterLoopGains>,
    pub velocity_limit: u32,
    pub velocity_filter_hz: u16,
    pub torque_filter_hz: u16,
    pub position_filter_hz: u16,
    pub flux_filter_hz: u16,
}

impl CalibrateEntryGains {
    fn from_tmc_state(
        tmc_state: &'static TmcChannelState,
        inner: Option<InnerLoopGains>,
        current_filter_default_hz: u16,
    ) -> Self {
        use core::sync::atomic::Ordering::Relaxed;

        Self {
            inner: inner.unwrap_or(crate::pid::DEFAULT_INNER_GAINS),
            outer: tmc_state.load_outer_gains(Relaxed).ok(),
            velocity_limit: tmc_state.velocity_limit_cfg.load(Relaxed),
            velocity_filter_hz: tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
            torque_filter_hz: tmc_state
                .torque_filter_hz
                .load(Relaxed)
                .unwrap_or(current_filter_default_hz),
            position_filter_hz: tmc_state.position_filter_hz.load_or_zero(Relaxed),
            flux_filter_hz: tmc_state
                .flux_filter_hz
                .load(Relaxed)
                .unwrap_or(current_filter_default_hz),
        }
    }
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum EngineMode {
    Inner {
        phase: InnerPhase,
        profile: CommissionProfile,
        sequence: InnerSequence,
        adc_cal: Option<AdcCalResult>,
        calibrate_entry_gains: Option<CalibrateEntryGains>,
    },
    Outer {
        phase: OuterPhase,
        profile: CommissionProfile,
        mode: OperatingMode,
        inner_lambda_us: u32,
        theta_e_us: u32,
        current_bandwidth_hz: u32,
        inner_warning_flags: u8,
        entry_snapshot: OuterGainRollbackSnapshot,
    },
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct TraceRunState {
    engine_seq: u32,
    phase_seq: u32,
    started: bool,
    terminal: bool,
    current_phase: Option<CommissionPhase>,
}

impl TraceRunState {
    const fn new(engine_seq: u32) -> Self {
        Self {
            engine_seq,
            phase_seq: 0,
            started: false,
            terminal: false,
            current_phase: None,
        }
    }
}

/// Cooperative commissioning state machine.
pub struct CommissioningEngine {
    oid: u8,
    tmc_state: &'static TmcChannelState,
    engine_mode: EngineMode,
    buf: TransientBuffer,
    selftest_results: crate::selftest::SelftestResults,
    diagnostics: crate::selftest::CommissionDiagnostics,
    trace_run: TraceRunState,
    trace_summary_seqs: TraceSummarySeqs,
    last_emitted_phase: Option<CommissionPhase>,
}

struct OuterScaffold {
    oid: u8,
    tmc_state: &'static TmcChannelState,
    profile: CommissionProfile,
    mode: OperatingMode,
    inner_lambda_us: u32,
    theta_e_us: u32,
    current_bandwidth_hz: u32,
    inner_warning_flags: u8,
    entry_snapshot: OuterGainRollbackSnapshot,
    trace_engine_seq: u32,
    safety: OuterSafetyLimits,
}

impl CommissioningEngine {
    #[cfg(test)]
    pub(crate) fn new_calibrate(oid: u8) -> Self {
        Self::new_calibrate_with_trace_seq(oid, 0)
    }

    #[cfg(test)]
    pub(crate) fn new_calibrate_with_trace_seq(oid: u8, trace_engine_seq: u32) -> Self {
        Self::new_calibrate_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            trace_engine_seq,
        )
    }

    #[cfg(test)]
    pub(crate) fn new_calibrate_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        trace_engine_seq: u32,
    ) -> Self {
        Self::new_calibrate_for_tmc_with_gains_and_trace_seq(
            oid,
            tmc_state,
            None,
            0,
            trace_engine_seq,
        )
    }

    #[cfg(test)]
    pub(crate) fn new_calibrate_for_tmc_with_gains(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        inner_gains: Option<InnerLoopGains>,
    ) -> Self {
        Self::new_calibrate_for_tmc_with_gains_and_trace_seq(oid, tmc_state, inner_gains, 0, 0)
    }

    /// Create a calibration engine (re-enable path) bound to a TMC channel state, stored gains,
    /// and trace sequence.
    ///
    /// Returns `StepResult::CalibrateDone` on success.
    pub fn new_calibrate_for_tmc_with_gains_and_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        inner_gains: Option<InnerLoopGains>,
        current_filter_default_hz: u16,
        trace_engine_seq: u32,
    ) -> Self {
        let calibrate_entry_gains = Some(CalibrateEntryGains::from_tmc_state(
            tmc_state,
            inner_gains,
            current_filter_default_hz,
        ));
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
                profile: CommissionProfile::Balanced,
                sequence: InnerSequence::Calibrate,
                adc_cal: None,
                calibrate_entry_gains,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
            last_emitted_phase: None,
        }
    }

    #[cfg(test)]
    pub(crate) fn new_selftest(oid: u8) -> Self {
        Self::new_selftest_with_trace_seq(oid, 0)
    }

    #[cfg(test)]
    pub(crate) fn new_selftest_with_trace_seq(oid: u8, trace_engine_seq: u32) -> Self {
        Self::new_selftest_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            trace_engine_seq,
        )
    }

    #[cfg(test)]
    pub(crate) fn new_selftest_for_tmc(oid: u8, tmc_state: &'static TmcChannelState) -> Self {
        Self::new_selftest_for_tmc_with_trace_seq(oid, tmc_state, 0)
    }

    /// Create a selftest engine (diagnostics + electrical ID) bound to a TMC channel state with a
    /// trace sequence.
    ///
    /// Stops with `StepResult::SelftestDone` after restoring safe state.
    pub fn new_selftest_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        trace_engine_seq: u32,
    ) -> Self {
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
                profile: CommissionProfile::Balanced,
                sequence: InnerSequence::Selftest,
                adc_cal: None,
                calibrate_entry_gains: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
            last_emitted_phase: None,
        }
    }

    #[cfg(test)]
    pub(crate) fn new_commission(oid: u8, profile: CommissionProfile) -> Self {
        Self::new_commission_with_trace_seq(oid, profile, 0)
    }

    #[cfg(test)]
    pub(crate) fn new_commission_with_trace_seq(
        oid: u8,
        profile: CommissionProfile,
        trace_engine_seq: u32,
    ) -> Self {
        Self::new_commission_for_tmc_with_trace_seq(
            oid,
            crate::state::tmc::tmc_channel0(),
            profile,
            trace_engine_seq,
        )
    }

    /// Create a current-loop commissioning engine bound to a TMC channel state and trace sequence.
    ///
    /// Returns `StepResult::CommissionDone` on success.
    pub fn new_commission_for_tmc_with_trace_seq(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        profile: CommissionProfile,
        trace_engine_seq: u32,
    ) -> Self {
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Inner {
                phase: InnerPhase::AdcCalibrate(inner::AdcCalState::Setup),
                profile,
                sequence: InnerSequence::Commission,
                adc_cal: None,
                calibrate_entry_gains: None,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
            last_emitted_phase: None,
        }
    }

    fn outer_scaffold(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterAutotuneInputs,
        entry_snapshot: OuterGainRollbackSnapshot,
        trace_engine_seq: u32,
    ) -> OuterScaffold {
        let OuterAutotuneInputs {
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_bandwidth_hz,
            inner_warning_flags,
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            slot_order: _,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
            homing_speed_mrev_s,
            max_accel_mrev_s2,
        } = params;
        let budget = OuterMotionBudget {
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
            homing_speed_mrev_s,
            max_accel_mrev_s2,
        };
        let safety = OuterSafetyLimits::new(0, budget);
        OuterScaffold {
            oid,
            tmc_state,
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_bandwidth_hz,
            inner_warning_flags,
            entry_snapshot,
            trace_engine_seq,
            safety,
        }
    }

    fn assemble_outer(scaffold: OuterScaffold, phase: OuterPhase) -> Self {
        let OuterScaffold {
            oid,
            tmc_state,
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_bandwidth_hz,
            inner_warning_flags,
            entry_snapshot,
            trace_engine_seq,
            safety: _,
        } = scaffold;
        Self {
            oid,
            tmc_state,
            engine_mode: EngineMode::Outer {
                phase,
                profile,
                mode,
                inner_lambda_us,
                theta_e_us,
                current_bandwidth_hz,
                inner_warning_flags,
                entry_snapshot,
            },
            buf: TransientBuffer::new(),
            selftest_results: crate::selftest::SelftestResults::default(),
            diagnostics: crate::selftest::CommissionDiagnostics::default(),
            trace_run: TraceRunState::new(trace_engine_seq),
            trace_summary_seqs: TraceSummarySeqs::new(),
            last_emitted_phase: None,
        }
    }

    /// Create a breakaway-campaign engine from an admitted, pre-planned campaign.
    ///
    /// # Errors
    ///
    /// Returns [`CommissionError::EncoderFault`] when the campaign's frozen encoder
    /// domain is invalid, [`CommissionError::VelocityEnvelopeCeilingZero`] when
    /// the independent safety envelope's effective ceiling is zero, and
    /// [`CommissionError::VelocityEnvelopeExceedsCeiling`] when its planned
    /// velocity's derived envelope exceeds that ceiling.
    pub(crate) fn new_outer_for_tmc_with_breakaway_campaign(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterAutotuneInputs,
        entry_snapshot: OuterGainRollbackSnapshot,
        campaign: crate::commissioning::outer::velocity::breakaway_campaign::BreakawayCampaign,
    ) -> Result<Self, CommissionError> {
        let run_sequence = campaign.run_sequence();
        let mut scaffold =
            Self::outer_scaffold(oid, tmc_state, params, entry_snapshot, run_sequence);
        scaffold
            .safety
            .configure_velocity_sweep_encoder_resolution(campaign.encoder_counts_per_rev())
            .map_err(|_| CommissionError::EncoderFault)?;
        scaffold
            .safety
            .configure_velocity_envelope_with_settle_ceiling(
                campaign.planned_velocity_mrev_s(),
                campaign.effective_ceiling_mrev_s(),
            )?;
        let phase = OuterPhase::BreakawayCampaign {
            campaign,
            workspace:
                crate::commissioning::outer::velocity::breakaway_campaign::BreakawayProbeWorkspace::new(),
            safety: scaffold.safety,
        };
        Ok(Self::assemble_outer(scaffold, phase))
    }

    pub(crate) fn breakaway_integral_execution_plan(
        &self,
    ) -> Option<crate::commissioning::outer::velocity::integral_plan::VelocityIntegralExecutionPlan>
    {
        let EngineMode::Outer {
            phase: OuterPhase::VelocityIntegral { data, .. },
            ..
        } = &self.engine_mode
        else {
            return None;
        };
        Some(data.plan)
    }

    #[cfg(feature = "trace")]
    pub(crate) fn integral_promotion_source_diagnostic(&self) -> (u8, bool) {
        let EngineMode::Outer {
            phase: OuterPhase::VelocityIntegral { .. },
            ..
        } = &self.engine_mode
        else {
            return (0, false);
        };
        (PROMOTION_SOURCE_BREAKAWAY, true)
    }

    /// Create a direct Integral resume engine from a preflighted exact plan.
    pub fn new_outer_for_tmc_with_integral_preflight(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterAutotuneInputs,
        entry_snapshot: OuterGainRollbackSnapshot,
        preflight: crate::commission_dispatch::integral::IntegralPreflight,
    ) -> Self {
        let scaffold = Self::outer_scaffold(
            oid,
            tmc_state,
            params,
            entry_snapshot,
            preflight.run_sequence,
        );
        let phase = OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Setup,
            data: VelocityIntegralRuntime::new(
                preflight.run_sequence,
                preflight.plan.execution_plan(),
            ),
            safety: scaffold.safety,
        };
        Self::assemble_outer(scaffold, phase)
    }

    /// Create a reversal-standstill robustness engine from a preflighted point.
    ///
    /// The gate's step reversals overshoot by roughly the torque limit over the
    /// velocity P gain, which is independent of the leg amplitude, so its
    /// safety envelope widens to the plan's effective ceiling, exactly like the
    /// breakaway campaign's; the gate's own reconvergence, residual, and IAE
    /// thresholds judge the overshoot.
    ///
    /// # Errors
    ///
    /// Returns [`CommissionError::EncoderFault`] for an invalid encoder domain,
    /// [`CommissionError::InvalidSchedule`] when the probe velocity is zero,
    /// its rpm-to-mrev/s conversion overflows, or the encoder-observable
    /// ceiling cannot be derived, and [`CommissionError::TargetRateInvalid`]
    /// when the envelope's proportional-margin arithmetic overflows.
    pub(crate) fn new_outer_for_tmc_with_robustness(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterAutotuneInputs,
        entry_snapshot: OuterGainRollbackSnapshot,
        preflight: crate::commission_dispatch::robustness::RobustnessPreflight,
        encoder_counts_per_rev: u32,
    ) -> Result<Self, CommissionError> {
        use crate::commissioning::outer::velocity::robustness_reversal::{
            RobustnessReversalSession, RobustnessReversalState, RobustnessReversalWorkspace,
        };

        let mut scaffold = Self::outer_scaffold(
            oid,
            tmc_state,
            params,
            entry_snapshot,
            preflight.run_sequence,
        );
        scaffold
            .safety
            .configure_robustness_encoder_resolution(encoder_counts_per_rev)
            .map_err(|_| CommissionError::EncoderFault)?;
        let probe_velocity_mrev_s = preflight
            .point
            .target_velocity_rpm
            .unsigned_abs()
            .checked_mul(1_000)
            .map(|value| value / 60)
            .ok_or(CommissionError::InvalidSchedule)?;
        let effective_ceiling_mrev_s =
            crate::commissioning::outer::velocity::sweep_plan::effective_velocity_ceiling_mrev_s(
                scaffold.safety.budget.machine_velocity_ceiling_mrev_s,
                encoder_counts_per_rev,
            )
            .map_err(|_| CommissionError::InvalidSchedule)?;
        scaffold.safety.configure_velocity_envelope_up_to_ceiling(
            probe_velocity_mrev_s,
            effective_ceiling_mrev_s,
        )?;
        let phase = OuterPhase::RobustnessReversal {
            state: RobustnessReversalState::Setup,
            workspace: RobustnessReversalWorkspace::new(),
            session: RobustnessReversalSession::new(
                preflight.point,
                preflight.run_sequence,
                encoder_counts_per_rev,
            ),
            safety: scaffold.safety,
        };
        Ok(Self::assemble_outer(scaffold, phase))
    }

    pub(crate) fn new_outer_for_tmc_with_position_tune(
        oid: u8,
        tmc_state: &'static TmcChannelState,
        params: OuterAutotuneInputs,
        entry_snapshot: OuterGainRollbackSnapshot,
        run_sequence: u32,
    ) -> Self {
        let scaffold = Self::outer_scaffold(oid, tmc_state, params, entry_snapshot, run_sequence);
        let phase = OuterPhase::PositionTune {
            state: crate::commissioning::outer::position_tune::PositionTuneState::new(
                entry_snapshot,
                params.profile,
            ),
            safety: scaffold.safety,
        };
        Self::assemble_outer(scaffold, phase)
    }

    /// The OID this engine was started for.
    pub fn oid(&self) -> u8 {
        self.oid
    }

    pub(crate) fn take_after_handled_stop(slot: &mut Option<Self>) -> Option<Self> {
        if matches!(
            slot.as_ref().map(|engine| &engine.engine_mode),
            Some(EngineMode::Outer {
                phase: OuterPhase::BreakawayCampaign { .. },
                ..
            })
        ) {
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
            );
        }

        slot.take()
    }

    /// Identify which constructor produced this engine.
    pub fn kind(&self) -> EngineKind {
        match &self.engine_mode {
            EngineMode::Inner {
                sequence: InnerSequence::Commission,
                ..
            } => EngineKind::Commission,
            EngineMode::Inner {
                sequence: InnerSequence::Calibrate,
                ..
            } => EngineKind::Calibrate,
            EngineMode::Inner {
                sequence: InnerSequence::Selftest,
                ..
            } => EngineKind::Selftest,
            EngineMode::Outer { .. } => EngineKind::OuterTune,
        }
    }

    pub(crate) fn velocity_operational_torque_limit(&self) -> Option<u16> {
        let EngineMode::Outer { phase, .. } = &self.engine_mode else {
            return None;
        };
        match phase {
            OuterPhase::VelocityIntegral { data, .. }
                if data.plan.plan_digest != 0 && data.plan.hard_torque_limit != 0 =>
            {
                Some(data.plan.hard_torque_limit)
            }
            OuterPhase::BreakawayCampaign { campaign, .. } if campaign.hard_torque_limit() != 0 => {
                Some(campaign.hard_torque_limit())
            }
            _ => None,
        }
    }

    pub(crate) fn mark_velocity_limit_restored(&mut self) {
        let EngineMode::Outer { phase, .. } = &mut self.engine_mode else {
            return;
        };
        match phase {
            OuterPhase::VelocityIntegral { data, .. } => data.plan.hard_torque_limit = 0,
            OuterPhase::BreakawayCampaign { .. } => {}
            _ => {}
        }
    }

    #[cfg(test)]
    pub(crate) fn calibrate_entry_inner_gains_for_test(
        &self,
    ) -> Option<crate::pid::InnerLoopGains> {
        match &self.engine_mode {
            EngineMode::Inner {
                sequence: InnerSequence::Calibrate,
                calibrate_entry_gains,
                ..
            } => calibrate_entry_gains.map(|gains| gains.inner),
            _ => None,
        }
    }

    #[cfg(test)]
    pub(crate) fn set_inductance_capture_for_test(
        &mut self,
        capture: inner::InductanceCaptureState,
    ) {
        let EngineMode::Inner { phase, .. } = &mut self.engine_mode else {
            panic!("inductance capture fixture requires an inner engine");
        };
        *phase = InnerPhase::ElectricalIdentification {
            state: inner::ElectricalIdentificationState::InductanceSample { capture },
            data: inner::ElectricalIdentificationData::new(
                1_024,
                crate::commission_dispatch::test_support::current_limit(2816),
            ),
        };
    }

    /// Return the entry snapshot for the outer engine, if active.
    pub fn entry_snapshot(&self) -> Option<OuterGainRollbackSnapshot> {
        match &self.engine_mode {
            EngineMode::Outer { entry_snapshot, .. } => Some(*entry_snapshot),
            _ => None,
        }
    }

    /// Return retained installed-tuning safety-envelope evidence for a safety terminal state.
    pub fn outer_safety_fault(&self) -> Option<crate::commissioning::types::OuterSafetyFault> {
        match &self.engine_mode {
            EngineMode::Outer {
                phase: OuterPhase::FailedWithSafety { safety_fault, .. },
                ..
            } => Some(*safety_fault),
            _ => None,
        }
    }

    /// Wire code of the position-P sweep's specific failure outcome, or `0`
    /// when this run is not a failed position-P sweep.
    pub fn position_tune_outcome_code(&self) -> u8 {
        match &self.engine_mode {
            EngineMode::Outer {
                phase: OuterPhase::PositionTuneFailed { outcome, .. },
                ..
            } => outcome.map_or(0, crate::autotune::PositionTuneOutcome::to_wire_code),
            _ => 0,
        }
    }

    /// The position-P sweep's evidence at the moment of failure, from either
    /// `PositionTuneFailed` or `FailedWithSafety`; `Default` (all-zero)
    /// otherwise.
    pub fn position_tune_failure_provenance(
        &self,
    ) -> crate::commissioning::types::PositionTuneProvenance {
        match &self.engine_mode {
            EngineMode::Outer {
                phase: OuterPhase::PositionTuneFailed { provenance, .. },
                ..
            } => *provenance,
            EngineMode::Outer {
                phase: OuterPhase::FailedWithSafety { position_tune, .. },
                ..
            } => *position_tune,
            _ => Default::default(),
        }
    }

    #[cfg(test)]
    pub(crate) fn position_tune_step_for_test(
        &self,
    ) -> Option<crate::commissioning::outer::position_tune::PositionTuneStep> {
        let EngineMode::Outer {
            phase: OuterPhase::PositionTune { state, .. },
            ..
        } = &self.engine_mode
        else {
            return None;
        };
        Some(state.step)
    }

    /// Consume the engine's collected selftest results.
    ///
    /// Caller should only invoke this when the most recent `step()`
    /// returned `StepResult::SelftestDone`.
    pub fn take_selftest_results(&mut self) -> crate::selftest::SelftestResults {
        core::mem::take(&mut self.selftest_results)
    }

    #[cfg(test)]
    pub(crate) fn diagnostics_mut(&mut self) -> &mut crate::selftest::CommissionDiagnostics {
        &mut self.diagnostics
    }

    /// Consume diagnostics collected during this run.
    pub fn take_diagnostics(&mut self) -> crate::selftest::CommissionDiagnostics {
        core::mem::take(&mut self.diagnostics)
    }

    pub(crate) fn trace_summary_seqs_mut(&mut self) -> &mut TraceSummarySeqs {
        &mut self.trace_summary_seqs
    }

    #[cfg(feature = "trace")]
    pub(crate) fn emit_outer_safety_timing_trace<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        run_sequence: u32,
    ) {
        let summary = self.trace_summary_seqs.take_outer_safety_timing();
        backend.record_velocity_sweep_trace_record(
            crate::ankyra::velocity_sweep_encoding::encode_outer_safety_timing_record(
                self.trace_run.engine_seq,
                run_sequence,
                summary,
            ),
        );
        backend.record_commission_trace_event(self.trace_event(
            TraceEventLevel::Info,
            EVENT_COMMISSION_OUTER_SAFETY_TIMING,
            [
                u32_arg(self.trace_run.engine_seq),
                u32_arg(summary.max_gap_us),
            ],
            2,
        ));
    }

    #[cfg(feature = "trace")]
    pub(crate) fn trace_velocity_run_sequence(&self) -> Option<u32> {
        match &self.engine_mode {
            EngineMode::Outer {
                phase: OuterPhase::VelocityIntegral { data, .. },
                ..
            } => Some(data.integrity.run_sequence()),
            EngineMode::Outer {
                phase: OuterPhase::BreakawayCampaign { campaign, .. },
                ..
            } => Some(campaign.run_sequence()),
            EngineMode::Inner { .. }
            | EngineMode::Outer {
                phase:
                    OuterPhase::Failed(_)
                    | OuterPhase::PositionTune { .. }
                    | OuterPhase::PositionTuneFailed { .. }
                    | OuterPhase::FailedWithSafety { .. }
                    | OuterPhase::RobustnessReversal { .. },
                ..
            } => None,
        }
    }

    pub(crate) fn attach_integral_rest_boundary(
        &self,
        terminal: &mut crate::commissioning::types::VelocityIntegralTerminal,
    ) -> Option<crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord> {
        let EngineMode::Outer {
            phase: OuterPhase::VelocityIntegral { data, .. },
            ..
        } = &self.engine_mode
        else {
            return None;
        };
        let boundary = data.positive_i_rest_boundary?;
        if terminal.rest_boundary_rung_plus_one != 0 || terminal.rest_boundary_slot_plus_one != 0 {
            return None;
        }
        let record =
            crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_rest_boundary(
                terminal.run_sequence,
                terminal.evidence_sequence,
                terminal.plan_digest,
                boundary,
            );
        let mut integrity = data.integrity;
        integrity.record_terminal_boundary(&record);
        terminal.digest = integrity.digest();
        terminal.rest_boundary_rung_plus_one = boundary.rung_index;
        terminal.rest_boundary_slot_plus_one = boundary
            .slot
            .checked_add(1)
            .expect("Integral boundary slot fits its one-based terminal field");
        Some(record)
    }

    #[cfg(test)]
    pub(crate) fn set_integral_rest_boundary_for_test(
        &mut self,
        boundary: crate::commissioning::outer::PositiveIRestBoundary,
    ) {
        let EngineMode::Outer {
            phase: OuterPhase::VelocityIntegral { data, .. },
            ..
        } = &mut self.engine_mode
        else {
            panic!("test requires an Integral engine");
        };
        data.positive_i_rest_boundary = Some(boundary);
    }

    pub(crate) fn current_phase_id(&self) -> Option<CommissionPhase> {
        match &self.engine_mode {
            EngineMode::Inner { phase, .. } => phase.current_phase_id(),
            EngineMode::Outer { phase, .. } => phase.current_phase_id(),
        }
    }

    #[cfg(test)]
    pub(crate) fn breakaway_nomination_p_below_for_test(&self) -> Option<u16> {
        let EngineMode::Outer {
            phase: crate::commissioning::outer::OuterPhase::BreakawayCampaign { campaign, .. },
            ..
        } = &self.engine_mode
        else {
            return None;
        };
        Some(campaign.nomination_p_below())
    }

    #[cfg(test)]
    pub(crate) fn outer_budget_for_test(
        &self,
    ) -> Option<crate::commissioning::outer::OuterMotionBudget> {
        let EngineMode::Outer {
            phase: crate::commissioning::outer::OuterPhase::PositionTune { safety, .. },
            ..
        } = &self.engine_mode
        else {
            return None;
        };
        Some(safety.budget)
    }

    fn unreported_outer_phase(&self) -> Option<CommissionPhase> {
        let EngineMode::Outer { phase, .. } = &self.engine_mode else {
            return None;
        };
        phase
            .current_phase_id()
            .filter(|active| Some(*active) != self.last_emitted_phase)
    }

    fn trace_profile_or_mode(&self) -> i32 {
        match &self.engine_mode {
            EngineMode::Inner {
                profile,
                sequence: InnerSequence::Commission,
                ..
            } => i32::from(profile.to_u8()),
            EngineMode::Outer { profile, .. } => i32::from(profile.to_u8()),
            EngineMode::Inner { .. } => 0,
        }
    }

    fn trace_event(
        &self,
        level: TraceEventLevel,
        event_id: u16,
        payload: [i32; 2],
        payload_count: u8,
    ) -> CommissionTraceEvent {
        CommissionTraceEvent {
            level,
            event_id,
            engine_kind: self.kind(),
            payload,
            payload_count,
        }
    }

    fn emit_trace_start<B: CommissioningBackend>(&mut self, backend: &mut B) {
        if self.trace_run.started {
            return;
        }

        self.trace_run.started = true;
        backend.record_commission_trace_event(self.trace_event(
            TraceEventLevel::Info,
            EVENT_COMMISSION_ENGINE_START,
            [
                self.trace_profile_or_mode(),
                u32_arg(self.trace_run.engine_seq),
            ],
            2,
        ));

        if let Some(phase) = self.current_phase_id() {
            self.trace_run.current_phase = Some(phase);
            backend.record_commission_trace_event(self.trace_event(
                TraceEventLevel::Info,
                EVENT_COMMISSION_PHASE_START,
                [i32::from(phase.to_u8()), u32_arg(self.trace_run.phase_seq)],
                2,
            ));
        }
    }

    fn emit_trace_phase_end<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        status_code: u8,
        level: TraceEventLevel,
    ) {
        if let Some(phase) = self.trace_run.current_phase.take() {
            backend.record_commission_trace_event(self.trace_event(
                level,
                EVENT_COMMISSION_PHASE_END,
                [i32::from(phase.to_u8()), i32::from(status_code)],
                2,
            ));
        }
    }

    fn emit_trace_phase_start<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        phase: CommissionPhase,
    ) {
        self.trace_run.current_phase = Some(phase);
        backend.record_commission_trace_event(self.trace_event(
            TraceEventLevel::Info,
            EVENT_COMMISSION_PHASE_START,
            [i32::from(phase.to_u8()), u32_arg(self.trace_run.phase_seq)],
            2,
        ));
    }

    fn emit_trace_terminal<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        status_code: u8,
        level: TraceEventLevel,
    ) {
        if self.trace_run.terminal {
            return;
        }
        self.trace_run.terminal = true;
        backend.record_commission_trace_event(self.trace_event(
            level,
            EVENT_COMMISSION_ENGINE_TERMINAL,
            [i32::from(status_code), u32_arg(self.trace_run.engine_seq)],
            2,
        ));
    }

    fn status_level(status: CommissionStatus) -> TraceEventLevel {
        match status {
            CommissionStatus::Accepted => TraceEventLevel::Info,
            CommissionStatus::AcceptedWithWarnings | CommissionStatus::InnerOnly => {
                TraceEventLevel::Warn
            }
            CommissionStatus::Failed(_) => TraceEventLevel::Error,
        }
    }

    #[cfg(feature = "trace")]
    #[inline(never)]
    pub(crate) fn emit_integral_terminal_trace<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        terminal: &crate::commissioning::types::VelocityIntegralTerminal,
        reproduction: Option<crate::commission_dispatch::integral::IntegralReproductionReport>,
    ) {
        self.emit_outer_safety_timing_trace(backend, terminal.run_sequence);
        backend.record_velocity_sweep_trace_record(
            crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_terminal(
                terminal,
            ),
        );
        if reproduction.is_some() {
            backend.record_commission_trace_event(CommissionTraceEvent {
                level: TraceEventLevel::Info,
                event_id: EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED,
                engine_kind: EngineKind::OuterTune,
                payload: [i32::from(terminal.evidence_sequence), 0],
                payload_count: 1,
            });
        }
        let level = match terminal.outcome {
            crate::commissioning::types::VelocityIntegralOutcome::FirstRunRetained
            | crate::commissioning::types::VelocityIntegralOutcome::RepeatabilityConfirmed => {
                TraceEventLevel::Info
            }
            crate::commissioning::types::VelocityIntegralOutcome::Inconclusive
            | crate::commissioning::types::VelocityIntegralOutcome::RejectedPlanMismatch
            | crate::commissioning::types::VelocityIntegralOutcome::Failed => TraceEventLevel::Warn,
            crate::commissioning::types::VelocityIntegralOutcome::Fault => TraceEventLevel::Error,
        };
        self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
        backend.record_commission_trace_event(CommissionTraceEvent {
            level,
            event_id: EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL,
            engine_kind: EngineKind::OuterTune,
            payload: [
                i32::from(terminal.cause.raw_value()),
                terminal.digest as i32,
            ],
            payload_count: 2,
        });
        if terminal.outcome
            == crate::commissioning::types::VelocityIntegralOutcome::RepeatabilityConfirmed
        {
            backend.record_commission_trace_event(CommissionTraceEvent {
                level: TraceEventLevel::Info,
                event_id: EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE,
                engine_kind: EngineKind::OuterTune,
                payload: [terminal.digest as i32, (terminal.digest >> 32) as i32],
                payload_count: 2,
            });
        }
        self.trace_run.terminal = true;
    }

    fn emit_trace_after_step<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        result: &StepResult,
    ) {
        match result {
            StepResult::ReadyNow | StepResult::At(_) => {}
            StepResult::PhaseChanged(phase) => {
                if self.trace_run.current_phase != Some(*phase) {
                    self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                    self.trace_run.phase_seq = self.trace_run.phase_seq.wrapping_add(1);
                    self.emit_trace_phase_start(backend, *phase);
                }
            }
            StepResult::VelocityIntegralEvent(event) => {
                #[cfg(feature = "trace")]
                backend.record_velocity_sweep_trace_record(
                    crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_event(
                        *event,
                    ),
                );
                let (event_id, sequence) = match event {
                    crate::commissioning::types::VelocityIntegralEvent::Observation(report) => (
                        EVENT_COMMISSION_VELOCITY_INTEGRAL_OBSERVATION,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::Rung(report) => (
                        EVENT_COMMISSION_VELOCITY_INTEGRAL_RUNG,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::Recovery(report) => (
                        EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::RecoveryRest(report) => (
                        EVENT_COMMISSION_SELECTED_RECOVERY_REST,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::RestSelection(report) => (
                        EVENT_COMMISSION_REST_SELECTION_DIAGNOSTIC,
                        report.evidence_sequence,
                    ),
                    crate::commissioning::types::VelocityIntegralEvent::Current(report) => (
                        EVENT_COMMISSION_VELOCITY_INTEGRAL_DERIVED,
                        report.evidence_sequence,
                    ),
                };
                backend.record_commission_trace_event(CommissionTraceEvent {
                    level: TraceEventLevel::Info,
                    event_id,
                    engine_kind: EngineKind::OuterTune,
                    payload: [i32::from(sequence), 0],
                    payload_count: 1,
                });
            }
            StepResult::VelocityIntegralTerminal(_) => {}
            StepResult::RobustnessReversalTerminal(_) => {}
            #[cfg(not(feature = "trace"))]
            StepResult::BreakawayCampaignEvent(_) => {}
            #[cfg(not(feature = "trace"))]
            StepResult::BreakawayIntegralPlanReady { .. } => {}
            #[cfg(feature = "trace")]
            StepResult::BreakawayCampaignEvent(event) => {
                self.emit_breakaway_campaign_trace(backend, *event);
            }
            #[cfg(feature = "trace")]
            StepResult::BreakawayIntegralPlanReady { run_sequence, plan } => {
                for record in
                    crate::commissioning::outer::velocity::integral_reporting::encode_velocity_integral_plan_trace(
                        *run_sequence,
                        *plan,
                    )
                {
                    backend.record_velocity_sweep_trace_record(record);
                }
            }
            StepResult::CommissionDone(result) => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                let level = Self::status_level(result.inner.status);
                self.emit_trace_terminal(backend, result.inner.status.to_u8(), level);
            }
            StepResult::OuterDone(result) => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                let level = Self::status_level(result.status);
                self.emit_trace_terminal(backend, result.status.to_u8(), level);
            }
            StepResult::CalibrateDone(_) | StepResult::SelftestDone => {
                self.emit_trace_phase_end(backend, PHASE_END_OK, TraceEventLevel::Info);
                self.emit_trace_terminal(backend, PHASE_END_OK, TraceEventLevel::Info);
            }
            StepResult::InnerFailed(err) | StepResult::OuterFailed(err) => {
                let status = err.status_code();
                self.emit_trace_phase_end(backend, status, TraceEventLevel::Error);
                self.emit_trace_terminal(backend, status, TraceEventLevel::Error);
            }
        }
    }

    #[cfg(feature = "trace")]
    fn emit_breakaway_campaign_trace<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        event: crate::commissioning::types::BreakawayCampaignEvent,
    ) {
        use crate::commissioning::outer::velocity::breakaway_reporting::confirmation_observation_report;
        use crate::commissioning::outer::velocity::breakaway_trace_reporting::{
            encode_campaign_terminal, encode_ceiling_source, encode_confirmation_observation,
            encode_confirmation_planned, encode_confirmation_terminal, encode_discovery_planned,
            encode_discovery_rung_margin, encode_discovery_terminal, encode_probe_decision,
            encode_probe_planned, encode_probe_resolved, encode_probe_terminal,
            encode_raw_observation, encode_recovery_origin_mismatch, encode_recovery_rest_rejected,
            encode_rung_zero_diagnostic,
        };
        use crate::commissioning::types::BreakawayCampaignEvent;

        let sequence = match &event {
            BreakawayCampaignEvent::ProbePlanned(report) => report.evidence_sequence,
            BreakawayCampaignEvent::ProbeResolved(report) => report.evidence_sequence,
            BreakawayCampaignEvent::ProbeTerminal(report) => report.evidence_sequence,
            BreakawayCampaignEvent::DiscoveryPlanned { planned, .. } => planned.evidence_sequence,
            BreakawayCampaignEvent::RawObservation(report) => report.evidence_sequence,
            BreakawayCampaignEvent::ProbeDecision(report) => report.evidence_sequence,
            BreakawayCampaignEvent::DiscoveryRungMargin(report) => report.evidence_sequence,
            BreakawayCampaignEvent::DiscoveryTerminal { terminal, .. } => {
                terminal.evidence_sequence
            }
            BreakawayCampaignEvent::ConfirmationPlanned(report) => report.evidence_sequence,
            BreakawayCampaignEvent::ConfirmationTerminal { terminal, .. } => {
                terminal.evidence_sequence
            }
            BreakawayCampaignEvent::CampaignTerminal(report) => report.evidence_sequence,
            BreakawayCampaignEvent::RecoveryRestRejected(report) => report.evidence_sequence,
            BreakawayCampaignEvent::RecoveryOriginMismatch(report) => report.evidence_sequence,
        };
        let event_id = match &event {
            BreakawayCampaignEvent::ProbePlanned(_) => EVENT_COMMISSION_BREAKAWAY_PROBE_PLAN,
            BreakawayCampaignEvent::ProbeResolved(_) => EVENT_COMMISSION_BREAKAWAY_PROBE_RESULT,
            BreakawayCampaignEvent::ProbeTerminal(_) => EVENT_COMMISSION_BREAKAWAY_PROBE_TERMINAL,
            BreakawayCampaignEvent::DiscoveryPlanned { .. } => {
                EVENT_COMMISSION_BREAKAWAY_DISCOVERY_PLAN
            }
            BreakawayCampaignEvent::RawObservation(_) => EVENT_COMMISSION_BREAKAWAY_RAW_OBSERVATION,
            BreakawayCampaignEvent::ProbeDecision(_) => EVENT_COMMISSION_BREAKAWAY_PROBE_DECISION,
            BreakawayCampaignEvent::DiscoveryRungMargin(_) => {
                EVENT_COMMISSION_BREAKAWAY_DISCOVERY_MARGIN
            }
            BreakawayCampaignEvent::DiscoveryTerminal { .. } => {
                EVENT_COMMISSION_BREAKAWAY_DISCOVERY_TERMINAL
            }
            BreakawayCampaignEvent::ConfirmationPlanned(_) => {
                EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_PLAN
            }
            BreakawayCampaignEvent::ConfirmationTerminal { .. } => {
                EVENT_COMMISSION_BREAKAWAY_CONFIRMATION_TERMINAL
            }
            BreakawayCampaignEvent::CampaignTerminal(_) => {
                EVENT_COMMISSION_BREAKAWAY_CAMPAIGN_TERMINAL
            }
            BreakawayCampaignEvent::RecoveryRestRejected(_) => {
                EVENT_COMMISSION_BREAKAWAY_RECOVERY_REST_REJECTED
            }
            BreakawayCampaignEvent::RecoveryOriginMismatch(_) => {
                EVENT_COMMISSION_BREAKAWAY_RECOVERY_ORIGIN_MISMATCH
            }
        };

        #[cfg(feature = "trace")]
        match event {
            BreakawayCampaignEvent::ProbePlanned(report) => {
                backend.record_velocity_sweep_trace_record(encode_probe_planned(report));
            }
            BreakawayCampaignEvent::ProbeResolved(report) => {
                backend.record_velocity_sweep_trace_record(encode_probe_resolved(report));
            }
            BreakawayCampaignEvent::ProbeTerminal(report) => {
                backend.record_velocity_sweep_trace_record(encode_probe_terminal(report));
            }
            BreakawayCampaignEvent::DiscoveryPlanned {
                planned,
                ceiling_source,
            } => {
                backend.record_velocity_sweep_trace_record(encode_discovery_planned(planned));
                backend.record_velocity_sweep_trace_record(encode_ceiling_source(ceiling_source));
            }
            BreakawayCampaignEvent::RawObservation(report) => {
                backend.record_velocity_sweep_trace_record(encode_raw_observation(report));
            }
            BreakawayCampaignEvent::ProbeDecision(report) => {
                backend.record_velocity_sweep_trace_record(encode_probe_decision(report));
            }
            BreakawayCampaignEvent::DiscoveryRungMargin(report) => {
                backend.record_velocity_sweep_trace_record(encode_discovery_rung_margin(report));
            }
            BreakawayCampaignEvent::DiscoveryTerminal {
                terminal,
                rung_zero,
            } => {
                if let Some(diagnostic) = rung_zero {
                    backend.record_velocity_sweep_trace_record(encode_rung_zero_diagnostic(
                        diagnostic,
                    ));
                }
                backend.record_velocity_sweep_trace_record(encode_discovery_terminal(terminal));
            }
            BreakawayCampaignEvent::ConfirmationPlanned(report) => {
                backend.record_velocity_sweep_trace_record(encode_confirmation_planned(report));
            }
            BreakawayCampaignEvent::ConfirmationTerminal {
                terminal,
                forward,
                reverse,
            } => {
                let run_sequence = terminal.run_sequence;
                let evidence_sequence = terminal.evidence_sequence;
                let plan_digest = terminal.plan_digest;
                for (slot_index, atom) in forward
                    .observations
                    .into_iter()
                    .enumerate()
                    .flat_map(|(slot_index, atom)| atom.map(|atom| (slot_index, atom)))
                {
                    if let Some(report) = confirmation_observation_report(
                        plan_digest,
                        slot_index as u8,
                        crate::autotune::velocity_measurement::VelocityDirection::Forward,
                        atom,
                        run_sequence,
                        evidence_sequence,
                    ) {
                        backend.record_velocity_sweep_trace_record(
                            encode_confirmation_observation(report),
                        );
                    }
                }
                for (slot_index, atom) in reverse
                    .observations
                    .into_iter()
                    .enumerate()
                    .flat_map(|(slot_index, atom)| atom.map(|atom| (slot_index, atom)))
                {
                    if let Some(report) = confirmation_observation_report(
                        plan_digest,
                        slot_index as u8 + 4,
                        crate::autotune::velocity_measurement::VelocityDirection::Reverse,
                        atom,
                        run_sequence,
                        evidence_sequence,
                    ) {
                        backend.record_velocity_sweep_trace_record(
                            encode_confirmation_observation(report),
                        );
                    }
                }
                backend.record_velocity_sweep_trace_record(encode_confirmation_terminal(terminal));
            }
            BreakawayCampaignEvent::CampaignTerminal(report) => {
                backend.record_velocity_sweep_trace_record(encode_campaign_terminal(report));
            }
            BreakawayCampaignEvent::RecoveryRestRejected(report) => {
                backend.record_velocity_sweep_trace_record(encode_recovery_rest_rejected(report));
            }
            BreakawayCampaignEvent::RecoveryOriginMismatch(report) => {
                backend.record_velocity_sweep_trace_record(encode_recovery_origin_mismatch(report));
            }
        }
        #[cfg(not(feature = "trace"))]
        let _ = event;

        backend.record_commission_trace_event(CommissionTraceEvent {
            level: TraceEventLevel::Info,
            event_id,
            engine_kind: EngineKind::OuterTune,
            payload: [i32::from(sequence), 0],
            payload_count: 1,
        });
    }

    /// Advance the state machine by one tick (max 1ms).
    ///
    /// Returns a [`StepResult`] indicating whether to continue, report
    /// a phase transition, or finalize.
    pub async fn step<B: CommissioningBackend>(&mut self, backend: &mut B) -> StepResult {
        if backend.is_shutdown_requested() {
            if let EngineMode::Inner { phase, .. } = &self.engine_mode {
                inner::electrical_identification::push_active_electrical_diagnostics(
                    phase,
                    &mut self.diagnostics,
                );
            }
            crate::commissioning::outer::position_tune::abort_stimulus(backend);
            self.tmc_state
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
            crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
            return self.abort(CommissionError::Shutdown);
        }

        self.emit_trace_start(backend);

        let result = if let Some(phase) = self.unreported_outer_phase() {
            self.last_emitted_phase = Some(phase);
            StepResult::PhaseChanged(phase)
        } else {
            let selftest_results = &mut self.selftest_results;
            let diagnostics = &mut self.diagnostics;
            let trace_summary_seqs = &mut self.trace_summary_seqs;
            let result = match &mut self.engine_mode {
                EngineMode::Inner {
                    phase,
                    profile,
                    sequence,
                    adc_cal,
                    calibrate_entry_gains,
                } => {
                    inner::step_inner(
                        phase,
                        *profile,
                        *sequence,
                        inner::InnerStepContext {
                            buf: &mut self.buf,
                            adc_cal,
                            calibrate_entry_gains: *calibrate_entry_gains,
                            selftest_results,
                            diagnostics,
                        },
                        backend,
                    )
                    .await
                }
                EngineMode::Outer { phase, .. } => {
                    outer::step_outer_with_trace(phase, &mut self.buf, trace_summary_seqs, backend)
                        .await
                }
            };
            if let StepResult::PhaseChanged(phase) = &result {
                self.last_emitted_phase = Some(*phase);
            }
            result
        };
        self.emit_trace_after_step(backend, &result);
        result
    }

    fn deadline_capture_active(&self) -> bool {
        matches!(
            &self.engine_mode,
            EngineMode::Inner {
                phase: InnerPhase::ElectricalIdentification {
                    state: inner::ElectricalIdentificationState::InductanceSample { .. },
                    ..
                },
                ..
            }
        )
    }

    pub(crate) async fn step_scheduled<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
    ) -> StepResult {
        loop {
            let capture_result = match &mut self.engine_mode {
                EngineMode::Inner { phase, .. } => {
                    inner::electrical_identification::drive_deadline_capture(
                        phase,
                        &mut self.diagnostics,
                        backend,
                    )
                    .await
                }
                EngineMode::Outer { .. } => None,
            };
            if let Some(result) = capture_result {
                self.emit_trace_after_step(backend, &result);
                return result;
            }

            let result = self.step(backend).await;
            let StepResult::At(_) = result else {
                return result;
            };
            if !self.deadline_capture_active() {
                return result;
            }
        }
    }

    fn abort(&mut self, error: CommissionError) -> StepResult {
        match &self.engine_mode {
            EngineMode::Inner { .. } => StepResult::InnerFailed(error),
            EngineMode::Outer { .. } => StepResult::OuterFailed(error),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{SpiOp, current_limit};
    use crate::commissioning::types::EngineKind;
    use foci_firmware_macros::fixture_test;

    #[test]
    fn size_regression_gates() {
        use core::mem::{size_of, size_of_val};

        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        let mut engine =
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced);
        let engine_step = size_of_val(&engine.step(&mut backend));

        let mut engine =
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced);
        let step_inner = match &mut engine.engine_mode {
            EngineMode::Inner {
                phase,
                profile,
                sequence,
                adc_cal,
                calibrate_entry_gains,
            } => size_of_val(&inner::step_inner(
                phase,
                *profile,
                *sequence,
                inner::InnerStepContext {
                    buf: &mut engine.buf,
                    adc_cal,
                    calibrate_entry_gains: *calibrate_entry_gains,
                    selftest_results: &mut engine.selftest_results,
                    diagnostics: &mut engine.diagnostics,
                },
                &mut backend,
            )),
            EngineMode::Outer { .. } => unreachable!(),
        };

        let mut phase = inner::InnerPhase::ElectricalIdentification {
            state: inner::ElectricalIdentificationState::ResistanceStart,
            data: inner::ElectricalIdentificationData::new(
                1024,
                crate::commission_dispatch::test_support::current_limit(2816),
            ),
        };
        let mut selftest_results = crate::selftest::SelftestResults::default();
        let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
        let electrical_id = size_of_val(
            &inner::electrical_identification::step_electrical_identification(
                &mut phase,
                crate::autotune::InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            ),
        );

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        let scheduled_step = size_of_val(&runtime.step_commissioning_scheduled(&mut backend));

        assert_eq!(size_of::<TraceRunState>(), 12);
        assert_eq!(size_of::<TraceSummarySeqs>(), 16);
        assert_eq!(
            size_of::<crate::commissioning::outer::VelocityIntegralRuntime>(),
            2_912
        );
        assert!(
            size_of::<crate::commissioning::outer::velocity::sweep_plan::VelocitySweepExecutionPlan>(
            ) == 32
        );
        assert_eq!(
            size_of::<crate::commissioning::types::VelocityIntegralEvent>(),
            288
        );
        assert_eq!(
            size_of::<crate::commissioning::types::RestSelectionDiagnostic>(),
            80
        );
        assert_eq!(
            size_of::<crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord>(),
            129
        );
        assert_eq!(size_of::<crate::commissioning::types::StepResult>(), 928);
        assert_eq!(
            size_of::<inner::InnerPhase>(),
            3_392,
            "the gain-floor capture buffer must stay inside the existing variant slack"
        );
        assert_eq!(size_of::<crate::commissioning::types::OuterResult>(), 92);
        assert_eq!(size_of::<CommissioningEngine>(), 5_936);
        assert!(size_of::<CommissioningEngine>() <= 5_936);
        assert!(size_of::<CommissioningEngine>() <= 6_144);
        assert!(size_of::<capture_timing::CaptureTiming>() <= 64);
        assert!(size_of::<capture_timing::CaptureTimingEvidence>() <= 32);
        assert_eq!(size_of::<[u16; crate::autotune::TRANSIENT_LEN]>(), 512);
        assert_eq!(size_of::<crate::autotune::DelayTimestampOffsets>(), 514);
        assert_eq!(engine_step, 4_400);
        assert!(engine_step <= 4_400);
        assert_eq!(step_inner, 4_352);
        assert!(step_inner <= 4_352);
        assert_eq!(electrical_id, 640);
        assert!(electrical_id <= 640);
        assert!(electrical_id <= 720);
        assert_eq!(scheduled_step, 4_480);
        assert!(scheduled_step <= 4_480);
    }

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    fn sample_outer_params() -> OuterAutotuneInputs {
        OuterAutotuneInputs {
            slot_order:
                crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder::ForwardFirst,
            profile: crate::autotune::CommissionProfile::Balanced,
            mode: crate::autotune::OperatingMode::Nominal,
            inner_lambda_us: 100,
            theta_e_us: 50,
            current_bandwidth_hz: 1000,
            inner_warning_flags: 0,
            requested_velocity_mrev_s: 6000,
            machine_velocity_ceiling_mrev_s: 10_000,
            requested_velocity_source:
                crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
            max_stroke_travel_mrev: 1000,
            settle_travel_reserve_mrev: 100,
            negative_position_headroom_mrev: 2000,
            positive_position_headroom_mrev: 2000,
            max_duration_ms: 3000,
            homing_speed_mrev_s: 4_375,
            max_accel_mrev_s2: 200_000,
        }
    }

    fn sample_entry_snapshot() -> OuterGainRollbackSnapshot {
        OuterGainRollbackSnapshot {
            velocity_p: crate::pid::p_gain(1152),
            velocity_i: crate::pid::i_gain(0),
            position_p: crate::pid::p_gain(640),
            position_i: crate::pid::i_gain(0),
            velocity_limit: 500_000,
            voltage_limit: crate::state::tmc::DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            velocity_filter_hz: 0,
            torque_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
        }
    }

    fn sample_inner_result(status: crate::autotune::CommissionStatus) -> InnerResult {
        InnerResult {
            status,
            r_count_milli: 1706,
            l_count_micro: 1245,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            candidate_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            candidate_axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            candidate_gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_measured_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 100,
            candidate_flux_i: 10,
            candidate_torque_p: 200,
            candidate_torque_i: 20,
            candidate_attempt: 0,
            lambda_us: 100,
            theta_e_us: 50,
            theta_source: 0,
            bandwidth_hz: 1000,
            inductance: crate::commissioning::types::InductanceEvidence::EMPTY,
            inner_warning_flags: 0,
            timing: [crate::commissioning::types::CommissionTimingSummary::NOT_RUN; 3],
            timing_summary: 0,
            resistance_selected_count_slope_milli: 1706,
            resistance_gain_path_count_slope_milli: 1706,
            resistance_axis0_count_slope_milli: 1700,
            resistance_axis1_count_slope_milli: 1712,
            resistance_axis0_intercept_count: 0,
            resistance_axis1_intercept_count: 0,
            resistance_axis0_rmse_permille: 0,
            resistance_axis1_rmse_permille: 0,
            resistance_selected_mask_axis0: 0b1111_1000,
            resistance_selected_mask_axis1: 0b1111_1000,
            resistance_axis0_signed_count_slope_milli: 1702,
            resistance_axis1_signed_count_slope_milli: 1710,
            resistance_axis0_signed_asymmetry_permille: 0,
            resistance_axis1_signed_asymmetry_permille: 0,
            resistance_axis0_drift_permille: 0,
            resistance_axis1_drift_permille: 0,
            resistance_status_flags_or: 0,
            resistance_warning_flags: 0,
            resistance_peak_abs_current_count: 1_440,
            resistance_max_abs_steady_mean_current_count: 1_280,
            resistance_current_ceiling_count: 1_600,
            resistance_power_stage_tripped: false,
            encoder_alignment_count: 0,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            current_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            current_validation_axes: crate::autotune::CURRENT_VALIDATION_AXIS_TORQUE,
            retry_budget_exhausted: 0,
            current_loop_failure_reason: crate::autotune::CURRENT_LOOP_FAILURE_NONE,
            closed_loop_activation_status: crate::autotune::CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN,
            closed_loop_activation_position_1: 0,
            closed_loop_activation_position_2: 0,
            closed_loop_activation_drift_count: 0,
            closed_loop_activation_threshold_count: 0,
            closed_loop_activation_runaway: 0,
            current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            current_hold_warning_flags: 0,
            current_hold_elapsed_us: 0,
            current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                as u16,
            current_hold_sample_count: 0,
            current_hold_position_span_count: 0,
            current_hold_position_drift_count: 0,
            current_hold_torque_mean_count: 0,
            current_hold_torque_rms_count: 0,
            current_hold_torque_peak_to_peak_count: 0,
            current_hold_torque_crossing_count: 0,
            current_hold_flux_mean_count: 0,
            current_hold_flux_rms_count: 0,
            current_hold_flux_peak_to_peak_count: 0,
            current_hold_flux_crossing_count: 0,
            current_hold_status_flags_or: 0,
            current_hold_actionable_status_count: 0,
            current_validation_step_amplitude: 128,
            current_validation_flux_step_amplitude: 128,
            current_validation_torque_step_amplitude: 128,
            current_validation_pidout_limit: 2400,
            current_validation_current_limited: 422,
            current_validation_voltage_limited: 128,
            current_validation_max_p: 3474,
            current_settled_validation_count: 0,
            current_settled_validation:
                [crate::commissioning::types::CurrentValidationSettledSampleEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
            current_flux_validation_count: 0,
            current_flux_validation:
                [crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation:
                [crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
        }
    }

    fn sample_commission_result(status: crate::autotune::CommissionStatus) -> CommissionResult {
        CommissionResult {
            inner: sample_inner_result(status),
            fallback_velocity_p: 1152,
            fallback_velocity_i: 0,
            fallback_position_p: 640,
            fallback_position_i: 0,
            fallback_velocity_limit: 500_000,
            velocity_filter_hz: 0,
            torque_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
        }
    }

    #[test]
    fn current_loop_terminal_path_uses_only_explicit_scheduling() {
        pollster::block_on(async {
            use foci_tmc4671::registers::{
                ABN_DECODER_PPR, ADC_I_SELECT, MOTOR_TYPE_N_POLE_PAIRS, PID_TORQUE_FLUX_LIMITS,
                PWM_SV_CHOP,
            };

            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            let EngineMode::Inner { phase, .. } = &mut engine.engine_mode else {
                unreachable!();
            };
            *phase = InnerPhase::ClosedLoopActivation {
                state: crate::autotune::ClosedLoopActivationState::WriteGains,
                flux_p: crate::pid::p_gain(100),
                flux_i: crate::pid::i_gain(10),
                torque_p: crate::pid::p_gain(200),
                torque_i: crate::pid::i_gain(20),
                velocity_p: crate::pid::p_gain(1_152),
                velocity_i: crate::pid::i_gain(0),
                position_p: crate::pid::p_gain(640),
                position_i: crate::pid::i_gain(0),
                velocity_limit: 500_000,
                velocity_filter_hz: 0,
                torque_filter_hz: 0,
                position_filter_hz: 0,
                flux_filter_hz: 0,
                inner_result: Some(sample_inner_result(
                    crate::autotune::CommissionStatus::Accepted,
                )),
                attempt: 0,
            };

            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.current_limit_value = current_limit(2_816);
            backend.configured_physical_width = Some(16);
            let required_registers = [
                (PWM_SV_CHOP.addr(), 7),
                (MOTOR_TYPE_N_POLE_PAIRS.addr(), (2 << 16) | 50),
                (
                    ABN_DECODER_PPR.addr(),
                    backend.mechanical_encoder_quadrature_counts_per_rev_value,
                ),
                (ADC_I_SELECT.addr(), backend.tmc_config.adc_i_select),
                (
                    PID_TORQUE_FLUX_LIMITS.addr(),
                    backend.current_limit_value.register_value(),
                ),
            ];
            for (address, value) in required_registers {
                backend.registers.insert(address, value).ok();
            }

            for _ in 0..512 {
                match engine.step_scheduled(&mut backend).await {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    StepResult::CommissionDone(result) => {
                        assert_eq!(
                            result.inner.status,
                            crate::autotune::CommissionStatus::Accepted
                        );
                        return;
                    }
                    other => panic!("unexpected current-loop terminal result: {other:?}"),
                }
            }
            panic!("current-loop terminal path exceeded its explicit schedule bound");
        });
    }

    fn sample_outer_result(status: crate::autotune::CommissionStatus) -> OuterResult {
        OuterResult {
            status,
            velocity_p: 1152,
            velocity_i: 0,
            velocity_limit: 500_000,
            position_p: 640,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
            warning_code: 0,
            provenance: crate::commissioning::types::TuneProvenance::default(),
            position_tune: crate::commissioning::types::PositionTuneProvenance::default(),
            position_tune_outcome_code: 0,
            motor_disarmed: false,
        }
    }

    fn sample_adc_cal_result() -> AdcCalResult {
        AdcCalResult {
            adc_i0: 11,
            adc_i1: 13,
            encoder_count: 17,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        }
    }

    fn initial_outer_phase_id() -> CommissionPhase {
        CommissionPhase::BreakawayProportionalAcquisition
    }

    fn assert_trace_event(
        event: &CommissionTraceEvent,
        event_id: u16,
        engine_kind: EngineKind,
        level: TraceEventLevel,
        payload: [i32; 2],
    ) {
        assert_eq!(event.event_id, event_id);
        assert_eq!(event.engine_kind, engine_kind);
        assert_eq!(event.level, level);
        assert_eq!(event.payload, payload);
        assert_eq!(event.payload_count, 2);
    }

    fn assert_terminal_trace(
        mut engine: CommissioningEngine,
        result: StepResult,
        engine_kind: EngineKind,
        phase_id: CommissionPhase,
        terminal_status_code: u8,
        phase_level: TraceEventLevel,
        terminal_level: TraceEventLevel,
    ) {
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &result);

        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            engine_kind,
            phase_level,
            [i32::from(phase_id.to_u8()), i32::from(PHASE_END_OK)],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_ENGINE_TERMINAL,
            engine_kind,
            terminal_level,
            [i32::from(terminal_status_code), 0],
        );
    }

    #[fixture_test]
    async fn calibrate_constructor_uses_stored_gains_for_closed_loop_activation() {
        use core::sync::atomic::Ordering::Relaxed;

        let tmc_state = tmc_state();
        let outer = crate::pid::test_outer_gains(627, 1, 1434, 2);
        tmc_state.store_outer_gains(outer, Relaxed);
        tmc_state.velocity_limit_cfg.store(500_000, Relaxed);
        tmc_state.velocity_filter_hz.store(Some(11), Relaxed);
        tmc_state.torque_filter_hz.store(Some(13), Relaxed);
        tmc_state.position_filter_hz.store(Some(17), Relaxed);
        tmc_state.flux_filter_hz.store(Some(19), Relaxed);

        let inner = crate::pid::InnerLoopGains {
            flux_p: crate::pid::p_gain(711),
            flux_i: crate::pid::i_gain(2_544),
            torque_p: crate::pid::p_gain(711),
            torque_i: crate::pid::i_gain(2_544),
        };

        let engine =
            CommissioningEngine::new_calibrate_for_tmc_with_gains(0, tmc_state, Some(inner));

        let EngineMode::Inner {
            calibrate_entry_gains: Some(gains),
            ..
        } = engine.engine_mode
        else {
            panic!("calibrate constructor should snapshot entry gains");
        };

        assert_eq!(gains.inner, inner);
        assert_eq!(gains.outer, Some(outer));
        assert_eq!(gains.velocity_limit, 500_000);
        assert_eq!(gains.velocity_filter_hz, 11);
        assert_eq!(gains.torque_filter_hz, 13);
        assert_eq!(gains.position_filter_hz, 17);
        assert_eq!(gains.flux_filter_hz, 19);
    }

    #[test]
    fn engine_kind_matches_constructor() {
        assert_eq!(
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced)
                .kind(),
            EngineKind::Commission,
        );
        assert_eq!(
            CommissioningEngine::new_calibrate(0).kind(),
            EngineKind::Calibrate
        );
        assert_eq!(
            CommissioningEngine::new_selftest(0).kind(),
            EngineKind::Selftest
        );
        assert_eq!(breakaway_safety_engine(42).kind(), EngineKind::OuterTune,);
    }

    #[test]
    fn new_outer_seeds_host_motion_budget() {
        use crate::commissioning::outer::velocity::breakaway_campaign::plan_breakaway_campaign_command;
        use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;

        let params = OuterAutotuneInputs {
            requested_velocity_mrev_s: 4_500,
            max_stroke_travel_mrev: 750,
            settle_travel_reserve_mrev: 75,
            negative_position_headroom_mrev: 900,
            positive_position_headroom_mrev: 1_100,
            max_duration_ms: 2_500,
            ..sample_outer_params()
        };
        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: params.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: params.requested_velocity_source,
            max_stroke_travel_mrev: params.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: params.negative_position_headroom_mrev,
            positive_position_headroom_mrev: params.positive_position_headroom_mrev,
            max_duration_ms: params.max_duration_ms,
            nomination_p_below: 0,
        };
        let (campaign, _workflow) =
            plan_breakaway_campaign_command(request, 4_000, 2_816, 0, 25_000, 0, 43)
                .expect("valid breakaway campaign");
        let engine = CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
            0,
            tmc_state(),
            params,
            sample_entry_snapshot(),
            campaign,
        )
        .expect("valid safety configuration");

        let EngineMode::Outer { phase, .. } = engine.engine_mode else {
            panic!("breakaway constructor should create an outer engine");
        };
        let safety = match phase {
            OuterPhase::BreakawayCampaign { safety, .. } => safety,
            _ => panic!("breakaway constructor should enter the campaign phase"),
        };

        assert_eq!(safety.budget.max_stroke_travel_mrev, 750);
        assert_eq!(safety.budget.requested_velocity_mrev_s, 4_500);
        assert_eq!(safety.budget.settle_travel_reserve_mrev, 75);
        assert_eq!(safety.budget.negative_position_headroom_mrev, 900);
        assert_eq!(safety.budget.positive_position_headroom_mrev, 1_100);
        assert_eq!(safety.budget.max_duration_ms, 2_500);
    }

    fn breakaway_safety_engine(run_sequence: u32) -> CommissioningEngine {
        use crate::commissioning::outer::velocity::breakaway_campaign::plan_breakaway_campaign_command;
        use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;

        let params = sample_outer_params();
        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: params.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: params.requested_velocity_source,
            max_stroke_travel_mrev: params.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: params.negative_position_headroom_mrev,
            positive_position_headroom_mrev: params.positive_position_headroom_mrev,
            max_duration_ms: params.max_duration_ms,
            nomination_p_below: 0,
        };
        let (campaign, _workflow) =
            plan_breakaway_campaign_command(request, 4_000, 2_816, 0, 25_000, 0, run_sequence)
                .expect("valid breakaway campaign");
        CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
            0,
            tmc_state(),
            params,
            sample_entry_snapshot(),
            campaign,
        )
        .expect("valid safety configuration")
    }

    fn backend_has_register_write(
        backend: &crate::commission_dispatch::test_support::MockBackend,
    ) -> bool {
        backend
            .spi_log
            .iter()
            .any(|operation| matches!(operation, SpiOp::Write(_, _)))
    }

    #[test]
    fn breakaway_planning_rejects_zero_encoder_resolution_before_motion() {
        use crate::autotune::TmcMechanicalRpm;
        use crate::commissioning::outer::velocity::breakaway_campaign::{
            BreakawayCampaign, BreakawayCampaignRequest,
        };
        use crate::commissioning::outer::velocity::velocity_clamp::VelocityLimitEvidenceCapability;

        let result = BreakawayCampaign::new(BreakawayCampaignRequest {
            target_velocity_rpm: TmcMechanicalRpm::from_register(100),
            planned_velocity_mrev_s: 4_394,
            effective_ceiling_mrev_s: 10_000,
            hard_torque_limit: 2_816,
            limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
            max_duration_ms: 3_000,
            nominal_workflow_ms: 400_000,
            maximum_workflow_ms: 450_000,
            channel: 0,
            run_sequence: 42,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 0,
            nomination_p_below: 0,
        });

        assert!(matches!(result, Err(CommissionError::CampaignPlanInvalid)));
    }

    #[fixture_test]
    fn robustness_safety_envelope_widens_to_the_effective_ceiling() {
        use crate::commission_dispatch::robustness::RobustnessPreflight;
        use crate::commissioning::outer::velocity::integral_reproduction::{
            AcceptancePoint, IntegralPlanIdentity,
        };

        let point = AcceptancePoint {
            identity: IntegralPlanIdentity {
                channel: 0,
                build_features: 0x1234,
                schema_revision: 1,
                plan_digest: 0xfeed_beef,
            },
            selected_p: 1_416,
            selected_i: 362,
            target_velocity_rpm: 322,
            selected_response_intervals_q: [[10, 20], [-20, -10]],
            confidence_q: 0,
            band_position_q: 0,
            qualifier_bits: 0,
            token: None,
        };
        let engine = CommissioningEngine::new_outer_for_tmc_with_robustness(
            0,
            tmc_state(),
            sample_outer_params(),
            sample_entry_snapshot(),
            RobustnessPreflight {
                point,
                run_sequence: 7,
            },
            4_000,
        )
        .expect("valid robustness safety configuration");

        let EngineMode::Outer {
            phase: OuterPhase::RobustnessReversal { safety, .. },
            ..
        } = &engine.engine_mode
        else {
            panic!("expected a robustness reversal engine");
        };
        // 322 rpm plans 5_366 mrev/s, whose proportional envelope is 8_049 mrev/s.
        // The gate is bounded by the effective ceiling instead: the 10_000 mrev/s
        // machine ceiling of `sample_outer_params`, below the 12_487 mrev/s a
        // 4_000-count encoder can observe, i.e. 40_000_000 milli-counts/s.
        assert_eq!(safety.active_velocity_cap_milli_counts_per_s(), 40_000_000);
    }

    #[fixture_test]
    fn breakaway_safety_constructor_rejects_velocity_envelope_overflow() {
        use crate::autotune::TmcMechanicalRpm;
        use crate::commissioning::outer::velocity::breakaway_campaign::{
            BreakawayCampaign, BreakawayCampaignRequest,
        };
        use crate::commissioning::outer::velocity::velocity_clamp::VelocityLimitEvidenceCapability;

        let campaign = BreakawayCampaign::new(BreakawayCampaignRequest {
            target_velocity_rpm: TmcMechanicalRpm::from_register(100),
            planned_velocity_mrev_s: u32::MAX / 3 + 1,
            effective_ceiling_mrev_s: u32::MAX,
            hard_torque_limit: 2_816,
            limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
            max_duration_ms: 3_000,
            nominal_workflow_ms: 400_000,
            maximum_workflow_ms: 450_000,
            channel: 0,
            run_sequence: 42,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            nomination_p_below: 0,
        })
        .expect("campaign planning does not consume the runtime velocity envelope");

        let result = CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
            0,
            tmc_state(),
            sample_outer_params(),
            sample_entry_snapshot(),
            campaign,
        );

        assert!(matches!(result, Err(CommissionError::TargetRateInvalid)));
    }

    #[test]
    fn breakaway_safety_is_configured_and_armed_before_the_first_gain_write() {
        use crate::commissioning::outer::velocity::breakaway_campaign::{
            BreakawayCampaignPhase, BreakawayProbeState,
        };
        use crate::commissioning::types::{BreakawayCampaignEvent, StepResult};

        pollster::block_on(async {
            let mut engine = breakaway_safety_engine(42);
            let EngineMode::Outer {
                phase: OuterPhase::BreakawayCampaign { campaign, .. },
                ..
            } = &engine.engine_mode
            else {
                panic!("breakaway constructor must install the campaign")
            };
            assert!(matches!(
                campaign.phase(),
                BreakawayCampaignPhase::Probe {
                    state: BreakawayProbeState::PlannedProbe,
                    ..
                }
            ));

            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.current_limit_value = current_limit(2_816);
            backend.mechanical_encoder_position_counts_value = 1_234;

            let entry = engine.step(&mut backend).await;
            assert!(matches!(
                entry,
                StepResult::PhaseChanged(CommissionPhase::BreakawayProportionalAcquisition)
            ));

            let first = engine.step(&mut backend).await;
            assert!(matches!(
                first,
                StepResult::BreakawayCampaignEvent(BreakawayCampaignEvent::ProbePlanned(_))
            ));
            assert!(!backend_has_register_write(&backend));

            let second = engine.step(&mut backend).await;
            assert!(matches!(second, StepResult::ReadyNow));
            assert!(!backend_has_register_write(&backend));

            let EngineMode::Outer {
                phase:
                    OuterPhase::BreakawayCampaign {
                        campaign, safety, ..
                    },
                ..
            } = &engine.engine_mode
            else {
                panic!("safety setup must retain the breakaway campaign")
            };
            assert!(matches!(
                campaign.phase(),
                BreakawayCampaignPhase::Probe {
                    state: BreakawayProbeState::WriteGain,
                    ..
                }
            ));
            assert_eq!(safety.torque_cap, 2_816);
            assert_eq!(safety.counts_per_rev(), 4_000);
            assert!(safety.next_safety_deadline().is_some());
        });
    }

    async fn assert_breakaway_safety_context_change_fails_before_write(
        change_backend: impl FnOnce(&mut crate::commission_dispatch::test_support::MockBackend),
    ) {
        use crate::commissioning::types::{BreakawayCampaignEvent, StepResult};

        let mut engine = breakaway_safety_engine(42);
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        backend.current_limit_value = current_limit(2_816);

        let entry = engine.step(&mut backend).await;
        assert!(matches!(
            entry,
            StepResult::PhaseChanged(CommissionPhase::BreakawayProportionalAcquisition)
        ));
        let planned = engine.step(&mut backend).await;
        assert!(matches!(
            planned,
            StepResult::BreakawayCampaignEvent(BreakawayCampaignEvent::ProbePlanned(_))
        ));
        change_backend(&mut backend);

        for _ in 0..4 {
            let step = engine.step(&mut backend).await;
            assert!(!backend_has_register_write(&backend));
            if matches!(
                step,
                StepResult::OuterFailed(CommissionError::PlanContractChanged)
            ) {
                return;
            }
        }
        panic!("changed breakaway context must terminate with PlanContractChanged");
    }

    #[test]
    fn breakaway_safety_rejects_changed_encoder_resolution_before_write() {
        pollster::block_on(assert_breakaway_safety_context_change_fails_before_write(
            |backend| backend.mechanical_encoder_quadrature_counts_per_rev_value = 8_000,
        ));
    }

    #[test]
    fn breakaway_safety_rejects_changed_current_limit_before_write() {
        pollster::block_on(assert_breakaway_safety_context_change_fails_before_write(
            |backend| backend.current_limit_value = current_limit(2_448),
        ));
    }

    #[fixture_test]
    fn breakaway_command_enters_the_campaign_and_reports_its_torque_limit() {
        use crate::commissioning::outer::velocity::breakaway_campaign::plan_breakaway_campaign_command;
        use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;
        use crate::commissioning::outer::velocity::workflow_plan::VelocityAutotuneWorkflowKind;

        let params = sample_outer_params();
        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: params.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: params.requested_velocity_source,
            max_stroke_travel_mrev: params.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: params.negative_position_headroom_mrev,
            positive_position_headroom_mrev: params.positive_position_headroom_mrev,
            max_duration_ms: params.max_duration_ms,
            nomination_p_below: 0,
        };
        let (campaign, workflow) =
            plan_breakaway_campaign_command(request, 4_000, 2_816, 0, 25_000, 0, 42)
                .expect("a valid host request must admit a breakaway campaign");

        assert_eq!(
            workflow.shape,
            VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral
        );
        assert_eq!(workflow.run_sequence, 42);
        assert!(workflow.nominal_workflow_ms > 0);
        assert!(workflow.nominal_workflow_ms <= workflow.maximum_workflow_ms);
        assert_eq!(campaign.hard_torque_limit(), 2_816);

        let engine = CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
            0,
            tmc_state(),
            params,
            sample_entry_snapshot(),
            campaign,
        )
        .expect("valid breakaway safety configuration");
        assert!(matches!(
            engine.engine_mode,
            EngineMode::Outer {
                phase: OuterPhase::BreakawayCampaign { .. },
                ..
            }
        ));
        assert_eq!(engine.velocity_operational_torque_limit(), Some(2_816));
    }

    #[cfg(feature = "trace")]
    #[fixture_test]
    async fn a_live_campaign_emits_its_probe_plan_onto_the_ftrc_trace_stream() {
        use crate::commissioning::outer::velocity::breakaway_campaign::plan_breakaway_campaign_command;
        use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;

        let params = sample_outer_params();
        let request = VelocitySweepRequest {
            requested_velocity_mrev_s: params.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: params.requested_velocity_source,
            max_stroke_travel_mrev: params.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: params.negative_position_headroom_mrev,
            positive_position_headroom_mrev: params.positive_position_headroom_mrev,
            max_duration_ms: params.max_duration_ms,
            nomination_p_below: 0,
        };
        let (campaign, _workflow) =
            plan_breakaway_campaign_command(request, 4_000, 2_816, 0, 25_000, 0, 42)
                .expect("a valid host request must admit a breakaway campaign");
        let mut engine = CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
            0,
            tmc_state(),
            params,
            sample_entry_snapshot(),
            campaign,
        )
        .expect("valid breakaway safety configuration");
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        let entry = engine.step(&mut backend).await;
        assert!(
            matches!(
                entry,
                StepResult::PhaseChanged(CommissionPhase::BreakawayProportionalAcquisition)
            ),
            "expected the workflow-entry phase change first, got {entry:?}"
        );
        let result = engine.step(&mut backend).await;
        assert!(
            matches!(
                result,
                StepResult::BreakawayCampaignEvent(
                    crate::commissioning::types::BreakawayCampaignEvent::ProbePlanned(_)
                )
            ),
            "expected the probe-planned event after the phase change, got {result:?}"
        );

        assert!(
            backend
                .trace_events
                .iter()
                .any(|event| event.event_id == EVENT_COMMISSION_BREAKAWAY_PROBE_PLAN),
            "no EVENT_COMMISSION_BREAKAWAY_PROBE_PLAN marker reached the FTRC trace stream: {:?}",
            backend.trace_events
        );
        let detail = backend
            .velocity_sweep_trace_records
            .iter()
            .find(|record| record.as_bytes().first() == Some(&0))
            .expect(
                "no tag-0 (breakaway probe-planned) canonical record reached the FTRC trace stream",
            );
        assert_eq!(detail.as_bytes().len(), 53);
    }

    #[test]
    fn commission_step_emits_engine_start_and_initial_phase_start() {
        pollster::block_on(async {
            let mut engine = CommissioningEngine::new_commission_with_trace_seq(
                0,
                crate::autotune::CommissionProfile::Balanced,
                42,
            );
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

            let _ = engine.step(&mut backend).await;

            let events = &backend.trace_events;
            assert_eq!(
                events[0].event_id,
                crate::trace::events::EVENT_COMMISSION_ENGINE_START
            );
            assert_eq!(events[0].engine_kind, EngineKind::Commission);
            assert_eq!(
                events[0].payload,
                [
                    crate::autotune::CommissionProfile::Balanced.to_u8() as i32,
                    42,
                ]
            );
            assert_eq!(
                events[1].event_id,
                crate::trace::events::EVENT_COMMISSION_PHASE_START
            );
            assert_eq!(
                events[1].payload[0],
                crate::autotune::CommissionPhase::AdcCalibrate.to_u8() as i32
            );
            assert_eq!(events[1].payload[1], 0);
        });
    }

    #[test]
    fn phase_changed_trace_ends_previous_phase_and_starts_next_sequence() {
        let mut engine = CommissioningEngine::new_commission_with_trace_seq(
            0,
            crate::autotune::CommissionProfile::Balanced,
            12,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(
            &mut backend,
            &StepResult::PhaseChanged(crate::autotune::CommissionPhase::CoilCheck),
        );

        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::Commission,
            TraceEventLevel::Info,
            [
                crate::autotune::CommissionPhase::AdcCalibrate.to_u8() as i32,
                0,
            ],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_PHASE_START,
            EngineKind::Commission,
            TraceEventLevel::Info,
            [
                crate::autotune::CommissionPhase::CoilCheck.to_u8() as i32,
                1,
            ],
        );
    }

    #[test]
    fn terminal_success_trace_covers_all_engine_paths() {
        assert_terminal_trace(
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced),
            StepResult::CommissionDone(sample_commission_result(
                crate::autotune::CommissionStatus::Accepted,
            )),
            EngineKind::Commission,
            crate::autotune::CommissionPhase::AdcCalibrate,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
        assert_terminal_trace(
            breakaway_safety_engine(0),
            StepResult::OuterDone(sample_outer_result(
                crate::autotune::CommissionStatus::Accepted,
            )),
            EngineKind::OuterTune,
            initial_outer_phase_id(),
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
        assert_terminal_trace(
            CommissioningEngine::new_calibrate(0),
            StepResult::CalibrateDone(sample_adc_cal_result()),
            EngineKind::Calibrate,
            crate::autotune::CommissionPhase::AdcCalibrate,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
        assert_terminal_trace(
            CommissioningEngine::new_selftest(0),
            StepResult::SelftestDone,
            EngineKind::Selftest,
            crate::autotune::CommissionPhase::AdcCalibrate,
            0,
            TraceEventLevel::Info,
            TraceEventLevel::Info,
        );
    }

    #[cfg(feature = "trace")]
    #[test]
    fn complete_integral_uses_dedicated_terminal_and_complete_events() {
        use crate::commissioning::outer::velocity::integral_evidence::{
            ExactRateInterval, IntegralRunEvidence,
        };
        use crate::commissioning::types::{
            IntegralOutcomeCause, VelocityIntegralOutcome, VelocityIntegralTerminal,
        };

        let mut engine = breakaway_safety_engine(9);
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let terminal = VelocityIntegralTerminal {
            run_sequence: 9,
            evidence_sequence: 57,
            plan_digest: 0x1111_2222_3333_4444,
            digest: 0x0123_4567_89ab_cdef,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: IntegralRunEvidence::new([
                ExactRateInterval::new(100, 120).unwrap(),
                ExactRateInterval::new(-120, -100).unwrap(),
            ]),
            expected_observations: 56,
            emitted_observations: 56,
            expected_rungs: 7,
            emitted_rungs: 7,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };

        engine.emit_trace_start(&mut backend);
        engine.emit_integral_terminal_trace(&mut backend, &terminal, None);

        assert_eq!(backend.velocity_sweep_trace_records.len(), 2);
        let ids: std::vec::Vec<_> = backend
            .trace_events
            .iter()
            .map(|event| event.event_id)
            .collect();
        assert_eq!(
            &ids[2..],
            &[
                crate::trace::events::EVENT_COMMISSION_OUTER_SAFETY_TIMING,
                crate::trace::events::EVENT_COMMISSION_PHASE_END,
                crate::trace::events::EVENT_COMMISSION_VELOCITY_INTEGRAL_TERMINAL,
                crate::trace::events::EVENT_COMMISSION_VELOCITY_INTEGRAL_COMPLETE,
            ]
        );
        assert!(engine.trace_run.terminal);
    }

    #[cfg(feature = "trace")]
    fn integral_engine_at_rest_boundary_for_test() -> (
        CommissioningEngine,
        crate::commissioning::types::VelocityIntegralTerminal,
        crate::commissioning::outer::PositiveIRestBoundary,
    ) {
        use crate::autotune::velocity_measurement::{
            StationarityExclusion, StationarityResult, VelocityDirection,
        };
        use crate::commission_dispatch::integral::IntegralPreflight;
        use crate::commissioning::outer::velocity::band_confirmation::{
            ConfirmationDirectionalInput, DiscoveryObservationAtom, evaluate_band_confirmation,
            plan_band_confirmation,
        };
        use crate::commissioning::outer::velocity::integral_evidence::{
            ExactRateInterval, IntegralRunEvidence,
        };
        use crate::commissioning::outer::velocity::integral_plan::{
            AcquisitionContext, BreakawayIntegralEvidence, IntegralPlanPhysics,
            bind_breakaway_integral_authority, plan_breakaway_velocity_integral_response,
        };
        use crate::commissioning::outer::velocity::integral_reproduction::IntegralPlanIdentity;
        use crate::commissioning::outer::velocity::sweep_consensus::{
            CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
        };
        use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
        use crate::commissioning::outer::velocity::velocity_clamp::{
            VelocityCurrentContact, VelocityCurrentPhaseEvidence, VelocityLimitEvidenceCapability,
        };
        use crate::commissioning::outer::{
            OuterMotionBudget, PositiveIRestBoundary, VelocitySettleMetrics,
        };
        use crate::commissioning::types::{
            IntegralOutcomeCause, VelocityIntegralOutcome, VelocityIntegralTerminal,
        };
        use foci_tmc4671::pid::PidPGain;

        let nominated_p = PidPGain::new(512).expect("raw gain within PidPGain range");
        let confirmation_plan = plan_band_confirmation(0x1111_2222_3333_4444, nominated_p)
            .expect("valid confirmation plan");

        let consensus = || DirectionalConsensus {
            classification: DirectionalConsensusClass::EligibleConsensus,
            components: [[0, 0]; 2],
            component_count: 1,
            collected_mask: 0b1111,
            eligible_mask: 0b1111,
            included_mask: 0b1111,
        };
        let mean_rate_q: i64 = 750_000;
        let variance_of_mean_q2: u128 = 58_752_225;
        let target_rate_q: i64 = 1_000_000;
        let forward = ConfirmationDirectionalInput {
            consensus: consensus(),
            classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            observations: [Some(DiscoveryObservationAtom {
                mean_rate_q,
                variance_of_mean_q2,
                target_rate_q,
                direction: VelocityDirection::Forward,
            }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        };
        let reverse = ConfirmationDirectionalInput {
            consensus: consensus(),
            classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            observations: [Some(DiscoveryObservationAtom {
                mean_rate_q: -mean_rate_q,
                variance_of_mean_q2,
                target_rate_q,
                direction: VelocityDirection::Reverse,
            }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        };
        let acceptance = evaluate_band_confirmation(nominated_p, forward, reverse)
            .expect("accepted confirmation");
        let intervals = acceptance.held_out_response_intervals_q;
        let token = bind_breakaway_integral_authority(
            confirmation_plan.plan_digest,
            confirmation_plan,
            Ok(acceptance),
        )
        .expect("bound breakaway authority");

        let context = AcquisitionContext {
            planned_velocity_mrev_s: 2_929,
            target_velocity_rpm: 176,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
            limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
            channel: 0,
            plan_time_pwm_hz: 25_000,
            plan_time_encoder_counts_per_rev: 4_000,
        };
        let plan = plan_breakaway_velocity_integral_response(
            token,
            confirmation_plan,
            context,
            BreakawayIntegralEvidence {
                held_out_response_intervals_q: intervals,
                confidence_q: acceptance.confidence_q,
                band_position_q: acceptance.band_position_q,
            },
            IntegralPlanPhysics {
                motion_budget: OuterMotionBudget {
                    requested_velocity_mrev_s: 2_929,
                    machine_velocity_ceiling_mrev_s: 10_000,
                    requested_velocity_source:
                        crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
                    max_stroke_travel_mrev: 2_000,
                    settle_travel_reserve_mrev: 250,
                    negative_position_headroom_mrev: 50_000,
                    positive_position_headroom_mrev: 50_000,
                    max_duration_ms: 60_000,
                    homing_speed_mrev_s: 4_375,
                    max_accel_mrev_s2: 200_000,
                },
                pwm_hz: 25_000,
                encoder_counts_per_rev: 4_000,
            },
        )
        .unwrap();

        let preflight = IntegralPreflight {
            plan,
            identity: IntegralPlanIdentity {
                channel: 0,
                build_features: 0,
                schema_revision: plan.schema_revision,
                plan_digest: plan.digest(),
            },
            run_sequence: 9,
        };

        let mut engine = CommissioningEngine::new_outer_for_tmc_with_integral_preflight(
            0,
            tmc_state(),
            sample_outer_params(),
            sample_entry_snapshot(),
            preflight,
        );

        let boundary = PositiveIRestBoundary {
            rung_index: 4,
            slot: 5,
            i_raw: 4,
            direction: -1,
            active_stationarity: StationarityResult::Excluded(
                StationarityExclusion::StructuredResidual,
            ),
            active_settle: VelocitySettleMetrics::EMPTY,
            zero_phase_current: VelocityCurrentPhaseEvidence {
                clamp_limit: 2_534,
                moving_pid_output_peak: 2_000,
                zero_pid_output_peak: 2_100,
                moving_status_flags: 0,
                zero_status_flags: 0,
                capability: VelocityLimitEvidenceCapability::BestEffortPolling,
                contact: VelocityCurrentContact::None,
            },
            cleanup_confirmed: true,
        };
        engine.set_integral_rest_boundary_for_test(boundary);

        let terminal = VelocityIntegralTerminal {
            run_sequence: 9,
            evidence_sequence: 57,
            plan_digest: plan.digest(),
            digest: 0,
            outcome: VelocityIntegralOutcome::RepeatabilityConfirmed,
            cause: IntegralOutcomeCause::Engine(0),
            breakaway_acceptance: false,
            fixed_p: 0,
            candidate_i: None,
            recovery_unavailable: false,
            recovered_with_current_headroom: false,
            recovery_quantization_exposed: false,
            probe_constrained_test_point: false,
            rest_boundary_rung_plus_one: 0,
            rest_boundary_slot_plus_one: 0,
            evidence: IntegralRunEvidence::new([
                ExactRateInterval::new(100, 120).unwrap(),
                ExactRateInterval::new(-120, -100).unwrap(),
            ]),
            expected_observations: 56,
            emitted_observations: 56,
            expected_rungs: 7,
            emitted_rungs: 7,
            run_started_us: 1_000,
            run_completed_us: 2_000,
            safety_fault: None,
            rest_rejection_after_sufficiency: false,
            rest_rejection_owner: None,
        };

        (engine, terminal, boundary)
    }

    #[cfg(feature = "trace")]
    #[fixture_test]
    async fn integral_terminal_digest_is_independent_of_emission_sink() {
        let (engine, mut terminal, boundary) = integral_engine_at_rest_boundary_for_test();
        let record = engine
            .attach_integral_rest_boundary(&mut terminal)
            .expect("a boundary record is produced");
        assert_ne!(terminal.digest, 0);
        assert_eq!(terminal.rest_boundary_rung_plus_one, boundary.rung_index);
        assert_eq!(
            terminal.rest_boundary_slot_plus_one,
            boundary.slot.checked_add(1).unwrap()
        );
        let folded_digest = terminal.digest;
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        backend.record_velocity_sweep_trace_record(record);
        assert_eq!(terminal.digest, folded_digest);
    }

    #[test]
    fn failure_trace_ends_active_inner_phase_and_is_idempotent() {
        let mut engine = CommissioningEngine::new_commission_with_trace_seq(
            0,
            crate::autotune::CommissionProfile::Balanced,
            21,
        );
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let err = crate::autotune::CommissionError::ChipResetDetected;

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &StepResult::InnerFailed(err));

        let status = err.status_code();
        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::Commission,
            TraceEventLevel::Error,
            [
                crate::autotune::CommissionPhase::AdcCalibrate.to_u8() as i32,
                i32::from(status),
            ],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_ENGINE_TERMINAL,
            EngineKind::Commission,
            TraceEventLevel::Error,
            [i32::from(status), 21],
        );

        engine.emit_trace_after_step(&mut backend, &StepResult::InnerFailed(err));
        assert_eq!(backend.trace_events.len(), 4);
    }

    #[test]
    fn failure_trace_ends_active_outer_phase_and_is_idempotent() {
        let mut engine = breakaway_safety_engine(22);
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let err = crate::autotune::CommissionError::VelocityValidationFailed;

        engine.emit_trace_start(&mut backend);
        engine.emit_trace_after_step(&mut backend, &StepResult::OuterFailed(err));

        let status = err.status_code();
        let events = &backend.trace_events;
        assert_eq!(events.len(), 4);
        assert_trace_event(
            &events[2],
            crate::trace::events::EVENT_COMMISSION_PHASE_END,
            EngineKind::OuterTune,
            TraceEventLevel::Error,
            [initial_outer_phase_id().to_u8() as i32, i32::from(status)],
        );
        assert_trace_event(
            &events[3],
            crate::trace::events::EVENT_COMMISSION_ENGINE_TERMINAL,
            EngineKind::OuterTune,
            TraceEventLevel::Error,
            [i32::from(status), 22],
        );

        engine.emit_trace_after_step(&mut backend, &StepResult::OuterFailed(err));
        assert_eq!(backend.trace_events.len(), 4);
    }

    #[test]
    fn calibrate_and_selftest_trace_start_use_zero_profile_or_mode() {
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let mut calibrate = CommissioningEngine::new_calibrate_with_trace_seq(0, 31);
        calibrate.emit_trace_start(&mut backend);
        assert_trace_event(
            &backend.trace_events[0],
            crate::trace::events::EVENT_COMMISSION_ENGINE_START,
            EngineKind::Calibrate,
            TraceEventLevel::Info,
            [0, 31],
        );

        backend.trace_events.clear();
        let mut selftest = CommissioningEngine::new_selftest_with_trace_seq(0, 32);
        selftest.emit_trace_start(&mut backend);
        assert_trace_event(
            &backend.trace_events[0],
            crate::trace::events::EVENT_COMMISSION_ENGINE_START,
            EngineKind::Selftest,
            TraceEventLevel::Info,
            [0, 32],
        );
    }

    #[test]
    fn warning_terminal_statuses_emit_warn_level() {
        assert_terminal_trace(
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced),
            StepResult::CommissionDone(sample_commission_result(
                crate::autotune::CommissionStatus::AcceptedWithWarnings,
            )),
            EngineKind::Commission,
            crate::autotune::CommissionPhase::AdcCalibrate,
            crate::autotune::CommissionStatus::AcceptedWithWarnings.to_u8(),
            TraceEventLevel::Info,
            TraceEventLevel::Warn,
        );
        assert_terminal_trace(
            breakaway_safety_engine(0),
            StepResult::OuterDone(sample_outer_result(
                crate::autotune::CommissionStatus::InnerOnly,
            )),
            EngineKind::OuterTune,
            initial_outer_phase_id(),
            crate::autotune::CommissionStatus::InnerOnly.to_u8(),
            TraceEventLevel::Info,
            TraceEventLevel::Warn,
        );
    }

    #[fixture_test]
    async fn step_restores_safe_state_when_shutdown_requested_mid_inner() {
        use core::sync::atomic::Ordering;
        let mut engine =
            CommissioningEngine::new_commission(0, crate::autotune::CommissionProfile::Balanced);
        if let EngineMode::Inner { phase, .. } = &mut engine.engine_mode {
            let data = inner::ElectricalIdentificationData::new(
                512,
                crate::commission_dispatch::test_support::current_limit(2816),
            );
            *phase = InnerPhase::ElectricalIdentification {
                state: inner::ElectricalIdentificationState::InductanceDisable { deadline_us: 20 },
                data,
            };
        }
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        backend.shutdown_requested = true;
        backend.enable_pin = true;
        tmc_state().motor_armed.store(true, Ordering::Relaxed);

        let result = engine.step(&mut backend).await;

        assert!(matches!(
            result,
            crate::commissioning::types::StepResult::InnerFailed(
                crate::autotune::CommissionError::Shutdown,
            )
        ));
        assert!(!backend.enable_pin, "shutdown abort must lower enable pin");
        assert!(
            !tmc_state().motor_armed.load(Ordering::Relaxed),
            "shutdown abort must gate step processing before reporting failure"
        );
    }

    #[test]
    fn shutdown_during_resistance_emits_current_evidence() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.tmc_config.resistance_max_expected_current_count = 321;
            let profile = backend
                .tmc_config
                .resistance_profile(backend.tmc_config.resistance_scale_metadata_validated);
            if let EngineMode::Inner { phase, .. } = &mut engine.engine_mode {
                *phase = InnerPhase::ElectricalIdentification {
                    state: inner::ElectricalIdentificationState::ResistanceIdentify {
                        runner: crate::resistance_identification::ResistanceRunner::new(
                            crate::resistance_identification::ResistanceRunMode::CommissioningManagedEnable,
                            profile,
                            0,
                        )
                        .unwrap(),
                    },
                    data: inner::ElectricalIdentificationData::new(
                        512,
                        crate::commission_dispatch::test_support::current_limit(2816),
                    ),
                };
            }
            backend.shutdown_requested = true;
            backend.enable_pin = true;

            let result = engine.step(&mut backend).await;

            assert!(matches!(
                result,
                crate::commissioning::types::StepResult::InnerFailed(
                    crate::autotune::CommissionError::Shutdown,
                )
            ));
            let diagnostics = engine.take_diagnostics();
            let current = diagnostics
                .iter()
                .find(|event| event.code == 40)
                .expect("shutdown resistance current diagnostic");
            assert_eq!(current.value0, 0);
            assert_eq!(current.value1, 0);
            assert_eq!(current.value2, 321);
            assert_eq!(current.status, 0);
        });
    }

    #[test]
    fn shutdown_during_inductance_restore_retains_completed_timing() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            let mut engine = CommissioningEngine::new_commission(
                0,
                crate::autotune::CommissionProfile::Balanced,
            );
            let timing = capture_timing::CaptureTimingEvidence {
                requested_period_us: 100,
                valid_samples: 7,
                missed_samples: 0,
                max_consecutive_misses: 0,
                max_lateness_us: 10,
                max_interval_us: 110,
                max_poll_wall_us: 15,
                max_spi_wall_us: 8,
            };
            if let EngineMode::Inner { phase, .. } = &mut engine.engine_mode {
                let mut data = inner::ElectricalIdentificationData::new(
                    512,
                    crate::commission_dispatch::test_support::current_limit(2816),
                );
                data.timing_states = crate::commissioning::types::COMMISSION_TIMING_REJECTED << 2;
                *phase = InnerPhase::ElectricalIdentification {
                    state: inner::ElectricalIdentificationState::InductanceRestore {
                        operation: inner::InductanceRestoreOperation::RetainedUqUdExt,
                        primary_error: None,
                        evidence: Some(crate::commissioning::types::InductanceEvidence::EMPTY),
                        timing: Some(timing),
                        timing_accepted: true,
                    },
                    data,
                };
            }
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            backend.shutdown_requested = true;
            backend.enable_pin = true;

            let result = engine.step(&mut backend).await;

            assert!(matches!(
                result,
                crate::commissioning::types::StepResult::InnerFailed(
                    crate::autotune::CommissionError::Shutdown,
                )
            ));
            assert!(!backend.enable_pin);
            assert!(matches!(
                backend.spi_log.first(),
                Some(SpiOp::EnablePin(false))
            ));
            assert!(
                backend.delay_us_log.is_empty(),
                "shutdown snapshot must not delay before GPIO-first restore"
            );
            let diagnostics = engine.take_diagnostics();
            let header = diagnostics
                .iter()
                .find(|event| event.code == 36)
                .expect("inductance restore timing diagnostic");
            assert_eq!(header.value0 >> 16, 7);
            assert_eq!((header.value1 >> 24) & 0x03, 2);
        });
    }
}
