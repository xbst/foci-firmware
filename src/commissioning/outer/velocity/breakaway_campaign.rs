use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidGainPair, PidIGain, PidPGain};
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, PID_VELOCITY_ACTUAL, PID_VELOCITY_OFFSET,
    PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET,
};

use crate::autotune::velocity_measurement::{
    DyadicBlocker, StationarityExclusion, StationarityResult, StationarityWork,
    VELOCITY_RATE_Q_SCALE, VelocityDirection, confidence_multiplier_95_df15,
};
use crate::autotune::{CommissionError, EquivalentTorqueTargetInterval, TmcMechanicalRpm};
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::encoder_domain::EncoderDomain;
use crate::commissioning::outer::state::{
    OuterPhase, VelocityIntegralRuntime, VelocityIntegralState,
};
use crate::commissioning::outer::state::{VELOCITY_SWEEP_BLOCK_LEVELS, VelocityObservationRing};
use crate::commissioning::outer::{OuterSafetyLimits, fail_outer, fail_outer_with_safety};
use crate::commissioning::schedule::next_grid_deadline;
use crate::commissioning::types::StepResult;
use crate::commissioning::types::{
    BreakawayCampaignEvent, BreakawayCeilingSource, OUTER_SAFETY_FAULT_CURRENT,
    OUTER_SAFETY_FAULT_OBSERVATION_GAP, OUTER_SAFETY_FAULT_QUARTER_TURN, OuterSafetyFault,
};
use crate::diagnostic::BreakawayCheckpoint;
use crate::tmc_register_values::MOTION_VELOCITY_ADVANCED_PI;

use super::band_confirmation::{
    BandConfirmationAcceptance, BandConfirmationFailureCause, BandConfirmationPlan,
    CONFIRMATION_BAND_LOWER_PERCENT, CONFIRMATION_BAND_UPPER_PERCENT, CONFIRMATION_FAMILY_SIZE,
    CONFIRMATION_SLOT_ORDER, ConfirmationDirectionalInput, DiscoveryObservationAtom,
    DiscoveryRungNominationBounds, DiscoveryRungNominationInput, NominatedConfirmationCandidate,
    NominationRejection, STEP_PEAK_ADMISSION_PERMILLE, TargetBandConfirmationInconclusive,
    evaluate_band_confirmation, nominate_transition_capable_candidate_from_bounds,
    plan_band_confirmation, rung_directional_pooled_bounds_percent, rung_nomination_bounds,
    rung_step_peak_permille, validate_fixed_contract,
};
use super::breakaway_excursion::{ProbeExcursionCapture, ProbeExcursionError, ProbeMotionClass};
use super::breakaway_plan::{
    AdditiveDiscoveryPlan, AdditiveDiscoveryRequest, AdditiveLadderPlanError,
    BreakawayProbeContext, GridFloorOrigin, PROBE_MAX_OBSERVATIONS, UpwardBreakawayProbePlan,
    plan_additive_discovery_ladder, plan_upward_breakaway_probe, resolve_grid_floor,
};
use super::breakaway_reporting::{
    BreakawayPhase, BreakawayTerminalCause, ProbeDecisionReportInput, RecoveryRestRejectedInput,
    campaign_terminal_report, confirmation_evaluated_terminal_report,
    confirmation_partial_evidence_terminal_report, confirmation_plan_report,
    confirmation_zero_stroke_cause, confirmation_zero_stroke_terminal_report,
    discovery_plan_report, discovery_terminal_cause, discovery_terminal_report, probe_plan_report,
    probe_resolved_report, probe_terminal_cause, probe_terminal_report,
    rung_zero_diagnostic_report,
};
use super::breakaway_search::{
    BreakawayMotionClass, BreakawaySearchError, ProbeStrokeRole, UpwardBreakawayCursor,
    UpwardProbeStep,
};
use super::integral_plan::integral_continuation_geometry;
use super::integral_plan::{
    AcquisitionContext, BreakawayIntegralEvidence, IntegralPlanPhysics, VelocityIntegralPlan,
    bind_breakaway_integral_authority, breakaway_workflow_envelope_ms, max_transition_capable_p,
    plan_breakaway_velocity_integral_response,
};
use super::peak_ratio::scored_peak_permille;
use super::recovery::{
    RecoveryBindingSource, RecoveryCurrentContactPolicy, RecoveryOriginMismatchEvidence,
    RecoveryStage, RungOriginRecoveryConfig, RungOriginRecoveryResources,
    RungOriginRecoveryRuntime, RungOriginRecoveryStep, commanded_side_lower_rate_q,
    plan_rung_origin_recovery, step_rung_origin_recovery,
};
use super::sweep_analysis::{
    ScoredRestPoll, VelocityObservationInputRequest, assemble_probe_observation_input,
    assemble_velocity_observation_input, bounded_rest_displacement, direction_sign,
    poll_scored_rest_confirmation, poll_stationarity,
};
use super::sweep_consensus::{
    CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, depth_three_consensus,
};
use super::sweep_evidence::{
    VelocityObservationClass, VelocityObservationInput, classify_velocity_observation,
};
use super::sweep_plan::current_gain_span;
use super::sweep_plan::{
    MAX_SAFETY_OBSERVATION_GAP_US, MAX_SWEEP_RUNGS, MIN_SETTLE_CAPTURE_US, MOVING_STROKE_US,
    OBSERVATIONS_PER_DIRECTION_PER_RUNG, SETTLE_ANALYSIS_BUDGET_US,
    VELOCITY_SWEEP_SAMPLE_PERIOD_US, VelocitySweepRequest, ZERO_SETTLE_US,
    plan_proportional_velocity_sweep,
};
use super::velocity_clamp::{
    VelocityCurrentClampSession, VelocityCurrentGuardError, VelocityCurrentObserver,
    VelocityCurrentZeroPhase, VelocityLimitEvidenceCapability, begin_velocity_current_primitive,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs,
};
use super::workflow_plan::CommissioningWorkflowPlan;
use super::{clear_velocity_error_sum, clear_velocity_offset};

#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayProbeWorkspace {
    previous_encoder_raw: u32,
    previous_read_us: u64,
    excursion: Option<ProbeExcursionCapture>,
    current_observer: VelocityCurrentObserver,
    settle_displacement_counts: i8,
    settle_min_displacement_counts: i8,
    settle_max_displacement_counts: i8,
    #[defmt(Debug2Format)]
    observations: VelocityObservationRing,
    #[defmt(Debug2Format)]
    blocker: DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    #[defmt(Debug2Format)]
    analysis: StationarityWork,
}

impl BreakawayProbeWorkspace {
    pub(crate) const fn new() -> Self {
        Self {
            previous_encoder_raw: 0,
            previous_read_us: 0,
            excursion: None,
            current_observer: VelocityCurrentObserver::EMPTY,
            settle_displacement_counts: 0,
            settle_min_displacement_counts: 0,
            settle_max_displacement_counts: 0,
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
        }
    }
}

#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayProbeSession {
    plan: UpwardBreakawayProbePlan,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    cursor: UpwardBreakawayCursor,
    pending_index: u8,
    pending_role: ProbeStrokeRole,
    pending_result: Option<Result<bool, BreakawayProbeOutcome>>,
    pending_lower_rate_q: Option<u64>,
    pending_observation: Option<CollectedObservation>,
}

impl BreakawayProbeSession {
    pub(crate) fn new(
        plan: UpwardBreakawayProbePlan,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Result<Self, BreakawaySearchError> {
        let (cursor, first) = UpwardBreakawayCursor::start(plan.search_count)?;
        let UpwardProbeStep::Probe {
            rung_index: pending_index,
            role: pending_role,
        } = first
        else {
            return Err(BreakawaySearchError::InvalidRequest);
        };
        Ok(Self {
            plan,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            cursor,
            pending_index,
            pending_role,
            pending_result: None,
            pending_lower_rate_q: None,
            pending_observation: None,
        })
    }

    fn pending_gain(&self) -> Result<PidPGain, BreakawaySearchError> {
        if self.pending_index >= self.plan.search_count {
            return Err(BreakawaySearchError::InvalidRequest);
        }
        self.plan
            .search_gains
            .get(usize::from(self.pending_index))
            .copied()
            .ok_or(BreakawaySearchError::InvalidRequest)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum BreakawayProbeState {
    PlannedProbe,
    SetupSafety,
    WriteGain,
    CommandTarget,
    Capture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    CommandZero,
    AnalyzeMoving {
        settle_deadline_us: u64,
        analysis_deadline_us: u64,
    },
    BeginSettleCapture {
        settle_deadline_us: u64,
    },
    SettleCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    AnalyzeSettle {
        analysis_deadline_us: u64,
    },
    Recover(RungOriginRecoveryRuntime),
    EmitDecision,
    Advance,
}

#[derive(Debug, defmt::Format)]
pub(crate) enum BreakawayProbeStep {
    ReadyNow,
    At(u64),
    Observation(CollectedObservation),
    Decision {
        direction: VelocityDirection,
        rung_index: u8,
        disposition: u8,
    },
    RestRejected(RecoveryRestRejectedInput),
    OriginMismatch(RecoveryOriginMismatchEvidence),
    Done(BreakawayProbeOutcome),
}

#[derive(Debug, defmt::Format)]
pub(crate) enum BreakawayProbeOutcome {
    BreakawayFound(ResolvedUpwardBreakaway),
    NoRepeatableMotionAtAuthority,
    ExcursionEvidenceInvalid,
    AuthorityFailure,
    SafetyFault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ResolvedUpwardBreakaway {
    pub(crate) breakaway_index: u8,
    pub(crate) observations: u8,
}

fn fault(error: CommissionError) -> BreakawayProbeStep {
    BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault {
        error,
        safety: None,
    })
}

fn safety_fault(safety: OuterSafetyFault, error: CommissionError) -> BreakawayProbeStep {
    defmt::warn!("probe safety fault: {}", safety);
    BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault {
        error,
        safety: Some(safety),
    })
}

fn current_guard_fault(error: VelocityCurrentGuardError) -> BreakawayProbeStep {
    fault(match error {
        VelocityCurrentGuardError::InvalidLimit => CommissionError::InvalidCurrentLimit,
        VelocityCurrentGuardError::RegisterIo => CommissionError::Spi,
        VelocityCurrentGuardError::ActionableStatus(status) => {
            defmt::warn!("probe current guard actionable status: {:#x}", status);
            CommissionError::SafetyEnvelopeViolation
        }
        // The probe climbs from the smallest representable gain; limiter contact
        // there means the axis cannot follow even the gentlest stroke, which is
        // still a fault rather than a reached authority ceiling.
        VelocityCurrentGuardError::LimiterContact(status) => {
            defmt::warn!("probe current guard limiter contact: {:#x}", status);
            CommissionError::SafetyEnvelopeViolation
        }
    })
}

fn checkpoint_breakaway_operation(checkpoint: BreakawayCheckpoint) {
    crate::diagnostic::checkpoint_breakaway_campaign(checkpoint);
}

fn search_error_outcome(error: BreakawaySearchError) -> BreakawayProbeStep {
    match error {
        BreakawaySearchError::InvalidRequest | BreakawaySearchError::Arithmetic => {
            fault(CommissionError::SearchGridInvalid)
        }
    }
}

pub(crate) async fn step_breakaway_probe<B: CommissioningBackend>(
    state: &mut BreakawayProbeState,
    workspace: &mut BreakawayProbeWorkspace,
    session: &mut BreakawayProbeSession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> BreakawayProbeStep {
    match state {
        BreakawayProbeState::PlannedProbe => {
            *state = BreakawayProbeState::SetupSafety;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::SetupSafety => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::SafetyConfigured,
            );
            let frozen = session.plan.context;
            if backend.mechanical_encoder_quadrature_counts_per_rev()
                != frozen.encoder_counts_per_rev
                || u16::from(backend.current_limit()) != frozen.hard_torque_limit
            {
                return fault(CommissionError::PlanContractChanged);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeSafetyEncoderRead);
            let encoder_raw = match backend.try_read_mechanical_encoder_position_counts().await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            if safety.set_encoder_baseline(encoder_raw).is_err() {
                return fault(CommissionError::EncoderFault);
            }
            safety.torque_cap = frozen.hard_torque_limit;
            if let Err(error) = safety.arm_safety_observation(backend.elapsed_us()) {
                return fault(error);
            }
            *state = BreakawayProbeState::WriteGain;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::WriteGain => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::GainProgramming,
            );
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeGainWrite);
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeRampModeWrite);
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, MOTION_VELOCITY_ADVANCED_PI)
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeGainIntegratorClear);
            if clear_velocity_error_sum(backend).await.is_err() {
                return fault(CommissionError::Spi);
            }
            *state = BreakawayProbeState::CommandTarget;
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::CommandTarget => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::TargetCommand,
            );
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.pending_role.direction()))
            {
                Ok(target) => target.raw(),
                Err(_) => return fault(CommissionError::TargetRateInvalid),
            };
            let hard_limit = session.plan.context.hard_torque_limit;
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return current_guard_fault(error),
                };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeClampInstall);
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return current_guard_fault(error);
            }
            let clamp_limit = clamp_session.probe_limit();
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeCurrentPrimitiveStart);
            if let Err(error) = begin_velocity_current_primitive(
                &mut workspace.current_observer,
                backend,
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            )
            .await
            {
                return current_guard_fault(error);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeTargetBaselineRead);
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            workspace.excursion = Some(ProbeExcursionCapture::new(encoder_raw, baseline_us));
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeTargetWrite);
            if backend
                .write_register_verified(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            if let Err(error) = safety.begin_velocity_step_primitive(backend.elapsed_us()) {
                return fault(error);
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fault(CommissionError::InvalidSchedule);
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return fault(CommissionError::InvalidSchedule);
            };
            workspace.observations.clear();
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::Capture,
            );
            *state = BreakawayProbeState::Capture {
                next_sample_us,
                deadline_us,
            };
            BreakawayProbeStep::At(next_sample_us)
        }
        BreakawayProbeState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BreakawayProbeStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(
                workspace,
                safety,
                backend,
                Some(BreakawayCheckpoint::ProbeMovingEncoderSample),
                Some(BreakawayCheckpoint::ProbeMovingCurrentInputSample),
                true,
            )
            .await
            {
                Ok(read_us) => read_us,
                Err(result) => return result,
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeMovingCurrentOutputSample);
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_moving_output(value),
                Err(error) => return current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                *state = BreakawayProbeState::CommandZero;
                BreakawayProbeStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BreakawayProbeStep::At(*next_sample_us)
                    }
                    Err(error) => fault(error),
                }
            }
        }
        BreakawayProbeState::CommandZero => {
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeMovingBoundary);
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeZeroTargetWrite);
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return fault(CommissionError::Spi);
            }
            safety.enter_settle_phase();
            if let Err(error) = boundary {
                return current_guard_fault(error);
            }
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeZeroIntegratorClear);
            if clear_velocity_error_sum(backend).await.is_err() {
                return fault(CommissionError::Spi);
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = now_us.checked_add(u64::from(ZERO_SETTLE_US)) else {
                return fault(CommissionError::InvalidSchedule);
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return fault(CommissionError::InvalidSchedule);
            };
            workspace.analysis.reset_with_structured_diagnostics();
            *state = BreakawayProbeState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fault(CommissionError::AnalysisOverrun);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let (stationarity, rejected_post_fit, drift_ratio_ppm) = match poll {
                Ok(Some((result, rejection, drift_ratio_ppm))) => {
                    (result, rejection, drift_ratio_ppm)
                }
                Ok(None) => return BreakawayProbeStep::ReadyNow,
                Err(error) => return fault(error),
            };
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            let input = match assemble_probe_observation_input(VelocityObservationInputRequest {
                direction: session.pending_role.direction(),
                p,
                stationarity,
                rejected_post_fit,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: session.plan.context.hard_torque_limit,
                initial_lead_in: false,
                confidence_family_size: 3,
                counts_per_rev: safety.counts_per_rev(),
            }) {
                Ok(input) => input,
                Err(error) => return fault(error),
            };
            session.pending_lower_rate_q = commanded_side_lower_rate_q(
                input.direction,
                input.settled_rate_low_q,
                input.settled_rate_high_q,
            );
            let capture = match workspace.excursion.take() {
                Some(capture) => capture,
                None => return fault(CommissionError::MissingCaptureState),
            };
            let wire_fields = capture.wire_fields();
            let excursion = capture.finish(
                session.pending_role.direction(),
                session.plan.motion_threshold_counts,
                session.plan.max_capture_interval_us,
            );
            session.pending_result = Some(
                if input.predicted_torque_target_abs >= input.hard_torque_limit {
                    defmt::warn!(
                        "probe torque prediction fault: dir={} p={} predicted_torque_target_abs={} usable_torque_limit={} hard_torque_limit={} moving_pid_output_peak={}",
                        input.direction,
                        input.p.raw(),
                        input.predicted_torque_target_abs,
                        input.usable_torque_limit,
                        input.hard_torque_limit,
                        workspace.current_observer.moving_pid_output_peak()
                    );
                    Err(BreakawayProbeOutcome::SafetyFault {
                        error: CommissionError::SafetyEnvelopeViolation,
                        safety: None,
                    })
                } else {
                    match excursion {
                        Err(error) => {
                            checkpoint_breakaway_operation(match error {
                                ProbeExcursionError::CaptureInterval => {
                                    BreakawayCheckpoint::ProbeExcursionIntervalInvalid
                                }
                                ProbeExcursionError::Arithmetic => {
                                    BreakawayCheckpoint::ProbeExcursionArithmeticInvalid
                                }
                            });
                            Err(BreakawayProbeOutcome::ExcursionEvidenceInvalid)
                        }
                        Ok(super::breakaway_excursion::ProbeExcursionEvidence {
                            motion: ProbeMotionClass::WrongDirection,
                            ..
                        }) => Err(BreakawayProbeOutcome::SafetyFault {
                            error: CommissionError::VelocityWrongDirection,
                            safety: None,
                        }),
                        Ok(evidence)
                            if matches!(evidence.motion, ProbeMotionClass::Moved)
                                && input.predicted_torque_target_abs
                                    >= input.usable_torque_limit =>
                        {
                            Err(BreakawayProbeOutcome::AuthorityFailure)
                        }
                        Ok(evidence) => Ok(matches!(evidence.motion, ProbeMotionClass::Moved)),
                    }
                },
            );
            let target_rate_q =
                match discovery_target_rate_q(session.target_velocity_rpm, safety.counts_per_rev())
                {
                    Ok(value) => value,
                    Err(error) => return fault(error),
                };
            let collected = CollectedObservation {
                atom: DiscoveryObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2: input.variance_of_mean_q2,
                    target_rate_q,
                    direction: session.pending_role.direction(),
                },
                rung_index: session.pending_index,
                slot_index: session.pending_role.slot_index(),
                classifier_inputs: observation_classifier_inputs(
                    &input,
                    workspace.current_observer.moving_pid_output_peak(),
                ),
                stroke_min_displacement_counts: wire_fields.map_or(0, |fields| fields.0),
                stroke_max_displacement_counts: wire_fields.map_or(0, |fields| fields.1),
                capture_max_interval_us: wire_fields.map_or(0, |fields| fields.2),
            };
            session.pending_observation = Some(collected);
            workspace.analysis.reset();
            workspace.observations.clear();
            if session.pending_result.as_ref().is_some_and(Result::is_err) {
                safety.end_primitive();
            }
            *state = match session.pending_result.as_ref() {
                Some(Ok(_)) => BreakawayProbeState::BeginSettleCapture {
                    settle_deadline_us: *settle_deadline_us,
                },
                Some(Err(_)) => BreakawayProbeState::Advance,
                None => return fault(CommissionError::MissingCaptureState),
            };
            BreakawayProbeStep::Observation(collected)
        }
        BreakawayProbeState::BeginSettleCapture { settle_deadline_us } => {
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeSettleBaselineRead);
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return fault(CommissionError::AnalysisOverrun);
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return fault(CommissionError::InvalidSchedule);
            };
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = BreakawayProbeState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
            };
            BreakawayProbeStep::At(next_sample_us)
        }
        BreakawayProbeState::SettleCapture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BreakawayProbeStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(
                workspace,
                safety,
                backend,
                Some(BreakawayCheckpoint::ProbeSettleEncoderSample),
                Some(BreakawayCheckpoint::ProbeSettleCurrentInputSample),
                false,
            )
            .await
            {
                Ok(read_us) => read_us,
                Err(result) => return result,
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeSettleCurrentOutputSample);
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_zero_output(value),
                Err(error) => return current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return fault(CommissionError::InvalidSchedule);
                };
                workspace
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        workspace.settle_min_displacement_counts,
                        workspace.settle_max_displacement_counts,
                    ));
                *state = BreakawayProbeState::AnalyzeSettle {
                    analysis_deadline_us,
                };
                BreakawayProbeStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BreakawayProbeStep::At(*next_sample_us)
                    }
                    Err(error) => fault(error),
                }
            }
        }
        BreakawayProbeState::AnalyzeSettle {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return fault(CommissionError::AnalysisOverrun);
            }
            let confirmation = match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending | ScoredRestPoll::Diagnostic(_)) => {
                    return BreakawayProbeStep::ReadyNow;
                }
                Ok(ScoredRestPoll::Complete(confirmation)) => confirmation,
                Err(error) => return fault(error),
            };
            checkpoint_breakaway_operation(BreakawayCheckpoint::ProbeZeroBoundary);
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut workspace.current_observer, backend).await
            {
                return current_guard_fault(error);
            }
            if workspace
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .is_err()
            {
                return fault(CommissionError::InvalidCurrentLimit);
            }
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                return fault(CommissionError::VelocityRestNotConfirmed);
            }
            let recovery_plan = match plan_rung_origin_recovery(
                safety.budget,
                safety.counts_per_rev(),
                session.planned_velocity_mrev_s,
            ) {
                Ok(plan) => plan,
                Err(_) => return fault(CommissionError::RecoveryPlanFailed),
            };
            let p = match session.pending_gain() {
                Ok(p) => p,
                Err(error) => return search_error_outcome(error),
            };
            let hard_limit = session.plan.context.hard_torque_limit;
            let clamp_limit = match PidTorqueFluxLimit::new(u32::from(hard_limit)) {
                Ok(limit) => limit,
                Err(_) => return fault(CommissionError::InvalidCurrentLimit),
            };
            workspace.current_observer.reset(
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            );
            let (lower_rate_q, authority_source) = match session.pending_lower_rate_q {
                Some(rate) => (Some(rate), RecoveryBindingSource::SettledRate),
                None => (None, RecoveryBindingSource::MissingRate),
            };
            let runtime = match RungOriginRecoveryRuntime::new(
                RungOriginRecoveryConfig {
                    stage: RecoveryStage::Proportional,
                    rung_index: session.pending_index,
                    p,
                    target_velocity_rpm: session.target_velocity_rpm,
                    planned_velocity_mrev_s: session.planned_velocity_mrev_s,
                    start_offset_counts: safety.unwrapped_position,
                    lower_rate_q,
                    authority_source,
                    current_contact_policy: RecoveryCurrentContactPolicy::StopImmediately,
                },
                recovery_plan,
            ) {
                Ok(runtime) => runtime,
                Err(error) => return fault(error),
            };
            *state = BreakawayProbeState::Recover(runtime);
            BreakawayProbeStep::ReadyNow
        }
        BreakawayProbeState::Recover(runtime) => {
            let mut resources = RungOriginRecoveryResources {
                safety,
                observations: &mut workspace.observations,
                blocker: &mut workspace.blocker,
                analysis: &mut workspace.analysis,
                settle_displacement_counts: &mut workspace.settle_displacement_counts,
                settle_min_displacement_counts: &mut workspace.settle_min_displacement_counts,
                settle_max_displacement_counts: &mut workspace.settle_max_displacement_counts,
                current_observer: &mut workspace.current_observer,
            };
            let step = step_rung_origin_recovery(runtime, &mut resources, backend).await;
            let settle = (
                *resources.settle_displacement_counts,
                *resources.settle_min_displacement_counts,
                *resources.settle_max_displacement_counts,
            );
            let p_raw = runtime.p_raw();
            match step {
                RungOriginRecoveryStep::ReadyNow => BreakawayProbeStep::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => BreakawayProbeStep::At(deadline_us),
                RungOriginRecoveryStep::RestSelection {
                    decision,
                    stage,
                    rung_index,
                } => match rest_rejected_input(
                    decision,
                    stage,
                    rung_index,
                    session.pending_role.slot_index(),
                    session.pending_role.direction(),
                    p_raw,
                    settle,
                ) {
                    Some(input) => BreakawayProbeStep::RestRejected(input),
                    None => BreakawayProbeStep::ReadyNow,
                },
                RungOriginRecoveryStep::RestAnalysis(_) => BreakawayProbeStep::ReadyNow,
                RungOriginRecoveryStep::OriginMismatch(evidence) => {
                    BreakawayProbeStep::OriginMismatch(evidence)
                }
                RungOriginRecoveryStep::Finished(_) | RungOriginRecoveryStep::Contained(_) => {
                    *state = if session.pending_result.as_ref().is_some_and(Result::is_ok) {
                        BreakawayProbeState::EmitDecision
                    } else {
                        BreakawayProbeState::Advance
                    };
                    BreakawayProbeStep::ReadyNow
                }
                RungOriginRecoveryStep::RestUnconfirmed => {
                    fault(CommissionError::VelocityRestNotConfirmed)
                }
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: Some(fault_report),
                } => safety_fault(fault_report, error),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => fault(error),
            }
        }
        BreakawayProbeState::EmitDecision => {
            let moved = session
                .pending_result
                .as_ref()
                .and_then(|result| result.as_ref().ok())
                .copied()
                .expect("only a motion verdict reaches decision emission");
            let observation = session
                .pending_observation
                .expect("raw observation retained until cursor commit");
            *state = BreakawayProbeState::Advance;
            BreakawayProbeStep::Decision {
                direction: observation.atom.direction,
                rung_index: observation.rung_index,
                disposition: u8::from(moved),
            }
        }
        BreakawayProbeState::Advance => {
            session
                .pending_observation
                .take()
                .expect("raw observation retained until cursor consumption");
            let result = session
                .pending_result
                .take()
                .expect("classified before recovery started");
            let moved = match result {
                Ok(moved) => moved,
                Err(outcome) => return BreakawayProbeStep::Done(outcome),
            };
            match session.cursor.advance(moved) {
                Ok(UpwardProbeStep::Probe { rung_index, role }) => {
                    session.pending_index = rung_index;
                    session.pending_role = role;
                    *state = BreakawayProbeState::WriteGain;
                    BreakawayProbeStep::ReadyNow
                }
                Ok(UpwardProbeStep::Resolved {
                    rung_index,
                    observations,
                }) => BreakawayProbeStep::Done(BreakawayProbeOutcome::BreakawayFound(
                    ResolvedUpwardBreakaway {
                        breakaway_index: rung_index,
                        observations,
                    },
                )),
                Ok(UpwardProbeStep::NoRepeatableMotionAtAuthority { .. }) => {
                    BreakawayProbeStep::Done(BreakawayProbeOutcome::NoRepeatableMotionAtAuthority)
                }
                Err(error) => search_error_outcome(error),
            }
        }
    }
}

#[allow(clippy::result_large_err)]
async fn sample_one_tick<B: CommissioningBackend>(
    workspace: &mut BreakawayProbeWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
    encoder_checkpoint: Option<BreakawayCheckpoint>,
    current_input_checkpoint: Option<BreakawayCheckpoint>,
    record_excursion: bool,
) -> Result<u64, BreakawayProbeStep> {
    if let Some(checkpoint) = encoder_checkpoint {
        checkpoint_breakaway_operation(checkpoint);
    }
    let encoder_raw = backend
        .read_capture_register(ABN_DECODER_COUNT)
        .await
        .map_err(|_| fault(CommissionError::Spi))?;
    let read_us = backend.elapsed_us();
    let elapsed_us = match read_us.checked_sub(workspace.previous_read_us) {
        Some(value @ 1..=MAX_SAFETY_OBSERVATION_GAP_US) => value,
        _ => {
            let fault_report = safety.fault(
                OUTER_SAFETY_FAULT_OBSERVATION_GAP,
                0,
                read_us.saturating_sub(workspace.previous_read_us),
                0,
                0,
            );
            return Err(safety_fault(
                fault_report,
                CommissionError::SafetyEnvelopeViolation,
            ));
        }
    };
    let domain = EncoderDomain::new(safety.counts_per_rev())
        .map_err(|_| fault(CommissionError::EncoderDomainInvalid))?;
    if record_excursion {
        let capture = workspace
            .excursion
            .as_mut()
            .ok_or_else(|| fault(CommissionError::MissingCaptureState))?;
        capture
            .observe(domain, encoder_raw, read_us)
            .map_err(|_| fault(CommissionError::EncoderDomainInvalid))?;
    }
    let delta = match domain.delta(
        domain.count(workspace.previous_encoder_raw),
        domain.count(encoder_raw),
    ) {
        Ok(delta) if delta.absolute_counts() < safety.quarter_turn_fault_counts => delta,
        _ => {
            let fault_report = safety.fault(OUTER_SAFETY_FAULT_QUARTER_TURN, 0, elapsed_us, 0, 0);
            return Err(safety_fault(
                fault_report,
                CommissionError::SafetyEnvelopeViolation,
            ));
        }
    };
    if let Some(checkpoint) = current_input_checkpoint {
        checkpoint_breakaway_operation(checkpoint);
    }
    let torque_demand = read_velocity_pid_input_abs(backend)
        .await
        .map_err(current_guard_fault)?;
    if torque_demand > workspace.current_observer.clamp_limit() {
        let fault_report = safety.fault(OUTER_SAFETY_FAULT_CURRENT, 0, elapsed_us, 0, 0);
        return Err(safety_fault(
            fault_report,
            CommissionError::SafetyEnvelopeViolation,
        ));
    }
    workspace.settle_displacement_counts = i32::from(workspace.settle_displacement_counts)
        .saturating_add(delta.signed_counts())
        .clamp(-2, 2) as i8;
    workspace.settle_min_displacement_counts = workspace
        .settle_min_displacement_counts
        .min(workspace.settle_displacement_counts);
    workspace.settle_max_displacement_counts = workspace
        .settle_max_displacement_counts
        .max(workspace.settle_displacement_counts);
    workspace
        .observations
        .push(delta.signed_counts(), elapsed_us as u16)
        .map_err(|_| fault(CommissionError::ZeroElapsedSample))?;
    workspace.previous_encoder_raw = encoder_raw;
    workspace.previous_read_us = read_us;
    Ok(read_us)
}

const MAX_IN_BAND_DISCOVERY_RUNGS: usize = MAX_SWEEP_RUNGS;

const EMPTY_DISCOVERY_RUNG_BOUNDS: DiscoveryRungNominationBounds = DiscoveryRungNominationBounds {
    p: PidPGain::ZERO,
    lower_percent: 0.0,
    upper_percent: 0.0,
    nominatable: false,
    step_peak_permille: 0,
};

/// Every discovery rung whose own response was contained in 70-80%, collected in
/// ascending rung order for [`nominate_confirmation_candidate_from_bounds`].
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct CollectedInBandRungs {
    rungs: [DiscoveryRungNominationBounds; MAX_IN_BAND_DISCOVERY_RUNGS],
    count: u8,
}

impl CollectedInBandRungs {
    const fn new() -> Self {
        Self {
            rungs: [EMPTY_DISCOVERY_RUNG_BOUNDS; MAX_IN_BAND_DISCOVERY_RUNGS],
            count: 0,
        }
    }

    pub(crate) fn as_slice(&self) -> &[DiscoveryRungNominationBounds] {
        &self.rungs[..usize::from(self.count)]
    }

    pub(crate) const fn count(&self) -> u8 {
        self.count
    }

    fn push(&mut self, bounds: DiscoveryRungNominationBounds) -> Result<(), CommissionError> {
        let slot = self
            .rungs
            .get_mut(usize::from(self.count))
            .ok_or(CommissionError::EvidenceBufferFull)?;
        *slot = bounds;
        self.count += 1;
        Ok(())
    }
}

#[derive(Debug, defmt::Format)]
struct DirectionAccumulator {
    classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    intervals: [Option<EquivalentTorqueTargetInterval>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    atoms: [Option<DiscoveryObservationAtom>; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    peaks: [u16; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
}

impl DirectionAccumulator {
    const fn new() -> Self {
        Self {
            classes: [VelocityObservationClass::InvalidEvidenceFault;
                CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            intervals: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            atoms: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
            peaks: [0; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        }
    }

    fn record(
        &mut self,
        slot: u8,
        classification: VelocityObservationClass,
        interval: Option<EquivalentTorqueTargetInterval>,
        atom: DiscoveryObservationAtom,
        step_peak_permille: u16,
    ) {
        let index = usize::from(slot);
        self.classes[index] = classification;
        self.intervals[index] = interval;
        self.atoms[index] = Some(atom);
        self.peaks[index] = step_peak_permille;
    }

    fn collected_mask(&self) -> u8 {
        self.atoms
            .iter()
            .enumerate()
            .filter(|(_, atom)| atom.is_some())
            .fold(0u8, |mask, (index, _)| mask | (1u8 << index))
    }

    fn consensus(&self) -> DirectionalConsensus {
        depth_three_consensus(self.collected_mask(), self.classes, self.intervals)
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingObservationAtom {
    mean_rate_q: i64,
    variance_of_mean_q2: u128,
    target_rate_q: i64,
}

impl PendingObservationAtom {
    fn with_direction(self, direction: VelocityDirection) -> DiscoveryObservationAtom {
        DiscoveryObservationAtom {
            mean_rate_q: self.mean_rate_q,
            variance_of_mean_q2: self.variance_of_mean_q2,
            target_rate_q: self.target_rate_q,
            direction,
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingDiscoveryRecord {
    classification: VelocityObservationClass,
    interval: Option<EquivalentTorqueTargetInterval>,
    atom: PendingObservationAtom,
    classifier_inputs: ObservationClassifierInputs,
}

#[derive(Debug, defmt::Format)]
pub(crate) struct AdditiveDiscoverySession {
    plan: AdditiveDiscoveryPlan,
    own_multiplier: f32,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    rung_index: u8,
    direction: VelocityDirection,
    slot: u8,
    forward: DirectionAccumulator,
    reverse: DirectionAccumulator,
    in_band: CollectedInBandRungs,
    pending_lower_rate_q: Option<u64>,
    pending_headroom_stop: bool,
    pending_step_peak_permille: u16,
    pending_record: Option<PendingDiscoveryRecord>,
    request: AdditiveDiscoveryRequest,
    floor_origin_clamped: bool,
    rung_zero_classification: Option<(BreakawayMotionClass, BreakawayMotionClass)>,
}

impl AdditiveDiscoverySession {
    #[allow(clippy::result_large_err)]
    pub(crate) fn new(
        request: AdditiveDiscoveryRequest,
        floor_origin_clamped: bool,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Result<Self, AdditiveDiscoveryOutcome> {
        let plan = plan_additive_discovery_ladder(request)
            .map_err(additive_discovery_plan_error_outcome)?;
        let own_multiplier = confidence_multiplier_95_df15(plan.confidence_family_size)
            .expect("plan_additive_discovery_ladder only returns audited family sizes");
        Ok(Self {
            plan,
            own_multiplier,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            rung_index: 0,
            direction: VelocityDirection::Forward,
            slot: 0,
            forward: DirectionAccumulator::new(),
            reverse: DirectionAccumulator::new(),
            in_band: CollectedInBandRungs::new(),
            pending_lower_rate_q: None,
            pending_headroom_stop: false,
            pending_step_peak_permille: 0,
            pending_record: None,
            request,
            floor_origin_clamped,
            rung_zero_classification: None,
        })
    }

    fn current_gain(&self) -> Result<PidPGain, CommissionError> {
        if self.rung_index >= self.plan.rung_count {
            return Err(CommissionError::RungIndexOutOfRange);
        }
        self.plan
            .gains
            .get(usize::from(self.rung_index))
            .copied()
            .ok_or(CommissionError::RungIndexOutOfRange)
    }
}

fn additive_discovery_plan_error_outcome(
    error: AdditiveLadderPlanError,
) -> AdditiveDiscoveryOutcome {
    match error {
        AdditiveLadderPlanError::InsufficientAdditiveSpan => {
            AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        }
        AdditiveLadderPlanError::AdditiveLadderResolutionBudgetExceeded => {
            AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded
        }
        AdditiveLadderPlanError::InvalidRequest
        | AdditiveLadderPlanError::Arithmetic
        | AdditiveLadderPlanError::ConfidenceFamilyTooLarge
        | AdditiveLadderPlanError::PrimitiveDurationExceeded
        | AdditiveLadderPlanError::WorkflowEnvelopeExceeded => {
            AdditiveDiscoveryOutcome::SafetyFault {
                error: CommissionError::CampaignPlanInvalid,
                safety: None,
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum AdditiveDiscoveryState {
    PlannedDiscovery,
    WriteGain,
    CommandTarget,
    Capture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    CommandZero,
    AnalyzeMoving {
        settle_deadline_us: u64,
        analysis_deadline_us: u64,
    },
    BeginSettleCapture {
        settle_deadline_us: u64,
        retry_available: bool,
    },
    SettleCapture {
        next_sample_us: u64,
        deadline_us: u64,
        retry_available: bool,
    },
    AnalyzeSettle {
        analysis_deadline_us: u64,
        retry_available: bool,
    },
    Recover(RungOriginRecoveryRuntime),
    Advance,
    StreamMargins {
        cursor: u8,
    },
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum AdditiveDiscoveryStep {
    ReadyNow,
    At(u64),
    Observation(CollectedObservation),
    Margin { collected_index: u8 },
    RestRejected(RecoveryRestRejectedInput),
    OriginMismatch(RecoveryOriginMismatchEvidence),
    Done(AdditiveDiscoveryOutcome),
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct CollectedObservation {
    pub(crate) atom: DiscoveryObservationAtom,
    pub(crate) rung_index: u8,
    pub(crate) slot_index: u8,
    pub(crate) classifier_inputs: ObservationClassifierInputs,
    pub(crate) stroke_min_displacement_counts: i32,
    pub(crate) stroke_max_displacement_counts: i32,
    pub(crate) capture_max_interval_us: u32,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct ObservationClassifierInputs {
    pub(crate) stationarity_code: u8,
    pub(crate) settled_rate_low_q: i64,
    pub(crate) settled_rate_high_q: i64,
    pub(crate) deficit_low_q: i64,
    pub(crate) deficit_high_q: i64,
    pub(crate) predicted_torque_target_abs: u16,
    pub(crate) moving_pid_output_peak: u16,
    pub(crate) usable_torque_limit: u16,
    pub(crate) hard_torque_limit: u16,
    pub(crate) disturbance_present: bool,
    pub(crate) disturbance_low_q16: i32,
    pub(crate) disturbance_high_q16: i32,
    pub(crate) drift_ratio_ppm: u32,
    pub(crate) post_fit_lag_one_state: u8,
}

fn stationarity_wire_code(result: StationarityResult) -> u8 {
    match result {
        StationarityResult::Usable(_) => 0,
        StationarityResult::Excluded(StationarityExclusion::NoBlockingPlateau) => 1,
        StationarityResult::Excluded(StationarityExclusion::TooFewDecorrelatedBlocks) => 2,
        StationarityResult::Excluded(StationarityExclusion::ResolvedSlope) => 3,
        StationarityResult::Excluded(StationarityExclusion::StructuredResidual) => 4,
        StationarityResult::Excluded(StationarityExclusion::Arithmetic) => 5,
        StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift) => 6,
    }
}

fn observation_classifier_inputs(
    input: &VelocityObservationInput,
    moving_pid_output_peak: u16,
) -> ObservationClassifierInputs {
    let (disturbance_present, disturbance_low_q16, disturbance_high_q16) = match input.disturbance {
        Some((_, interval)) => (true, interval.low().raw(), interval.high().raw()),
        None => (false, 0, 0),
    };
    ObservationClassifierInputs {
        stationarity_code: stationarity_wire_code(input.stationarity),
        settled_rate_low_q: input.settled_rate_low_q,
        settled_rate_high_q: input.settled_rate_high_q,
        deficit_low_q: input.deficit_low_q,
        deficit_high_q: input.deficit_high_q,
        predicted_torque_target_abs: input.predicted_torque_target_abs,
        moving_pid_output_peak,
        usable_torque_limit: input.usable_torque_limit,
        hard_torque_limit: input.hard_torque_limit,
        disturbance_present,
        disturbance_low_q16,
        disturbance_high_q16,
        drift_ratio_ppm: input.drift_ratio_ppm,
        post_fit_lag_one_state: input.post_fit_lag_one_state,
    }
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum AdditiveDiscoveryOutcome {
    TargetBandSelected(CollectedInBandRungs),
    TargetBandSkipped,
    CurrentHeadroomBeforeBand,
    InsufficientAdditiveSpan,
    AdditiveLadderResolutionBudgetExceeded,
    DiscoveryEvidenceExcluded {
        error: CommissionError,
    },
    SafetyFault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

fn discovery_fault(error: CommissionError) -> AdditiveDiscoveryStep {
    AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::SafetyFault {
        error,
        safety: None,
    })
}

fn discovery_safety_fault(
    safety: OuterSafetyFault,
    error: CommissionError,
) -> AdditiveDiscoveryStep {
    AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::SafetyFault {
        error,
        safety: Some(safety),
    })
}

fn discovery_current_guard_fault(error: VelocityCurrentGuardError) -> AdditiveDiscoveryStep {
    discovery_fault(match error {
        VelocityCurrentGuardError::InvalidLimit => CommissionError::InvalidCurrentLimit,
        VelocityCurrentGuardError::RegisterIo => CommissionError::Spi,
        VelocityCurrentGuardError::ActionableStatus(_)
        | VelocityCurrentGuardError::LimiterContact(_) => CommissionError::SafetyEnvelopeViolation,
    })
}

/// Record limiter contact observed at a discovery phase boundary.
///
/// The stroke drove the current loops or the voltage output into a configured
/// limit at this rung, so the ladder has reached its current authority. Finish
/// this rung's bookkeeping, then stop climbing and route the walk through the
/// current-headroom terminus so every in-band rung already collected is kept,
/// instead of discarding the campaign as an internal fault.
fn note_discovery_limiter_contact(session: &mut AdditiveDiscoverySession, status: u32) {
    defmt::warn!(
        "discovery rung={} limiter contact: status={:#x}, ending the ladder on current authority",
        session.rung_index,
        status,
    );
    session.pending_headroom_stop = true;
}

fn discovery_step_from_probe_fault(step: BreakawayProbeStep) -> AdditiveDiscoveryStep {
    match step {
        BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault { error, safety }) => {
            AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::SafetyFault { error, safety })
        }
        _ => unreachable!(
            "sample_one_tick only ever returns a SafetyFault-shaped Done, see its own docs"
        ),
    }
}

fn discovery_target_rate_q(
    target_velocity_rpm: TmcMechanicalRpm,
    counts_per_rev: u32,
) -> Result<i64, CommissionError> {
    i64::from(target_velocity_rpm.raw().unsigned_abs())
        .checked_mul(i64::from(counts_per_rev))
        .and_then(|value| value.checked_mul(VELOCITY_RATE_Q_SCALE))
        .map(|value| value / 60)
        .ok_or(CommissionError::TargetRateInvalid)
}

const DISCOVERY_CONTAINMENT_C_SIGMA: f32 = 1.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum RungBandVerdict {
    InBand,
    ConfidentlyAboveBand,
    Ambiguous,
}

fn direction_motion_class(
    classes: [VelocityObservationClass; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
) -> BreakawayMotionClass {
    let moved = classes.iter().any(|class| {
        matches!(
            class,
            VelocityObservationClass::Eligible | VelocityObservationClass::DeficitUnresolved
        )
    });
    if moved {
        BreakawayMotionClass::Moving
    } else {
        BreakawayMotionClass::Inert
    }
}

fn classify_rung_band(rung: &DiscoveryRungNominationInput, own_multiplier: f32) -> RungBandVerdict {
    match rung_directional_pooled_bounds_percent(
        rung,
        own_multiplier,
        DISCOVERY_CONTAINMENT_C_SIGMA,
    ) {
        Some(
            [
                (forward_lower, forward_upper),
                (reverse_lower, reverse_upper),
            ],
        ) if forward_lower >= f32::from(CONFIRMATION_BAND_LOWER_PERCENT)
            && forward_upper <= f32::from(CONFIRMATION_BAND_UPPER_PERCENT)
            && reverse_lower >= f32::from(CONFIRMATION_BAND_LOWER_PERCENT)
            && reverse_upper <= f32::from(CONFIRMATION_BAND_UPPER_PERCENT) =>
        {
            RungBandVerdict::InBand
        }
        Some([(forward_lower, _), (reverse_lower, _)])
            if forward_lower > f32::from(CONFIRMATION_BAND_UPPER_PERCENT)
                && reverse_lower > f32::from(CONFIRMATION_BAND_UPPER_PERCENT) =>
        {
            RungBandVerdict::ConfidentlyAboveBand
        }
        Some(_) | None => RungBandVerdict::Ambiguous,
    }
}

fn finish_discovery_walk(
    state: &mut AdditiveDiscoveryState,
    in_band_count: u8,
    empty_outcome: AdditiveDiscoveryOutcome,
) -> AdditiveDiscoveryStep {
    if in_band_count > 0 {
        *state = AdditiveDiscoveryState::StreamMargins { cursor: 0 };
        AdditiveDiscoveryStep::ReadyNow
    } else {
        AdditiveDiscoveryStep::Done(empty_outcome)
    }
}

pub(crate) async fn step_additive_discovery<B: CommissioningBackend>(
    state: &mut AdditiveDiscoveryState,
    workspace: &mut BreakawayProbeWorkspace,
    session: &mut AdditiveDiscoverySession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> AdditiveDiscoveryStep {
    match state {
        AdditiveDiscoveryState::PlannedDiscovery => {
            *state = AdditiveDiscoveryState::WriteGain;
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::WriteGain => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::GainProgramming,
            );
            if session.rung_index == 0 {
                let velocity_offset = backend
                    .read_register(PID_VELOCITY_OFFSET)
                    .await
                    .map(|value| value as i32)
                    .unwrap_or(i32::MIN);
                let velocity_actual = backend
                    .read_capture_register(PID_VELOCITY_ACTUAL)
                    .await
                    .map(|value| value as i32)
                    .unwrap_or(i32::MIN);
                defmt::info!(
                    "ladder start: velocity_offset={} velocity_actual={}",
                    velocity_offset,
                    velocity_actual,
                );
                if clear_velocity_offset(backend).await.is_err() {
                    return discovery_fault(CommissionError::Spi);
                }
            }
            let p = match session.current_gain() {
                Ok(p) => p,
                Err(error) => return discovery_fault(error),
            };
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_VELOCITY_ADVANCED_PI)
                    .await
                    .is_err()
                || clear_velocity_error_sum(backend).await.is_err()
            {
                return discovery_fault(CommissionError::Spi);
            }
            *state = AdditiveDiscoveryState::CommandTarget;
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::CommandTarget => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::TargetCommand,
            );
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return discovery_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.direction))
            {
                Ok(target) => target.raw(),
                Err(_) => return discovery_fault(CommissionError::TargetRateInvalid),
            };
            let hard_limit = safety.torque_cap;
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return discovery_current_guard_fault(error),
                };
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return discovery_current_guard_fault(error);
            }
            let clamp_limit = clamp_session.probe_limit();
            if let Err(error) = begin_velocity_current_primitive(
                &mut workspace.current_observer,
                backend,
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            )
            .await
            {
                return discovery_current_guard_fault(error);
            }
            if backend
                .write_register_verified(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                return discovery_fault(CommissionError::Spi);
            }
            if let Err(error) = safety.begin_velocity_step_primitive(backend.elapsed_us()) {
                return discovery_fault(error);
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            workspace.observations.clear();
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::Capture,
            );
            *state = AdditiveDiscoveryState::Capture {
                next_sample_us,
                deadline_us,
            };
            AdditiveDiscoveryStep::At(next_sample_us)
        }
        AdditiveDiscoveryState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return AdditiveDiscoveryStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return discovery_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_moving_output(value),
                Err(error) => return discovery_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                *state = AdditiveDiscoveryState::CommandZero;
                AdditiveDiscoveryStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        AdditiveDiscoveryStep::At(*next_sample_us)
                    }
                    Err(error) => discovery_fault(error),
                }
            }
        }
        AdditiveDiscoveryState::CommandZero => {
            session.pending_step_peak_permille = scored_peak_permille(
                safety.primitive_peak_milli_counts_per_s(),
                session.target_velocity_rpm.raw().unsigned_abs(),
                safety.counts_per_rev(),
                STEP_PEAK_ADMISSION_PERMILLE,
            );
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return discovery_fault(CommissionError::Spi);
            }
            safety.enter_settle_phase();
            match boundary {
                Ok(()) => {}
                Err(VelocityCurrentGuardError::LimiterContact(status)) => {
                    note_discovery_limiter_contact(session, status);
                }
                Err(error) => return discovery_current_guard_fault(error),
            }
            if clear_velocity_error_sum(backend).await.is_err() {
                return discovery_fault(CommissionError::Spi);
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = now_us.checked_add(u64::from(ZERO_SETTLE_US)) else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            workspace.analysis.reset_with_structured_diagnostics();
            *state = AdditiveDiscoveryState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return discovery_fault(CommissionError::AnalysisOverrun);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let (stationarity, drift_ratio_ppm) = match poll {
                Ok(Some((result, _rejection, drift_ratio_ppm))) => (result, drift_ratio_ppm),
                Ok(None) => return AdditiveDiscoveryStep::ReadyNow,
                Err(error) => return discovery_fault(error),
            };
            let p = match session.current_gain() {
                Ok(p) => p,
                Err(error) => return discovery_fault(error),
            };
            let input = match assemble_velocity_observation_input(VelocityObservationInputRequest {
                direction: session.direction,
                p,
                stationarity,
                rejected_post_fit: None,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: safety.torque_cap,
                initial_lead_in: false,
                confidence_family_size: session.plan.confidence_family_size,
                counts_per_rev: safety.counts_per_rev(),
            }) {
                Ok(input) => input,
                Err(error) => return discovery_fault(error),
            };
            let classification = classify_velocity_observation(input).classification;
            match classification {
                VelocityObservationClass::WrongDirectionFault => {
                    return discovery_fault(CommissionError::VelocityWrongDirection);
                }
                VelocityObservationClass::HardClipFault => {
                    return discovery_fault(CommissionError::SafetyEnvelopeViolation);
                }
                VelocityObservationClass::InvalidEvidenceFault => {
                    return AdditiveDiscoveryStep::Done(
                        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                            error: CommissionError::VelocityInvalidEvidence,
                        },
                    );
                }
                VelocityObservationClass::CurrentHeadroom => {
                    session.pending_headroom_stop = true;
                }
                VelocityObservationClass::Eligible
                | VelocityObservationClass::Inert
                | VelocityObservationClass::DeficitUnresolved
                | VelocityObservationClass::StationarityExcluded(_) => {}
            }
            session.pending_lower_rate_q = commanded_side_lower_rate_q(
                input.direction,
                input.settled_rate_low_q,
                input.settled_rate_high_q,
            );
            let variance_of_mean_q2 = match stationarity {
                StationarityResult::Usable(suffix) => suffix.selection.variance_of_mean_q2,
                StationarityResult::Excluded(_) => 0,
            };
            let target_rate_q =
                match discovery_target_rate_q(session.target_velocity_rpm, safety.counts_per_rev())
                {
                    Ok(value) => value,
                    Err(error) => return discovery_fault(error),
                };
            session.pending_record = Some(PendingDiscoveryRecord {
                classification,
                interval: input.disturbance.map(|(_, interval)| interval),
                atom: PendingObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2,
                    target_rate_q,
                },
                classifier_inputs: observation_classifier_inputs(
                    &input,
                    workspace.current_observer.moving_pid_output_peak(),
                ),
            });
            workspace.analysis.reset();
            workspace.observations.clear();
            *state = AdditiveDiscoveryState::BeginSettleCapture {
                settle_deadline_us: *settle_deadline_us,
                retry_available: true,
            };
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::BeginSettleCapture {
            settle_deadline_us,
            retry_available,
        } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return discovery_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return discovery_fault(CommissionError::AnalysisOverrun);
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return discovery_fault(CommissionError::InvalidSchedule);
            };
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = AdditiveDiscoveryState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
                retry_available: *retry_available,
            };
            AdditiveDiscoveryStep::At(next_sample_us)
        }
        AdditiveDiscoveryState::SettleCapture {
            next_sample_us,
            deadline_us,
            retry_available,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return AdditiveDiscoveryStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return discovery_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_zero_output(value),
                Err(error) => return discovery_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return discovery_fault(CommissionError::InvalidSchedule);
                };
                workspace
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        workspace.settle_min_displacement_counts,
                        workspace.settle_max_displacement_counts,
                    ));
                *state = AdditiveDiscoveryState::AnalyzeSettle {
                    analysis_deadline_us,
                    retry_available: *retry_available,
                };
                AdditiveDiscoveryStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        AdditiveDiscoveryStep::At(*next_sample_us)
                    }
                    Err(error) => discovery_fault(error),
                }
            }
        }
        AdditiveDiscoveryState::AnalyzeSettle {
            analysis_deadline_us,
            retry_available,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return discovery_fault(CommissionError::AnalysisOverrun);
            }
            let confirmation = match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending) => return AdditiveDiscoveryStep::ReadyNow,
                Ok(ScoredRestPoll::Diagnostic(decision)) => {
                    let velocity_offset = backend
                        .read_register(PID_VELOCITY_OFFSET)
                        .await
                        .map(|value| value as i32)
                        .unwrap_or(i32::MIN);
                    let velocity_actual = backend
                        .read_capture_register(PID_VELOCITY_ACTUAL)
                        .await
                        .map(|value| value as i32)
                        .unwrap_or(i32::MIN);
                    settle_rest_rejected_input(
                        decision,
                        session.rung_index,
                        *retry_available,
                        (velocity_offset, velocity_actual),
                        (
                            workspace.settle_displacement_counts,
                            workspace.settle_min_displacement_counts,
                            workspace.settle_max_displacement_counts,
                        ),
                    );
                    return AdditiveDiscoveryStep::ReadyNow;
                }
                Ok(ScoredRestPoll::Complete(confirmation)) => confirmation,
                Err(error) => return discovery_fault(error),
            };
            match finish_velocity_current_zero_phase(&mut workspace.current_observer, backend).await
            {
                Ok(()) => {}
                Err(VelocityCurrentGuardError::LimiterContact(status)) => {
                    note_discovery_limiter_contact(session, status);
                }
                Err(error) => return discovery_current_guard_fault(error),
            }
            if workspace
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .is_err()
            {
                return discovery_fault(CommissionError::InvalidCurrentLimit);
            }
            if !confirmation.confirmed_rest && *retry_available {
                workspace.analysis.reset();
                workspace.observations.clear();
                let Some(settle_deadline_us) =
                    backend.elapsed_us().checked_add(u64::from(ZERO_SETTLE_US))
                else {
                    return discovery_fault(CommissionError::InvalidSchedule);
                };
                *state = AdditiveDiscoveryState::BeginSettleCapture {
                    settle_deadline_us,
                    retry_available: false,
                };
                return AdditiveDiscoveryStep::ReadyNow;
            }
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                defmt::warn!(
                    "settle rest not confirmed at rung={} (capping ladder from collected evidence if any)",
                    session.rung_index,
                );
                return finish_discovery_walk(
                    state,
                    session.in_band.count(),
                    AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                        error: CommissionError::VelocityRestNotConfirmed,
                    },
                );
            }
            let recovery_plan = match plan_rung_origin_recovery(
                safety.budget,
                safety.counts_per_rev(),
                session.planned_velocity_mrev_s,
            ) {
                Ok(plan) => plan,
                Err(_) => return discovery_fault(CommissionError::RecoveryPlanFailed),
            };
            let p = match session.current_gain() {
                Ok(p) => p,
                Err(error) => return discovery_fault(error),
            };
            let hard_limit = safety.torque_cap;
            let clamp_limit = match PidTorqueFluxLimit::new(u32::from(hard_limit)) {
                Ok(limit) => limit,
                Err(_) => return discovery_fault(CommissionError::InvalidCurrentLimit),
            };
            workspace.current_observer.reset(
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            );
            let (lower_rate_q, authority_source) = match session.pending_lower_rate_q {
                Some(rate) => (Some(rate), RecoveryBindingSource::SettledRate),
                None => (None, RecoveryBindingSource::MissingRate),
            };
            let runtime = match RungOriginRecoveryRuntime::new(
                RungOriginRecoveryConfig {
                    stage: RecoveryStage::Proportional,
                    rung_index: session.rung_index,
                    p,
                    target_velocity_rpm: session.target_velocity_rpm,
                    planned_velocity_mrev_s: session.planned_velocity_mrev_s,
                    start_offset_counts: safety.unwrapped_position,
                    lower_rate_q,
                    authority_source,
                    current_contact_policy: RecoveryCurrentContactPolicy::StopImmediately,
                },
                recovery_plan,
            ) {
                Ok(runtime) => runtime,
                Err(error) => return discovery_fault(error),
            };
            *state = AdditiveDiscoveryState::Recover(runtime);
            AdditiveDiscoveryStep::ReadyNow
        }
        AdditiveDiscoveryState::Recover(runtime) => {
            let mut resources = RungOriginRecoveryResources {
                safety,
                observations: &mut workspace.observations,
                blocker: &mut workspace.blocker,
                analysis: &mut workspace.analysis,
                settle_displacement_counts: &mut workspace.settle_displacement_counts,
                settle_min_displacement_counts: &mut workspace.settle_min_displacement_counts,
                settle_max_displacement_counts: &mut workspace.settle_max_displacement_counts,
                current_observer: &mut workspace.current_observer,
            };
            let step = step_rung_origin_recovery(runtime, &mut resources, backend).await;
            let settle = (
                *resources.settle_displacement_counts,
                *resources.settle_min_displacement_counts,
                *resources.settle_max_displacement_counts,
            );
            let p_raw = runtime.p_raw();
            match step {
                RungOriginRecoveryStep::ReadyNow => AdditiveDiscoveryStep::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => AdditiveDiscoveryStep::At(deadline_us),
                RungOriginRecoveryStep::RestSelection {
                    decision,
                    stage,
                    rung_index,
                } => match rest_rejected_input(
                    decision,
                    stage,
                    rung_index,
                    session.slot,
                    session.direction,
                    p_raw,
                    settle,
                ) {
                    Some(input) => AdditiveDiscoveryStep::RestRejected(input),
                    None => AdditiveDiscoveryStep::ReadyNow,
                },
                RungOriginRecoveryStep::RestAnalysis(_) => AdditiveDiscoveryStep::ReadyNow,
                RungOriginRecoveryStep::OriginMismatch(evidence) => {
                    AdditiveDiscoveryStep::OriginMismatch(evidence)
                }
                RungOriginRecoveryStep::Finished(_) | RungOriginRecoveryStep::Contained(_) => {
                    *state = AdditiveDiscoveryState::Advance;
                    match session.pending_record.as_ref() {
                        Some(record) => AdditiveDiscoveryStep::Observation(CollectedObservation {
                            atom: record.atom.with_direction(session.direction),
                            rung_index: session.rung_index,
                            slot_index: session.slot,
                            classifier_inputs: record.classifier_inputs,
                            stroke_min_displacement_counts: 0,
                            stroke_max_displacement_counts: 0,
                            capture_max_interval_us: 0,
                        }),
                        None => AdditiveDiscoveryStep::ReadyNow,
                    }
                }
                RungOriginRecoveryStep::RestUnconfirmed => finish_discovery_walk(
                    state,
                    session.in_band.count(),
                    AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded {
                        error: CommissionError::VelocityRestNotConfirmed,
                    },
                ),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: Some(fault_report),
                } => discovery_safety_fault(fault_report, error),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => discovery_fault(error),
            }
        }
        AdditiveDiscoveryState::Advance => {
            let record = session
                .pending_record
                .take()
                .expect("classified before recovery started");
            let atom = record.atom.with_direction(session.direction);
            let step_peak_permille = session.pending_step_peak_permille;
            match session.direction {
                VelocityDirection::Forward => session.forward.record(
                    session.slot,
                    record.classification,
                    record.interval,
                    atom,
                    step_peak_permille,
                ),
                VelocityDirection::Reverse => session.reverse.record(
                    session.slot,
                    record.classification,
                    record.interval,
                    atom,
                    step_peak_permille,
                ),
            }

            if session.pending_headroom_stop {
                return finish_discovery_walk(
                    state,
                    session.in_band.count(),
                    AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand,
                );
            }

            let Some(next_slot) = session.slot.checked_add(1) else {
                return discovery_fault(CommissionError::RungIndexOutOfRange);
            };
            if usize::from(next_slot) < CONSENSUS_OBSERVATIONS_PER_DIRECTION {
                session.slot = next_slot;
                *state = AdditiveDiscoveryState::WriteGain;
                return AdditiveDiscoveryStep::ReadyNow;
            }

            match session.direction {
                VelocityDirection::Forward => {
                    session.direction = VelocityDirection::Reverse;
                    session.slot = 0;
                    *state = AdditiveDiscoveryState::WriteGain;
                    AdditiveDiscoveryStep::ReadyNow
                }
                VelocityDirection::Reverse => {
                    let p = match session.current_gain() {
                        Ok(p) => p,
                        Err(error) => return discovery_fault(error),
                    };
                    let rung_input = DiscoveryRungNominationInput {
                        p,
                        forward_consensus: session.forward.consensus(),
                        forward_observations: session.forward.atoms,
                        reverse_consensus: session.reverse.consensus(),
                        reverse_observations: session.reverse.atoms,
                    };
                    let step_peak_permille = rung_step_peak_permille(
                        rung_input.forward_consensus.included_mask,
                        &session.forward.peaks,
                        rung_input.reverse_consensus.included_mask,
                        &session.reverse.peaks,
                    );
                    let verdict = classify_rung_band(&rung_input, session.own_multiplier);
                    let [(fwd_lo, fwd_hi), (rev_lo, rev_hi)] =
                        rung_directional_pooled_bounds_percent(
                            &rung_input,
                            session.own_multiplier,
                            DISCOVERY_CONTAINMENT_C_SIGMA,
                        )
                        .unwrap_or([(f32::NAN, f32::NAN), (f32::NAN, f32::NAN)]);
                    defmt::info!(
                        "discovery rung classified: rung={} p={} verdict={} fwd_pct=[{},{}] rev_pct=[{},{}] step_peak_permille={}",
                        session.rung_index,
                        p.raw(),
                        verdict,
                        fwd_lo,
                        fwd_hi,
                        rev_lo,
                        rev_hi,
                        step_peak_permille,
                    );
                    if session.rung_index == 0 {
                        session.rung_zero_classification = Some((
                            direction_motion_class(session.forward.classes),
                            direction_motion_class(session.reverse.classes),
                        ));
                    }
                    if verdict == RungBandVerdict::InBand {
                        let bounds = rung_nomination_bounds(&rung_input, step_peak_permille);
                        if session.in_band.push(bounds).is_err() {
                            return discovery_fault(CommissionError::EvidenceBufferFull);
                        }
                    }
                    if verdict == RungBandVerdict::ConfidentlyAboveBand {
                        return finish_discovery_walk(
                            state,
                            session.in_band.count(),
                            AdditiveDiscoveryOutcome::TargetBandSkipped,
                        );
                    }

                    session.forward = DirectionAccumulator::new();
                    session.reverse = DirectionAccumulator::new();

                    let Some(next_rung) = session.rung_index.checked_add(1) else {
                        return discovery_fault(CommissionError::RungIndexOutOfRange);
                    };
                    if usize::from(next_rung) >= usize::from(session.plan.rung_count) {
                        return finish_discovery_walk(
                            state,
                            session.in_band.count(),
                            AdditiveDiscoveryOutcome::TargetBandSkipped,
                        );
                    }
                    session.rung_index = next_rung;
                    session.direction = VelocityDirection::Forward;
                    session.slot = 0;
                    *state = AdditiveDiscoveryState::WriteGain;
                    AdditiveDiscoveryStep::ReadyNow
                }
            }
        }
        AdditiveDiscoveryState::StreamMargins { cursor } => {
            let index = *cursor;
            if usize::from(index) < usize::from(session.in_band.count()) {
                *cursor = index.saturating_add(1);
                AdditiveDiscoveryStep::Margin {
                    collected_index: index,
                }
            } else {
                AdditiveDiscoveryStep::Done(AdditiveDiscoveryOutcome::TargetBandSelected(
                    session.in_band,
                ))
            }
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct PendingConfirmationRecord {
    classification: VelocityObservationClass,
    interval: Option<EquivalentTorqueTargetInterval>,
    atom: PendingObservationAtom,
    classifier_inputs: ObservationClassifierInputs,
}

/// Nomination evidence carried on the campaign terminal.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, defmt::Format)]
pub(crate) struct NominationEvidence {
    /// Step velocity peak of the nominated rung, in permille of target; `0` when none was
    /// nominated or it was unscored.
    pub nominated_step_peak_permille: u16,
    /// Rungs excluded from nomination for exceeding the step peak bound.
    pub overshoot_excluded_rungs: u8,
}

#[derive(Debug, defmt::Format)]
pub(crate) enum BandConfirmationOutcome {
    Accepted {
        acceptance: BandConfirmationAcceptance,
        plan: BandConfirmationPlan,
    },
    TargetBandUnconfirmableFromDiscovery,
    TargetBandNoTransitionCapableCandidate,
    /// Every band-admissible, transition-capable rung exceeded the step peak bound.
    TargetBandNoDampedCandidate {
        /// Rungs excluded by the step peak bound.
        excluded: u8,
    },
    /// Every band-admissible, transition-capable, damped rung sat at or above the nomination
    /// ceiling.
    TargetBandNoRungBelowCeiling {
        /// The nomination ceiling that excluded every remaining rung.
        ceiling: u16,
        /// Rungs excluded by the step peak bound.
        excluded: u8,
    },
    TargetBandNotReachedAtAuthority,
    TargetBandConfirmationInconclusive(BandConfirmationFailureCause),
    ConfirmationEvidenceExcluded,
    SafetyFault {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BandConfirmationEntry {
    Ready(BandConfirmationSession),
    Done(BandConfirmationOutcome),
}

pub(crate) fn begin_band_confirmation(
    discovery_outcome: AdditiveDiscoveryOutcome,
    discovery_plan_digest: u64,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    transition_ceiling: u16,
    nomination_p_below: u16,
) -> BandConfirmationEntry {
    let rungs = match discovery_outcome {
        AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand => {
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandNotReachedAtAuthority,
            );
        }
        AdditiveDiscoveryOutcome::TargetBandSelected(rungs) => rungs,
        AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded
        | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. } => {
            return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
                error: CommissionError::DiscoveryBandNotSelected,
                safety: None,
            });
        }
        AdditiveDiscoveryOutcome::SafetyFault { error, safety } => {
            return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
                error,
                safety,
            });
        }
    };
    let candidate = match nominate_transition_capable_candidate_from_bounds(
        rungs.as_slice(),
        transition_ceiling,
        nomination_p_below,
    ) {
        Ok(candidate) => candidate,
        Err(NominationRejection::Unconfirmable) => {
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery,
            );
        }
        Err(NominationRejection::NoTransitionCapableCandidate) => {
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandNoTransitionCapableCandidate,
            );
        }
        Err(NominationRejection::NoDampedCandidate { excluded }) => {
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandNoDampedCandidate { excluded },
            );
        }
        Err(NominationRejection::NoRungBelowCeiling { ceiling, excluded }) => {
            defmt::info!("nomination: no rung below ceiling p={}", ceiling);
            return BandConfirmationEntry::Done(
                BandConfirmationOutcome::TargetBandNoRungBelowCeiling { ceiling, excluded },
            );
        }
    };
    let plan = match plan_band_confirmation(discovery_plan_digest, candidate.p) {
        Ok(plan) => plan,
        Err(_) => {
            return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
                error: CommissionError::CampaignPlanInvalid,
                safety: None,
            });
        }
    };
    if validate_fixed_contract(&plan).is_err() {
        return BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
            error: CommissionError::PlanContractChanged,
            safety: None,
        });
    }
    let mut session = BandConfirmationSession::new(
        plan,
        candidate.margin_percent,
        target_velocity_rpm,
        planned_velocity_mrev_s,
    );
    session.nomination_evidence = NominationEvidence {
        nominated_step_peak_permille: candidate.step_peak_permille,
        overshoot_excluded_rungs: candidate.overshoot_excluded_rungs,
    };
    BandConfirmationEntry::Ready(session)
}

#[derive(Debug, defmt::Format)]
pub(crate) struct BandConfirmationSession {
    plan: BandConfirmationPlan,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    stroke_index: u8,
    forward: DirectionAccumulator,
    reverse: DirectionAccumulator,
    pending_lower_rate_q: Option<u64>,
    pending_record: Option<PendingConfirmationRecord>,
    nominated_margin_percent: f32,
    nomination_evidence: NominationEvidence,
}

impl BandConfirmationSession {
    fn new(
        plan: BandConfirmationPlan,
        nominated_margin_percent: f32,
        target_velocity_rpm: TmcMechanicalRpm,
        planned_velocity_mrev_s: u32,
    ) -> Self {
        Self {
            plan,
            target_velocity_rpm,
            planned_velocity_mrev_s,
            stroke_index: 0,
            forward: DirectionAccumulator::new(),
            reverse: DirectionAccumulator::new(),
            pending_lower_rate_q: None,
            pending_record: None,
            nominated_margin_percent,
            nomination_evidence: NominationEvidence::default(),
        }
    }

    fn current_direction(&self) -> VelocityDirection {
        CONFIRMATION_SLOT_ORDER[usize::from(self.stroke_index)]
    }

    fn slot_within_direction(&self) -> u8 {
        self.stroke_index % 4
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum BandConfirmationState {
    PlannedConfirmation,
    WriteGain,
    CommandTarget,
    Capture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    CommandZero,
    AnalyzeMoving {
        settle_deadline_us: u64,
        analysis_deadline_us: u64,
    },
    BeginSettleCapture {
        settle_deadline_us: u64,
    },
    SettleCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    AnalyzeSettle {
        analysis_deadline_us: u64,
    },
    Recover(RungOriginRecoveryRuntime),
    Advance,
}

#[derive(Debug, defmt::Format)]
pub(crate) enum BandConfirmationStep {
    ReadyNow,
    At(u64),
    Observation(CollectedObservation),
    RestRejected(RecoveryRestRejectedInput),
    OriginMismatch(RecoveryOriginMismatchEvidence),
    Done(BandConfirmationOutcome),
}

fn confirmation_fault(error: CommissionError) -> BandConfirmationStep {
    BandConfirmationStep::Done(BandConfirmationOutcome::SafetyFault {
        error,
        safety: None,
    })
}

fn confirmation_safety_fault(
    safety: OuterSafetyFault,
    error: CommissionError,
) -> BandConfirmationStep {
    BandConfirmationStep::Done(BandConfirmationOutcome::SafetyFault {
        error,
        safety: Some(safety),
    })
}

fn confirmation_current_guard_fault(error: VelocityCurrentGuardError) -> BandConfirmationStep {
    match error {
        VelocityCurrentGuardError::InvalidLimit => {
            confirmation_fault(CommissionError::InvalidCurrentLimit)
        }
        VelocityCurrentGuardError::RegisterIo => confirmation_fault(CommissionError::Spi),
        VelocityCurrentGuardError::ActionableStatus(_) => {
            confirmation_fault(CommissionError::SafetyEnvelopeViolation)
        }
        // A held-out stroke that drives the current loops or the voltage output
        // into a limit cannot confirm the band at the nominated gain. Report the
        // same current-authority verdict a headroom-excluded observation would,
        // so the campaign ends inconclusive instead of as an internal fault.
        VelocityCurrentGuardError::LimiterContact(status) => {
            defmt::warn!(
                "confirmation limiter contact: status={:#x}, ending on current authority",
                status,
            );
            BandConfirmationStep::Done(BandConfirmationOutcome::TargetBandConfirmationInconclusive(
                super::band_confirmation::BandConfirmationFailureCause::CurrentAuthority,
            ))
        }
    }
}

fn confirmation_step_from_probe_fault(step: BreakawayProbeStep) -> BandConfirmationStep {
    match step {
        BreakawayProbeStep::Done(BreakawayProbeOutcome::SafetyFault { error, safety }) => {
            BandConfirmationStep::Done(BandConfirmationOutcome::SafetyFault { error, safety })
        }
        _ => unreachable!(
            "sample_one_tick only ever returns a SafetyFault-shaped Done, see its own docs"
        ),
    }
}

fn confirmation_evaluation_outcome(
    result: Result<BandConfirmationAcceptance, TargetBandConfirmationInconclusive>,
    plan: BandConfirmationPlan,
) -> BandConfirmationOutcome {
    match result {
        Ok(acceptance) => BandConfirmationOutcome::Accepted { acceptance, plan },
        Err(TargetBandConfirmationInconclusive(cause)) => {
            BandConfirmationOutcome::TargetBandConfirmationInconclusive(cause)
        }
    }
}

fn evaluate_confirmation_session(session: &BandConfirmationSession) -> BandConfirmationOutcome {
    let forward = ConfirmationDirectionalInput {
        consensus: session.forward.consensus(),
        classes: session.forward.classes,
        observations: session.forward.atoms,
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: session.reverse.consensus(),
        classes: session.reverse.classes,
        observations: session.reverse.atoms,
    };
    confirmation_evaluation_outcome(
        evaluate_band_confirmation(session.plan.nominated_p, forward, reverse),
        session.plan,
    )
}

pub(crate) async fn step_band_confirmation<B: CommissioningBackend>(
    state: &mut BandConfirmationState,
    workspace: &mut BreakawayProbeWorkspace,
    session: &mut BandConfirmationSession,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> BandConfirmationStep {
    match state {
        BandConfirmationState::PlannedConfirmation => {
            *state = BandConfirmationState::WriteGain;
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::WriteGain => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::GainProgramming,
            );
            let p = session.plan.nominated_p;
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
                || backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_VELOCITY_ADVANCED_PI)
                    .await
                    .is_err()
                || clear_velocity_error_sum(backend).await.is_err()
            {
                return confirmation_fault(CommissionError::Spi);
            }
            *state = BandConfirmationState::CommandTarget;
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::CommandTarget => {
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::TargetCommand,
            );
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return confirmation_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            let target = match session
                .target_velocity_rpm
                .with_direction(direction_sign(session.current_direction()))
            {
                Ok(target) => target.raw(),
                Err(_) => return confirmation_fault(CommissionError::TargetRateInvalid),
            };
            let hard_limit = safety.torque_cap;
            let mut clamp_session =
                match VelocityCurrentClampSession::try_new(u32::from(hard_limit)) {
                    Ok(session) => session,
                    Err(error) => return confirmation_current_guard_fault(error),
                };
            if let Err(error) = clamp_session.install_probe_limit(backend).await {
                return confirmation_current_guard_fault(error);
            }
            let clamp_limit = clamp_session.probe_limit();
            if let Err(error) = begin_velocity_current_primitive(
                &mut workspace.current_observer,
                backend,
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            )
            .await
            {
                return confirmation_current_guard_fault(error);
            }
            if backend
                .write_register_verified(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                return confirmation_fault(CommissionError::Spi);
            }
            if let Err(error) = safety.begin_velocity_step_primitive(backend.elapsed_us()) {
                return confirmation_fault(error);
            }
            let started_us = backend.elapsed_us();
            let Some(next_sample_us) =
                started_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            let Some(deadline_us) = started_us.checked_add(u64::from(MOVING_STROKE_US)) else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            workspace.observations.clear();
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            crate::diagnostic::checkpoint_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::Capture,
            );
            *state = BandConfirmationState::Capture {
                next_sample_us,
                deadline_us,
            };
            BandConfirmationStep::At(next_sample_us)
        }
        BandConfirmationState::Capture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BandConfirmationStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return confirmation_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_moving_output(value),
                Err(error) => return confirmation_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                *state = BandConfirmationState::CommandZero;
                BandConfirmationStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BandConfirmationStep::At(*next_sample_us)
                    }
                    Err(error) => confirmation_fault(error),
                }
            }
        }
        BandConfirmationState::CommandZero => {
            let boundary =
                finish_velocity_current_moving_phase(&mut workspace.current_observer, backend)
                    .await;
            if backend
                .write_register(PID_VELOCITY_TARGET, 0)
                .await
                .is_err()
            {
                return confirmation_fault(CommissionError::Spi);
            }
            safety.enter_settle_phase();
            if let Err(error) = boundary {
                return confirmation_current_guard_fault(error);
            }
            if clear_velocity_error_sum(backend).await.is_err() {
                return confirmation_fault(CommissionError::Spi);
            }
            let now_us = backend.elapsed_us();
            let Some(settle_deadline_us) = now_us.checked_add(u64::from(ZERO_SETTLE_US)) else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            let Some(analysis_deadline_us) =
                now_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
            else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            workspace.analysis.reset_with_structured_diagnostics();
            *state = BandConfirmationState::AnalyzeMoving {
                settle_deadline_us,
                analysis_deadline_us,
            };
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::AnalyzeMoving {
            settle_deadline_us,
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return confirmation_fault(CommissionError::AnalysisOverrun);
            }
            let poll = poll_stationarity(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            );
            let (stationarity, drift_ratio_ppm) = match poll {
                Ok(Some((result, _rejection, drift_ratio_ppm))) => (result, drift_ratio_ppm),
                Ok(None) => return BandConfirmationStep::ReadyNow,
                Err(error) => return confirmation_fault(error),
            };
            let p = session.plan.nominated_p;
            let input = match assemble_velocity_observation_input(VelocityObservationInputRequest {
                direction: session.current_direction(),
                p,
                stationarity,
                rejected_post_fit: None,
                drift_ratio_ppm,
                moving_pid_output_peak: workspace.current_observer.moving_pid_output_peak(),
                target_velocity_rpm: session.target_velocity_rpm,
                usable_torque_limit: workspace.current_observer.clamp_limit(),
                hard_torque_limit: safety.torque_cap,
                initial_lead_in: false,
                confidence_family_size: CONFIRMATION_FAMILY_SIZE,
                counts_per_rev: safety.counts_per_rev(),
            }) {
                Ok(input) => input,
                Err(error) => return confirmation_fault(error),
            };
            let classification = classify_velocity_observation(input).classification;
            match classification {
                VelocityObservationClass::WrongDirectionFault => {
                    return confirmation_fault(CommissionError::VelocityWrongDirection);
                }
                VelocityObservationClass::HardClipFault => {
                    return confirmation_fault(CommissionError::SafetyEnvelopeViolation);
                }
                VelocityObservationClass::InvalidEvidenceFault => {
                    return BandConfirmationStep::Done(
                        BandConfirmationOutcome::ConfirmationEvidenceExcluded,
                    );
                }
                VelocityObservationClass::Eligible
                | VelocityObservationClass::Inert
                | VelocityObservationClass::DeficitUnresolved
                | VelocityObservationClass::CurrentHeadroom
                | VelocityObservationClass::StationarityExcluded(_) => {}
            }
            session.pending_lower_rate_q = commanded_side_lower_rate_q(
                input.direction,
                input.settled_rate_low_q,
                input.settled_rate_high_q,
            );
            let variance_of_mean_q2 = match stationarity {
                StationarityResult::Usable(suffix) => suffix.selection.variance_of_mean_q2,
                StationarityResult::Excluded(_) => 0,
            };
            let target_rate_q =
                match discovery_target_rate_q(session.target_velocity_rpm, safety.counts_per_rev())
                {
                    Ok(value) => value,
                    Err(error) => return confirmation_fault(error),
                };
            session.pending_record = Some(PendingConfirmationRecord {
                classification,
                interval: input.disturbance.map(|(_, interval)| interval),
                atom: PendingObservationAtom {
                    mean_rate_q: input.settled_rate_mean_q,
                    variance_of_mean_q2,
                    target_rate_q,
                },
                classifier_inputs: observation_classifier_inputs(
                    &input,
                    workspace.current_observer.moving_pid_output_peak(),
                ),
            });
            workspace.analysis.reset();
            workspace.observations.clear();
            *state = BandConfirmationState::BeginSettleCapture {
                settle_deadline_us: *settle_deadline_us,
            };
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::BeginSettleCapture { settle_deadline_us } => {
            let encoder_raw = match backend.read_capture_register(ABN_DECODER_COUNT).await {
                Ok(value) => value,
                Err(_) => return confirmation_fault(CommissionError::Spi),
            };
            let baseline_us = backend.elapsed_us();
            if baseline_us
                .checked_add(u64::from(MIN_SETTLE_CAPTURE_US))
                .is_none_or(|minimum_end_us| minimum_end_us > *settle_deadline_us)
            {
                return confirmation_fault(CommissionError::AnalysisOverrun);
            }
            let Some(next_sample_us) =
                baseline_us.checked_add(u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US))
            else {
                return confirmation_fault(CommissionError::InvalidSchedule);
            };
            workspace.previous_encoder_raw = encoder_raw;
            workspace.previous_read_us = baseline_us;
            workspace.settle_displacement_counts = 0;
            workspace.settle_min_displacement_counts = 0;
            workspace.settle_max_displacement_counts = 0;
            *state = BandConfirmationState::SettleCapture {
                next_sample_us,
                deadline_us: *settle_deadline_us,
            };
            BandConfirmationStep::At(next_sample_us)
        }
        BandConfirmationState::SettleCapture {
            next_sample_us,
            deadline_us,
        } => {
            let entry_us = backend.elapsed_us();
            if entry_us < *next_sample_us {
                return BandConfirmationStep::At(*next_sample_us);
            }
            let read_us = match sample_one_tick(workspace, safety, backend, None, None, false).await
            {
                Ok(read_us) => read_us,
                Err(step) => return confirmation_step_from_probe_fault(step),
            };
            match read_velocity_pid_output_abs(backend).await {
                Ok(value) => workspace.current_observer.observe_zero_output(value),
                Err(error) => return confirmation_current_guard_fault(error),
            }
            if read_us >= *deadline_us {
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return confirmation_fault(CommissionError::InvalidSchedule);
                };
                workspace
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        workspace.settle_min_displacement_counts,
                        workspace.settle_max_displacement_counts,
                    ));
                *state = BandConfirmationState::AnalyzeSettle {
                    analysis_deadline_us,
                };
                BandConfirmationStep::ReadyNow
            } else {
                match next_grid_deadline(
                    *next_sample_us,
                    u64::from(VELOCITY_SWEEP_SAMPLE_PERIOD_US),
                    read_us,
                ) {
                    Ok(next) => {
                        *next_sample_us = next.min(*deadline_us);
                        BandConfirmationStep::At(*next_sample_us)
                    }
                    Err(error) => confirmation_fault(error),
                }
            }
        }
        BandConfirmationState::AnalyzeSettle {
            analysis_deadline_us,
        } => {
            if backend.elapsed_us() >= *analysis_deadline_us {
                return confirmation_fault(CommissionError::AnalysisOverrun);
            }
            let confirmation = match poll_scored_rest_confirmation(
                &mut workspace.analysis,
                &workspace.observations,
                &mut workspace.blocker,
            ) {
                Ok(ScoredRestPoll::Pending | ScoredRestPoll::Diagnostic(_)) => {
                    return BandConfirmationStep::ReadyNow;
                }
                Ok(ScoredRestPoll::Complete(confirmation)) => confirmation,
                Err(error) => return confirmation_fault(error),
            };
            if let Err(error) =
                finish_velocity_current_zero_phase(&mut workspace.current_observer, backend).await
            {
                return confirmation_current_guard_fault(error);
            }
            if workspace
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .is_err()
            {
                return confirmation_fault(CommissionError::InvalidCurrentLimit);
            }
            workspace.analysis.reset();
            workspace.observations.clear();
            safety.end_primitive();
            if !confirmation.confirmed_rest {
                return BandConfirmationStep::Done(
                    BandConfirmationOutcome::ConfirmationEvidenceExcluded,
                );
            }
            let recovery_plan = match plan_rung_origin_recovery(
                safety.budget,
                safety.counts_per_rev(),
                session.planned_velocity_mrev_s,
            ) {
                Ok(plan) => plan,
                Err(_) => return confirmation_fault(CommissionError::RecoveryPlanFailed),
            };
            let p = session.plan.nominated_p;
            let hard_limit = safety.torque_cap;
            let clamp_limit = match PidTorqueFluxLimit::new(u32::from(hard_limit)) {
                Ok(limit) => limit,
                Err(_) => return confirmation_fault(CommissionError::InvalidCurrentLimit),
            };
            workspace.current_observer.reset(
                clamp_limit,
                VelocityLimitEvidenceCapability::BestEffortPolling,
            );
            let (lower_rate_q, authority_source) = match session.pending_lower_rate_q {
                Some(rate) => (Some(rate), RecoveryBindingSource::SettledRate),
                None => (None, RecoveryBindingSource::MissingRate),
            };
            let runtime = match RungOriginRecoveryRuntime::new(
                RungOriginRecoveryConfig {
                    stage: RecoveryStage::Proportional,
                    rung_index: session.stroke_index,
                    p,
                    target_velocity_rpm: session.target_velocity_rpm,
                    planned_velocity_mrev_s: session.planned_velocity_mrev_s,
                    start_offset_counts: safety.unwrapped_position,
                    lower_rate_q,
                    authority_source,
                    current_contact_policy: RecoveryCurrentContactPolicy::StopImmediately,
                },
                recovery_plan,
            ) {
                Ok(runtime) => runtime,
                Err(error) => return confirmation_fault(error),
            };
            *state = BandConfirmationState::Recover(runtime);
            BandConfirmationStep::ReadyNow
        }
        BandConfirmationState::Recover(runtime) => {
            let mut resources = RungOriginRecoveryResources {
                safety,
                observations: &mut workspace.observations,
                blocker: &mut workspace.blocker,
                analysis: &mut workspace.analysis,
                settle_displacement_counts: &mut workspace.settle_displacement_counts,
                settle_min_displacement_counts: &mut workspace.settle_min_displacement_counts,
                settle_max_displacement_counts: &mut workspace.settle_max_displacement_counts,
                current_observer: &mut workspace.current_observer,
            };
            let step = step_rung_origin_recovery(runtime, &mut resources, backend).await;
            let settle = (
                *resources.settle_displacement_counts,
                *resources.settle_min_displacement_counts,
                *resources.settle_max_displacement_counts,
            );
            let p_raw = runtime.p_raw();
            match step {
                RungOriginRecoveryStep::ReadyNow => BandConfirmationStep::ReadyNow,
                RungOriginRecoveryStep::At(deadline_us) => BandConfirmationStep::At(deadline_us),
                RungOriginRecoveryStep::RestSelection {
                    decision,
                    stage,
                    rung_index,
                } => match rest_rejected_input(
                    decision,
                    stage,
                    rung_index,
                    session.slot_within_direction(),
                    session.current_direction(),
                    p_raw,
                    settle,
                ) {
                    Some(input) => BandConfirmationStep::RestRejected(input),
                    None => BandConfirmationStep::ReadyNow,
                },
                RungOriginRecoveryStep::RestAnalysis(_) => BandConfirmationStep::ReadyNow,
                RungOriginRecoveryStep::OriginMismatch(evidence) => {
                    BandConfirmationStep::OriginMismatch(evidence)
                }
                RungOriginRecoveryStep::Finished(_) | RungOriginRecoveryStep::Contained(_) => {
                    let slot_index = match session.current_direction() {
                        VelocityDirection::Forward => session.slot_within_direction(),
                        VelocityDirection::Reverse => session.slot_within_direction() + 4,
                    };
                    *state = BandConfirmationState::Advance;
                    match session.pending_record.as_ref() {
                        Some(record) => BandConfirmationStep::Observation(CollectedObservation {
                            atom: record.atom.with_direction(session.current_direction()),
                            rung_index: 0,
                            slot_index,
                            classifier_inputs: record.classifier_inputs,
                            stroke_min_displacement_counts: 0,
                            stroke_max_displacement_counts: 0,
                            capture_max_interval_us: 0,
                        }),
                        None => BandConfirmationStep::ReadyNow,
                    }
                }
                RungOriginRecoveryStep::RestUnconfirmed => BandConfirmationStep::Done(
                    BandConfirmationOutcome::ConfirmationEvidenceExcluded,
                ),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: Some(fault_report),
                } => confirmation_safety_fault(fault_report, error),
                RungOriginRecoveryStep::Fault {
                    error,
                    safety: None,
                } => confirmation_fault(error),
            }
        }
        BandConfirmationState::Advance => {
            let record = session
                .pending_record
                .take()
                .expect("classified before recovery started");
            let atom = record.atom.with_direction(session.current_direction());
            match session.current_direction() {
                VelocityDirection::Forward => session.forward.record(
                    session.slot_within_direction(),
                    record.classification,
                    record.interval,
                    atom,
                    0,
                ),
                VelocityDirection::Reverse => session.reverse.record(
                    session.slot_within_direction(),
                    record.classification,
                    record.interval,
                    atom,
                    0,
                ),
            }

            let Some(next_stroke) = session.stroke_index.checked_add(1) else {
                return confirmation_fault(CommissionError::RungIndexOutOfRange);
            };
            if usize::from(next_stroke) < CONFIRMATION_SLOT_ORDER.len() {
                session.stroke_index = next_stroke;
                *state = BandConfirmationState::WriteGain;
                return BandConfirmationStep::ReadyNow;
            }

            BandConfirmationStep::Done(evaluate_confirmation_session(session))
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct BreakawayCampaignInputs {
    p_start: PidPGain,
    p_top: PidPGain,
    target_velocity_rpm: TmcMechanicalRpm,
    planned_velocity_mrev_s: u32,
    effective_ceiling_mrev_s: u32,
    hard_torque_limit: u16,
    usable_torque_limit: u16,
    limit_evidence_capability: VelocityLimitEvidenceCapability,
    observations_per_direction: u16,
    max_duration_ms: u16,
    workflow_envelope_ms: u32,
    channel: u8,
    run_sequence: u32,
    pwm_hz: u32,
    encoder_counts_per_rev: u32,
    ceiling_binding_source: u8,
    nomination_p_below: u16,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct BreakawayCampaignRequest {
    pub(crate) target_velocity_rpm: TmcMechanicalRpm,
    pub(crate) planned_velocity_mrev_s: u32,
    pub(crate) effective_ceiling_mrev_s: u32,
    pub(crate) hard_torque_limit: u16,
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
    pub(crate) max_duration_ms: u16,
    pub(crate) nominal_workflow_ms: u32,
    pub(crate) maximum_workflow_ms: u32,
    pub(crate) channel: u8,
    pub(crate) run_sequence: u32,
    pub(crate) pwm_hz: u32,
    pub(crate) encoder_counts_per_rev: u32,
    pub(crate) nomination_p_below: u16,
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BreakawayCampaignPhase {
    Probe {
        state: BreakawayProbeState,
        session: BreakawayProbeSession,
    },
    Discovery {
        state: AdditiveDiscoveryState,
        session: AdditiveDiscoverySession,
    },
    Confirmation {
        state: BandConfirmationState,
        session: BandConfirmationSession,
    },
}

#[derive(Debug, defmt::Format)]
pub(crate) struct BreakawayCampaign {
    inputs: BreakawayCampaignInputs,
    phase: BreakawayCampaignPhase,
    evidence_sequence: u16,
    pending_event: Option<crate::commissioning::types::BreakawayCampaignTerminal>,
    pending_done: Option<BreakawayCampaignOutcome>,
    pending_phase_terminal: Option<crate::commissioning::types::AdditiveDiscoveryTerminal>,
    nomination_evidence: NominationEvidence,
}

impl BreakawayCampaign {
    pub(crate) fn new(request: BreakawayCampaignRequest) -> Result<Self, CommissionError> {
        let (p_start, p_top, usable_torque_limit, ceiling_binding_source) =
            current_gain_span(request.target_velocity_rpm, request.hard_torque_limit)
                .map_err(|_| CommissionError::CampaignPlanInvalid)?;
        let workflow = CommissioningWorkflowPlan::new_breakaway(
            request.run_sequence,
            request.nominal_workflow_ms,
            request.maximum_workflow_ms,
        )
        .map_err(|_| CommissionError::CampaignPlanInvalid)?;
        let probe_context = BreakawayProbeContext {
            workflow_digest: workflow.digest(),
            target_velocity_rpm: request.target_velocity_rpm.raw(),
            hard_torque_limit: request.hard_torque_limit,
            usable_torque_limit,
            pwm_hz: request.pwm_hz,
            encoder_counts_per_rev: request.encoder_counts_per_rev,
        };
        let probe_plan = plan_upward_breakaway_probe(p_start, p_top, probe_context)
            .map_err(|_| CommissionError::CampaignPlanInvalid)?;
        let session = BreakawayProbeSession::new(
            probe_plan,
            request.target_velocity_rpm,
            request.planned_velocity_mrev_s,
        )
        .map_err(|_| CommissionError::CampaignPlanInvalid)?;
        Ok(Self {
            inputs: BreakawayCampaignInputs {
                p_start,
                p_top,
                target_velocity_rpm: request.target_velocity_rpm,
                planned_velocity_mrev_s: request.planned_velocity_mrev_s,
                effective_ceiling_mrev_s: request.effective_ceiling_mrev_s,
                hard_torque_limit: request.hard_torque_limit,
                usable_torque_limit,
                limit_evidence_capability: request.limit_evidence_capability,
                observations_per_direction: CONSENSUS_OBSERVATIONS_PER_DIRECTION as u16,
                max_duration_ms: request.max_duration_ms,
                workflow_envelope_ms: request.maximum_workflow_ms,
                channel: request.channel,
                run_sequence: request.run_sequence,
                pwm_hz: request.pwm_hz,
                encoder_counts_per_rev: request.encoder_counts_per_rev,
                ceiling_binding_source,
                nomination_p_below: request.nomination_p_below,
            },
            phase: BreakawayCampaignPhase::Probe {
                state: BreakawayProbeState::PlannedProbe,
                session,
            },
            evidence_sequence: 0,
            pending_event: None,
            pending_done: None,
            pending_phase_terminal: None,
            nomination_evidence: NominationEvidence::default(),
        })
    }

    pub(crate) const fn run_sequence(&self) -> u32 {
        self.inputs.run_sequence
    }

    pub(crate) const RECOVERY_SLOT_COUNT: u32 = PROBE_MAX_OBSERVATIONS as u32
        + MAX_SWEEP_RUNGS as u32 * 2 * OBSERVATIONS_PER_DIRECTION_PER_RUNG as u32
        + CONFIRMATION_FAMILY_SIZE as u32;

    pub(crate) const fn hard_torque_limit(&self) -> u16 {
        self.inputs.hard_torque_limit
    }

    pub(crate) const fn encoder_counts_per_rev(&self) -> u32 {
        self.inputs.encoder_counts_per_rev
    }

    pub(crate) const fn planned_velocity_mrev_s(&self) -> u32 {
        self.inputs.planned_velocity_mrev_s
    }

    pub(crate) const fn effective_ceiling_mrev_s(&self) -> u32 {
        self.inputs.effective_ceiling_mrev_s
    }

    pub(crate) const fn phase(&self) -> &BreakawayCampaignPhase {
        &self.phase
    }

    #[cfg(test)]
    pub(crate) const fn nomination_p_below(&self) -> u16 {
        self.inputs.nomination_p_below
    }
}

pub(crate) fn plan_breakaway_campaign_command(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    hard_torque_limit: u16,
    limit_evidence_capability: u32,
    pwm_hz: u32,
    channel: u8,
    run_sequence: u32,
) -> Result<(BreakawayCampaign, CommissioningWorkflowPlan), CommissionError> {
    let plan = plan_proportional_velocity_sweep(
        request,
        encoder_counts_per_rev,
        hard_torque_limit,
        limit_evidence_capability,
    )
    .map_err(|_| CommissionError::CampaignPlanInvalid)?;
    let geometry = integral_continuation_geometry(plan, pwm_hz)
        .map_err(|_| CommissionError::CampaignPlanInvalid)?;
    let integral_nominal_us = u64::from(geometry.i_nominal_ms)
        .checked_mul(1_000)
        .ok_or(CommissionError::InvalidSchedule)?;
    let integral_maximum_us = u64::from(geometry.i_maximum_ms)
        .checked_mul(1_000)
        .ok_or(CommissionError::InvalidSchedule)?;
    let nominal_workflow_ms = breakaway_workflow_envelope_ms(
        BreakawayCampaign::RECOVERY_SLOT_COUNT,
        plan.recovery_nominal_slot_us,
        integral_nominal_us,
    )
    .map_err(|_| CommissionError::CampaignPlanInvalid)?;
    let maximum_workflow_ms = breakaway_workflow_envelope_ms(
        BreakawayCampaign::RECOVERY_SLOT_COUNT,
        plan.recovery_maximum_slot_us,
        integral_maximum_us,
    )
    .map_err(|_| CommissionError::CampaignPlanInvalid)?;
    let workflow = CommissioningWorkflowPlan::new_breakaway(
        run_sequence,
        nominal_workflow_ms,
        maximum_workflow_ms,
    )
    .map_err(|_| CommissionError::CampaignPlanInvalid)?;
    let campaign = BreakawayCampaign::new(BreakawayCampaignRequest {
        target_velocity_rpm: plan.target_velocity_rpm,
        planned_velocity_mrev_s: plan.planned_velocity_mrev_s,
        effective_ceiling_mrev_s: plan.effective_ceiling_mrev_s,
        hard_torque_limit,
        limit_evidence_capability: plan.limit_evidence_capability,
        max_duration_ms: request.max_duration_ms,
        nominal_workflow_ms,
        maximum_workflow_ms,
        channel,
        run_sequence,
        pwm_hz,
        encoder_counts_per_rev,
        nomination_p_below: request.nomination_p_below,
    })?;
    Ok((campaign, workflow))
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BreakawayCampaignOutcome {
    IntegralPlanReady(VelocityIntegralPlan),
    Terminal {
        error: CommissionError,
        safety: Option<OuterSafetyFault>,
    },
}

#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum BreakawayCampaignStep {
    ReadyNow,
    At(u64),
    Event(BreakawayCampaignEvent),
    Done(BreakawayCampaignOutcome),
}

#[allow(clippy::large_enum_variant)]
enum CampaignAdvance {
    Step(BreakawayCampaignStep),
    Emit {
        now: BreakawayCampaignEvent,
        next: EmitContinuation,
    },
}

#[allow(clippy::large_enum_variant)]
enum EmitContinuation {
    EnterPhase(BreakawayCampaignPhase),
    Terminal {
        campaign_terminal: crate::commissioning::types::BreakawayCampaignTerminal,
        done: BreakawayCampaignOutcome,
    },
}

fn probe_terminal(outcome: BreakawayProbeOutcome) -> BreakawayCampaignOutcome {
    match outcome {
        BreakawayProbeOutcome::BreakawayFound(_) => {
            unreachable!("BreakawayFound is advanced to discovery by the orchestrator")
        }
        BreakawayProbeOutcome::SafetyFault { error, safety } => {
            BreakawayCampaignOutcome::Terminal { error, safety }
        }
        BreakawayProbeOutcome::NoRepeatableMotionAtAuthority
        | BreakawayProbeOutcome::ExcursionEvidenceInvalid
        | BreakawayProbeOutcome::AuthorityFailure => BreakawayCampaignOutcome::Terminal {
            error: CommissionError::VelocityValidationFailed,
            safety: None,
        },
    }
}

fn confirmation_terminal(outcome: BandConfirmationOutcome) -> BreakawayCampaignOutcome {
    match outcome {
        BandConfirmationOutcome::Accepted { .. } => {
            unreachable!("Accepted is routed to Integral by the orchestrator")
        }
        BandConfirmationOutcome::SafetyFault { error, safety } => {
            BreakawayCampaignOutcome::Terminal { error, safety }
        }
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery
        | BandConfirmationOutcome::TargetBandNoTransitionCapableCandidate
        | BandConfirmationOutcome::TargetBandNoDampedCandidate { .. }
        | BandConfirmationOutcome::TargetBandNoRungBelowCeiling { .. }
        | BandConfirmationOutcome::TargetBandNotReachedAtAuthority
        | BandConfirmationOutcome::TargetBandConfirmationInconclusive(_)
        | BandConfirmationOutcome::ConfirmationEvidenceExcluded => {
            BreakawayCampaignOutcome::Terminal {
                error: CommissionError::VelocityValidationFailed,
                safety: None,
            }
        }
    }
}

fn next_sequence(evidence_sequence: &mut u16) -> u16 {
    let sequence = *evidence_sequence;
    *evidence_sequence = evidence_sequence.wrapping_add(1);
    sequence
}

fn raw_observation_step(
    plan_digest: u64,
    phase: super::breakaway_reporting::BreakawayPhase,
    collected: CollectedObservation,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let sequence = next_sequence(evidence_sequence);
    let report = super::breakaway_reporting::raw_observation_report(
        plan_digest,
        phase,
        &collected,
        run_sequence,
        sequence,
    );
    BreakawayCampaignStep::Event(BreakawayCampaignEvent::RawObservation(report))
}

fn rest_rejected_step(
    plan_digest: u64,
    phase: super::breakaway_reporting::BreakawayPhase,
    input: RecoveryRestRejectedInput,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let sequence = next_sequence(evidence_sequence);
    let report = super::breakaway_reporting::recovery_rest_rejected_report(
        plan_digest,
        phase,
        input,
        run_sequence,
        sequence,
    );
    BreakawayCampaignStep::Event(BreakawayCampaignEvent::RecoveryRestRejected(report))
}

fn origin_mismatch_step(
    plan_digest: u64,
    phase: super::breakaway_reporting::BreakawayPhase,
    evidence: RecoveryOriginMismatchEvidence,
    slot_index: u8,
    direction: VelocityDirection,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let sequence = next_sequence(evidence_sequence);
    let report = super::breakaway_reporting::recovery_origin_mismatch_report(
        plan_digest,
        phase,
        evidence,
        slot_index,
        direction,
        run_sequence,
        sequence,
    );
    BreakawayCampaignStep::Event(BreakawayCampaignEvent::RecoveryOriginMismatch(report))
}

fn rest_rejected_input(
    decision: crate::autotune::velocity_measurement::RestSelectionDecision,
    stage: RecoveryStage,
    rung_index: u8,
    slot_index: u8,
    direction: VelocityDirection,
    p_raw: u16,
    settle: (i8, i8, i8),
) -> Option<RecoveryRestRejectedInput> {
    use crate::autotune::velocity_measurement::RestSelectionDisposition;

    if decision.disposition == RestSelectionDisposition::Rejected {
        defmt::warn!(
            "recovery rest rejected: stage={} rung={} slot={} dir={} p_raw={} decision={}",
            stage,
            rung_index,
            slot_index,
            direction,
            p_raw,
            decision,
        );
    }

    (decision.disposition == RestSelectionDisposition::Rejected).then_some(
        RecoveryRestRejectedInput {
            decision,
            stage: stage as u8,
            rung_index,
            slot_index,
            direction,
            p_raw,
            scored_net_displacement_counts: settle.0,
            scored_min_displacement_counts: settle.1,
            scored_max_displacement_counts: settle.2,
        },
    )
}

fn settle_rest_rejected_input(
    decision: crate::autotune::velocity_measurement::RestSelectionDecision,
    rung_index: u8,
    retry_available: bool,
    velocity_bias: (i32, i32),
    settle: (i8, i8, i8),
) {
    use crate::autotune::velocity_measurement::RestSelectionDisposition;

    if decision.disposition == RestSelectionDisposition::Rejected {
        defmt::warn!(
            "settle rest rejected: rung={} retry_available={} velocity_offset={} velocity_actual={} net_counts={} min_counts={} max_counts={} decision={}",
            rung_index,
            retry_available,
            velocity_bias.0,
            velocity_bias.1,
            settle.0,
            settle.1,
            settle.2,
            decision,
        );
    }
}

fn probe_decision_step(
    plan_digest: u64,
    input: ProbeDecisionReportInput,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let sequence = next_sequence(evidence_sequence);
    let report = super::breakaway_reporting::probe_decision_report(
        plan_digest,
        input,
        run_sequence,
        sequence,
    );
    BreakawayCampaignStep::Event(BreakawayCampaignEvent::ProbeDecision(report))
}

fn margin_step(
    plan_digest: u64,
    in_band: &[DiscoveryRungNominationBounds],
    collected_index: u8,
    run_sequence: u32,
    evidence_sequence: &mut u16,
) -> BreakawayCampaignStep {
    let nominatable = in_band
        .get(usize::from(collected_index))
        .is_some_and(|bounds| bounds.nominatable);
    if !nominatable {
        return BreakawayCampaignStep::ReadyNow;
    }
    let sequence = next_sequence(evidence_sequence);
    match super::breakaway_reporting::discovery_rung_margin_report(
        in_band,
        collected_index,
        plan_digest,
        run_sequence,
        sequence,
    ) {
        Some(report) => {
            BreakawayCampaignStep::Event(BreakawayCampaignEvent::DiscoveryRungMargin(report))
        }
        None => BreakawayCampaignStep::ReadyNow,
    }
}

#[allow(clippy::too_many_arguments)]
fn campaign_terminal_event(
    phase: BreakawayPhase,
    terminal_cause: super::breakaway_reporting::BreakawayTerminalCause,
    accepted: bool,
    integral_plan_digest: u64,
    error_code: u8,
    nomination: NominationEvidence,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> crate::commissioning::types::BreakawayCampaignTerminal {
    let sequence = next_sequence(evidence_sequence);
    campaign_terminal_report(
        phase,
        terminal_cause,
        accepted,
        integral_plan_digest,
        error_code,
        nomination,
        inputs.run_sequence,
        sequence,
    )
}

pub(crate) fn breakaway_campaign_fault_terminal(
    phase: &mut OuterPhase,
    error: CommissionError,
    safety_fault: Option<OuterSafetyFault>,
) -> StepResult {
    let OuterPhase::BreakawayCampaign { campaign, .. } = phase else {
        return StepResult::OuterFailed(error);
    };
    let raised_safety = safety_fault.is_some();
    let (owning_phase, terminal_cause) = match campaign.phase() {
        BreakawayCampaignPhase::Probe { .. } => (
            BreakawayPhase::BreakawayProbe,
            if raised_safety {
                super::breakaway_reporting::BreakawayTerminalCause::ProbeSafetyFault
            } else {
                super::breakaway_reporting::BreakawayTerminalCause::ProbeInternalFault
            },
        ),
        BreakawayCampaignPhase::Discovery { .. } => (
            BreakawayPhase::ProportionalResponseDiscovery,
            if raised_safety {
                super::breakaway_reporting::BreakawayTerminalCause::DiscoverySafetyFault
            } else {
                super::breakaway_reporting::BreakawayTerminalCause::DiscoveryInternalFault
            },
        ),
        BreakawayCampaignPhase::Confirmation { .. } => (
            BreakawayPhase::ProportionalBandConfirmation,
            if raised_safety {
                super::breakaway_reporting::BreakawayTerminalCause::ConfirmationSafetyFault
            } else {
                super::breakaway_reporting::BreakawayTerminalCause::ConfirmationInternalFault
            },
        ),
    };
    let phase_terminal = match &campaign.phase {
        BreakawayCampaignPhase::Discovery { session, .. } => {
            let sequence = next_sequence(&mut campaign.evidence_sequence);
            Some(super::breakaway_reporting::discovery_terminal_fields(
                &session.plan,
                super::breakaway_reporting::DiscoveryTerminalEvidence {
                    cause: terminal_cause,
                    has_safety_fault: u8::from(raised_safety),
                    collected_count: session.in_band.count(),
                    stop_rung_index: session.rung_index,
                    error_code: error.status_code(),
                },
                campaign.inputs.run_sequence,
                sequence,
            ))
        }
        BreakawayCampaignPhase::Probe { .. } | BreakawayCampaignPhase::Confirmation { .. } => None,
    };
    let campaign_terminal = campaign_terminal_event(
        owning_phase,
        terminal_cause,
        false,
        0,
        error.status_code(),
        campaign.nomination_evidence,
        &campaign.inputs,
        &mut campaign.evidence_sequence,
    );
    campaign.pending_phase_terminal = phase_terminal;
    campaign.pending_event = Some(campaign_terminal);
    campaign.pending_done = Some(BreakawayCampaignOutcome::Terminal {
        error,
        safety: safety_fault,
    });
    StepResult::ReadyNow
}

pub(crate) fn breakaway_campaign_pending_terminal(phase: &OuterPhase) -> bool {
    matches!(
        phase,
        OuterPhase::BreakawayCampaign { campaign, .. }
            if campaign.pending_event.is_some() || campaign.pending_done.is_some()
    )
}

fn discovery_construction_fault(
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> EmitContinuation {
    let campaign_terminal = campaign_terminal_event(
        BreakawayPhase::ProportionalResponseDiscovery,
        super::breakaway_reporting::BreakawayTerminalCause::DiscoveryInternalFault,
        false,
        0,
        CommissionError::DiscoveryBandNotSelected.status_code(),
        NominationEvidence::default(),
        inputs,
        evidence_sequence,
    );
    EmitContinuation::Terminal {
        campaign_terminal,
        done: BreakawayCampaignOutcome::Terminal {
            error: CommissionError::DiscoveryBandNotSelected,
            safety: None,
        },
    }
}

fn route_discovery_outcome(
    outcome: AdditiveDiscoveryOutcome,
    discovery_plan_digest: u64,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
    nomination_evidence: &mut NominationEvidence,
) -> EmitContinuation {
    let entry = match max_transition_capable_p(
        inputs.target_velocity_rpm.raw(),
        inputs.encoder_counts_per_rev,
        inputs.pwm_hz,
        MOVING_STROKE_US,
        inputs.usable_torque_limit,
    ) {
        Ok(transition_ceiling) => begin_band_confirmation(
            outcome,
            discovery_plan_digest,
            inputs.target_velocity_rpm,
            inputs.planned_velocity_mrev_s,
            transition_ceiling,
            inputs.nomination_p_below,
        ),
        Err(_) => BandConfirmationEntry::Done(BandConfirmationOutcome::SafetyFault {
            error: CommissionError::CampaignPlanInvalid,
            safety: None,
        }),
    };
    match entry {
        BandConfirmationEntry::Ready(session) => {
            *nomination_evidence = session.nomination_evidence;
            EmitContinuation::EnterPhase(BreakawayCampaignPhase::Confirmation {
                state: BandConfirmationState::PlannedConfirmation,
                session,
            })
        }
        BandConfirmationEntry::Done(outcome) => {
            let cause = confirmation_zero_stroke_cause(&outcome).0;
            let error_code = match &outcome {
                BandConfirmationOutcome::SafetyFault { error, .. } => error.status_code(),
                _ => 0,
            };
            if let BandConfirmationOutcome::TargetBandNoDampedCandidate { excluded }
            | BandConfirmationOutcome::TargetBandNoRungBelowCeiling { excluded, .. } = outcome
            {
                *nomination_evidence = NominationEvidence {
                    nominated_step_peak_permille: 0,
                    overshoot_excluded_rungs: excluded,
                };
            }
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::ProportionalBandConfirmation,
                cause,
                false,
                0,
                error_code,
                *nomination_evidence,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: confirmation_terminal(outcome),
            }
        }
    }
}

fn route_confirmed_accept<B: CommissioningBackend>(
    acceptance: BandConfirmationAcceptance,
    plan: BandConfirmationPlan,
    inputs: &BreakawayCampaignInputs,
    safety: &OuterSafetyLimits,
    backend: &B,
) -> BreakawayCampaignOutcome {
    let token = match bind_breakaway_integral_authority(plan.plan_digest, plan, Ok(acceptance)) {
        Ok(token) => token,
        Err(_) => {
            return BreakawayCampaignOutcome::Terminal {
                error: CommissionError::IntegralAuthorityDenied,
                safety: None,
            };
        }
    };
    let context = AcquisitionContext {
        planned_velocity_mrev_s: inputs.planned_velocity_mrev_s,
        target_velocity_rpm: inputs.target_velocity_rpm.raw(),
        hard_torque_limit: inputs.hard_torque_limit,
        usable_torque_limit: inputs.usable_torque_limit,
        limit_evidence_capability: inputs.limit_evidence_capability,
        channel: inputs.channel,
        plan_time_pwm_hz: inputs.pwm_hz,
        plan_time_encoder_counts_per_rev: inputs.encoder_counts_per_rev,
    };
    match plan_breakaway_velocity_integral_response(
        token,
        plan,
        context,
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: acceptance.held_out_response_intervals_q,
            confidence_q: acceptance.confidence_q,
            band_position_q: acceptance.band_position_q,
        },
        IntegralPlanPhysics {
            motion_budget: safety.budget,
            pwm_hz: backend.tmc_config().pwm_freq_hz(),
            encoder_counts_per_rev: safety.counts_per_rev(),
        },
    ) {
        Ok(integral_plan) => BreakawayCampaignOutcome::IntegralPlanReady(integral_plan),
        Err(_) => BreakawayCampaignOutcome::Terminal {
            error: CommissionError::VelocityIntegralPlanInvalid,
            safety: None,
        },
    }
}

fn enter_discovery(
    resolved: &ResolvedUpwardBreakaway,
    probe_plan: &UpwardBreakawayProbePlan,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> EmitContinuation {
    let (floor, floor_origin) = match resolve_grid_floor(
        inputs.p_start,
        inputs.p_top,
        probe_plan.search_count,
        resolved.breakaway_index,
    ) {
        Ok(resolved_floor) => resolved_floor,
        Err(_) => return discovery_construction_fault(inputs, evidence_sequence),
    };
    let Some(breakaway) = probe_plan
        .search_gains
        .get(usize::from(resolved.breakaway_index))
        .copied()
    else {
        return discovery_construction_fault(inputs, evidence_sequence);
    };
    let request = AdditiveDiscoveryRequest {
        floor,
        breakaway,
        authority_ceiling: inputs.p_top,
        observations_per_direction: inputs.observations_per_direction,
        max_duration_ms: inputs.max_duration_ms,
        workflow_envelope_ms: inputs.workflow_envelope_ms,
        probe_plan_digest: probe_plan.plan_digest,
    };
    let floor_origin_clamped = matches!(floor_origin, GridFloorOrigin::ClampedAtBreakaway);
    match AdditiveDiscoverySession::new(
        request,
        floor_origin_clamped,
        inputs.target_velocity_rpm,
        inputs.planned_velocity_mrev_s,
    ) {
        Ok(session) => EmitContinuation::EnterPhase(BreakawayCampaignPhase::Discovery {
            state: AdditiveDiscoveryState::PlannedDiscovery,
            session,
        }),
        Err(outcome) => {
            let (cause, error, safety, error_code) = match outcome {
                AdditiveDiscoveryOutcome::InsufficientAdditiveSpan => (
                    BreakawayTerminalCause::DiscoveryInsufficientAdditiveSpan,
                    CommissionError::DiscoveryBandNotSelected,
                    None,
                    0,
                ),
                AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => (
                    BreakawayTerminalCause::DiscoveryAdditiveLadderResolutionBudgetExceeded,
                    CommissionError::DiscoveryBandNotSelected,
                    None,
                    0,
                ),
                AdditiveDiscoveryOutcome::SafetyFault { error, safety } => (
                    if safety.is_some() {
                        BreakawayTerminalCause::DiscoverySafetyFault
                    } else {
                        BreakawayTerminalCause::DiscoveryInternalFault
                    },
                    error,
                    safety,
                    error.status_code(),
                ),
                AdditiveDiscoveryOutcome::TargetBandSelected(_)
                | AdditiveDiscoveryOutcome::TargetBandSkipped
                | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand
                | AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { .. } => (
                    BreakawayTerminalCause::DiscoveryInternalFault,
                    CommissionError::DiscoveryBandNotSelected,
                    None,
                    CommissionError::DiscoveryBandNotSelected.status_code(),
                ),
            };
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::ProportionalResponseDiscovery,
                cause,
                false,
                0,
                error_code,
                NominationEvidence::default(),
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal { error, safety },
            }
        }
    }
}

fn probe_success_advance(
    resolved: ResolvedUpwardBreakaway,
    probe_plan: &UpwardBreakawayProbePlan,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let report = probe_resolved_report(
        probe_plan,
        resolved.breakaway_index,
        resolved.observations,
        inputs.run_sequence,
        sequence,
    );
    let now = BreakawayCampaignEvent::ProbeResolved(report);
    let next = enter_discovery(&resolved, probe_plan, inputs, evidence_sequence);
    CampaignAdvance::Emit { now, next }
}

fn probe_failure_advance(
    outcome: BreakawayProbeOutcome,
    probe_plan: &UpwardBreakawayProbePlan,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let terminal = probe_terminal_report(probe_plan, &outcome, inputs.run_sequence, sequence);
    let cause = probe_terminal_cause(&outcome).0;
    let error_code = match &outcome {
        BreakawayProbeOutcome::SafetyFault { error, .. } => error.status_code(),
        _ => 0,
    };
    let now = BreakawayCampaignEvent::ProbeTerminal(terminal);
    let campaign_terminal = campaign_terminal_event(
        BreakawayPhase::BreakawayProbe,
        cause,
        false,
        0,
        error_code,
        NominationEvidence::default(),
        inputs,
        evidence_sequence,
    );
    CampaignAdvance::Emit {
        now,
        next: EmitContinuation::Terminal {
            campaign_terminal,
            done: probe_terminal(outcome),
        },
    }
}

fn discovery_done_advance(
    outcome: AdditiveDiscoveryOutcome,
    session: &AdditiveDiscoverySession,
    inputs: &BreakawayCampaignInputs,
    evidence_sequence: &mut u16,
    nomination_evidence: &mut NominationEvidence,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let terminal = discovery_terminal_report(
        &session.plan,
        &outcome,
        session.in_band.count(),
        session.rung_index,
        inputs.run_sequence,
        sequence,
    );
    let rung_zero = session.rung_zero_classification.map(|(forward, reverse)| {
        rung_zero_diagnostic_report(
            &session.plan,
            forward,
            reverse,
            inputs.run_sequence,
            sequence,
        )
    });
    let now = BreakawayCampaignEvent::DiscoveryTerminal {
        terminal,
        rung_zero,
    };
    let plan_digest = session.plan.plan_digest;
    let next = match outcome {
        AdditiveDiscoveryOutcome::TargetBandSelected(_)
        | AdditiveDiscoveryOutcome::CurrentHeadroomBeforeBand => route_discovery_outcome(
            outcome,
            plan_digest,
            inputs,
            evidence_sequence,
            nomination_evidence,
        ),
        AdditiveDiscoveryOutcome::TargetBandSkipped
        | AdditiveDiscoveryOutcome::InsufficientAdditiveSpan
        | AdditiveDiscoveryOutcome::AdditiveLadderResolutionBudgetExceeded => {
            let cause = discovery_terminal_cause(&outcome).0;
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::ProportionalResponseDiscovery,
                cause,
                false,
                0,
                0,
                *nomination_evidence,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal {
                    error: CommissionError::DiscoveryBandNotSelected,
                    safety: None,
                },
            }
        }
        AdditiveDiscoveryOutcome::DiscoveryEvidenceExcluded { error } => {
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::ProportionalResponseDiscovery,
                super::breakaway_reporting::BreakawayTerminalCause::DiscoveryEvidenceExcluded,
                false,
                0,
                0,
                *nomination_evidence,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal {
                    error,
                    safety: None,
                },
            }
        }
        AdditiveDiscoveryOutcome::SafetyFault { error, safety } => {
            let campaign_terminal = campaign_terminal_event(
                BreakawayPhase::ProportionalResponseDiscovery,
                if safety.is_some() {
                    super::breakaway_reporting::BreakawayTerminalCause::DiscoverySafetyFault
                } else {
                    super::breakaway_reporting::BreakawayTerminalCause::DiscoveryInternalFault
                },
                false,
                0,
                error.status_code(),
                *nomination_evidence,
                inputs,
                evidence_sequence,
            );
            EmitContinuation::Terminal {
                campaign_terminal,
                done: BreakawayCampaignOutcome::Terminal { error, safety },
            }
        }
    };
    CampaignAdvance::Emit { now, next }
}

fn confirmation_directional_input(
    session: &BandConfirmationSession,
    direction: VelocityDirection,
) -> ConfirmationDirectionalInput {
    let accumulator = match direction {
        VelocityDirection::Forward => &session.forward,
        VelocityDirection::Reverse => &session.reverse,
    };
    ConfirmationDirectionalInput {
        consensus: accumulator.consensus(),
        classes: accumulator.classes,
        observations: accumulator.atoms,
    }
}

const EMPTY_CONFIRMATION_DIRECTIONAL_INPUT: ConfirmationDirectionalInput =
    ConfirmationDirectionalInput {
        consensus: DirectionalConsensus {
            classification: super::sweep_consensus::DirectionalConsensusClass::IncompleteRung,
            components: [[0; 2]; 2],
            component_count: 0,
            collected_mask: 0,
            eligible_mask: 0,
            included_mask: 0,
        },
        classes: [VelocityObservationClass::InvalidEvidenceFault;
            CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [None; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };

fn confirmation_done_advance<B: CommissioningBackend>(
    outcome: BandConfirmationOutcome,
    session: &BandConfirmationSession,
    inputs: &BreakawayCampaignInputs,
    safety: &OuterSafetyLimits,
    backend: &B,
    nomination_evidence: NominationEvidence,
    evidence_sequence: &mut u16,
) -> CampaignAdvance {
    let sequence = next_sequence(evidence_sequence);
    let (cause, has_safety_fault) = confirmation_zero_stroke_cause(&outcome);
    let (terminal, forward, reverse) = match &outcome {
        BandConfirmationOutcome::Accepted { acceptance, .. } => {
            let forward = confirmation_directional_input(session, VelocityDirection::Forward);
            let reverse = confirmation_directional_input(session, VelocityDirection::Reverse);
            let terminal = confirmation_evaluated_terminal_report(
                &session.plan,
                &forward,
                &reverse,
                Some(acceptance.nominated_p().raw()),
                None,
                inputs.run_sequence,
                sequence,
            );
            (terminal, forward, reverse)
        }
        BandConfirmationOutcome::TargetBandConfirmationInconclusive(failure_cause) => {
            let forward = confirmation_directional_input(session, VelocityDirection::Forward);
            let reverse = confirmation_directional_input(session, VelocityDirection::Reverse);
            let terminal = confirmation_evaluated_terminal_report(
                &session.plan,
                &forward,
                &reverse,
                None,
                Some(*failure_cause),
                inputs.run_sequence,
                sequence,
            );
            (terminal, forward, reverse)
        }
        BandConfirmationOutcome::ConfirmationEvidenceExcluded
        | BandConfirmationOutcome::SafetyFault { .. } => {
            let forward = confirmation_directional_input(session, VelocityDirection::Forward);
            let reverse = confirmation_directional_input(session, VelocityDirection::Reverse);
            let terminal = confirmation_partial_evidence_terminal_report(
                &session.plan,
                &forward,
                &reverse,
                cause,
                has_safety_fault,
                inputs.run_sequence,
                sequence,
            );
            (terminal, forward, reverse)
        }
        BandConfirmationOutcome::TargetBandUnconfirmableFromDiscovery
        | BandConfirmationOutcome::TargetBandNoTransitionCapableCandidate
        | BandConfirmationOutcome::TargetBandNoDampedCandidate { .. }
        | BandConfirmationOutcome::TargetBandNoRungBelowCeiling { .. }
        | BandConfirmationOutcome::TargetBandNotReachedAtAuthority => {
            let terminal = confirmation_zero_stroke_terminal_report(
                session.plan.plan_digest,
                session.plan.discovery_plan_digest,
                session.plan.family_size,
                &outcome,
                inputs.run_sequence,
                sequence,
            );
            (
                terminal,
                EMPTY_CONFIRMATION_DIRECTIONAL_INPUT,
                EMPTY_CONFIRMATION_DIRECTIONAL_INPUT,
            )
        }
    };
    let now = BreakawayCampaignEvent::ConfirmationTerminal {
        terminal,
        forward,
        reverse,
    };
    let (done, campaign_cause, error_code) = match outcome {
        BandConfirmationOutcome::Accepted { acceptance, plan } => {
            let done = route_confirmed_accept(acceptance, plan, inputs, safety, backend);
            let (campaign_cause, error_code) = match &done {
                BreakawayCampaignOutcome::IntegralPlanReady(_) => (cause, 0),
                BreakawayCampaignOutcome::Terminal { error, .. } => (
                    super::breakaway_reporting::BreakawayTerminalCause::ConfirmationIntegralPlanRefused,
                    error.status_code(),
                ),
            };
            (done, campaign_cause, error_code)
        }
        other => {
            let error_code = match &other {
                BandConfirmationOutcome::SafetyFault { error, .. } => error.status_code(),
                _ => 0,
            };
            (confirmation_terminal(other), cause, error_code)
        }
    };
    let (accepted, integral_plan_digest) = match &done {
        BreakawayCampaignOutcome::IntegralPlanReady(plan) => (true, plan.digest()),
        BreakawayCampaignOutcome::Terminal { .. } => (false, 0),
    };
    let campaign_terminal = campaign_terminal_event(
        BreakawayPhase::ProportionalBandConfirmation,
        campaign_cause,
        accepted,
        integral_plan_digest,
        error_code,
        nomination_evidence,
        inputs,
        evidence_sequence,
    );
    CampaignAdvance::Emit {
        now,
        next: EmitContinuation::Terminal {
            campaign_terminal,
            done,
        },
    }
}

fn emit_phase_planned_event(campaign: &mut BreakawayCampaign) -> Option<BreakawayCampaignStep> {
    let BreakawayCampaign {
        inputs,
        phase,
        evidence_sequence,
        ..
    } = campaign;
    match phase {
        BreakawayCampaignPhase::Probe { state, session } => {
            if !matches!(*state, BreakawayProbeState::PlannedProbe) {
                return None;
            }
            let sequence = next_sequence(evidence_sequence);
            let report = probe_plan_report(
                &session.plan,
                inputs.p_start,
                inputs.p_top,
                inputs.run_sequence,
                sequence,
            );
            *state = BreakawayProbeState::SetupSafety;
            Some(BreakawayCampaignStep::Event(
                BreakawayCampaignEvent::ProbePlanned(report),
            ))
        }
        BreakawayCampaignPhase::Discovery { state, session } => {
            if !matches!(*state, AdditiveDiscoveryState::PlannedDiscovery) {
                return None;
            }
            let sequence = next_sequence(evidence_sequence);
            let planned = discovery_plan_report(
                &session.request,
                session.floor_origin_clamped,
                &session.plan,
                inputs.run_sequence,
                sequence,
            );
            let ceiling_source = BreakawayCeilingSource {
                run_sequence: inputs.run_sequence,
                evidence_sequence: sequence,
                plan_digest: session.plan.plan_digest,
                binding_source: inputs.ceiling_binding_source,
            };
            *state = AdditiveDiscoveryState::WriteGain;
            Some(BreakawayCampaignStep::Event(
                BreakawayCampaignEvent::DiscoveryPlanned {
                    planned,
                    ceiling_source,
                },
            ))
        }
        BreakawayCampaignPhase::Confirmation { state, session } => {
            if !matches!(*state, BandConfirmationState::PlannedConfirmation) {
                return None;
            }
            let sequence = next_sequence(evidence_sequence);
            let nomination = NominatedConfirmationCandidate {
                p: session.plan.nominated_p,
                margin_percent: session.nominated_margin_percent,
                step_peak_permille: session.nomination_evidence.nominated_step_peak_permille,
                overshoot_excluded_rungs: session.nomination_evidence.overshoot_excluded_rungs,
            };
            let planned =
                confirmation_plan_report(&session.plan, &nomination, inputs.run_sequence, sequence);
            *state = BandConfirmationState::WriteGain;
            Some(BreakawayCampaignStep::Event(
                BreakawayCampaignEvent::ConfirmationPlanned(planned),
            ))
        }
    }
}

fn checkpoint_event_routing(step: BreakawayCampaignStep) -> BreakawayCampaignStep {
    if matches!(&step, BreakawayCampaignStep::Event(_)) {
        crate::diagnostic::checkpoint_breakaway_campaign(
            crate::diagnostic::BreakawayCheckpoint::EventRouting,
        );
    }
    step
}

pub(crate) async fn step_breakaway_campaign<B: CommissioningBackend>(
    campaign: &mut BreakawayCampaign,
    workspace: &mut BreakawayProbeWorkspace,
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> BreakawayCampaignStep {
    if let Some(terminal) = campaign.pending_phase_terminal.take() {
        return checkpoint_event_routing(BreakawayCampaignStep::Event(
            BreakawayCampaignEvent::DiscoveryTerminal {
                terminal,
                rung_zero: None,
            },
        ));
    }
    if let Some(campaign_terminal) = campaign.pending_event.take() {
        return checkpoint_event_routing(BreakawayCampaignStep::Event(
            BreakawayCampaignEvent::CampaignTerminal(campaign_terminal),
        ));
    }
    if let Some(outcome) = campaign.pending_done.take() {
        return BreakawayCampaignStep::Done(outcome);
    }
    if let Some(step) = emit_phase_planned_event(campaign) {
        return checkpoint_event_routing(step);
    }

    let advance = {
        let BreakawayCampaign {
            inputs,
            phase,
            evidence_sequence,
            nomination_evidence,
            ..
        } = &mut *campaign;
        match phase {
            BreakawayCampaignPhase::Probe { state, session } => {
                match step_breakaway_probe(state, workspace, session, safety, backend).await {
                    BreakawayProbeStep::ReadyNow => {
                        CampaignAdvance::Step(BreakawayCampaignStep::ReadyNow)
                    }
                    BreakawayProbeStep::At(deadline_us) => {
                        CampaignAdvance::Step(BreakawayCampaignStep::At(deadline_us))
                    }
                    BreakawayProbeStep::Observation(collected) => {
                        CampaignAdvance::Step(raw_observation_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::BreakawayProbe,
                            collected,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    BreakawayProbeStep::Decision {
                        direction,
                        rung_index,
                        disposition,
                    } => CampaignAdvance::Step(probe_decision_step(
                        session.plan.plan_digest,
                        ProbeDecisionReportInput {
                            direction,
                            rung_index,
                            disposition,
                        },
                        inputs.run_sequence,
                        evidence_sequence,
                    )),
                    BreakawayProbeStep::RestRejected(input) => {
                        CampaignAdvance::Step(rest_rejected_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::BreakawayProbe,
                            input,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    BreakawayProbeStep::OriginMismatch(evidence) => {
                        CampaignAdvance::Step(origin_mismatch_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::BreakawayProbe,
                            evidence,
                            session.pending_role.slot_index(),
                            session.pending_role.direction(),
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    BreakawayProbeStep::Done(BreakawayProbeOutcome::BreakawayFound(resolved)) => {
                        probe_success_advance(resolved, &session.plan, inputs, evidence_sequence)
                    }
                    BreakawayProbeStep::Done(outcome) => {
                        probe_failure_advance(outcome, &session.plan, inputs, evidence_sequence)
                    }
                }
            }
            BreakawayCampaignPhase::Discovery { state, session } => {
                match step_additive_discovery(state, workspace, session, safety, backend).await {
                    AdditiveDiscoveryStep::ReadyNow => {
                        CampaignAdvance::Step(BreakawayCampaignStep::ReadyNow)
                    }
                    AdditiveDiscoveryStep::At(deadline_us) => {
                        CampaignAdvance::Step(BreakawayCampaignStep::At(deadline_us))
                    }
                    AdditiveDiscoveryStep::Observation(collected) => {
                        CampaignAdvance::Step(raw_observation_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::ProportionalResponseDiscovery,
                            collected,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    AdditiveDiscoveryStep::Margin { collected_index } => {
                        CampaignAdvance::Step(margin_step(
                            session.plan.plan_digest,
                            session.in_band.as_slice(),
                            collected_index,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    AdditiveDiscoveryStep::RestRejected(input) => {
                        CampaignAdvance::Step(rest_rejected_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::ProportionalResponseDiscovery,
                            input,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    AdditiveDiscoveryStep::OriginMismatch(evidence) => {
                        CampaignAdvance::Step(origin_mismatch_step(
                            session.plan.plan_digest,
                            super::breakaway_reporting::BreakawayPhase::ProportionalResponseDiscovery,
                            evidence,
                            session.slot,
                            session.direction,
                            inputs.run_sequence,
                            evidence_sequence,
                        ))
                    }
                    AdditiveDiscoveryStep::Done(outcome) => {
                        discovery_done_advance(
                            outcome,
                            session,
                            inputs,
                            evidence_sequence,
                            nomination_evidence,
                        )
                    }
                }
            }
            BreakawayCampaignPhase::Confirmation { state, session } => match step_band_confirmation(
                state, workspace, session, safety, backend,
            )
            .await
            {
                BandConfirmationStep::ReadyNow => {
                    CampaignAdvance::Step(BreakawayCampaignStep::ReadyNow)
                }
                BandConfirmationStep::At(deadline_us) => {
                    CampaignAdvance::Step(BreakawayCampaignStep::At(deadline_us))
                }
                BandConfirmationStep::Observation(collected) => {
                    CampaignAdvance::Step(raw_observation_step(
                        session.plan.plan_digest,
                        super::breakaway_reporting::BreakawayPhase::ProportionalBandConfirmation,
                        collected,
                        inputs.run_sequence,
                        evidence_sequence,
                    ))
                }
                BandConfirmationStep::RestRejected(input) => {
                    CampaignAdvance::Step(rest_rejected_step(
                        session.plan.plan_digest,
                        super::breakaway_reporting::BreakawayPhase::ProportionalBandConfirmation,
                        input,
                        inputs.run_sequence,
                        evidence_sequence,
                    ))
                }
                BandConfirmationStep::OriginMismatch(evidence) => {
                    CampaignAdvance::Step(origin_mismatch_step(
                        session.plan.plan_digest,
                        super::breakaway_reporting::BreakawayPhase::ProportionalBandConfirmation,
                        evidence,
                        session.slot_within_direction(),
                        session.current_direction(),
                        inputs.run_sequence,
                        evidence_sequence,
                    ))
                }
                BandConfirmationStep::Done(outcome) => confirmation_done_advance(
                    outcome,
                    session,
                    inputs,
                    safety,
                    backend,
                    *nomination_evidence,
                    evidence_sequence,
                ),
            },
        }
    };
    let step = match advance {
        CampaignAdvance::Step(step) => step,
        CampaignAdvance::Emit { now, next } => {
            match next {
                EmitContinuation::EnterPhase(next_phase) => campaign.phase = next_phase,
                EmitContinuation::Terminal {
                    campaign_terminal,
                    done,
                } => {
                    campaign.pending_event = Some(campaign_terminal);
                    campaign.pending_done = Some(done);
                }
            }
            BreakawayCampaignStep::Event(now)
        }
    };
    checkpoint_event_routing(step)
}

pub(crate) async fn step_breakaway_campaign_with_trace<B: CommissioningBackend>(
    phase: &mut OuterPhase,
    backend: &mut B,
) -> StepResult {
    let step = {
        let OuterPhase::BreakawayCampaign {
            campaign,
            workspace,
            safety,
        } = phase
        else {
            unreachable!("step_breakaway_campaign_with_trace requires a breakaway campaign phase")
        };
        step_breakaway_campaign(campaign, workspace, safety, backend).await
    };
    match step {
        BreakawayCampaignStep::ReadyNow => StepResult::ReadyNow,
        BreakawayCampaignStep::At(deadline_us) => StepResult::At(deadline_us),
        BreakawayCampaignStep::Event(event) => StepResult::BreakawayCampaignEvent(event),
        BreakawayCampaignStep::Done(BreakawayCampaignOutcome::IntegralPlanReady(plan)) => {
            let old =
                core::mem::replace(phase, OuterPhase::Failed(CommissionError::InvalidSchedule));
            let OuterPhase::BreakawayCampaign {
                campaign, safety, ..
            } = old
            else {
                unreachable!("phase was matched as a breakaway campaign above")
            };
            let run_sequence = campaign.run_sequence();
            *phase = OuterPhase::VelocityIntegral {
                state: VelocityIntegralState::Setup,
                data: VelocityIntegralRuntime::new(run_sequence, plan.execution_plan()),
                safety,
            };
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::IntegralHandedOff,
            );
            StepResult::BreakawayIntegralPlanReady { run_sequence, plan }
        }
        BreakawayCampaignStep::Done(BreakawayCampaignOutcome::Terminal {
            error,
            safety: Some(fault),
        }) => {
            let result = fail_outer_with_safety(phase, error, fault, Default::default());
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
            );
            result
        }
        BreakawayCampaignStep::Done(BreakawayCampaignOutcome::Terminal {
            error,
            safety: None,
        }) => {
            let result = fail_outer(phase, error);
            crate::diagnostic::complete_breakaway_campaign(
                crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
            );
            result
        }
    }
}

#[cfg(test)]
#[path = "breakaway_campaign_tests.rs"]
mod tests;
