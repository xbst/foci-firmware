//! Cooperative state machine for rung-origin recovery.

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::outer::{OuterSafetyLimits, check_outer_safety};
use crate::commissioning::types::{OUTER_SAFETY_FAULT_CURRENT, OuterSafetyFault};
use crate::diagnostic::BreakawayCheckpoint;
use foci_tmc4671::pid::{PidGainPair, PidIGain};
use foci_tmc4671::registers::{PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET};

use super::super::clear_velocity_error_sum;
use super::super::sweep_analysis::{
    ScoredRestPoll, bounded_rest_displacement, poll_scored_rest_confirmation,
};
use super::super::sweep_plan::{SETTLE_ANALYSIS_BUDGET_US, ZERO_SETTLE_US};
use super::super::velocity_clamp::{
    VelocityCurrentContact, VelocityCurrentGuardError, VelocityCurrentZeroPhase,
    finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    read_velocity_pid_input_abs, read_velocity_pid_output_abs, restart_velocity_current_primitive,
};
use super::{
    ExactU64Parts, RECOVERY_STABILIZATION_US, RecoveryBindingSource,
    RecoveryOriginMismatchEvidence, RecoveryRestAnalysis, RecoveryStabilizationSummary,
    RecoveryState, RungOriginRecoveryOutcome, RungOriginRecoveryResources,
    RungOriginRecoveryRuntime, RungOriginRecoveryStep, recovery_return_distance_counts,
};

/// Advance one bounded unit of shared rung-origin recovery work.
pub(crate) async fn step_rung_origin_recovery<B: CommissioningBackend>(
    runtime: &mut RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &mut B,
) -> RungOriginRecoveryStep {
    if backend.is_shutdown_requested() {
        return recovery_fault(CommissionError::Shutdown, None);
    }
    match runtime.state {
        RecoveryState::Terminal {
            outcome,
            binding_source,
        } => {
            let report = runtime.report(
                outcome,
                binding_source,
                resources.safety.unwrapped_position,
                backend.elapsed_us(),
                resources.current_observer.peak_pid_output_abs(),
            );
            if matches!(
                outcome,
                RungOriginRecoveryOutcome::AlreadyCentered
                    | RungOriginRecoveryOutcome::Recovered
                    | RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom
            ) {
                RungOriginRecoveryStep::Finished(report)
            } else {
                RungOriginRecoveryStep::Contained(report)
            }
        }
        RecoveryState::RestTerminal {
            confirmed_rest,
            in_band,
            contained,
            current_contact,
        } => {
            if current_contact {
                let outcome = if runtime.proves_passive_settle() && confirmed_rest && in_band {
                    RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom
                } else {
                    RungOriginRecoveryOutcome::RecoveryCurrentHeadroom
                };
                let report = runtime.report(
                    outcome,
                    RecoveryBindingSource::CurrentHeadroom,
                    resources.safety.unwrapped_position,
                    backend.elapsed_us(),
                    resources.current_observer.peak_pid_output_abs(),
                );
                return if matches!(
                    outcome,
                    RungOriginRecoveryOutcome::RecoveredWithCurrentHeadroom
                ) {
                    RungOriginRecoveryStep::Finished(report)
                } else {
                    RungOriginRecoveryStep::Contained(report)
                };
            }
            if !confirmed_rest {
                return RungOriginRecoveryStep::RestUnconfirmed;
            }
            if contained || !in_band {
                let report = runtime.report(
                    RungOriginRecoveryOutcome::ContainedDeadline,
                    RecoveryBindingSource::Deadline,
                    resources.safety.unwrapped_position,
                    backend.elapsed_us(),
                    resources.current_observer.peak_pid_output_abs(),
                );
                return RungOriginRecoveryStep::Contained(report);
            }
            let report = runtime.report(
                RungOriginRecoveryOutcome::Recovered,
                runtime.authority_source,
                resources.safety.unwrapped_position,
                backend.elapsed_us(),
                resources.current_observer.peak_pid_output_abs(),
            );
            RungOriginRecoveryStep::Finished(report)
        }
        RecoveryState::OriginMismatchContained => {
            runtime.state = RecoveryState::Terminal {
                outcome: RungOriginRecoveryOutcome::ContainedDeadline,
                binding_source: RecoveryBindingSource::LocalTravel,
            };
            RungOriginRecoveryStep::Contained(runtime.report(
                RungOriginRecoveryOutcome::ContainedDeadline,
                RecoveryBindingSource::LocalTravel,
                resources.safety.unwrapped_position,
                backend.elapsed_us(),
                resources.current_observer.peak_pid_output_abs(),
            ))
        }
        RecoveryState::WriteGains => {
            let start_side_window = if runtime.start_offset_counts < 0 {
                resources.safety.negative_position_window
            } else {
                resources.safety.positive_position_window
            };
            if runtime.start_offset_counts.unsigned_abs() > start_side_window {
                runtime.state = RecoveryState::Terminal {
                    outcome: RungOriginRecoveryOutcome::ContainedDeadline,
                    binding_source: RecoveryBindingSource::LocalTravel,
                };
                return RungOriginRecoveryStep::Contained(runtime.report(
                    RungOriginRecoveryOutcome::ContainedDeadline,
                    RecoveryBindingSource::LocalTravel,
                    resources.safety.unwrapped_position,
                    backend.elapsed_us(),
                    resources.current_observer.peak_pid_output_abs(),
                ));
            }
            let origin_drift_counts = resources
                .safety
                .unwrapped_position
                .saturating_sub(runtime.start_offset_counts);
            if origin_drift_counts.unsigned_abs() > runtime.plan.origin_band_counts {
                runtime.state = RecoveryState::OriginMismatchContained;
                return RungOriginRecoveryStep::OriginMismatch(RecoveryOriginMismatchEvidence {
                    expected_counts: runtime.start_offset_counts,
                    observed_counts: resources.safety.unwrapped_position,
                    position_window: resources.safety.position_window,
                    stage: runtime.stage,
                    rung_index: runtime.rung_index,
                    p_raw: runtime.p_raw(),
                });
            }
            checkpoint(BreakawayCheckpoint::RecoveryGainWrite);
            if backend
                .write_register(
                    PID_VELOCITY_P_VELOCITY_I,
                    PidGainPair::new(runtime.p, PidIGain::ZERO).register_value(),
                )
                .await
                .is_err()
            {
                return recovery_fault(CommissionError::Spi, None);
            }
            runtime.state = RecoveryState::ClearMovingIntegrator;
            RungOriginRecoveryStep::ReadyNow
        }
        RecoveryState::ClearMovingIntegrator => {
            checkpoint(BreakawayCheckpoint::RecoveryMovingIntegratorClear);
            if clear_velocity_error_sum(backend).await.is_err() {
                return recovery_fault(CommissionError::Spi, None);
            }
            runtime.state = RecoveryState::CommandTowardOrigin;
            RungOriginRecoveryStep::ReadyNow
        }
        RecoveryState::CommandTowardOrigin => {
            let direction = if resources.safety.unwrapped_position > 0 {
                -1
            } else {
                1
            };
            let target = match runtime
                .target_velocity_rpm
                .with_direction(direction)
                .map_err(|_| CommissionError::TargetRateInvalid)
            {
                Ok(target) => target.raw(),
                Err(error) => return recovery_fault(error, None),
            };
            let started_us = backend.elapsed_us();
            if resources
                .safety
                .begin_velocity_step_primitive(started_us)
                .is_err()
            {
                return recovery_fault(CommissionError::PrimitiveAlreadyOpen, None);
            }
            checkpoint(BreakawayCheckpoint::RecoveryCurrentPrimitiveStart);
            if let Err(error) =
                restart_velocity_current_primitive(resources.current_observer, backend).await
            {
                resources.safety.end_primitive();
                return current_guard_fault(runtime, resources, backend, error);
            }
            checkpoint(BreakawayCheckpoint::RecoveryTargetWrite);
            if backend
                .write_register(PID_VELOCITY_TARGET, target as u32)
                .await
                .is_err()
            {
                resources.safety.end_primitive();
                return recovery_fault(CommissionError::Spi, None);
            }
            let Some(moving_budget_us) =
                runtime.runtime_us.checked_sub(runtime.rest_allowance_us())
            else {
                resources.safety.end_primitive();
                return recovery_fault(CommissionError::InvalidSchedule, None);
            };
            let Some(moving_deadline_us) = started_us.checked_add(u64::from(moving_budget_us))
            else {
                resources.safety.end_primitive();
                return recovery_fault(CommissionError::ScheduleDeadlineOverflow, None);
            };
            runtime.recovery_started_us = ExactU64Parts::from_u64(started_us);
            runtime.mark_motion_started();
            runtime.state = RecoveryState::Observe {
                moving_deadline_us: ExactU64Parts::from_u64(moving_deadline_us),
            };
            RungOriginRecoveryStep::At(
                resources
                    .safety
                    .next_safety_deadline()
                    .unwrap_or(started_us),
            )
        }
        RecoveryState::Observe { moving_deadline_us } => {
            let moving_deadline_us = moving_deadline_us.to_u64();
            if let Some(step) = recovery_safety_sample(runtime, resources, backend, false).await {
                return step;
            }
            let offset = resources.safety.unwrapped_position;
            update_closest(runtime, offset);
            if wrong_way(
                runtime.start_offset_counts,
                offset,
                runtime.plan.origin_band_counts,
            ) {
                let elapsed_us = backend
                    .elapsed_us()
                    .saturating_sub(runtime.recovery_started_us.to_u64());
                let fault = resources
                    .safety
                    .recovery_wrong_way_fault(runtime.start_offset_counts, elapsed_us);
                return recovery_fault(CommissionError::SafetyEnvelopeViolation, Some(fault));
            }
            let in_band = offset.unsigned_abs() <= runtime.plan.origin_band_counts;
            let reached_target = offset.unsigned_abs() <= runtime.moving_target_counts;
            let deadline_reached = backend.elapsed_us() >= moving_deadline_us;
            if reached_target || deadline_reached {
                runtime.first_in_band_offset_counts = offset;
                runtime.entry_to_band_duration_us = backend
                    .elapsed_us()
                    .saturating_sub(runtime.recovery_started_us.to_u64())
                    .min(u64::from(u32::MAX))
                    as u32;
                runtime.lead_in_first_or_net_counts = offset;
                runtime.lead_in_min_or_span_counts = offset;
                checkpoint(BreakawayCheckpoint::RecoveryMovingBoundary);
                let current_result =
                    finish_velocity_current_moving_phase(resources.current_observer, backend).await;
                checkpoint(BreakawayCheckpoint::RecoveryZeroTargetWrite);
                let zero_result = backend.write_register(PID_VELOCITY_TARGET, 0).await;
                if zero_result.is_err() {
                    return recovery_fault(CommissionError::Spi, None);
                }
                resources.safety.enter_settle_phase();
                let limiter_contact = match current_result {
                    Ok(()) => false,
                    Err(VelocityCurrentGuardError::LimiterContact(status)) => {
                        defmt::warn!(
                            "recovery moving boundary limiter contact: status={:#x}",
                            status
                        );
                        true
                    }
                    Err(error) => return current_guard_fault(runtime, resources, backend, error),
                };
                if (limiter_contact || recovery_current_contact(resources.current_observer))
                    && !runtime.current_contact_detected()
                {
                    return recovery_current_headroom(runtime, resources, backend).await;
                }
                runtime.zero_commanded_us = ExactU64Parts::from_u64(backend.elapsed_us());
                runtime.state = RecoveryState::ClearSettleIntegrator {
                    settle_started_us: runtime.zero_commanded_us,
                    contained: deadline_reached && !in_band,
                    lead_in: true,
                };
                RungOriginRecoveryStep::ReadyNow
            } else {
                RungOriginRecoveryStep::At(
                    resources
                        .safety
                        .next_safety_deadline()
                        .unwrap_or(backend.elapsed_us()),
                )
            }
        }
        RecoveryState::ClearSettleIntegrator {
            settle_started_us,
            contained,
            lead_in,
        } => {
            if !runtime.settle_integrator_cleared() {
                checkpoint(BreakawayCheckpoint::RecoverySettleIntegratorClear);
                if clear_velocity_error_sum(backend).await.is_err() {
                    return recovery_fault(CommissionError::Spi, None);
                }
            }
            let capture_us = if lead_in {
                RECOVERY_STABILIZATION_US
            } else {
                ZERO_SETTLE_US
            };
            let Some(deadline_us) = settle_started_us
                .to_u64()
                .checked_add(u64::from(capture_us))
            else {
                return recovery_fault(CommissionError::InvalidSchedule, None);
            };
            resources.observations.clear();
            resources.analysis.reset();
            *resources.settle_displacement_counts = 0;
            *resources.settle_min_displacement_counts = 0;
            *resources.settle_max_displacement_counts = 0;
            let next_sample_us = resources
                .safety
                .next_safety_deadline()
                .unwrap_or(backend.elapsed_us());
            runtime.state = RecoveryState::CaptureSettle {
                next_sample_us: ExactU64Parts::from_u64(next_sample_us),
                deadline_us: ExactU64Parts::from_u64(deadline_us),
                contained,
                lead_in,
            };
            RungOriginRecoveryStep::At(next_sample_us)
        }
        RecoveryState::CaptureSettle {
            next_sample_us,
            deadline_us,
            contained,
            lead_in,
        } => {
            let next_sample_us = next_sample_us.to_u64();
            let deadline_us = deadline_us.to_u64();
            if backend.elapsed_us() < next_sample_us {
                return RungOriginRecoveryStep::At(next_sample_us);
            }
            if let Some(step) = recovery_safety_sample(runtime, resources, backend, true).await {
                return step;
            }
            let offset = resources.safety.unwrapped_position;
            update_closest(runtime, offset);
            if wrong_way(
                runtime.start_offset_counts,
                offset,
                runtime.plan.origin_band_counts,
            ) {
                let elapsed_us = backend
                    .elapsed_us()
                    .saturating_sub(runtime.recovery_started_us.to_u64());
                let fault = resources
                    .safety
                    .recovery_wrong_way_fault(runtime.start_offset_counts, elapsed_us);
                return recovery_fault(CommissionError::SafetyEnvelopeViolation, Some(fault));
            }
            let Some((delta, elapsed_us)) = resources.safety.last_accepted_observation() else {
                return recovery_fault(CommissionError::MissingCaptureState, None);
            };
            let Some(elapsed_us) = u16::try_from(elapsed_us).ok() else {
                return recovery_fault(CommissionError::ElapsedTimeConversionInvalid, None);
            };
            if lead_in {
                runtime.lead_in_first_or_net_counts =
                    runtime.lead_in_first_or_net_counts.min(offset);
                runtime.lead_in_min_or_span_counts = runtime.lead_in_min_or_span_counts.max(offset);
            } else {
                *resources.settle_displacement_counts =
                    i32::from(*resources.settle_displacement_counts)
                        .saturating_add(delta)
                        .clamp(-2, 2) as i8;
                *resources.settle_min_displacement_counts = (*resources
                    .settle_min_displacement_counts)
                    .min(*resources.settle_displacement_counts);
                *resources.settle_max_displacement_counts = (*resources
                    .settle_max_displacement_counts)
                    .max(*resources.settle_displacement_counts);
            }
            if resources.observations.push(delta, elapsed_us).is_err() {
                return recovery_fault(CommissionError::ZeroElapsedSample, None);
            }
            let read_us = backend.elapsed_us();
            if read_us >= deadline_us {
                if lead_in {
                    let minimum = runtime.lead_in_first_or_net_counts;
                    let maximum = runtime.lead_in_min_or_span_counts;
                    runtime.lead_in_first_or_net_counts =
                        offset.saturating_sub(runtime.first_in_band_offset_counts);
                    runtime.lead_in_min_or_span_counts =
                        i32::try_from(maximum.saturating_sub(minimum).unsigned_abs())
                            .unwrap_or(i32::MAX);
                    resources.observations.clear();
                    resources.analysis.reset();
                    *resources.settle_displacement_counts = 0;
                    *resources.settle_min_displacement_counts = 0;
                    *resources.settle_max_displacement_counts = 0;
                    let Some(scored_deadline_us) = read_us.checked_add(u64::from(ZERO_SETTLE_US))
                    else {
                        return recovery_fault(CommissionError::InvalidSchedule, None);
                    };
                    let next = resources.safety.next_safety_deadline().unwrap_or(read_us);
                    runtime.state = RecoveryState::CaptureSettle {
                        next_sample_us: ExactU64Parts::from_u64(next),
                        deadline_us: ExactU64Parts::from_u64(scored_deadline_us),
                        contained,
                        lead_in: false,
                    };
                    return RungOriginRecoveryStep::At(next);
                }
                let Some(analysis_deadline_us) =
                    read_us.checked_add(u64::from(SETTLE_ANALYSIS_BUDGET_US))
                else {
                    return recovery_fault(CommissionError::InvalidSchedule, None);
                };
                resources
                    .analysis
                    .reset_for_scored_rest(bounded_rest_displacement(
                        *resources.settle_min_displacement_counts,
                        *resources.settle_max_displacement_counts,
                    ));
                runtime.state = RecoveryState::AnalyzeSettle {
                    analysis_deadline_us: ExactU64Parts::from_u64(analysis_deadline_us),
                    contained,
                };
                RungOriginRecoveryStep::ReadyNow
            } else {
                let next = resources.safety.next_safety_deadline().unwrap_or(read_us);
                runtime.state = RecoveryState::CaptureSettle {
                    next_sample_us: ExactU64Parts::from_u64(next),
                    deadline_us: ExactU64Parts::from_u64(deadline_us),
                    contained,
                    lead_in,
                };
                RungOriginRecoveryStep::At(next)
            }
        }
        RecoveryState::AnalyzeSettle {
            analysis_deadline_us,
            contained,
        } => {
            let analysis_deadline_us = analysis_deadline_us.to_u64();
            if backend.elapsed_us() >= analysis_deadline_us {
                return recovery_fault(CommissionError::AnalysisOverrun, None);
            }
            let confirmation = match poll_scored_rest_confirmation(
                resources.analysis,
                resources.observations,
                resources.blocker,
            ) {
                Ok(ScoredRestPoll::Pending) => return RungOriginRecoveryStep::ReadyNow,
                Ok(ScoredRestPoll::Diagnostic(decision)) => {
                    return RungOriginRecoveryStep::RestSelection {
                        decision,
                        stage: runtime.stage,
                        rung_index: runtime.rung_index,
                    };
                }
                Ok(ScoredRestPoll::Complete(result)) => result,
                Err(error) => return recovery_fault(error, None),
            };
            checkpoint(BreakawayCheckpoint::RecoveryZeroBoundary);
            let limiter_contact =
                match finish_velocity_current_zero_phase(resources.current_observer, backend).await
                {
                    Ok(()) => false,
                    Err(VelocityCurrentGuardError::LimiterContact(status)) => {
                        defmt::warn!(
                            "recovery zero boundary limiter contact: status={:#x}",
                            status
                        );
                        true
                    }
                    Err(error) => return current_guard_fault(runtime, resources, backend, error),
                };
            let current_contact = if runtime.current_contact_detected() {
                true
            } else if limiter_contact || recovery_current_contact(resources.current_observer) {
                if let Some(fault) = hold_zero_for_current_contact(resources.safety, backend).await
                {
                    return fault;
                }
                true
            } else {
                false
            };
            resources.analysis.reset();
            resources.observations.clear();
            let final_offset = resources.safety.unwrapped_position;
            let in_band = final_offset.unsigned_abs() <= runtime.plan.origin_band_counts;
            resources.safety.end_primitive();
            let confirmed = confirmation.confirmed_rest && in_band;
            runtime.state = RecoveryState::RestTerminal {
                confirmed_rest: confirmation.confirmed_rest,
                in_band,
                contained,
                current_contact,
            };
            let exact_velocity_interval_q =
                confirmation.exact_velocity_interval_q.unwrap_or([0; 2]);
            let interval_present = u8::from(confirmation.exact_velocity_interval_q.is_some());
            let current_contact_code = resources
                .current_observer
                .evidence(VelocityCurrentZeroPhase::Recovery)
                .map_or(VelocityCurrentContact::None as u8, |evidence| {
                    evidence.contact as u8
                });
            RungOriginRecoveryStep::RestAnalysis(RecoveryRestAnalysis {
                exact_velocity_interval_q,
                start_offset_counts: runtime.start_offset_counts,
                closest_offset_counts: runtime.closest_offset_counts,
                final_offset_counts: final_offset,
                planned_return_distance_counts: recovery_return_distance_counts(
                    runtime.start_offset_counts,
                    runtime.plan.origin_band_counts,
                )
                .unwrap_or(0),
                first_in_band_offset_counts: runtime.first_in_band_offset_counts,
                displacement_to_band_counts: runtime
                    .first_in_band_offset_counts
                    .saturating_sub(runtime.start_offset_counts),
                entry_to_band_duration_us: runtime.entry_to_band_duration_us,
                lead_in: RecoveryStabilizationSummary {
                    net_displacement_counts: runtime.lead_in_first_or_net_counts,
                    displacement_span_counts: runtime.lead_in_min_or_span_counts.unsigned_abs(),
                },
                scored_net_displacement_counts: i32::from(*resources.settle_displacement_counts),
                scored_min_displacement_counts: i32::from(
                    *resources.settle_min_displacement_counts,
                ),
                scored_max_displacement_counts: i32::from(
                    *resources.settle_max_displacement_counts,
                ),
                p_raw: runtime.p.raw(),
                peak_torque_target_abs: resources.current_observer.peak_pid_output_abs(),
                stage: runtime.stage,
                rung_index: runtime.rung_index,
                stationarity_code: confirmation.recent_structural_code as u8,
                interval_present,
                current_contact_code,
                disposition: u8::from(!confirmed),
            })
        }
    }
}

async fn recovery_safety_sample<B: CommissioningBackend>(
    runtime: &mut RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &mut B,
    zero_phase: bool,
) -> Option<RungOriginRecoveryStep> {
    checkpoint(if zero_phase {
        BreakawayCheckpoint::RecoverySettleSafetySample
    } else {
        BreakawayCheckpoint::RecoveryMovingSafetySample
    });
    match check_outer_safety(resources.safety, backend).await {
        Ok(Some(fault)) => {
            return Some(recovery_fault(
                CommissionError::SafetyEnvelopeViolation,
                Some(fault),
            ));
        }
        Err(error) => return Some(recovery_fault(error, None)),
        Ok(None) => {}
    }
    checkpoint(if zero_phase {
        BreakawayCheckpoint::RecoverySettleCurrentInputSample
    } else {
        BreakawayCheckpoint::RecoveryMovingCurrentInputSample
    });
    let torque_demand = match read_velocity_pid_input_abs(backend).await {
        Ok(value) => value,
        Err(error) => return Some(current_guard_fault(runtime, resources, backend, error)),
    };
    if torque_demand > resources.current_observer.clamp_limit() {
        return Some(current_limit_fault(runtime, resources, backend));
    }
    checkpoint(if zero_phase {
        BreakawayCheckpoint::RecoverySettleCurrentOutputSample
    } else {
        BreakawayCheckpoint::RecoveryMovingCurrentOutputSample
    });
    let torque_target = match read_velocity_pid_output_abs(backend).await {
        Ok(value) => value,
        Err(error) => return Some(current_guard_fault(runtime, resources, backend, error)),
    };
    if zero_phase {
        resources
            .current_observer
            .observe_zero_output(torque_target);
    } else {
        resources
            .current_observer
            .observe_moving_output(torque_target);
    }
    if recovery_current_contact(resources.current_observer) && !runtime.current_contact_detected() {
        return Some(recovery_current_headroom(runtime, resources, backend).await);
    }
    None
}

async fn hold_zero_for_current_contact<B: CommissioningBackend>(
    safety: &mut OuterSafetyLimits,
    backend: &mut B,
) -> Option<RungOriginRecoveryStep> {
    checkpoint(BreakawayCheckpoint::RecoveryZeroTargetWrite);
    if backend
        .write_register(PID_VELOCITY_TARGET, 0)
        .await
        .is_err()
    {
        return Some(recovery_fault(CommissionError::Spi, None));
    }
    safety.enter_settle_phase();
    checkpoint(BreakawayCheckpoint::RecoverySettleIntegratorClear);
    if clear_velocity_error_sum(backend).await.is_err() {
        return Some(recovery_fault(CommissionError::Spi, None));
    }
    None
}

async fn recovery_current_headroom<B: CommissioningBackend>(
    runtime: &mut RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &mut B,
) -> RungOriginRecoveryStep {
    if let Some(fault) = hold_zero_for_current_contact(resources.safety, backend).await {
        return fault;
    }
    if runtime.proves_passive_settle() {
        runtime.mark_current_contact_detected();
        if matches!(runtime.state, RecoveryState::Observe { .. }) {
            runtime.zero_commanded_us = ExactU64Parts::from_u64(backend.elapsed_us());
            runtime.state = RecoveryState::ClearSettleIntegrator {
                settle_started_us: runtime.zero_commanded_us,
                contained: false,
                lead_in: true,
            };
            return RungOriginRecoveryStep::ReadyNow;
        }
        return RungOriginRecoveryStep::ReadyNow;
    }

    resources.safety.end_primitive();
    let outcome = RungOriginRecoveryOutcome::RecoveryCurrentHeadroom;
    let binding_source = RecoveryBindingSource::CurrentHeadroom;
    runtime.state = RecoveryState::Terminal {
        outcome,
        binding_source,
    };
    RungOriginRecoveryStep::Contained(runtime.report(
        outcome,
        binding_source,
        resources.safety.unwrapped_position,
        backend.elapsed_us(),
        resources.current_observer.peak_pid_output_abs(),
    ))
}

fn recovery_current_contact(
    observer: &super::super::velocity_clamp::VelocityCurrentObserver,
) -> bool {
    observer
        .evidence(VelocityCurrentZeroPhase::Recovery)
        .map(|evidence| evidence.contact != VelocityCurrentContact::None)
        .unwrap_or(true)
}

fn current_guard_fault<B: CommissioningBackend>(
    runtime: &RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &B,
    error: VelocityCurrentGuardError,
) -> RungOriginRecoveryStep {
    match error {
        VelocityCurrentGuardError::RegisterIo => recovery_fault(CommissionError::Spi, None),
        VelocityCurrentGuardError::InvalidLimit
        | VelocityCurrentGuardError::ActionableStatus(_)
        | VelocityCurrentGuardError::LimiterContact(_) => {
            current_limit_fault(runtime, resources, backend)
        }
    }
}

fn current_limit_fault<B: CommissioningBackend>(
    runtime: &RungOriginRecoveryRuntime,
    resources: &mut RungOriginRecoveryResources<'_>,
    backend: &B,
) -> RungOriginRecoveryStep {
    let elapsed_us = backend
        .elapsed_us()
        .saturating_sub(runtime.recovery_started_us.to_u64());
    let fault = resources
        .safety
        .fault(OUTER_SAFETY_FAULT_CURRENT, 0, 0, 0, elapsed_us);
    recovery_fault(CommissionError::SafetyEnvelopeViolation, Some(fault))
}

fn update_closest(runtime: &mut RungOriginRecoveryRuntime, offset: i32) {
    if offset.unsigned_abs() < runtime.closest_offset_counts.unsigned_abs() {
        runtime.closest_offset_counts = offset;
    }
}

fn wrong_way(start_offset: i32, current_offset: i32, origin_band_counts: u32) -> bool {
    current_offset.unsigned_abs()
        > start_offset
            .unsigned_abs()
            .saturating_add(origin_band_counts)
}

fn recovery_fault(
    error: CommissionError,
    safety: Option<OuterSafetyFault>,
) -> RungOriginRecoveryStep {
    RungOriginRecoveryStep::Fault { error, safety }
}

fn checkpoint(checkpoint: BreakawayCheckpoint) {
    crate::diagnostic::checkpoint_active_breakaway_campaign(checkpoint);
}
