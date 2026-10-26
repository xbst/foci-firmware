use foci_tmc4671::current_limit::PidTorqueFluxLimit;

use super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use super::sweep_plan::{RequestedVelocitySource, VelocitySweepRequest, plan_velocity_sweep};
use super::velocity_clamp::{
    PhaseObservation, VelocityCurrentClampSession, VelocityCurrentContact,
    VelocityCurrentGuardError, VelocityCurrentObserver, VelocityCurrentZeroPhase,
    VelocityLimitEvidenceCapability, begin_velocity_current_primitive, classify_phase_evidence,
    derive_probe_limit, finish_velocity_current_moving_phase, finish_velocity_current_zero_phase,
    restart_velocity_current_primitive,
};
use crate::commission_dispatch::test_support::{
    MockBackend, SpiOp, breakaway_confirmation_for_test,
};
use crate::commissioning::inner::closed_loop_activation::STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE;
use crate::commissioning::outer::OuterMotionBudget;
use foci_tmc4671::registers::{PID_TORQUE_FLUX_LIMITS, STATUS_FLAGS};

const PID_V_OUTPUT_LIMIT: u32 = 1 << 7;
const ADC_I_CLIPPED_BIT: u32 = 1 << 26;

#[test]
fn velocity_current_guard_contract() {
    for (operational, expected_probe) in [(2_816, Some(2_534)), (10, Some(9)), (1, None)] {
        let result = derive_probe_limit(PidTorqueFluxLimit::new(operational).unwrap());
        assert_eq!(result.ok().map(PidTorqueFluxLimit::raw), expected_probe);
    }
    assert!(PidTorqueFluxLimit::new(0x8000).is_err());
    assert!(
        classify_phase_evidence(
            PidTorqueFluxLimit::ZERO,
            PhaseObservation {
                pid_output_peak: 0,
                status_flags: 0
            },
            PhaseObservation {
                pid_output_peak: 0,
                status_flags: 0
            },
            VelocityLimitEvidenceCapability::BestEffortPolling,
            VelocityCurrentZeroPhase::Braking
        )
        .is_err()
    );

    let moving_flags = PID_V_OUTPUT_LIMIT | (1 << 3) | (1 << 26);
    let moving = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        PhaseObservation {
            pid_output_peak: 2_533,
            status_flags: moving_flags,
        },
        PhaseObservation {
            pid_output_peak: 0,
            status_flags: 0,
        },
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(moving.contact, VelocityCurrentContact::Moving);
    assert_eq!(moving.moving_status_flags, moving_flags);
    assert!(!moving.moving_clip_absence_proven());
    assert!(!moving.moving_clip_uncertain());

    let zero_flags = PID_V_OUTPUT_LIMIT | (1 << 15) | (1 << 26);
    let braking = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        PhaseObservation {
            pid_output_peak: 2_533,
            status_flags: 1 << 3,
        },
        PhaseObservation {
            pid_output_peak: 2_534,
            status_flags: zero_flags,
        },
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(braking.contact, VelocityCurrentContact::Braking);
    assert_eq!(braking.moving_status_flags, 1 << 3);
    assert_eq!(braking.zero_status_flags, zero_flags);
    assert!(braking.moving_clip_absence_proven());

    let recovery = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        PhaseObservation {
            pid_output_peak: 2_000,
            status_flags: 0,
        },
        PhaseObservation {
            pid_output_peak: 2_534,
            status_flags: PID_V_OUTPUT_LIMIT,
        },
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Recovery,
    )
    .unwrap();
    assert_eq!(recovery.contact, VelocityCurrentContact::Recovery);

    let best_effort = classify_phase_evidence(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        PhaseObservation {
            pid_output_peak: 2_533,
            status_flags: 0,
        },
        PhaseObservation {
            pid_output_peak: 2_000,
            status_flags: 0,
        },
        VelocityLimitEvidenceCapability::BestEffortPolling,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(best_effort.contact, VelocityCurrentContact::None);
    assert!(!best_effort.moving_clip_absence_proven());
    assert!(best_effort.moving_clip_uncertain());

    let target_rpm = 176_i32;
    let advanced_meter_quantum_rpm = 375_i32;
    let p_raw = 2_048_i32;
    let spin_up_output = (p_raw * target_rpm / 256).unsigned_abs() as u16;
    let reversal_output =
        (p_raw * (target_rpm - advanced_meter_quantum_rpm) / 256).unsigned_abs() as u16;
    assert_eq!((spin_up_output, reversal_output), (1_408, 1_592));

    let latched_quantized = classify_phase_evidence(
        PidTorqueFluxLimit::new(1_592).unwrap(),
        PhaseObservation {
            pid_output_peak: 1_591,
            status_flags: PID_V_OUTPUT_LIMIT,
        },
        PhaseObservation {
            pid_output_peak: 0,
            status_flags: 0,
        },
        VelocityLimitEvidenceCapability::LatchedFlag,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(latched_quantized.contact, VelocityCurrentContact::Moving);

    let polled_quantized = classify_phase_evidence(
        PidTorqueFluxLimit::new(1_592).unwrap(),
        PhaseObservation {
            pid_output_peak: 1_591,
            status_flags: 0,
        },
        PhaseObservation {
            pid_output_peak: 0,
            status_flags: 0,
        },
        VelocityLimitEvidenceCapability::BestEffortPolling,
        VelocityCurrentZeroPhase::Braking,
    )
    .unwrap();
    assert_eq!(polled_quantized.contact, VelocityCurrentContact::None);
    assert!(polled_quantized.moving_clip_uncertain());

    let integral = breakaway_integral_plan(VelocityLimitEvidenceCapability::LatchedFlag);
    assert_eq!(
        integral.limit_evidence_capability,
        VelocityLimitEvidenceCapability::LatchedFlag
    );
    assert_eq!(
        integral.execution_plan().limit_evidence_capability,
        VelocityLimitEvidenceCapability::LatchedFlag
    );
    let best_effort_integral =
        breakaway_integral_plan(VelocityLimitEvidenceCapability::BestEffortPolling);
    assert_eq!(
        best_effort_integral.limit_evidence_capability,
        VelocityLimitEvidenceCapability::BestEffortPolling
    );
    assert_ne!(integral.digest(), best_effort_integral.digest());
}

fn breakaway_integral_plan(
    limit_evidence_capability: VelocityLimitEvidenceCapability,
) -> crate::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan {
    use crate::commissioning::outer::velocity::integral_plan::{
        AcquisitionContext, BreakawayIntegralEvidence, IntegralPlanPhysics,
        plan_breakaway_velocity_integral_response,
    };

    let confirmation = breakaway_confirmation_for_test();

    let context = AcquisitionContext {
        planned_velocity_mrev_s: 2_929,
        target_velocity_rpm: 176,
        hard_torque_limit: 2_816,
        usable_torque_limit: 2_534,
        limit_evidence_capability,
        channel: 0,
        plan_time_pwm_hz: 25_000,
        plan_time_encoder_counts_per_rev: 4_000,
    };
    plan_breakaway_velocity_integral_response(
        confirmation.token,
        confirmation.confirmation_plan,
        context,
        BreakawayIntegralEvidence {
            held_out_response_intervals_q: confirmation.held_out_response_intervals_q,
            confidence_q: confirmation.confidence_q,
            band_position_q: confirmation.band_position_q,
        },
        IntegralPlanPhysics {
            motion_budget: OuterMotionBudget {
                requested_velocity_mrev_s: 6_666,
                machine_velocity_ceiling_mrev_s: 10_000,
                requested_velocity_source: RequestedVelocitySource::Defaulted,
                max_stroke_travel_mrev: 2_500,
                settle_travel_reserve_mrev: 250,
                negative_position_headroom_mrev: 7_025,
                positive_position_headroom_mrev: 7_025,
                max_duration_ms: 3_000,
                homing_speed_mrev_s: 4_375,
                max_accel_mrev_s2: 200_000,
            },
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
        },
    )
    .unwrap()
}

#[test]
fn velocity_current_prevention_configuration_matrix() {
    const SUPPORTED_PWM_HZ: [u32; 2] = [25_000, 50_000];
    const SUPPORTED_VELOCITY_METERS: [u8; 1] = [0];
    const SUPPORTED_VELOCITY_FILTERS_ENABLED: [bool; 2] = [false, true];

    for encoder_counts_per_rev in [1_000_u32, 65_536] {
        for pwm_hz in SUPPORTED_PWM_HZ {
            let meter_quantum_rpm =
                STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE.div_ceil(encoder_counts_per_rev);
            for target_rpm in [
                meter_quantum_rpm.saturating_sub(1).max(1),
                meter_quantum_rpm,
                meter_quantum_rpm + 1,
            ] {
                for velocity_meter in SUPPORTED_VELOCITY_METERS {
                    for filter_enabled in SUPPORTED_VELOCITY_FILTERS_ENABLED {
                        for operational_limit in [10_u32, 32_767] {
                            for (bed_mm, max_stroke_travel_mrev, headroom_mrev) in
                                [(120_u16, 2_500_u32, 3_000_u32), (300, 6_250, 7_500)]
                            {
                                let probe = derive_probe_limit(
                                    PidTorqueFluxLimit::new(operational_limit).unwrap(),
                                )
                                .unwrap();
                                assert_eq!(
                                    u32::from(probe.raw()),
                                    operational_limit * 9 / 10,
                                    "encoder={encoder_counts_per_rev} pwm={pwm_hz} \
                                     target={target_rpm} meter={velocity_meter} \
                                     filter={filter_enabled} bed={bed_mm}"
                                );

                                let request = VelocitySweepRequest {
                                    requested_velocity_mrev_s: 6_666,
                                    machine_velocity_ceiling_mrev_s: 10_000,
                                    requested_velocity_source: RequestedVelocitySource::Defaulted,
                                    max_stroke_travel_mrev,
                                    settle_travel_reserve_mrev: 250,
                                    negative_position_headroom_mrev: headroom_mrev,
                                    positive_position_headroom_mrev: headroom_mrev,
                                    max_duration_ms: 3_000,
                                    nomination_p_below: 0,
                                };
                                let planned = plan_velocity_sweep(
                                    request,
                                    encoder_counts_per_rev,
                                    u16::try_from(operational_limit).unwrap(),
                                    VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
                                );
                                let plan = planned.unwrap_or_else(|error| {
                                    panic!(
                                        "sweep plan failed: encoder={encoder_counts_per_rev} \
                                         pwm={pwm_hz} target={target_rpm} \
                                         meter={velocity_meter} filter={filter_enabled} \
                                         limit={operational_limit} bed={bed_mm}: {error:?}"
                                    )
                                });
                                assert_eq!(plan.usable_torque_limit, probe.raw());

                                let below = classify_phase_evidence(
                                    probe,
                                    PhaseObservation {
                                        pid_output_peak: probe.raw() - 1,
                                        status_flags: 0,
                                    },
                                    PhaseObservation {
                                        pid_output_peak: 0,
                                        status_flags: 0,
                                    },
                                    VelocityLimitEvidenceCapability::LatchedFlag,
                                    VelocityCurrentZeroPhase::Braking,
                                )
                                .unwrap();
                                assert_eq!(below.contact, VelocityCurrentContact::None);
                                assert!(below.moving_clip_absence_proven());

                                let at_boundary = classify_phase_evidence(
                                    probe,
                                    PhaseObservation {
                                        pid_output_peak: probe.raw(),
                                        status_flags: 0,
                                    },
                                    PhaseObservation {
                                        pid_output_peak: 0,
                                        status_flags: 0,
                                    },
                                    VelocityLimitEvidenceCapability::LatchedFlag,
                                    VelocityCurrentZeroPhase::Braking,
                                )
                                .unwrap();
                                assert_eq!(at_boundary.contact, VelocityCurrentContact::Moving);

                                let p_raw = 2_048_i64;
                                let spin_up_output =
                                    (p_raw * i64::from(target_rpm) / 256).unsigned_abs();
                                let reversal_output = (p_raw
                                    * (i64::from(target_rpm) - i64::from(meter_quantum_rpm))
                                    / 256)
                                    .unsigned_abs();
                                let unconstrained_peak = spin_up_output.max(reversal_output);
                                let sampled_peak =
                                    unconstrained_peak.min(u64::from(probe.raw())) as u16;
                                let flags = if unconstrained_peak >= u64::from(probe.raw()) {
                                    PID_V_OUTPUT_LIMIT
                                } else {
                                    0
                                };
                                let quantized = classify_phase_evidence(
                                    probe,
                                    PhaseObservation {
                                        pid_output_peak: sampled_peak,
                                        status_flags: flags,
                                    },
                                    PhaseObservation {
                                        pid_output_peak: 0,
                                        status_flags: 0,
                                    },
                                    VelocityLimitEvidenceCapability::LatchedFlag,
                                    VelocityCurrentZeroPhase::Braking,
                                )
                                .unwrap();
                                assert_eq!(
                                    quantized.contact == VelocityCurrentContact::Moving,
                                    unconstrained_peak >= u64::from(probe.raw()),
                                    "encoder={encoder_counts_per_rev} pwm={pwm_hz} \
                                     target={target_rpm} meter={velocity_meter} \
                                     filter={filter_enabled} limit={operational_limit} \
                                     bed={bed_mm}"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn velocity_current_clamp_session() {
    pollster::block_on(async {
        let mut session = VelocityCurrentClampSession::try_new(2_816).unwrap();
        assert_eq!(session.operational_limit().raw(), 2_816);
        assert_eq!(session.probe_limit().raw(), 2_534);
        assert!(!session.probe_is_installed());

        let mut backend = MockBackend::new();
        session.install_probe_limit(&mut backend).await.unwrap();
        assert!(session.probe_is_installed());
        assert!(matches!(
            backend.spi_log.as_slice(),
            [
                SpiOp::Write(addr, 2_534),
                SpiOp::Read,
            ] if *addr == PID_TORQUE_FLUX_LIMITS.addr()
        ));

        let retained_log_len = backend.spi_log.len();
        assert!(session.probe_is_installed());
        assert_eq!(backend.spi_log.len(), retained_log_len);

        session.install_probe_limit(&mut backend).await.unwrap();
        assert!(session.probe_is_installed());
        assert_eq!(backend.spi_log.len(), retained_log_len + 2);

        session
            .restore_operational_limit(&mut backend)
            .await
            .unwrap();
        assert!(!session.probe_is_installed());
        assert!(matches!(
            &backend.spi_log.as_slice()[retained_log_len + 2..],
            [
                SpiOp::Write(addr, 2_816),
                SpiOp::Read,
            ] if *addr == PID_TORQUE_FLUX_LIMITS.addr()
        ));

        #[derive(Clone, Copy)]
        enum FailureMode {
            Write,
            Read,
            CompleteWordMismatch,
        }
        for failure in [
            FailureMode::Write,
            FailureMode::Read,
            FailureMode::CompleteWordMismatch,
        ] {
            let mut session = VelocityCurrentClampSession::try_new(2_816).unwrap();
            let mut backend = MockBackend::new();
            match failure {
                FailureMode::Write => {
                    backend.fail_on_write = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::Read => {
                    backend.fail_on_read = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::CompleteWordMismatch => {
                    backend.read_override = Some((PID_TORQUE_FLUX_LIMITS.addr(), 0x0001_09e6));
                }
            }
            assert!(session.install_probe_limit(&mut backend).await.is_err());
            assert!(!session.probe_is_installed());
        }

        for failure in [
            FailureMode::Write,
            FailureMode::Read,
            FailureMode::CompleteWordMismatch,
        ] {
            let mut session = VelocityCurrentClampSession::try_new(2_816).unwrap();
            let mut backend = MockBackend::new();
            session.install_probe_limit(&mut backend).await.unwrap();
            match failure {
                FailureMode::Write => {
                    backend.fail_on_write = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::Read => {
                    backend.fail_on_read = Some(PID_TORQUE_FLUX_LIMITS.addr());
                }
                FailureMode::CompleteWordMismatch => {
                    backend.read_override = Some((PID_TORQUE_FLUX_LIMITS.addr(), 0x0001_0b00));
                }
            }
            assert!(
                session
                    .restore_operational_limit(&mut backend)
                    .await
                    .is_err()
            );
            assert!(session.probe_is_installed());
        }

        for invalid in [0, 1, 0x8000, u32::MAX] {
            assert!(VelocityCurrentClampSession::try_new(invalid).is_err());
        }
    });
}

#[test]
fn velocity_current_clamp_session_accepts_an_explicit_bounded_limit() {
    let historical = VelocityCurrentClampSession::try_new(2_816).unwrap();
    assert_eq!(historical.operational_limit().raw(), 2_816);
    assert_eq!(historical.probe_limit().raw(), 2_534);

    let operational = VelocityCurrentClampSession::try_new_with_limit(2_816, 2_816).unwrap();
    assert_eq!(operational.operational_limit().raw(), 2_816);
    assert_eq!(operational.probe_limit().raw(), 2_816);

    let conservative = VelocityCurrentClampSession::try_new_with_limit(2_816, 2_534).unwrap();
    assert_eq!(conservative.operational_limit().raw(), 2_816);
    assert_eq!(conservative.probe_limit().raw(), 2_534);

    for active_limit in [0, 2_817, 0x8000, u32::MAX] {
        assert!(VelocityCurrentClampSession::try_new_with_limit(2_816, active_limit).is_err());
    }
}

#[test]
fn velocity_current_observer_attributes_phases() {
    let limit = PidTorqueFluxLimit::new(2_816).unwrap();
    let mut observer =
        VelocityCurrentObserver::new(limit, VelocityLimitEvidenceCapability::LatchedFlag);

    for peak in [100, 2_815, 800] {
        observer.observe_moving_output(peak);
    }
    observer.observe_moving_flags((1 << 3) | (1 << 26));
    observer.observe_zero_output(2_000);
    observer.observe_zero_flags(1 << 15);
    let clear = observer
        .evidence(VelocityCurrentZeroPhase::Braking)
        .unwrap();
    assert_eq!(clear.contact, VelocityCurrentContact::None);
    assert_eq!(clear.moving_pid_output_peak, 2_815);
    assert_eq!(clear.zero_pid_output_peak, 2_000);
    assert_eq!(clear.moving_status_flags, (1 << 3) | (1 << 26));
    assert_eq!(clear.zero_status_flags, 1 << 15);
    assert!(clear.moving_clip_absence_proven());

    for contact_index in 0..3 {
        observer.reset(limit, VelocityLimitEvidenceCapability::LatchedFlag);
        for (index, peak) in [100, 200, 300].into_iter().enumerate() {
            observer.observe_moving_output(if index == contact_index {
                limit.raw()
            } else {
                peak
            });
        }
        assert_eq!(
            observer
                .evidence(VelocityCurrentZeroPhase::Braking)
                .unwrap()
                .contact,
            VelocityCurrentContact::Moving
        );
    }

    observer.reset(limit, VelocityLimitEvidenceCapability::LatchedFlag);
    observer.observe_moving_output(2_815);
    observer.observe_zero_flags(PID_V_OUTPUT_LIMIT | (1 << 15));
    let braking = observer
        .evidence(VelocityCurrentZeroPhase::Braking)
        .unwrap();
    assert_eq!(braking.contact, VelocityCurrentContact::Braking);
    assert_eq!(braking.zero_status_flags, PID_V_OUTPUT_LIMIT | (1 << 15));

    observer.reset(limit, VelocityLimitEvidenceCapability::LatchedFlag);
    observer.observe_zero_output(limit.raw());
    assert_eq!(
        observer
            .evidence(VelocityCurrentZeroPhase::Recovery)
            .unwrap()
            .contact,
        VelocityCurrentContact::Recovery
    );

    observer.reset(limit, VelocityLimitEvidenceCapability::BestEffortPolling);
    observer.observe_moving_output(2_815);
    let best_effort = observer
        .evidence(VelocityCurrentZeroPhase::Braking)
        .unwrap();
    assert_eq!(best_effort.contact, VelocityCurrentContact::None);
    assert!(best_effort.moving_clip_uncertain());
}

#[test]
fn begin_clears_demoted_bits_without_faulting() {
    for preload in [
        1u32 << 8,
        1 << 10,
        1 << 12,
        1 << 14,
        1 << 16,
        1 << 17,
        1 << 18,
        1 << 20,
        1 << 21,
        1 << 22,
        1 << 24,
        1 << 25,
        0x0377_5500,
    ] {
        let mut backend = MockBackend::new();
        let _ = backend.registers.insert(STATUS_FLAGS.addr(), preload);
        let mut observer = VelocityCurrentObserver::EMPTY;
        let r = pollster::block_on(begin_velocity_current_primitive(
            &mut observer,
            &mut backend,
            PidTorqueFluxLimit::new(2_534).unwrap(),
            VelocityLimitEvidenceCapability::BestEffortPolling,
        ));
        assert!(
            r.is_ok(),
            "demoted preload {preload:#x} must not fault: {r:?}"
        );
        assert_eq!(
            backend.registers.get(&STATUS_FLAGS.addr()),
            Some(&0),
            "must clear baseline"
        );
    }
}

#[test]
fn begin_faults_on_genuine_fault_bit() {
    let mut backend = MockBackend::new();
    let _ = backend
        .registers
        .insert(STATUS_FLAGS.addr(), ADC_I_CLIPPED_BIT);
    let mut observer = VelocityCurrentObserver::EMPTY;
    let r = pollster::block_on(begin_velocity_current_primitive(
        &mut observer,
        &mut backend,
        PidTorqueFluxLimit::new(2_534).unwrap(),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    ));
    assert!(
        matches!(r, Err(VelocityCurrentGuardError::ActionableStatus(s)) if s == ADC_I_CLIPPED_BIT)
    );
}

#[test]
fn begin_reports_register_io_and_does_not_clear() {
    let mut backend = MockBackend::new();
    let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0x0377_0000);
    backend.fail_on_read = Some(STATUS_FLAGS.addr());
    let mut observer = VelocityCurrentObserver::EMPTY;
    let r = pollster::block_on(begin_velocity_current_primitive(
        &mut observer,
        &mut backend,
        PidTorqueFluxLimit::new(2_534).unwrap(),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    ));
    assert!(matches!(r, Err(VelocityCurrentGuardError::RegisterIo)));
    assert_eq!(
        backend.registers.get(&STATUS_FLAGS.addr()),
        Some(&0x0377_0000),
        "must not clear on read failure"
    );
}

#[test]
fn restart_clears_demoted_bits_without_faulting() {
    for preload in [
        1u32 << 8,
        1 << 10,
        1 << 12,
        1 << 14,
        1 << 16,
        1 << 17,
        1 << 18,
        1 << 20,
        1 << 21,
        1 << 22,
        1 << 24,
        1 << 25,
        0x0377_5500,
    ] {
        let mut backend = MockBackend::new();
        let _ = backend.registers.insert(STATUS_FLAGS.addr(), preload);
        let mut observer = VelocityCurrentObserver::new(
            PidTorqueFluxLimit::new(2_534).unwrap(),
            VelocityLimitEvidenceCapability::BestEffortPolling,
        );
        let r = pollster::block_on(restart_velocity_current_primitive(
            &mut observer,
            &mut backend,
        ));
        assert!(
            r.is_ok(),
            "demoted preload {preload:#x} must not fault at restart: {r:?}"
        );
        assert_eq!(backend.registers.get(&STATUS_FLAGS.addr()), Some(&0));
    }
}

#[test]
fn restart_faults_on_genuine_fault_bit() {
    let mut backend = MockBackend::new();
    let _ = backend
        .registers
        .insert(STATUS_FLAGS.addr(), ADC_I_CLIPPED_BIT);
    let mut observer = VelocityCurrentObserver::new(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    let r = pollster::block_on(restart_velocity_current_primitive(
        &mut observer,
        &mut backend,
    ));
    assert!(
        matches!(r, Err(VelocityCurrentGuardError::ActionableStatus(s)) if s == ADC_I_CLIPPED_BIT)
    );
}

#[test]
fn finish_moving_reports_limiter_contact_for_limiter_bits_set_during_stroke() {
    for bit in [8u32, 10, 12, 14, 16, 17, 18, 24, 25] {
        let mut backend = MockBackend::new();
        let _ = backend.registers.insert(STATUS_FLAGS.addr(), 1 << bit);
        let mut observer = VelocityCurrentObserver::new(
            PidTorqueFluxLimit::new(2_534).unwrap(),
            VelocityLimitEvidenceCapability::BestEffortPolling,
        );
        let r = pollster::block_on(finish_velocity_current_moving_phase(
            &mut observer,
            &mut backend,
        ));
        assert!(
            matches!(r, Err(VelocityCurrentGuardError::LimiterContact(s)) if s == (1 << bit)),
            "finish_moving must report limiter contact on bit {bit} set during the stroke: {r:?}"
        );
    }
}

#[test]
fn finish_moving_still_faults_on_reference_switch_bits_set_during_stroke() {
    for bit in [20u32, 21, 22] {
        let mut backend = MockBackend::new();
        let _ = backend.registers.insert(STATUS_FLAGS.addr(), 1 << bit);
        let mut observer = VelocityCurrentObserver::new(
            PidTorqueFluxLimit::new(2_534).unwrap(),
            VelocityLimitEvidenceCapability::BestEffortPolling,
        );
        let r = pollster::block_on(finish_velocity_current_moving_phase(
            &mut observer,
            &mut backend,
        ));
        assert!(
            matches!(r, Err(VelocityCurrentGuardError::ActionableStatus(s)) if s == (1 << bit)),
            "finish_moving must fault on reference switch bit {bit} set during the stroke"
        );
    }
}

#[test]
fn finish_moving_keeps_the_fail_closed_consequence_when_a_fault_bit_joins_limiter_bits() {
    const ERRSUM_LIMITS: u32 = (1 << 10) | (1 << 14);
    let mut backend = MockBackend::new();
    let _ = backend
        .registers
        .insert(STATUS_FLAGS.addr(), ERRSUM_LIMITS | ADC_I_CLIPPED_BIT);
    let mut observer = VelocityCurrentObserver::new(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    let r = pollster::block_on(finish_velocity_current_moving_phase(
        &mut observer,
        &mut backend,
    ));
    assert!(
        matches!(
            r,
            Err(VelocityCurrentGuardError::ActionableStatus(s))
                if s == ERRSUM_LIMITS | ADC_I_CLIPPED_BIT
        ),
        "a genuine fault bit must keep the whole word fail-closed: {r:?}"
    );
}

#[test]
fn finish_zero_reports_limiter_contact_for_current_loop_errsum_limits() {
    // The word a TMC4671 leaves behind when a high-gain velocity stroke saturates
    // both current-loop integrators: pid_id_errsum_limit and pid_iq_errsum_limit.
    const ERRSUM_LIMITS: u32 = (1 << 10) | (1 << 14);
    let mut backend = MockBackend::new();
    let _ = backend.registers.insert(STATUS_FLAGS.addr(), ERRSUM_LIMITS);
    let mut observer = VelocityCurrentObserver::new(
        PidTorqueFluxLimit::new(2_534).unwrap(),
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );
    let r = pollster::block_on(finish_velocity_current_zero_phase(
        &mut observer,
        &mut backend,
    ));
    assert!(
        matches!(r, Err(VelocityCurrentGuardError::LimiterContact(s)) if s == ERRSUM_LIMITS),
        "finish_zero must report limiter contact for errsum limits: {r:?}"
    );
}
