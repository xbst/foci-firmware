use super::*;
use crate::commission_dispatch::test_support::breakaway_campaign_engine;
use crate::commission_dispatch::test_support::{
    MockBackend, MockEvent, SpiOp, StimulusOp, accepted_commission_result, current_limit,
    sample_entry_snapshot, sample_outer_params,
};
use crate::commission_dispatch::{PostActions, ReplyPayload, StepperPostAction};
use crate::commissioning::CommissioningEngine;
use crate::config::TmcConfig;
use crate::pid::InnerLoopGains;
use crate::tmc::{TmcRequestKind, decode_signed_upper_half};
use crate::tmc_control::dispatch::{
    RegisterRequestError, SharedTmcDispatch, apply_shared_tmc_request,
};
use crate::tmc_control::helpers::{
    LiveGainWriteStatus, apply_live_inner_gains, apply_live_outer_gains, apply_motion_scale,
    refresh_velocity_scale_from_state,
};
use crate::tmc_control::request::live_gain_failure_outcome;
use crate::tmc_control::state::{
    HOMING_CLAMP_MAX_RETRIES, HomingClockWindow, homing_clamp_due, homing_clamp_lead_ticks,
};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use crate::tmc_control::test_spi::{
    RuntimeMockSpi, assert_limit_plus_offset_within, assert_limit_plus_offset_within_schedule,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_tmc4671::registers::{
    ABN_DECODER_MODE, ABN_DECODER_PPR, CONFIG_BIQUAD_T_ENABLE, MODE_RAMP_MODE_MOTION,
    PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM,
    PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_FLUX_P_FLUX_I, PID_POSITION_ACTUAL,
    PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET,
    PID_TORQUE_P_TORQUE_I, PID_VELOCITY_OFFSET, PID_VELOCITY_P_VELOCITY_I, PIDOUT_UQ_UD_LIMITS,
    STATUS_FLAGS, STEP_WIDTH,
};
use foci_tmc4671::spi::Tmc4671Spi;

fn assert_no_actions(actions: &BoardActions) {
    assert_eq!(actions.len(), 0, "expected no board actions");
}

fn assert_single_stop_now(actions: &BoardActions) {
    assert_eq!(actions.len(), 1, "expected one StopNow action");
    assert_stop_now_at(actions, 0);
}

fn assert_stop_now_at(actions: &BoardActions, index: usize) {
    match &actions[index] {
        BoardAction::Stepper(StepperAction::StopNow) => {}
        BoardAction::Stepper(_) => panic!("expected StopNow stepper action at index {index}"),
        BoardAction::Shutdown(_) => panic!("expected StopNow, got Shutdown at index {index}"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected StopNow, got SignalEmergencyDisable at index {index}")
        }
    }
}

#[test]
fn runtime_starts_disarmed_and_uncalibrated() {
    let runtime = TmcControlRuntime::new_for_channel(0);

    assert_eq!(runtime.channel(), 0);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert_eq!(runtime.stored_gains(), None);
    assert_eq!(runtime.pending_commission_kind(), None);
}

#[test]
fn channel_runtime_size_stays_bounded() {
    #[cfg(feature = "trace")]
    assert_eq!(core::mem::size_of::<TmcControlRuntime>(), 8_336);
    #[cfg(not(feature = "trace"))]
    assert_eq!(core::mem::size_of::<TmcControlRuntime>(), 8_320);
}

#[test]
fn runtime_stores_inner_gains_without_spi_side_effects() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let gains = InnerLoopGains {
        flux_p: crate::pid::p_gain(11),
        flux_i: crate::pid::i_gain(12),
        torque_p: crate::pid::p_gain(13),
        torque_i: crate::pid::i_gain(14),
    };

    runtime.set_stored_gains(gains);

    assert_eq!(runtime.stored_gains(), Some(gains));
}

#[test]
fn refresh_velocity_scale_caches_exact_planner_scale() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc_state = crate::state::tmc::tmc_channel0();
    let stepper_state = crate::state::stepper::stepper_channel0();
    tmc_state.reset();
    stepper_state.reset();
    let scale = tmc_state
        .configure_motion_scale(stepper_state, 1000, 3200)
        .unwrap();

    refresh_velocity_scale_from_state(&mut runtime, tmc_state, 84_000_000, 25_000);

    assert_eq!(
        runtime.planner_velocity_scale(),
        scale.planner_velocity_scale(84_000_000, 25_000)
    );
    tmc_state.reset();
    stepper_state.reset();
}

#[test]
fn apply_live_inner_gains_publishes_only_after_both_writes_succeed() {
    pollster::block_on(async {
        let previous = InnerLoopGains {
            flux_p: crate::pid::p_gain(1),
            flux_i: crate::pid::i_gain(2),
            torque_p: crate::pid::p_gain(3),
            torque_i: crate::pid::i_gain(4),
        };
        let requested = InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        };
        let cases = [
            (false, false, true, true),
            (true, false, false, true),
            (false, true, true, false),
            (true, true, false, false),
        ];

        for (fail_flux, fail_torque, expected_first_ok, expected_second_ok) in cases {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_stored_gains(previous);
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            tmc_state.mark_pi_representation_established();
            let mut spi = RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
            if fail_flux {
                spi = spi.fail_writes_to(PID_FLUX_P_FLUX_I.addr());
            }
            if fail_torque {
                spi = spi.fail_writes_to(PID_TORQUE_P_TORQUE_I.addr());
            }
            let mut tmc = Tmc4671Spi::new(spi);

            let status =
                apply_live_inner_gains(runtime.stored_gains_mut(), &tmc_state, &mut tmc, requested)
                    .await;
            let spi = tmc.into_inner();

            assert_eq!(status.first_write_ok, expected_first_ok);
            assert_eq!(status.second_write_ok, expected_second_ok);
            assert_eq!(spi.write_attempts.len(), 3);
            assert_eq!(
                spi.write_attempts,
                std::vec![
                    (
                        foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                            .addr
                            .addr(),
                        u32::from(foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT.sub_addr,),
                    ),
                    (PID_FLUX_P_FLUX_I.addr(), (11u32 << 16) | 12,),
                    (PID_TORQUE_P_TORQUE_I.addr(), (13u32 << 16) | 14,),
                ],
            );
            assert_eq!(
                runtime.stored_gains(),
                if status.all_ok() {
                    Some(requested)
                } else {
                    Some(previous)
                },
            );
        }
    });
}

#[test]
fn apply_live_outer_gains_publishes_only_after_both_writes_succeed() {
    pollster::block_on(async {
        let previous = crate::pid::test_outer_gains(101, 102, 103, 104);
        let requested = crate::pid::test_outer_gains(201, 202, 203, 204);
        let cases = [
            (false, false, true, true),
            (true, false, false, true),
            (false, true, true, false),
            (true, true, false, false),
        ];

        for (fail_velocity, fail_position, expected_first_ok, expected_second_ok) in cases {
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            tmc_state.store_outer_gains(previous, Relaxed);
            tmc_state.mark_pi_representation_established();
            let mut spi = RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
            if fail_velocity {
                spi = spi.fail_writes_to(PID_VELOCITY_P_VELOCITY_I.addr());
            }
            if fail_position {
                spi = spi.fail_writes_to(PID_POSITION_P_POSITION_I.addr());
            }
            let mut tmc = Tmc4671Spi::new(spi);

            let status = apply_live_outer_gains(&tmc_state, &mut tmc, requested).await;
            let spi = tmc.into_inner();
            let published = tmc_state.load_outer_gains(Relaxed).unwrap();

            assert_eq!(status.first_write_ok, expected_first_ok);
            assert_eq!(status.second_write_ok, expected_second_ok);
            assert_eq!(spi.write_attempts.len(), 3);
            assert_eq!(
                spi.write_attempts,
                std::vec![
                    (
                        foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                            .addr
                            .addr(),
                        u32::from(foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT.sub_addr,),
                    ),
                    (PID_VELOCITY_P_VELOCITY_I.addr(), (203u32 << 16) | 204,),
                    (PID_POSITION_P_POSITION_I.addr(), (201u32 << 16) | 202,),
                ],
            );
            assert_eq!(
                published,
                if status.all_ok() { requested } else { previous },
            );
        }
    });
}

#[test]
fn live_gain_write_refuses_before_pi_representation_establishment() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let requested = InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        };
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let status =
            apply_live_inner_gains(runtime.stored_gains_mut(), &tmc_state, &mut tmc, requested)
                .await;
        let spi = tmc.into_inner();

        assert!(!status.all_ok());
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn live_gain_write_refuses_mismatching_pi_representation_before_pid_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.mark_pi_representation_established();
        let requested = InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        };
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(0));

        let status =
            apply_live_inner_gains(runtime.stored_gains_mut(), &tmc_state, &mut tmc, requested)
                .await;
        let spi = tmc.into_inner();

        assert!(!status.all_ok());
        assert!(!spi.attempted_write_to(PID_FLUX_P_FLUX_I));
        assert!(!spi.attempted_write_to(PID_TORQUE_P_TORQUE_I));
        assert!(!tmc_state.pi_representation_established());
    });
}

#[test]
fn live_gain_failure_outcome_maps_incomplete_pairs_to_shutdown() {
    let cases = [
        (true, true, None),
        (
            false,
            true,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
        (
            true,
            false,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
        (
            false,
            false,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
    ];

    for (first_write_ok, second_write_ok, expected) in cases {
        let status = LiveGainWriteStatus {
            first_write_ok,
            second_write_ok,
        };

        assert_eq!(live_gain_failure_outcome(status), expected);
    }
}

#[test]
fn shared_dispatch_set_current_writes_current_limit_and_updates_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let expected_limit = ((1155.0_f32 * core::f32::consts::SQRT_2) / 1.155_f32) as u32;

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetCurrent { run_ma: 1155 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(PID_TORQUE_FLUX_LIMITS.addr(), expected_limit)]
        );
        assert_eq!(
            u32::from(tmc_state.load_current_limit(Relaxed).raw()),
            expected_limit
        );
    });
}

#[test]
fn shared_dispatch_set_current_rejects_invalid_limit_distinctly() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetCurrent { run_ma: u32::MAX },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;

        assert_eq!(result, Err(RegisterRequestError::CurrentLimitInvalid));
    });
}

#[test]
fn apply_motion_scale_writes_encoder_then_physical_width_and_publishes_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let scale = crate::motion_scale::MotionScale::new(1000, 3200).unwrap();

        let result = apply_motion_scale(&mut tmc, &tmc_state, &stepper_state, scale, |_| {
            Ok::<(), core::convert::Infallible>(())
        })
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(()));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_PPR.addr(), 4000), (STEP_WIDTH.addr(), 16)]
        );
        assert_eq!(tmc_state.encoder_counts_per_rev(), Some(4000));
        assert_eq!(stepper_state.planner_steps_per_rev(&tmc_state), Some(3200));
        assert_eq!(tmc_state.physical_step_width(), Some(16));
    });
}

#[test]
fn apply_motion_scale_spi_failures_leave_scale_unconfigured() {
    pollster::block_on(async {
        for failed_register in [ABN_DECODER_PPR.addr(), STEP_WIDTH.addr()] {
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            let stepper_state = crate::state::stepper::StepperChannelState::new();
            let mut tmc =
                Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(failed_register));
            let scale = crate::motion_scale::MotionScale::new(1000, 3200).unwrap();

            let expected = if failed_register == ABN_DECODER_PPR.addr() {
                crate::motion_scale::MotionScaleRejection::EncoderRegisterWriteFailed
            } else {
                crate::motion_scale::MotionScaleRejection::StepWidthRegisterWriteFailed
            };
            assert_eq!(
                apply_motion_scale(&mut tmc, &tmc_state, &stepper_state, scale, |_| {
                    Ok::<(), core::convert::Infallible>(())
                })
                .await,
                Err(expected)
            );
            assert!(!tmc_state.motion_scale_configured());
            assert_eq!(tmc_state.physical_step_width(), None);
        }
    });
}

#[test]
fn shared_dispatch_set_encoder_dir_writes_mode_and_marks_reset() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetEncoderDir { invert: true },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_MODE.addr(), 1 << 12)]
        );
        assert!(tmc_state.encoder_reversed.load(Relaxed));
        assert!(tmc_state.feedforward_reset.load(Relaxed));
    });
}

#[test]
fn shared_dispatch_set_voltage_limit_propagates_write_failure() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr()));

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetVoltageLimit {
                voltage_limit: 32_000,
            },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Err(RegisterRequestError::Other));
        assert!(spi.attempted_write_to(PIDOUT_UQ_UD_LIMITS));
    });
}

#[test]
fn shared_dispatch_set_torque_filter_writes_biquad_and_updates_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetTorqueFilter { filter_hz: 3000 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.contains(&(
            CONFIG_BIQUAD_T_ENABLE.addr.addr(),
            CONFIG_BIQUAD_T_ENABLE.sub_addr as u32,
        )));
        assert!(
            spi.write_attempts
                .contains(&(CONFIG_BIQUAD_T_ENABLE.data.addr(), 1))
        );
        assert_eq!(tmc_state.torque_filter_hz.load(Relaxed), Some(3000));
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn shared_dispatch_set_torque_filter_zero_disables_filter() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.torque_filter_hz.store(Some(3000), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetTorqueFilter { filter_hz: 0 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.contains(&(
            CONFIG_BIQUAD_T_ENABLE.addr.addr(),
            CONFIG_BIQUAD_T_ENABLE.sub_addr as u32,
        )));
        assert!(
            spi.write_attempts
                .contains(&(CONFIG_BIQUAD_T_ENABLE.data.addr(), 0))
        );
        assert_eq!(tmc_state.torque_filter_hz.load(Relaxed), Some(0));
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn shared_dispatch_main_shell_request_returns_unhandled_without_spi() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::Calibrate { oid: 7 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Unhandled));
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    });
}

fn commissioning_runtime_config() -> TmcControlConfig {
    let tmc = TmcConfig::default_for_test();
    TmcControlConfig::new(84_000_000, tmc.pwm_freq_hz(), 0.03125, tmc)
}

#[test]
fn scheduled_step_preserves_explicit_engine_deadline_without_consuming_engine() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let tmc = crate::state::tmc::tmc_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 42_000;
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));
        assert!(matches!(
            runtime
                .step_commissioning_scheduled(&mut backend)
                .await
                .expect("engine should be active"),
            crate::commissioning::schedule::CommissionPoll::At {
                timestamp_us: 292_000,
                event: None,
            }
        ));
    });
}

#[test]
fn scheduled_step_keeps_inductance_capture_inside_one_deadline_loop() {
    pollster::block_on(async {
        use crate::commissioning::capture_timing::{CaptureTiming, TimingEnvelope};
        use crate::commissioning::encoder_domain::EncoderDomain;
        use crate::commissioning::inner::InductanceCaptureState;

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut engine =
            CommissioningEngine::new_commission(7, crate::autotune::CommissionProfile::Balanced);
        engine.set_inductance_capture_for_test(InductanceCaptureState {
            encoder_domain: EncoderDomain::new(4_000).unwrap(),
            encoder_start: 0,
            start_us: 900,
            previous_elapsed_us: 0,
            previous_phi: 0,
            accumulated_phi_delta_counts: 0,
            accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
            first_half: crate::electrical_capture::AcFrameAccumulator::new(),
            second_half: crate::electrical_capture::AcFrameAccumulator::new(),
            saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
            sample_count: 0,
            status_flags_or: 0,
            next_sample_us: 1_000,
            timing: CaptureTiming::new(200, TimingEnvelope::INDUCTANCE),
            current_envelope: 563,
        });
        runtime.set_commission_engine(engine);

        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 1_138;
        backend.advance_elapsed_on_delay = true;
        let mut phi_samples = heapless::Vec::<u32, 128>::new();
        let mut current_samples = heapless::Vec::<u32, 128>::new();
        let actual = ((-84i16 as u16 as u32) << 16) | 20;
        for index in 1..=100u32 {
            phi_samples
                .push((index.wrapping_mul(20 * 65_536) / 100) & 0xffff)
                .unwrap();
            current_samples.push(actual).unwrap();
        }
        backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
        backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

        let poll = runtime
            .step_commissioning_scheduled(&mut backend)
            .await
            .expect("engine should be active");

        assert!(matches!(
            poll,
            crate::commissioning::schedule::CommissionPoll::ReadyNow { event: None }
        ));
        assert_eq!(backend.register_pair_reads, 100);
    });
}

#[test]
fn scheduled_capture_shutdown_uses_canonical_engine_failure_path() {
    pollster::block_on(async {
        use crate::commissioning::capture_timing::{CaptureTiming, TimingEnvelope};
        use crate::commissioning::encoder_domain::EncoderDomain;
        use crate::commissioning::inner::InductanceCaptureState;

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut engine =
            CommissioningEngine::new_commission(7, crate::autotune::CommissionProfile::Balanced);
        engine.set_inductance_capture_for_test(InductanceCaptureState {
            encoder_domain: EncoderDomain::new(4_000).unwrap(),
            encoder_start: 0,
            start_us: 900,
            previous_elapsed_us: 0,
            previous_phi: 0,
            accumulated_phi_delta_counts: 0,
            accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
            first_half: crate::electrical_capture::AcFrameAccumulator::new(),
            second_half: crate::electrical_capture::AcFrameAccumulator::new(),
            saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
            sample_count: 0,
            status_flags_or: 0,
            next_sample_us: 1_000,
            timing: CaptureTiming::new(200, TimingEnvelope::INDUCTANCE),
            current_envelope: 563,
        });
        runtime.set_commission_engine(engine);

        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        backend.elapsed_us_value = 900;
        backend.shutdown_requested = true;

        let poll = runtime
            .step_commissioning_scheduled(&mut backend)
            .await
            .expect("engine should be active");

        assert!(matches!(
            poll,
            crate::commissioning::schedule::CommissionPoll::Terminal {
                result: crate::commissioning::schedule::CommissionTerminal::InnerFailed(
                    crate::autotune::CommissionError::Shutdown
                )
            }
        ));
        assert!(!backend.enable_pin);
        assert_eq!(backend.register_pair_reads, 0);
    });
}

#[test]
fn scheduled_classification_is_synchronous_for_at_action() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();
    let tmc = crate::state::tmc::tmc_channel0();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

    let action = runtime
        .classify_commission_poll(crate::commissioning::schedule::CommissionPoll::At {
            timestamp_us: 43_000,
            event: None,
        })
        .expect("engine should be active");

    assert!(matches!(
        action,
        ScheduledCommissionTickAction::At {
            timestamp_us: 43_000,
            event: None,
        }
    ));
}

#[test]
fn scheduled_classification_preserves_phase_event_context_without_delay() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_commission(
        37,
        crate::autotune::CommissionProfile::Balanced,
    ));
    let poll = crate::commissioning::schedule::adapt_step_result(
        crate::commissioning::types::StepResult::PhaseChanged(
            crate::autotune::CommissionPhase::ElectricalIdentification,
        ),
        81_250,
        Some((37, crate::commissioning::types::EngineKind::Commission)),
    )
    .expect("phase change should produce a scheduled poll");

    assert!(matches!(
        &poll,
        crate::commissioning::schedule::CommissionPoll::ReadyNow {
            event: Some(
                crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                    oid: 37,
                    phase_id: crate::autotune::CommissionPhase::ElectricalIdentification,
                    engine_kind: crate::commissioning::types::EngineKind::Commission,
                }
            ),
        }
    ));

    let action = runtime
        .classify_commission_poll(poll)
        .expect("engine should be active");

    assert!(matches!(
        action,
        ScheduledCommissionTickAction::ReadyNow {
            event: Some(
                crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                    oid: 37,
                    phase_id: crate::autotune::CommissionPhase::ElectricalIdentification,
                    engine_kind: crate::commissioning::types::EngineKind::Commission,
                }
            ),
        }
    ));
    assert!(runtime.is_commission_active());
}

#[test]
fn scheduled_dispatch_preserves_terminal_reply_and_actions() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let tmc = crate::state::tmc::tmc_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

        let mut integral =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let action = runtime
            .drive_commissioning_terminal_for_test(
                crate::commissioning::schedule::CommissionTerminal::SelftestDone,
                &mut backend,
                &mut crate::commission_dispatch::CommissionRetention::new(&mut integral),
                &config,
            )
            .await;

        match action {
            ScheduledCommissionTickAction::Terminal {
                reply: ReplyPayload::Selftest { oid: 7, .. },
                actions,
            } => {
                assert_eq!(actions.stop_trace, PostActions::NONE.stop_trace);
                assert_eq!(actions.stepper, PostActions::NONE.stepper);
            }
            _ => panic!("expected Selftest terminal action"),
        }
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn scheduled_dispatch_preserves_accepted_commission_reply_and_actions() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;

        const MOTOR_SENTINEL: u32 = 0x0002_0032;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.reset();
        stepper.reset();
        tmc.mark_pi_representation_established();
        tmc.store_current_limit(current_limit(0x10), Ordering::Relaxed);
        tmc.motor_sentinel.store(MOTOR_SENTINEL, Ordering::Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                MOTOR_SENTINEL,
            )
            .await
            .expect("test motor sentinel write should succeed");
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            23,
            crate::autotune::CommissionProfile::Balanced,
        ));

        let mut integral =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let action = runtime
            .drive_commissioning_terminal_for_test(
                crate::commissioning::schedule::CommissionTerminal::CommissionDone(
                    accepted_commission_result(),
                ),
                &mut backend,
                &mut crate::commission_dispatch::CommissionRetention::new(&mut integral),
                &config,
            )
            .await;

        match action {
            ScheduledCommissionTickAction::Terminal {
                reply:
                    ReplyPayload::CommissionResult {
                        oid,
                        status,
                        result,
                    },
                actions,
            } => {
                assert_eq!(oid, 23);
                assert_eq!(status, 0);
                assert_eq!(result.inner.r_count_milli, 1706);
                assert_eq!(result.inner.l_count_micro, 1245);
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmPreservePosition);
            }
            _ => panic!("expected accepted CommissionResult terminal action"),
        }
        assert!(!runtime.is_commission_active());
        assert!(runtime.is_calibrated());
        assert!(runtime.is_motor_enabled());
        assert!(runtime.stored_gains().is_some());
        assert!(backend.enable_pin);
        assert!(!tmc.motor_armed.load(Ordering::Relaxed));
        assert!(!tmc.commission_in_flight.load(Ordering::Relaxed));

        tmc.reset();
        stepper.reset();
    });
}

#[test]
fn state_refs_carry_channel_state_without_ownership() {
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let refs = TmcControlStateRefs::new(&tmc, &stepper);

    assert!(core::ptr::eq(refs.tmc, &tmc));
    assert!(core::ptr::eq(refs.stepper, &stepper));
}

#[test]
fn dispatch_state_view_mutates_runtime_owned_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc: &'static crate::state::tmc::TmcChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::tmc::TmcChannelState::new()),
    );
    let stepper: &'static crate::state::stepper::StepperChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::stepper::StepperChannelState::new()),
    );

    {
        let dispatch = runtime.dispatch_state(tmc, stepper);
        *dispatch.calibrated = true;
        *dispatch.motor_enabled = true;
    }

    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());

    runtime.set_calibrated_for_test(false);
    runtime.set_motor_enabled_for_test(false);
}

#[test]
fn config_reset_runtime_state_clears_owned_flags_and_defaults() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(1234);

    runtime.reset_runtime_state();

    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(runtime.planner_velocity_scale(), None);
    assert_eq!(runtime.planner_rpm_scale(), None);
}

#[test]
fn reset_runtime_state_releases_the_offset_headroom_reservation() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let tmc_state = crate::state::tmc::tmc_channel0();
    tmc_state.store_current_limit(current_limit(2_557), Relaxed);
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.hold_offset_headroom_for_test(tmc_state, 0x0400_0000);
    assert!(tmc_state.offset_headroom_reserved(Relaxed));

    runtime.reset_runtime_state();

    assert!(!tmc_state.offset_headroom_reserved(Relaxed));
    assert!(!runtime.offset_headroom_engaged());
    assert_eq!(
        crate::tmc_control::current_limit::operational_limit_register(tmc_state),
        current_limit(2_557).register_value()
    );
}

#[test]
fn config_reset_channel_state_restores_known_defaults() {
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();

    tmc.store_current_limit(current_limit(99), Relaxed);
    tmc.velocity_limit_cfg.store(123, Relaxed);
    stepper.position.store(44, Relaxed);

    TmcControlRuntime::reset_channel_state(&tmc, &stepper);

    assert_eq!(tmc.load_current_limit(Relaxed).raw(), 0);
    assert_eq!(tmc.velocity_limit_cfg.load(Relaxed), 500_000);
    assert_eq!(stepper.position.load(Relaxed), 0);
}

#[test]
fn queue_flush_without_signal_does_not_mutate_state() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.store_current_limit(current_limit(88), Relaxed);
    stepper.position.store(55, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert_no_actions(&actions);
    assert!(!queue_flush.load(Relaxed));
    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());
    assert_eq!(tmc.load_current_limit(Relaxed).raw(), 88);
    assert_eq!(stepper.position.load(Relaxed), 55);
    assert!(fault_active.load(Relaxed));
}

#[test]
fn queue_flush_signal_resets_runtime_and_channel_state() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.store_current_limit(current_limit(88), Relaxed);
    stepper.position.store(55, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(!queue_flush.load(Relaxed));
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(tmc.load_current_limit(Relaxed).raw(), 0);
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert!(!fault_active.load(Relaxed));
    assert_single_stop_now(&actions);
}

#[test]
fn queue_flush_keeps_the_fault_latch_while_the_status_pin_is_asserted() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    stepper.position.store(55, Relaxed);
    tmc.status_pin_asserted.store(true, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(
        fault_active.load(Relaxed),
        "a flush must not retire a latch the STATUS pin still reports"
    );
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert_single_stop_now(&actions);
}

#[test]
fn queue_flush_keeps_the_fault_latch_while_the_disable_signal_is_pending() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    disable_signal.signal(());
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    stepper.position.store(55, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(
        fault_active.load(Relaxed),
        "a flush must not retire a latch while an emergency disable is pending"
    );
    assert!(
        disable_signal.signaled(),
        "the pending disable must survive for the board's containment check"
    );
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert_single_stop_now(&actions);
}

#[test]
fn pid_error_clear_request_loads_hold_settle_countdown() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();

    tmc.pid_error_sums_clear.store(true, Relaxed);
    runtime.poll_pid_error_clear_request(&tmc);

    assert_eq!(
        runtime.pid_error_clear_countdown_for_test(),
        crate::tmc_control::HOLD_SETTLE_DELAY_MS
    );
}

#[test]
fn pid_error_settle_tick_waits_for_inactive_timer_then_clears() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        runtime.set_motor_enabled_for_test(true);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .expect("motion scale should configure");
        tmc_state.pid_error_sums_clear.store(true, Relaxed);
        stepper_state.timer_active_mirror.store(true, Relaxed);

        let mut busy_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_pid_error_settle_tick(
                &mut busy_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
            )
            .await;

        assert!(!tmc_state.pid_error_sums_clear.load(Relaxed));
        assert_eq!(
            runtime.pid_error_clear_countdown_for_test(),
            crate::tmc_control::HOLD_SETTLE_DELAY_MS
        );
        assert!(busy_tmc.into_inner().writes.is_empty());

        stepper_state.timer_active_mirror.store(false, Relaxed);
        runtime.set_pid_error_clear_countdown_ms(1);
        let mut settle_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000).with_read(1_000));

        runtime
            .apply_pid_error_settle_tick(
                &mut settle_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
            )
            .await;

        assert_eq!(runtime.pid_error_clear_countdown_for_test(), 0);
        let raw_spi = settle_tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (PID_POSITION_ACTUAL.addr(), 1_000),
                (
                    PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
                (STATUS_FLAGS.addr(), 0),
            ]
        );
    });
}

#[test]
fn homing_channel_tick_suppresses_position_i_without_global_advance() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_P_POSITION_I.addr(), 123u32 << 16)]
        );
    });
}

#[test]
fn homing_channel_tick_preserves_openffboard_ungated_suppression() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(77, 0, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_P_POSITION_I.addr(), 77u32 << 16)]
        );
    });
}

#[test]
fn homing_channel_tick_restores_position_i_and_clears_integrator() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(12, 34, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![
                (PID_POSITION_P_POSITION_I.addr(), (12u32 << 16) | 34),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
            ]
        );
    });
}

#[test]
fn homing_channel_tick_preserves_openffboard_ungated_restore() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(7, 9, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![
                (PID_POSITION_P_POSITION_I.addr(), (7u32 << 16) | 9),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
            ]
        );
    });
}

fn post_homing_restore_limit_writes(reserved: bool) -> std::vec::Vec<u32> {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(7, 9, 1152, 0), Relaxed);
        tmc_state.store_current_limit(current_limit(2557), Relaxed);
        tmc_state.homing_clamp_active.store(true, Relaxed);
        tmc_state.set_offset_headroom_reserved(reserved, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        tmc.into_inner()
            .writes
            .iter()
            .filter(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr())
            .map(|(_, value)| *value)
            .collect()
    })
}

#[test]
fn post_homing_restore_writes_the_reserved_limit_while_the_reservation_is_set() {
    assert_eq!(
        post_homing_restore_limit_writes(true),
        std::vec![2557 - 1278]
    );
}

#[test]
fn post_homing_restore_writes_the_committed_limit_while_the_reservation_is_clear() {
    assert_eq!(post_homing_restore_limit_writes(false), std::vec![2557]);
}

#[test]
fn homing_state_1_outer_gain_failure_retries_twice_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.corrupt_outer_gain_for_test(0x8000);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
    });
}

const HOMING_CURRENT_SCALE_MA_PER_LSB: f32 = 1.155;

fn homing_runtime_config() -> TmcControlConfig {
    let mut config = commissioning_runtime_config();
    config.current_scale_ma_per_lsb = HOMING_CURRENT_SCALE_MA_PER_LSB;
    config
}

fn due_homing_window() -> HomingClockWindow {
    HomingClockWindow { now: 0, start: 0 }
}

fn homing_limit_for(run_ma: u32) -> foci_tmc4671::current_limit::PidTorqueFluxLimit {
    crate::tmc_control::current_limit::current_limit_from_run_ma(
        run_ma,
        HOMING_CURRENT_SCALE_MA_PER_LSB,
    )
    .expect("current must be representable")
}

fn homing_state(runtime: &mut TmcControlRuntime) -> crate::state::tmc::TmcChannelState {
    let tmc_state = crate::state::tmc::TmcChannelState::new();
    tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
    tmc_state.homing_current_ma.store(700, Relaxed);
    tmc_state.stall_ceiling_units.store(100, Relaxed);
    tmc_state.stall_margin_units.store(99, Relaxed);
    tmc_state.stall_persistence.store(2, Relaxed);
    tmc_state.store_current_limit(homing_limit_for(2300), Relaxed);
    tmc_state
        .configure_motion_scale(
            &crate::state::stepper::StepperChannelState::new(),
            1_000,
            3_200,
        )
        .unwrap();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_calibrated_for_test(true);
    tmc_state
}

async fn homing_tick(
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
    refs: TmcControlStateRefs<'_>,
    state: u8,
) -> HomingMaintenanceResult {
    runtime
        .apply_homing_channel_tick(
            tmc,
            refs,
            &homing_runtime_config(),
            state,
            due_homing_window(),
        )
        .await
}

async fn three_homing_state_1_attempts(
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
    refs: TmcControlStateRefs<'_>,
) -> [HomingMaintenanceResult; 3] {
    [
        homing_tick(runtime, tmc, refs, 1).await,
        homing_tick(runtime, tmc, refs, 1).await,
        homing_tick(runtime, tmc, refs, 1).await,
    ]
}

const fn expected_exhausted_homing_state_1_budget() -> [HomingMaintenanceResult; 3] {
    [
        HomingMaintenanceResult::RetryCurrentState,
        HomingMaintenanceResult::RetryCurrentState,
        HomingMaintenanceResult::FailClosed,
    ]
}

#[test]
fn homing_prepare_fails_closed_without_a_motion_scale() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.clear_motion_scale(&stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::FailClosed);
        let writes = tmc.into_inner().writes;
        assert!(
            !writes
                .iter()
                .any(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr())
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_prepare_clamps_torque_limit_and_arms_detector() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        let expected_limit = homing_limit_for(700).register_value();
        let writes = tmc.into_inner().writes;
        assert!(writes.contains(&(PID_TORQUE_FLUX_LIMITS.addr(), expected_limit)));
        assert!(tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!tmc_state.stall_latched.load(Relaxed));
        assert!(runtime.stall_detector_armed_for_test());
        assert_eq!(
            tmc_state.load_current_limit(Relaxed).raw(),
            homing_limit_for(2300).raw(),
            "clamp must not overwrite the stored run current"
        );
    });
}

#[test]
fn homing_clamp_due_opens_a_fixed_lead_before_the_start_clock() {
    let clock_freq = 84_000_000;
    let lead = homing_clamp_lead_ticks(clock_freq);
    assert_eq!(lead, 336_000, "4 ms lead at 84 MHz");
    let start = 10_000_000;
    let window = |now| HomingClockWindow { now, start };
    assert!(!homing_clamp_due(window(start - lead - 1), clock_freq));
    assert!(homing_clamp_due(window(start - lead), clock_freq));
    assert!(homing_clamp_due(window(start + 5), clock_freq));
    let wrapped = HomingClockWindow {
        now: u32::MAX - 100,
        start: lead + 50,
    };
    assert!(!homing_clamp_due(wrapped, clock_freq));
}

#[test]
fn homing_state_1_defers_the_clamp_until_the_start_window_opens() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let config = homing_runtime_config();
        let lead = homing_clamp_lead_ticks(config.clock_freq);
        let start = 50_000_000;

        let early = HomingClockWindow {
            now: start - lead - 1_000,
            start,
        };
        let deferred = runtime
            .apply_homing_channel_tick(&mut tmc, refs, &config, 1, early)
            .await;

        assert_eq!(deferred, HomingMaintenanceResult::Deferred);
        assert!(
            tmc.inner().writes.is_empty(),
            "no register may change while the previous move is still running"
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());

        let due = HomingClockWindow {
            now: start - lead,
            start,
        };
        let applied = runtime
            .apply_homing_channel_tick(&mut tmc, refs, &config, 1, due)
            .await;

        assert_eq!(applied, HomingMaintenanceResult::ReadyToAdvance);
        assert!(tmc_state.homing_clamp_active.load(Relaxed));
        assert!(runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_state_1_deferral_does_not_consume_the_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let config = homing_runtime_config();
        let start = 50_000_000;
        let early = HomingClockWindow {
            now: start - homing_clamp_lead_ticks(config.clock_freq) - 1,
            start,
        };

        for _ in 0..(HOMING_CLAMP_MAX_RETRIES + 2) {
            let result = runtime
                .apply_homing_channel_tick(&mut tmc, refs, &config, 1, early)
                .await;
            assert_eq!(result, HomingMaintenanceResult::Deferred);
        }
        assert!(runtime.homing_state_1_retries_for_test() == 0);
    });
}

#[test]
fn homing_prepare_without_homing_current_leaves_limit_alone() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        tmc_state.homing_current_ma.store(0, Relaxed);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_LIMITS));
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_prepare_fails_closed_for_stall_endstop_without_current() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        tmc_state.homing_current_ma.store(0, Relaxed);
        tmc_state.stall_endstop_configured.store(true, Relaxed);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::FailClosed);
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
    });
}

#[test]
fn homing_prepare_fails_closed_when_not_armed_for_steps() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        runtime.set_calibrated_for_test(false);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::FailClosed);
        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_LIMITS));
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
    });
}

#[test]
fn homing_prepare_fails_closed_when_homing_current_not_below_run_current() {
    pollster::block_on(async {
        for homing_ma in [2300u32, 3000] {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let tmc_state = homing_state(&mut runtime);
            tmc_state.homing_current_ma.store(homing_ma, Relaxed);
            let stepper_state = crate::state::stepper::StepperChannelState::new();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let result = runtime
                .apply_homing_channel_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &homing_runtime_config(),
                    1,
                    due_homing_window(),
                )
                .await;

            assert_eq!(result, HomingMaintenanceResult::FailClosed);
            assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_LIMITS));
            assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        }
    });
}

#[test]
fn homing_shutdown_quiesce_drops_detector_and_retry_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        assert!(runtime.stall_detector_armed_for_test());

        let _ = runtime.prepare_shutdown_quiesce();

        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_state_forced_idle_drops_detector_and_clamp_flag() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        tmc_state.stall_latched.store(true, Relaxed);
        assert!(tmc_state.homing_clamp_active.load(Relaxed));

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                0,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert!(!runtime.stall_detector_armed_for_test());
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!tmc_state.stall_latched.load(Relaxed));
    });
}

#[test]
fn homing_state_1_clamp_write_failure_retries_twice_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );

        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_state_1_mixed_failure_causes_share_one_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        tmc_state.corrupt_outer_gain_for_test(0x8000);
        let mut outer_gain_failure = Tmc4671Spi::new(RuntimeMockSpi::default());
        let first = homing_tick(&mut runtime, &mut outer_gain_failure, refs, 1).await;

        tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
        let mut clamp_write_failure = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );
        let second = homing_tick(&mut runtime, &mut clamp_write_failure, refs, 1).await;

        tmc_state.corrupt_outer_gain_for_test(0x8000);
        let third = homing_tick(&mut runtime, &mut outer_gain_failure, refs, 1).await;

        assert_eq!(first, HomingMaintenanceResult::RetryCurrentState);
        assert_eq!(second, HomingMaintenanceResult::RetryCurrentState);
        assert_eq!(third, HomingMaintenanceResult::FailClosed);
    });
}

#[test]
fn homing_state_1_success_resets_its_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        tmc_state.corrupt_outer_gain_for_test(0x8000);

        for _ in 0..2 {
            assert_eq!(
                homing_tick(&mut runtime, &mut tmc, refs, 1).await,
                HomingMaintenanceResult::RetryCurrentState
            );
        }

        tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
        assert_eq!(
            homing_tick(&mut runtime, &mut tmc, refs, 1).await,
            HomingMaintenanceResult::ReadyToAdvance
        );

        tmc_state.corrupt_outer_gain_for_test(0x8000);
        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
    });
}

#[test]
fn homing_state_1_attempts_preserve_state_3_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut successful_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            homing_tick(&mut runtime, &mut successful_tmc, refs, 1).await,
            HomingMaintenanceResult::ReadyToAdvance
        );

        let mut restore_failure = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );
        assert_eq!(
            homing_tick(&mut runtime, &mut restore_failure, refs, 1).await,
            HomingMaintenanceResult::RetryCurrentState
        );

        assert_eq!(
            homing_tick(&mut runtime, &mut restore_failure, refs, 3).await,
            HomingMaintenanceResult::RetryCurrentState
        );

        let mut successful_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            homing_tick(&mut runtime, &mut successful_tmc, refs, 1).await,
            HomingMaintenanceResult::ReadyToAdvance
        );

        assert_eq!(
            homing_tick(&mut runtime, &mut restore_failure, refs, 3).await,
            HomingMaintenanceResult::FailClosed
        );
    });
}

#[test]
fn homing_detect_sets_latch_after_persistence() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default()
                .with_read(1000)
                .with_read(950)
                .with_read(1000)
                .with_read(850)
                .with_read(1000)
                .with_read(850),
        );
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(
            runtime
                .apply_homing_channel_tick(
                    &mut tmc,
                    refs,
                    &homing_runtime_config(),
                    2,
                    due_homing_window()
                )
                .await,
            HomingMaintenanceResult::ReadyToAdvance
        );
        assert!(!tmc_state.stall_latched.load(Relaxed));
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;
        assert!(!tmc_state.stall_latched.load(Relaxed));
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;
        assert!(tmc_state.stall_latched.load(Relaxed));

        let spi = tmc.into_inner();
        assert!(spi.read_requested(PID_POSITION_TARGET));
        assert!(spi.read_requested(PID_POSITION_ACTUAL));
    });
}

#[test]
fn homing_restore_writes_run_current_and_publishes_result() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default()
                .with_read(1000)
                .with_read(0)
                .with_read(1000)
                .with_read(0),
        );
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        let run_limit = tmc_state.load_current_limit(Relaxed).register_value();
        assert_eq!(
            tmc.into_inner().writes.last(),
            Some(&(PID_TORQUE_FLUX_LIMITS.addr(), run_limit))
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!tmc_state.stall_latched.load(Relaxed));
        assert!(tmc_state.stall_result_latched.load(Relaxed));
        assert_eq!(tmc_state.stall_peak_error_units.load(Relaxed), 1000);
        assert_eq!(tmc_state.stall_trigger_tick.load(Relaxed), 2);
        assert_eq!(tmc_state.stall_trigger_path.load(Relaxed), 1);
        assert!(!runtime.stall_detector_armed_for_test());
        assert_eq!(tmc_state.stall_peak_margin_delta_units.load(Relaxed), 0);
    });
}

#[test]
fn homing_restore_failure_retries_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );

        let first = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;
        let second = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(first, HomingMaintenanceResult::RetryCurrentState);
        assert_eq!(second, HomingMaintenanceResult::FailClosed);
        assert!(
            tmc_state.homing_clamp_active.load(Relaxed),
            "clamp stays flagged until a successful restore or the board disable path"
        );
    });
}

#[test]
fn advance_homing_state_after_channels_waits_for_all_channels() {
    let homing_state = core::sync::atomic::AtomicU8::new(1);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(false);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);

    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, false);
    assert_eq!(homing_state.load(Relaxed), 1);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, true);
    assert_eq!(homing_state.load(Relaxed), 2);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 2);

    homing_triggered.store(0, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 3);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(homing_state.load(Relaxed), 0);
}

#[test]
fn advance_homing_state_does_not_regress_a_move_the_host_already_ended() {
    // The group opened while state 1 was preparing; before it finalized, the
    // endstop fired and the host's trsync_trigger handed the move to restore.
    let homing_state = core::sync::atomic::AtomicU8::new(3);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(false);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, true);
    assert_eq!(
        homing_state.load(Relaxed),
        3,
        "a stale state-1 snapshot must not pull a finished move back to 2"
    );

    // Same for a stale state-2 snapshot whose trigger latch is still set.
    homing_triggered.store(5, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 3);
}

#[test]
fn advance_homing_state_re_enters_preparation_after_a_restore_with_a_queued_pass() {
    let homing_state = core::sync::atomic::AtomicU8::new(3);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(true);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, false);
    assert_eq!(
        homing_state.load(Relaxed),
        3,
        "an unfinished restore keeps waiting"
    );
    assert!(homing_reenter.load(Relaxed));

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(
        homing_state.load(Relaxed),
        1,
        "the pass that registered during the restore gets its own preparation"
    );
    assert!(!homing_reenter.load(Relaxed));
}

#[test]
fn restore_leaves_state_3_before_consuming_the_re_entry_flag() {
    let homing_state = core::sync::atomic::AtomicU8::new(3);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(false);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(homing_state.load(Relaxed), 0);

    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    crate::state::trsync::HOMING_STATE.store(0, Relaxed);
    crate::state::trsync::HOMING_REENTER_PENDING.store(false, Relaxed);
    assert!(
        crate::state::trsync::begin_homing_preparation(900),
        "a registration that arrives once state 3 is gone takes the idle path directly"
    );
    assert!(!crate::state::trsync::HOMING_REENTER_PENDING.load(Relaxed));
    crate::state::trsync::HOMING_STATE.store(0, Relaxed);

    homing_state.store(3, Relaxed);
    homing_reenter.store(true, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(
        homing_state.load(Relaxed),
        1,
        "a registration that arrived while state 3 was still held is promoted after the exchange"
    );
    assert!(!homing_reenter.load(Relaxed));

    homing_state.store(0, Relaxed);
    homing_reenter.store(true, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(
        homing_state.load(Relaxed),
        0,
        "a stale snapshot never re-enters from idle"
    );
    assert!(
        homing_reenter.load(Relaxed),
        "the flag is left for the path that owns the state"
    );
}

#[test]
fn shutdown_quiesce_clears_safe_state_without_config_reset() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let gains = InnerLoopGains {
        flux_p: crate::pid::p_gain(11),
        flux_i: crate::pid::i_gain(12),
        torque_p: crate::pid::p_gain(13),
        torque_i: crate::pid::i_gain(14),
    };

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(crate::commissioning::CommissioningEngine::new_calibrate(0));
    runtime.set_stored_gains(gains);
    runtime.set_planner_velocity_scale_for_test(123_456);
    runtime.set_feedforward_was_active(true);
    runtime.set_pid_error_clear_countdown_ms(9);

    let actions = runtime.prepare_shutdown_quiesce();

    assert_single_stop_now(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_commission_active());
    assert!(!runtime.feedforward_was_active());
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    assert_eq!(runtime.stored_gains(), Some(gains));
    assert_eq!(
        runtime.planner_velocity_scale(),
        Some(crate::motion_scale::PlannerVelocityScale::from_reduced(
            123_456, 1
        ))
    );
}

#[test]
fn shutdown_quiesce_clears_torque_flux_offset_activity() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(&mut active_tmc, refs, &config, false, false, true)
                .await,
            Ok(())
        );
        assert!(
            active_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET)
        );

        assert_single_stop_now(&runtime.prepare_shutdown_quiesce());
        tmc_state.accel_feedforward.store(false, Relaxed);
        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            inactive_tmc.into_inner().writes,
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 10_000)
            ],
            "the held reservation is released once, offset first"
        );
        let mut idle_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut idle_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert!(
            !idle_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET),
            "shutdown quiesce should clear the offset-active latch so later inactive ticks do not emit a cleanup write"
        );
    });
}

#[test]
fn feedforward_and_pid_settle_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    assert!(!runtime.feedforward_was_active());
    runtime.set_feedforward_was_active(true);
    assert!(runtime.feedforward_was_active());

    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    runtime.set_pid_error_clear_countdown_ms(5);
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 5);
}

#[test]
fn dispatch_state_remains_usable_after_shutdown_quiesce() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc: &'static crate::state::tmc::TmcChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::tmc::TmcChannelState::new()),
    );
    let stepper: &'static crate::state::stepper::StepperChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::stepper::StepperChannelState::new()),
    );

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.prepare_shutdown_quiesce();

    {
        let dispatch = runtime.dispatch_state(tmc, stepper);
        *dispatch.calibrated = true;
        *dispatch.motor_enabled = true;
    }

    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());
}

#[test]
fn adc_current_clipped_true_when_bit_set() {
    assert!(is_adc_current_clipped(Ok(1 << 26)));
}

#[test]
fn adc_current_clipped_false_for_clean_read() {
    assert!(!is_adc_current_clipped(Ok(0)));
}

#[test]
fn adc_current_clipped_false_when_status_read_fails() {
    assert!(!is_adc_current_clipped(Err(())));
}

#[test]
fn classify_emergency_disable_fault_prefers_adc_clipped_over_read_failure() {
    assert_eq!(
        classify_emergency_disable_fault(Ok(1 << 26)),
        EmergencyDisableFault::AdcCurrentClipped
    );
}

#[test]
fn classify_emergency_disable_fault_is_read_failed_on_err() {
    assert_eq!(
        classify_emergency_disable_fault(Err(())),
        EmergencyDisableFault::StatusReadFailed
    );
}

#[test]
fn classify_emergency_disable_fault_is_generic_for_other_fault_bits() {
    assert_eq!(
        classify_emergency_disable_fault(Ok(1 << 3)),
        EmergencyDisableFault::Generic
    );
}

#[test]
fn combine_emergency_disable_fault_clipped_beats_read_failed_either_order() {
    use EmergencyDisableFault::{AdcCurrentClipped, StatusReadFailed};
    assert_eq!(
        combine_emergency_disable_fault(AdcCurrentClipped, StatusReadFailed),
        AdcCurrentClipped
    );
    assert_eq!(
        combine_emergency_disable_fault(StatusReadFailed, AdcCurrentClipped),
        AdcCurrentClipped
    );
}

#[test]
fn combine_emergency_disable_fault_read_failed_beats_generic_either_order() {
    use EmergencyDisableFault::{Generic, StatusReadFailed};
    assert_eq!(
        combine_emergency_disable_fault(StatusReadFailed, Generic),
        StatusReadFailed
    );
    assert_eq!(
        combine_emergency_disable_fault(Generic, StatusReadFailed),
        StatusReadFailed
    );
}

#[test]
fn multi_channel_fold_retains_earlier_channel_failure_after_later_channel_succeeds() {
    let mut fault = EmergencyDisableFault::Generic;
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Err(())));
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Ok(0)));
    assert_eq!(fault, EmergencyDisableFault::StatusReadFailed);
}

#[test]
fn multi_channel_fold_retains_earlier_channel_clip_after_later_channel_read_fails() {
    let mut fault = EmergencyDisableFault::Generic;
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Ok(1 << 26)));
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Err(())));
    assert_eq!(fault, EmergencyDisableFault::AdcCurrentClipped);
}

#[test]
fn commission_tick_without_engine_is_empty() {
    let runtime = TmcControlRuntime::new_for_channel(0);
    assert!(runtime.pending_commission_kind().is_none());
    assert!(!runtime.is_commission_active());
}

#[test]
fn take_commission_engine_is_none_when_idle() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert!(runtime.take_commission_engine().is_none());
    assert!(!runtime.is_commission_active());
}

#[test]
fn raw_take_preserves_breakaway_cancellation_interrupted_state() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(breakaway_campaign_engine());
    crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
    crate::diagnostic::begin_breakaway_campaign();

    assert!(runtime.take_commission_engine().is_some());

    assert!(!runtime.is_commission_active());
    assert_eq!(
        crate::diagnostic::current_breakaway_breadcrumb_for_test(),
        Some(crate::diagnostic::BreakawayBreadcrumb {
            checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignInstalled,
            interrupted: true,
        })
    );
}

#[test]
fn non_breakaway_handled_stop_preserves_completed_integral_breadcrumb() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(8));
    crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
    crate::diagnostic::complete_breakaway_campaign(
        crate::diagnostic::BreakawayCheckpoint::IntegralHandedOff,
    );

    assert!(
        runtime
            .take_commission_engine_after_handled_stop()
            .is_some()
    );

    assert!(!runtime.is_commission_active());
    assert_eq!(
        crate::diagnostic::current_breakaway_breadcrumb_for_test(),
        Some(crate::diagnostic::BreakawayBreadcrumb {
            checkpoint: crate::diagnostic::BreakawayCheckpoint::IntegralHandedOff,
            interrupted: false,
        })
    );
}

#[test]
fn clear_calibrated_resets_flag() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.clear_calibrated();
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_before_calibration_without_gate_is_noop() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    tmc.auto_calibrate_on_enable_allowed.store(false, Relaxed);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, None);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_before_calibration_with_auto_calibrate_gate_still_does_not_energize() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    tmc.auto_calibrate_on_enable_allowed.store(true, Relaxed);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, None);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_after_calibration_emits_only_arm_preserve_position_check() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, Some(StepperPostAction::ArmPreservePosition));
    assert!(!runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
}

#[test]
fn enable_when_already_enabled_is_noop() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, None);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
}

#[test]
fn disable_request_stops_and_clears_runtime_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.prepare_disable_request();

    assert_single_stop_now(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn diagnostic_trip_clears_runtime_and_step_acceptance_state() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    tmc.reset();
    stepper.reset();
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.motor_armed.store(true, Relaxed);
    stepper.timer_active_mirror.store(true, Relaxed);

    runtime.contain_diagnostic_trip();

    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!tmc.motor_armed.load(Relaxed));
    assert!(!stepper.timer_active_mirror.load(Relaxed));
    tmc.reset();
    stepper.reset();
}

#[test]
fn feedforward_tick_writes_velocity_offset_and_marks_active() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let expected = crate::tmc::compute_planner_command_rpm_reference(
            crate::motion_scale::PlannerRpmScale::from_reduced(84_000_000, 1),
            2_800,
            false,
        ) as u32;
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), expected)]
        );
        assert!(runtime.feedforward_was_active());
        assert!(!tmc_state.velocity_clamp_latched());
    });
}

#[test]
fn feedforward_tick_stays_suppressed_while_commissioning_without_the_stimulus_opt_in() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_commission_engine(breakaway_campaign_engine());
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert!(tmc.into_inner().writes.is_empty());
        assert!(!runtime.feedforward_was_active());
        assert!(tmc_state.feedforward_surfaces_idle.load(Relaxed));
    });
}

#[test]
fn feedforward_tick_runs_under_commissioning_when_the_stimulus_opt_in_is_set() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_commission_engine(breakaway_campaign_engine());
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.commission_stimulus_active.store(true, Relaxed);
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let expected = crate::tmc::compute_planner_command_rpm_reference(
            crate::motion_scale::PlannerRpmScale::from_reduced(84_000_000, 1),
            2_800,
            false,
        ) as u32;
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), expected)]
        );
        assert!(runtime.feedforward_was_active());
        assert!(!tmc_state.feedforward_surfaces_idle.load(Relaxed));
    });
}

#[test]
fn clearing_the_stimulus_opt_in_restores_the_velocity_offset_and_raises_the_idle_mirror() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_commission_engine(breakaway_campaign_engine());
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.commission_stimulus_active.store(true, Relaxed);
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_feedforward_tick(&mut tmc, refs, &config, false, false, true)
            .await
            .unwrap();
        assert!(!tmc_state.feedforward_surfaces_idle.load(Relaxed));

        tmc_state.commission_stimulus_active.store(false, Relaxed);
        runtime
            .apply_feedforward_tick(&mut tmc, refs, &config, false, false, true)
            .await
            .unwrap();

        let writes = tmc.into_inner().writes;
        assert_eq!(writes.last(), Some(&(PID_VELOCITY_OFFSET.addr(), 0)));
        assert!(!runtime.feedforward_was_active());
        assert!(tmc_state.feedforward_surfaces_idle.load(Relaxed));
    });
}

#[test]
fn torque_flux_offset_stays_zero_under_stimulus_and_clears_after_the_opt_in_clears() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_commission_engine(breakaway_campaign_engine());
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.commission_stimulus_active.store(true, Relaxed);
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);

        runtime
            .apply_feedforward_tick(&mut tmc, refs, &config, false, false, true)
            .await
            .unwrap();
        assert!(runtime.torque_flux_offset_was_active_for_test());
        assert!(runtime.feedforward_restoration_pending());
        assert!(!tmc_state.feedforward_surfaces_idle.load(Relaxed));

        tmc_state.commission_stimulus_active.store(false, Relaxed);
        runtime
            .apply_feedforward_tick(&mut tmc, refs, &config, false, false, true)
            .await
            .unwrap();

        let writes = tmc.into_inner().writes;
        assert!(
            writes
                .iter()
                .all(|write| *write == (PID_TORQUE_FLUX_OFFSET.addr(), 0)),
            "commissioning keeps the accel offset at zero and never reserves: {writes:?}"
        );
        assert_eq!(writes.last(), Some(&(PID_TORQUE_FLUX_OFFSET.addr(), 0)));
        assert!(!runtime.torque_flux_offset_was_active_for_test());
        assert!(!runtime.feedforward_restoration_pending());
        assert!(tmc_state.feedforward_surfaces_idle.load(Relaxed));
    });
}

#[test]
fn control_period_goes_fast_under_commissioning_only_with_the_stimulus_opt_in() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    runtime.set_commission_engine(breakaway_campaign_engine());
    let tmc_state = crate::state::tmc::TmcChannelState::new();
    let stepper_state = crate::state::stepper::StepperChannelState::new();
    let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);

    assert_eq!(
        runtime.control_period_us(refs),
        crate::tmc::CONTROL_PERIOD_IDLE_US
    );

    tmc_state.commission_stimulus_active.store(true, Relaxed);
    assert_eq!(runtime.control_period_us(refs), 1_000);
}

#[test]
fn feedforward_tick_clamps_velocity_offset_to_command_multiple_and_latches() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(60_000, Relaxed);
        stepper_state.interval.store(8_400_000, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let command = crate::tmc::compute_planner_command_rpm_reference(
            crate::motion_scale::PlannerRpmScale::from_reduced(84_000_000, 1),
            8_400_000,
            false,
        );
        let expected = command * i32::from(crate::tmc::VELOCITY_FF_CLAMP_K);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(command, 10);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), expected as u32)]
        );
        assert!(tmc_state.velocity_clamp_latched());
        assert_eq!(tmc_state.velocity_clamp_count(), 1);
    });
}

#[test]
fn feedforward_tick_clears_velocity_offset_once_after_deactivation() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_feedforward_was_active(true);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            first_tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), 0)]
        );
        assert!(!runtime.feedforward_was_active());

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(second_tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn feedforward_tick_retries_failed_velocity_offset_clear() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut active_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        let active_writes = active_tmc.into_inner().writes;
        assert_eq!(active_writes.len(), 1);
        let active_offset = active_writes[0].1 as i32;
        assert_ne!(active_offset, 0);
        assert!(runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, active_offset);
        }

        tmc_state.velocity_feedforward.store(false, Relaxed);
        let mut failed_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PID_VELOCITY_OFFSET.addr()));

        let failed_tick = runtime
            .apply_feedforward_tick(
                &mut failed_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                false,
            )
            .await;

        assert_eq!(failed_tick, Err(()));
        assert!(runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, active_offset);
        }

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            retry_tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), 0)]
        );
        assert!(!runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, 0);
        }

        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        assert!(inactive_tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn feedforward_tick_retries_failed_torque_flux_offset_clear() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut active_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let active_writes = active_tmc.into_inner().writes;
        assert_eq!(active_writes.len(), 2);
        assert_eq!(active_writes[0].0, PID_TORQUE_FLUX_LIMITS.addr());
        assert_eq!(active_writes[1].0, PID_TORQUE_FLUX_OFFSET.addr());
        let active_offset = ((active_writes[1].1 >> 16) as u16) as i16 as i32;
        assert_ne!(active_offset, 0);
        assert!(runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        let (active_gain, active_flags) = {
            let trace = runtime.trace_state();
            assert_eq!(trace.cached_accel_torque_offset, active_offset);
            assert_ne!(trace.cached_accel_gain_permille, 0);
            assert_ne!(trace.cached_accel_flags, 0);
            (trace.cached_accel_gain_permille, trace.cached_accel_flags)
        };

        tmc_state.accel_feedforward.store(false, Relaxed);
        let mut failed_tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr()),
        );

        let failed_tick = runtime
            .apply_feedforward_tick(
                &mut failed_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await;

        assert_eq!(failed_tick, Err(()));
        assert!(runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        {
            assert_eq!(
                runtime.trace_state().cached_accel_torque_offset,
                active_offset
            );
            assert_eq!(
                runtime.trace_state().cached_accel_gain_permille,
                active_gain
            );
            assert_eq!(runtime.trace_state().cached_accel_flags, active_flags);
        }

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            retry_tmc.into_inner().writes,
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 10_000)
            ]
        );
        assert!(!runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_accel_torque_offset, 0);
            assert_eq!(runtime.trace_state().cached_accel_gain_permille, 0);
            assert_eq!(runtime.trace_state().cached_accel_flags, 0);
        }

        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            !inactive_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET)
        );
    });
}

#[test]
fn feedforward_tick_skips_unchanged_velocity_offset_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(first_tmc.into_inner().writes.len(), 1);

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            second_tmc.into_inner().writes.is_empty(),
            "constant cruise must not rewrite an unchanged velocity offset"
        );
    });
}

#[test]
fn feedforward_tick_retries_velocity_offset_after_a_failed_change() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_feedforward_tick(
                &mut first_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await
            .expect("first tick applies the initial offset");
        let first_offset = first_tmc.into_inner().writes[0].1;

        stepper_state.interval.store(1_400, Relaxed);
        let mut failed_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PID_VELOCITY_OFFSET.addr()));
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut failed_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(()),
            "active-tick writes stay best-effort; only inactive-transition clears propagate errors"
        );
        assert_eq!(failed_tmc.into_inner().write_attempts.len(), 1);

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let retry_writes = retry_tmc.into_inner().writes;
        assert_eq!(
            retry_writes.len(),
            1,
            "a failed write must leave the cache invalid so the changed value is retried"
        );
        assert_ne!(retry_writes[0].1, first_offset);
    });
}

#[test]
fn feedforward_tick_skips_unchanged_torque_flux_offset_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            first_tmc
                .into_inner()
                .writes
                .iter()
                .filter(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr())
                .count(),
            1
        );

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            second_tmc.into_inner().writes.is_empty(),
            "steady acceleration must not rewrite an unchanged torque/flux offset"
        );
    });
}

#[test]
fn feedforward_tick_retries_torque_flux_offset_after_a_failed_change() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(200, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_feedforward_tick(
                &mut first_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await
            .expect("first tick applies the initial offset");
        let first_offset = first_tmc
            .into_inner()
            .writes
            .iter()
            .find(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr())
            .expect("the first tick writes an offset")
            .1;

        stepper_state.interval_add.store(10, Relaxed);
        let mut failed_tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr()),
        );
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut failed_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(()),
            "active-tick writes stay best-effort; only inactive-transition clears propagate errors"
        );
        assert_eq!(failed_tmc.into_inner().write_attempts.len(), 1);

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let retry_writes = retry_tmc.into_inner().writes;
        assert_eq!(
            retry_writes.len(),
            1,
            "a failed write must leave the cache invalid so the changed value is retried"
        );
        assert_ne!(retry_writes[0].1, first_offset);
    });
}

#[test]
fn feedforward_fast_tick_skips_torque_flux_offset_slow_work() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );

        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_OFFSET));
    });
}

#[test]
fn command_velocity_reference_zeroes_when_reset_is_requested() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, true);

    assert_eq!(reference, 0);
}

#[test]
fn command_velocity_reference_can_use_single_consumed_reset_latch() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    tmc.feedforward_reset.store(true, Relaxed);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reset = tmc.feedforward_reset.swap(false, Relaxed);
    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, reset);

    assert!(reset);
    assert_eq!(reference, 0);
    assert!(!tmc.feedforward_reset.load(Relaxed));
}

#[test]
fn command_velocity_reference_uses_motion_when_reset_false() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(
        reference,
        crate::tmc::compute_planner_command_velocity_reference(
            crate::motion_scale::PlannerVelocityScale::from_reduced(84_000_000, 1),
            2_800,
            false,
        )
    );
}

#[test]
fn command_velocity_reference_zeroes_when_runtime_inactive() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(reference, 0);
}

#[test]
fn planner_velocity_scale_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert_eq!(runtime.planner_velocity_scale(), None);
    assert_eq!(runtime.planner_rpm_scale(), None);
    runtime.set_planner_velocity_scale_for_test(123_456);
    assert_eq!(
        runtime.planner_velocity_scale(),
        Some(crate::motion_scale::PlannerVelocityScale::from_reduced(
            123_456, 1
        ))
    );
    assert_eq!(
        runtime.planner_rpm_scale(),
        Some(crate::motion_scale::PlannerRpmScale::from_reduced(
            123_456, 1
        ))
    );
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_returns_sample_before_incremented_control_tick() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 0);
    assert_eq!(tick.control_tick, 1);
    assert!(!tick.status_detail_due);
    assert!(!tick.metadata_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_marks_status_detail_due_at_100() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(99);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 99);
    assert_eq!(tick.control_tick, 100);
    assert!(tick.status_detail_due);
    assert!(!tick.metadata_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_marks_metadata_due_at_1000() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(999);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 999);
    assert_eq!(tick.control_tick, 1000);
    assert!(tick.status_detail_due);
    assert!(tick.metadata_due);
}

fn zero_cal_result() -> crate::commissioning::types::AdcCalResult {
    crate::commissioning::types::AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
        encoder_alignment_electrical_residual_counts: 0,
        encoder_alignment_stability_counts: 0,
        encoder_alignment_movement_counts: 0,
        encoder_alignment_min_movement_counts: 0,
        encoder_alignment_counts_per_electrical_rev: 0,
        adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
    }
}

fn armed_runtime_with_restore_started(runtime: &mut TmcControlRuntime, backend: &mut MockBackend) {
    use crate::commissioning::backend::CommissionMotorControl;

    let tmc = crate::state::tmc::tmc_channel0();
    tmc.mark_pi_representation_established();
    tmc.motor_armed.store(true, Relaxed);
    tmc.commission_in_flight.store(true, Relaxed);
    backend.set_enable_pin(true);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(0));

    runtime.begin_commissioning_terminal(
        crate::commissioning::schedule::CommissionTerminal::CalibrateDone(zero_cal_result()),
        &commissioning_runtime_config(),
    );
    assert!(runtime.is_restore_pending());
    assert!(runtime.is_commission_active());
}

async fn armed_runtime_mid_restore(
    runtime: &mut TmcControlRuntime,
    backend: &mut MockBackend,
    slot: &mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
) {
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    armed_runtime_with_restore_started(runtime, backend);

    let mut retention = crate::commission_dispatch::CommissionRetention::new(slot);
    assert!(
        runtime
            .poll_commissioning_terminal(backend, &mut retention, tmc, stepper)
            .await
            .is_none(),
        "the calibrate restore must need more than one poll"
    );
    assert!(runtime.is_restore_pending());
    assert!(runtime.is_commission_active());
}

async fn assert_drain_reaches_safe_state(
    runtime: &mut TmcControlRuntime,
    backend: &mut MockBackend,
    slot: &mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
) {
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();

    assert!(
        runtime.restore_cancel_pending(),
        "the exit must latch a cancel before the drain runs"
    );
    assert!(
        runtime.is_restore_pending() && runtime.is_commission_active(),
        "both slots must survive until the drain has run"
    );

    let _ = runtime
        .drive_restore_cancel(
            backend,
            tmc,
            stepper,
            crate::autotune::CommissionError::Shutdown,
        )
        .await;

    assert!(!backend.enable_pin, "the motor must be de-energized");
    assert!(!tmc.motor_armed.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!tmc.commission_in_flight.load(Relaxed));
    assert!(
        !runtime.is_restore_pending(),
        "restore machine must be gone"
    );
    assert!(!runtime.is_commission_active(), "engine slot must be gone");
    assert!(!runtime.restore_cancel_pending());

    let before = backend.spi_log.len();
    let mut retention = crate::commission_dispatch::CommissionRetention::new(slot);
    assert!(
        runtime
            .poll_commissioning_terminal(backend, &mut retention, tmc, stepper)
            .await
            .is_none()
    );
    assert_eq!(backend.spi_log.len(), before, "a later tick must be inert");
    assert!(!backend.enable_pin);
}

fn cancel_test_signals<'a>(
    shutdown_active: &'a AtomicBool,
    queue_flush: &'a AtomicBool,
    fault_active: &'a AtomicBool,
    disable_signal: &'a Signal<CriticalSectionRawMutex, ()>,
) -> TmcControlSignals<'a> {
    TmcControlSignals::new(shutdown_active, queue_flush, fault_active, disable_signal)
}

#[test]
fn shutdown_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let _ = runtime.prepare_shutdown_quiesce();

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn shutdown_safe_state_drains_a_pending_restore_before_disarming() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        assert!(!runtime.restore_cancel_pending());
        crate::tmc_control::lifecycle::apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )
        .await;

        assert!(!backend.enable_pin);
        assert!(!crate::state::tmc::tmc_channel0().motor_armed.load(Relaxed));
        assert!(!runtime.is_motor_enabled());
        assert!(!runtime.is_restore_pending());
        assert!(!runtime.is_commission_active());
        assert!(!runtime.restore_cancel_pending());
    });
}

#[test]
fn queue_flush_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let shutdown_active = AtomicBool::new(false);
        let queue_flush = AtomicBool::new(true);
        let fault_active = AtomicBool::new(false);
        let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
        let signals = cancel_test_signals(
            &shutdown_active,
            &queue_flush,
            &fault_active,
            &disable_signal,
        );
        let _ = runtime.poll_queue_flush(
            &signals,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        );

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn lease_expiry_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        runtime.request_restore_cancel();

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn request_restore_cancel_without_a_restore_is_inert() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.request_restore_cancel();
    assert!(!runtime.restore_cancel_pending());
    assert!(!runtime.is_restore_pending());
}

#[test]
fn cancelled_mid_restore_reports_cancelled_status_not_shutdown() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.cancel_requested.store(true, Relaxed);

        let pwm_maxcnt = commissioning_runtime_config().tmc.pwm_maxcnt;
        let outcome = runtime
            .drive_operator_cancel(&mut backend, tmc, stepper, pwm_maxcnt)
            .await
            .expect("a cancel latched mid-restore must produce a terminal outcome");

        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, .. } => match reply {
                crate::commission_dispatch::ReplyPayload::CalibrateResult { status, .. } => {
                    assert_eq!(
                        status,
                        crate::calibration::CalibrationError::Cancelled.status_code(),
                        "a mid-restore operator cancel must report Cancelled, not Shutdown"
                    );
                }
                _ => panic!("expected a CalibrateResult reply from a CalibrateDone restore"),
            },
            _ => panic!("expected a Terminal outcome"),
        }

        assert!(!backend.enable_pin, "the motor must be de-energized");
        assert!(!tmc.commission_in_flight.load(Relaxed));
        assert!(
            !tmc.cancel_requested.load(Relaxed),
            "the cancel flag must be consumed"
        );
        assert!(!runtime.is_restore_pending());
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn cancelled_mid_phase_disarms_only_the_targeted_channel_and_reports_cancelled() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();

        use crate::commissioning::backend::CommissionMotorControl;

        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.mark_pi_representation_established();
        tmc.motor_armed.store(true, Relaxed);
        tmc.commission_in_flight.store(true, Relaxed);
        tmc.cancel_requested.store(true, Relaxed);

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend.set_enable_pin(true);
        runtime.set_calibrated_for_test(true);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            7,
            crate::autotune::CommissionProfile::Balanced,
        ));
        assert!(runtime.is_commission_active());
        assert!(!runtime.is_restore_pending(), "no restore machine yet");

        let pwm_maxcnt = commissioning_runtime_config().tmc.pwm_maxcnt;
        let outcome = runtime
            .drive_operator_cancel(&mut backend, tmc, stepper, pwm_maxcnt)
            .await
            .expect("a cancel latched mid-phase must produce a terminal outcome");

        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, .. } => match reply {
                crate::commission_dispatch::ReplyPayload::CommissionResult { status, .. } => {
                    assert_eq!(
                        status,
                        crate::autotune::CommissionError::Cancelled.status_code()
                    );
                }
                _ => panic!("expected a CommissionResult reply from a Commission-kind engine"),
            },
            _ => panic!("expected a Terminal outcome"),
        }

        assert!(
            !backend.enable_pin,
            "channel 0's motor must be de-energized"
        );
        assert!(!tmc.motor_armed.load(Relaxed));
        assert!(!tmc.commission_in_flight.load(Relaxed));
        assert!(
            !tmc.cancel_requested.load(Relaxed),
            "the cancel flag must be consumed"
        );
        assert!(!runtime.is_commission_active());
        assert!(!runtime.is_restore_pending());

        let other_channel = crate::state::tmc::tmc_channel(1)
            .expect("a second channel's static state must exist on this build");
        assert!(
            !other_channel.motor_armed.load(Relaxed),
            "the other channel's motor state must be untouched"
        );
        assert!(
            !other_channel.commission_in_flight.load(Relaxed),
            "the other channel's commissioning state must be untouched"
        );
        assert!(
            !other_channel.cancel_requested.load(Relaxed),
            "the other channel's cancel flag must be untouched"
        );
    });
}

fn position_tune_cancel_backend() -> MockBackend {
    let mut backend = MockBackend::new();
    backend.spi_elapsed_step_us = 0;
    backend.elapsed_us_value = 1_000_000;
    backend.current_limit_value = current_limit(5_000);
    backend.mechanical_encoder_quadrature_counts_per_rev_value = 4_000;
    backend.configured_physical_width = Some(8);
    backend.configured_planner_steps = Some(8_192);
    let _ = backend.registers.insert(
        PID_VELOCITY_P_VELOCITY_I.addr(),
        foci_tmc4671::pid::PidGainPair::new(
            foci_tmc4671::pid::PidPGain::new(350).unwrap(),
            foci_tmc4671::pid::PidIGain::ZERO,
        )
        .register_value(),
    );
    backend
}

fn position_tune_engine_mid_stroke(backend: &mut MockBackend) -> CommissioningEngine {
    let mut engine = CommissioningEngine::new_outer_for_tmc_with_position_tune(
        7,
        crate::state::tmc::tmc_channel0(),
        sample_outer_params(),
        sample_entry_snapshot(),
        0,
    );
    let mut clock_us = backend.elapsed_us_value;
    for _ in 0..6 {
        backend.elapsed_us_value = clock_us;
        let result = pollster::block_on(engine.step(backend));
        match result {
            crate::commissioning::types::StepResult::At(next_us) => clock_us = next_us,
            crate::commissioning::types::StepResult::ReadyNow
            | crate::commissioning::types::StepResult::PhaseChanged(_) => {}
            other => panic!("unexpected early terminal driving to mid-stroke: {other:?}"),
        }
    }
    engine
}

fn assert_stimulus_stopped_before_safe_state_restore(event_log: &[MockEvent]) {
    let restore_write_index = event_log
        .iter()
        .position(|event| {
            matches!(event, MockEvent::Spi(SpiOp::Write(addr, 0)) if *addr == MODE_RAMP_MODE_MOTION.addr())
        })
        .expect("the safe-state restore's MODE_RAMP_MODE_MOTION zero write must be recorded");
    let preceding = &event_log[..restore_write_index];
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
        "abort_stimulus's Stop then Feedforward(false) must be the last events before the \
         safe-state restore begins, with nothing else between them and the write"
    );
}

#[test]
fn an_operator_cancel_mid_stroke_stops_the_stimulus_before_the_disarm() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();

        let mut backend = position_tune_cancel_backend();
        let engine = position_tune_engine_mid_stroke(&mut backend);

        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(engine);
        assert!(
            !runtime.is_restore_pending(),
            "a mid-stroke engine has not yet produced a terminal restore"
        );
        tmc.cancel_requested.store(true, Relaxed);

        let pwm_maxcnt = commissioning_runtime_config().tmc.pwm_maxcnt;
        let outcome = runtime
            .drive_operator_cancel(&mut backend, tmc, stepper, pwm_maxcnt)
            .await
            .expect("a cancel latched mid-stroke must produce a terminal outcome");
        assert!(matches!(
            outcome,
            crate::commission_dispatch::CommissionOutcome::Terminal { .. }
        ));

        assert_stimulus_stopped_before_safe_state_restore(&backend.event_log);
    });
}

#[test]
fn a_shutdown_safe_state_mid_stroke_stops_the_stimulus_before_the_disarm() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();

        let mut backend = position_tune_cancel_backend();
        let engine = position_tune_engine_mid_stroke(&mut backend);

        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(engine);
        assert!(
            !runtime.is_restore_pending(),
            "a mid-stroke engine has not yet produced a terminal restore"
        );

        let _ = crate::tmc_control::lifecycle::apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            tmc,
            stepper,
        )
        .await;

        assert_stimulus_stopped_before_safe_state_restore(&backend.event_log);
    });
}

fn assert_dropped_mid_operation(backend: &MockBackend, ops_before: usize) {
    assert!(
        backend.spi_log.len() > ops_before,
        "the suspension must interrupt a real backend operation, not gate entry to one"
    );
    assert!(
        matches!(backend.spi_log.last(), Some(SpiOp::Write(_, _))),
        "the interrupted verified write must have landed its write and not yet read back, \
         got {:?}",
        backend.spi_log.last()
    );
}

fn lock_and_reset_shared_state() -> impl core::ops::DerefMut<Target = ()> + 'static {
    let guard =
        pollster::block_on(crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.lock());
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();
    guard
}

#[test]
fn a_torn_down_restore_poll_keeps_both_slots_installed() {
    let _guard = lock_and_reset_shared_state();
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    let mut slot =
        crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut backend = MockBackend::new();
    armed_runtime_with_restore_started(&mut runtime, &mut backend);
    runtime.request_restore_cancel();
    pollster::block_on(runtime.drive_restore_cancel(
        &mut backend,
        tmc,
        stepper,
        crate::autotune::CommissionError::Shutdown,
    ));
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(0));
    runtime.begin_commissioning_terminal(
        crate::commissioning::schedule::CommissionTerminal::InnerFailed(
            crate::autotune::CommissionError::EncoderFault,
        ),
        &commissioning_runtime_config(),
    );

    backend.pend_next_op = true;
    let ops_before = backend.spi_log.len();
    let suspended = {
        let mut retention = crate::commission_dispatch::CommissionRetention::new(&mut slot);
        let mut future = std::boxed::Box::pin(runtime.poll_commissioning_terminal(
            &mut backend,
            &mut retention,
            tmc,
            stepper,
        ));
        let mut context = std::task::Context::from_waker(std::task::Waker::noop());
        matches!(future.as_mut().poll(&mut context), std::task::Poll::Pending)
    };

    assert!(
        suspended,
        "the mock must suspend so the poll is genuinely in flight when dropped"
    );
    assert_dropped_mid_operation(&backend, ops_before);
    assert!(
        runtime.is_restore_pending(),
        "a torn-down poll must not lose the restore machine"
    );
    assert!(
        runtime.is_commission_active(),
        "a torn-down poll must not lose the engine"
    );
}

#[test]
fn a_torn_down_restore_cancel_keeps_the_slots_and_latch_and_stays_resumable() {
    let _guard = lock_and_reset_shared_state();
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    let mut slot =
        crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut backend = MockBackend::new();
    pollster::block_on(armed_runtime_mid_restore(
        &mut runtime,
        &mut backend,
        &mut slot,
    ));
    runtime.request_restore_cancel();

    backend.pend_next_op = true;
    let ops_before = backend.spi_log.len();
    let suspended = {
        let mut future = std::boxed::Box::pin(runtime.drive_restore_cancel(
            &mut backend,
            tmc,
            stepper,
            crate::autotune::CommissionError::Shutdown,
        ));
        let mut context = std::task::Context::from_waker(std::task::Waker::noop());
        matches!(future.as_mut().poll(&mut context), std::task::Poll::Pending)
    };

    assert!(
        suspended,
        "the mock must suspend so the drain is genuinely in flight when dropped"
    );
    assert_dropped_mid_operation(&backend, ops_before);
    assert!(
        runtime.is_restore_pending(),
        "a torn-down drain must not lose the restore machine"
    );
    assert!(
        runtime.is_commission_active(),
        "a torn-down drain must not lose the engine"
    );
    assert!(
        runtime.restore_cancel_pending(),
        "a torn-down drain must leave the cancel latched so it is retried"
    );
    pollster::block_on(assert_drain_reaches_safe_state(
        &mut runtime,
        &mut backend,
        &mut slot,
    ));
}

async fn last_accel_offset_written(limit_raw: u32, interval_add: i32) -> Option<i16> {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    let tmc_state = crate::state::tmc::TmcChannelState::new();
    let stepper_state = crate::state::stepper::StepperChannelState::new();
    let config = commissioning_runtime_config();
    tmc_state.accel_feedforward.store(true, Relaxed);
    tmc_state
        .accel_feedforward_accel_gain_permille
        .store(1_000, Relaxed);
    tmc_state
        .accel_feedforward_decel_gain_permille
        .store(1_000, Relaxed);
    tmc_state.store_current_limit(current_limit(limit_raw), Relaxed);
    stepper_state.interval.store(2_800, Relaxed);
    stepper_state.interval_add.store(interval_add, Relaxed);
    let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

    runtime
        .apply_feedforward_tick(
            &mut tmc,
            TmcControlStateRefs::new(&tmc_state, &stepper_state),
            &config,
            false,
            false,
            true,
        )
        .await
        .unwrap();

    tmc.into_inner()
        .writes
        .iter()
        .rev()
        .find(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr())
        .map(|(_, value)| decode_signed_upper_half(*value))
}

#[test]
fn accel_offset_is_capped_at_half_the_current_limit() {
    pollster::block_on(async {
        let offset = last_accel_offset_written(2557, -90).await;

        assert_eq!(offset.map(i16::unsigned_abs), Some(1278));
    });
}

#[test]
fn accel_offset_is_zero_when_headroom_is_zero() {
    pollster::block_on(async {
        let offset = last_accel_offset_written(1, -90).await;

        assert_eq!(offset.unwrap_or(0), 0);
    });
}

struct ReservationRig {
    runtime: TmcControlRuntime,
    tmc_state: &'static crate::state::tmc::TmcChannelState,
    stepper_state: &'static crate::state::stepper::StepperChannelState,
    config: TmcControlConfig,
    log: std::vec::Vec<(u8, u32)>,
    initial_limit: u32,
    committed_at_write: std::rc::Rc<std::cell::RefCell<std::vec::Vec<u32>>>,
}

impl ReservationRig {
    fn eligible(limit_raw: u32) -> Self {
        let tmc_state: &'static _ = std::boxed::Box::leak(std::boxed::Box::new(
            crate::state::tmc::TmcChannelState::new(),
        ));
        let stepper_state: &'static _ = std::boxed::Box::leak(std::boxed::Box::new(
            crate::state::stepper::StepperChannelState::new(),
        ));
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(limit_raw), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-90, Relaxed);
        Self {
            runtime,
            tmc_state,
            stepper_state,
            config: homing_runtime_config(),
            log: std::vec::Vec::new(),
            initial_limit: limit_raw,
            committed_at_write: std::rc::Rc::default(),
        }
    }

    fn observed(&self, spi: RuntimeMockSpi) -> RuntimeMockSpi {
        let committed_at_write = self.committed_at_write.clone();
        let tmc_state = self.tmc_state;
        spi.observe_writes(move |_, _| {
            committed_at_write
                .borrow_mut()
                .push(tmc_state.load_current_limit(Relaxed).register_value());
        })
    }

    fn record(&mut self, spi: RuntimeMockSpi) {
        self.log.extend(spi.writes);
        let (limit, offset) = applied_pair(&self.log, self.initial_limit);
        assert!(
            limit + offset <= self.committed(),
            "applied limit {limit} + |offset| {offset} exceeds committed limit {} between ticks: \
             {:?}",
            self.committed(),
            self.log
        );
    }

    fn assert_observed_invariant(&self) {
        let committed_at_write = self.committed_at_write.borrow();
        let mut previous = self.initial_limit;
        let schedule: std::vec::Vec<(usize, u32)> = committed_at_write
            .iter()
            .enumerate()
            .filter_map(|(index, &committed)| {
                let changed = committed != previous;
                previous = committed;
                changed.then_some((index, committed))
            })
            .collect();
        assert_limit_plus_offset_within_schedule(&self.log, self.initial_limit, &schedule);
    }

    fn refs(&self) -> TmcControlStateRefs<'static> {
        TmcControlStateRefs::new(self.tmc_state, self.stepper_state)
    }

    async fn tick_on(
        &mut self,
        spi: RuntimeMockSpi,
        homing_active: bool,
        reset: bool,
    ) -> Result<(), ()> {
        let mut tmc = Tmc4671Spi::new(self.observed(spi));
        let refs = self.refs();
        let result = self
            .runtime
            .apply_feedforward_tick(&mut tmc, refs, &self.config, homing_active, reset, true)
            .await;
        self.record(tmc.into_inner());
        result
    }

    async fn tick(&mut self) -> Result<(), ()> {
        self.tick_on(RuntimeMockSpi::default(), false, false).await
    }

    async fn maintenance_unit(
        &mut self,
        spi: RuntimeMockSpi,
        homing_snapshot: u8,
    ) -> HomingMaintenanceResult {
        let mut tmc = Tmc4671Spi::new(self.observed(spi));
        let refs = self.refs();
        let _ = self
            .runtime
            .apply_feedforward_tick(
                &mut tmc,
                refs,
                &self.config,
                homing_snapshot != 0,
                false,
                true,
            )
            .await;
        let result = self
            .runtime
            .apply_slow_channel_maintenance_tick(
                &mut tmc,
                refs,
                &self.config,
                homing_snapshot,
                due_homing_window(),
            )
            .await;
        self.record(tmc.into_inner());
        result
    }

    fn committed(&self) -> u32 {
        self.tmc_state.load_current_limit(Relaxed).register_value()
    }

    fn phase(&self) -> crate::tmc_control::state::OffsetHeadroomPhase {
        self.runtime.offset_headroom_phase_for_test()
    }

    fn reserved_for(limit_raw: u32) -> u32 {
        crate::tmc_control::current_limit::reserved_limit(current_limit(limit_raw)).register_value()
    }
}

fn applied_pair(log: &[(u8, u32)], initial_limit: u32) -> (u32, u32) {
    log.iter()
        .fold((initial_limit, 0), |(limit, offset), &(addr, value)| {
            if addr == PID_TORQUE_FLUX_LIMITS.addr() {
                (value & 0xffff, offset)
            } else if addr == PID_TORQUE_FLUX_OFFSET.addr() {
                (
                    limit,
                    u32::from(decode_signed_upper_half(value).unsigned_abs()),
                )
            } else {
                (limit, offset)
            }
        })
}

fn limit_offset_writes(log: &[(u8, u32)]) -> std::vec::Vec<(u8, u32)> {
    log.iter()
        .copied()
        .filter(|(addr, _)| {
            *addr == PID_TORQUE_FLUX_LIMITS.addr() || *addr == PID_TORQUE_FLUX_OFFSET.addr()
        })
        .collect()
}

fn is_nonzero_offset(write: &(u8, u32)) -> bool {
    write.0 == PID_TORQUE_FLUX_OFFSET.addr() && decode_signed_upper_half(write.1) != 0
}

fn position_of(log: &[(u8, u32)], write: (u8, u32)) -> usize {
    log.iter()
        .position(|entry| *entry == write)
        .unwrap_or_else(|| panic!("expected write {write:?} in {log:?}"))
}

#[test]
fn engage_lowers_the_limit_before_any_nonzero_offset() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);

        rig.tick().await.unwrap();

        let first_limit = rig
            .log
            .iter()
            .position(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr())
            .expect("engage writes the reserved limit");
        assert_eq!(rig.log[first_limit].1, ReservationRig::reserved_for(1_000));
        let first_nonzero = rig
            .log
            .iter()
            .position(is_nonzero_offset)
            .expect("a held reservation applies the accel offset");
        assert!(first_limit < first_nonzero, "{:?}", rig.log);
        assert!(rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn engage_limit_write_failure_keeps_the_offset_zero() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);

        for _ in 0..3 {
            let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr());
            let _ = rig.tick_on(failing, false, false).await;
        }

        assert!(!rig.log.iter().any(is_nonzero_offset), "{:?}", rig.log);
        assert!(rig.tmc_state.offset_headroom_reserved(Relaxed));

        rig.tick().await.unwrap();

        assert!(rig.log.contains(&(
            PID_TORQUE_FLUX_LIMITS.addr(),
            ReservationRig::reserved_for(1_000)
        )));
        assert!(rig.log.iter().any(is_nonzero_offset), "{:?}", rig.log);
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn disengage_clears_the_offset_before_restoring_the_limit() {
    type Trigger = fn(&mut ReservationRig) -> bool;
    let triggers: [(&str, Trigger); 4] = [
        ("gains to zero", |rig| {
            rig.tmc_state
                .accel_feedforward_accel_gain_permille
                .store(0, Relaxed);
            rig.tmc_state
                .accel_feedforward_decel_gain_permille
                .store(0, Relaxed);
            false
        }),
        ("flag off", |rig| {
            rig.tmc_state.accel_feedforward.store(false, Relaxed);
            false
        }),
        ("motor disabled", |rig| {
            rig.runtime.set_motor_enabled_for_test(false);
            false
        }),
        ("reset", |_| true),
    ];
    for (name, trigger) in triggers {
        pollster::block_on(async {
            let mut rig = ReservationRig::eligible(1_000);
            rig.tick().await.unwrap();
            let held_len = rig.log.len();

            let reset = trigger(&mut rig);
            rig.tick_on(RuntimeMockSpi::default(), false, reset)
                .await
                .unwrap();

            let release = limit_offset_writes(&rig.log[held_len..]);
            assert_eq!(
                release,
                std::vec![
                    (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                    (PID_TORQUE_FLUX_LIMITS.addr(), 1_000)
                ],
                "{name}"
            );
            assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed), "{name}");
            assert_limit_plus_offset_within(&rig.log, 1_000);
        });
    }
}

#[test]
fn disengage_offset_write_failure_never_raises_the_limit() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tick().await.unwrap();
        rig.tmc_state.accel_feedforward.store(false, Relaxed);

        for _ in 0..3 {
            let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr());
            assert_eq!(rig.tick_on(failing, false, false).await, Err(()));
        }

        let reserved = ReservationRig::reserved_for(1_000);
        assert!(
            !rig.log
                .iter()
                .any(|(addr, value)| *addr == PID_TORQUE_FLUX_LIMITS.addr() && *value > reserved),
            "{:?}",
            rig.log
        );
        assert!(rig.tmc_state.offset_headroom_reserved(Relaxed));

        rig.tick().await.unwrap();

        let zero = position_of(&rig.log, (PID_TORQUE_FLUX_OFFSET.addr(), 0));
        let full = position_of(&rig.log, (PID_TORQUE_FLUX_LIMITS.addr(), 1_000));
        assert!(zero < full);
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn disengage_limit_write_failure_is_retried() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tick().await.unwrap();
        rig.tmc_state.accel_feedforward.store(false, Relaxed);

        let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr());
        assert_eq!(rig.tick_on(failing, false, false).await, Err(()));
        assert!(rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert!(rig.runtime.offset_headroom_engaged());

        let retry_start = rig.log.len();
        rig.tick().await.unwrap();

        assert_eq!(
            limit_offset_writes(&rig.log[retry_start..]),
            std::vec![(PID_TORQUE_FLUX_LIMITS.addr(), 1_000)]
        );
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert!(!rig.runtime.offset_headroom_engaged());
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn back_emf_only_mode_never_touches_the_limit() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tmc_state
            .accel_feedforward_accel_gain_permille
            .store(0, Relaxed);
        rig.tmc_state
            .accel_feedforward_decel_gain_permille
            .store(0, Relaxed);

        for _ in 0..4 {
            rig.tick().await.unwrap();
        }

        assert!(
            !rig.log
                .iter()
                .any(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr()),
            "{:?}",
            rig.log
        );
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn decel_gain_alone_reserves() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tmc_state
            .accel_feedforward_accel_gain_permille
            .store(0, Relaxed);
        rig.stepper_state.interval_add.store(90, Relaxed);

        rig.tick().await.unwrap();

        assert!(rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert!(rig.log.contains(&(
            PID_TORQUE_FLUX_LIMITS.addr(),
            ReservationRig::reserved_for(1_000)
        )));
        assert!(rig.log.iter().any(is_nonzero_offset), "{:?}", rig.log);
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn tiny_limit_has_no_headroom_and_no_error() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1);

        for _ in 0..3 {
            assert_eq!(rig.tick().await, Ok(()));
        }

        assert!(!rig.log.iter().any(is_nonzero_offset), "{:?}", rig.log);
        assert_limit_plus_offset_within(&rig.log, 1);
    });
}

#[test]
fn downward_current_change_from_a_full_reservation_stays_within_bounds() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tick().await.unwrap();
        assert!(rig.log.contains(&(
            PID_TORQUE_FLUX_LIMITS.addr(),
            ReservationRig::reserved_for(1_000)
        )));
        assert!(
            rig.log
                .iter()
                .any(|write| write.0 == PID_TORQUE_FLUX_OFFSET.addr()
                    && decode_signed_upper_half(write.1).unsigned_abs() == 500),
            "{:?}",
            rig.log
        );
        let change_start = rig.log.len();

        rig.tmc_state.store_pending_run_limit(current_limit(600));
        rig.tick().await.unwrap();

        let change = limit_offset_writes(&rig.log[change_start..]);
        assert_eq!(
            change,
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 600)
            ]
        );
        assert_eq!(rig.committed(), 600);
        let commit_at = position_of(&rig.log, (PID_TORQUE_FLUX_LIMITS.addr(), 600));

        rig.tick().await.unwrap();

        let reserved = ReservationRig::reserved_for(600);
        assert_eq!(reserved, 300);
        assert_eq!(
            rig.log.last().map(|write| write.0),
            Some(PID_TORQUE_FLUX_OFFSET.addr())
        );
        assert!(position_of(&rig.log, (PID_TORQUE_FLUX_LIMITS.addr(), reserved)) > commit_at);
        assert_eq!(
            rig.log
                .last()
                .map(|write| decode_signed_upper_half(write.1).unsigned_abs()),
            Some(300)
        );
        rig.assert_observed_invariant();
    });
}

fn configure_homing(rig: &mut ReservationRig) {
    let tmc_state = rig.tmc_state;
    tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
    tmc_state.homing_current_ma.store(700, Relaxed);
    tmc_state.stall_ceiling_units.store(100, Relaxed);
    tmc_state.stall_margin_units.store(99, Relaxed);
    tmc_state.stall_persistence.store(2, Relaxed);
    tmc_state
        .configure_motion_scale(
            &crate::state::stepper::StepperChannelState::new(),
            1_000,
            3_200,
        )
        .unwrap();
    rig.runtime.set_calibrated_for_test(true);
}

#[test]
fn homing_snapshot_releases_before_the_clamp() {
    pollster::block_on(async {
        let run_limit = homing_limit_for(2300).register_value();
        let clamp = homing_limit_for(700).register_value();
        let mut rig = ReservationRig::eligible(run_limit);
        configure_homing(&mut rig);
        rig.tick().await.unwrap();
        assert!(rig.runtime.offset_headroom_engaged());
        let held_len = rig.log.len();

        let result = rig.maintenance_unit(RuntimeMockSpi::default(), 1).await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            limit_offset_writes(&rig.log[held_len..]),
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), run_limit),
                (PID_TORQUE_FLUX_LIMITS.addr(), clamp),
            ]
        );
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert_limit_plus_offset_within(&rig.log, run_limit);
    });
}

#[test]
fn homing_clamp_entry_waits_for_a_pending_release() {
    pollster::block_on(async {
        let run_limit = homing_limit_for(2300).register_value();
        let clamp = homing_limit_for(700).register_value();
        let mut rig = ReservationRig::eligible(run_limit);
        configure_homing(&mut rig);
        rig.tick().await.unwrap();
        let held_len = rig.log.len();

        let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr());
        let blocked = rig.maintenance_unit(failing, 1).await;

        assert_eq!(blocked, HomingMaintenanceResult::RetryCurrentState);
        assert!(
            limit_offset_writes(&rig.log[held_len..]).is_empty(),
            "{:?}",
            rig.log
        );
        assert!(!rig.tmc_state.homing_clamp_active.load(Relaxed));

        let result = rig.maintenance_unit(RuntimeMockSpi::default(), 1).await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            limit_offset_writes(&rig.log[held_len..]),
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), run_limit),
                (PID_TORQUE_FLUX_LIMITS.addr(), clamp),
            ]
        );
        assert_limit_plus_offset_within(&rig.log, run_limit);
    });
}

#[test]
fn no_engagement_while_homing_clamp_is_active() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tmc_state.homing_clamp_active.store(true, Relaxed);

        for _ in 0..3 {
            rig.tick().await.unwrap();
        }

        assert!(
            !rig.log
                .iter()
                .any(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr()),
            "{:?}",
            rig.log
        );
        assert!(!rig.log.iter().any(is_nonzero_offset), "{:?}", rig.log);
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));

        rig.tmc_state.homing_clamp_active.store(false, Relaxed);
        rig.tick().await.unwrap();

        assert!(rig.runtime.offset_headroom_engaged());
        assert!(rig.log.iter().any(is_nonzero_offset), "{:?}", rig.log);
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn no_engagement_while_a_commissioning_engine_or_restore_is_pending() {
    type Setup = fn(&mut ReservationRig);
    let setups: [(&str, Setup); 2] = [
        ("commissioning engine", |rig| {
            rig.runtime
                .set_commission_engine(breakaway_campaign_engine());
            rig.tmc_state
                .commission_stimulus_active
                .store(true, Relaxed);
        }),
        ("restore pending", |rig| {
            let config = rig.config;
            rig.runtime.begin_commissioning_terminal(
                crate::commissioning::schedule::CommissionTerminal::CalibrateDone(
                    zero_cal_result(),
                ),
                &config,
            );
        }),
    ];
    for (name, setup) in setups {
        pollster::block_on(async {
            let mut rig = ReservationRig::eligible(1_000);
            setup(&mut rig);

            for _ in 0..3 {
                rig.tick().await.unwrap();
            }

            assert!(
                !rig.log
                    .iter()
                    .any(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr()),
                "{name}: {:?}",
                rig.log
            );
            assert!(
                !rig.log.iter().any(is_nonzero_offset),
                "{name}: {:?}",
                rig.log
            );
            assert!(!rig.runtime.offset_headroom_engaged(), "{name}");
            assert_limit_plus_offset_within(&rig.log, 1_000);
        });
    }
}

#[test]
fn set_current_arriving_during_a_transition_write_is_neither_lost_nor_unsafe() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        let tmc_state = rig.tmc_state;

        let engage_race = RuntimeMockSpi::default().race_pending_config_on_write(
            PID_TORQUE_FLUX_LIMITS.addr(),
            move || {
                tmc_state.store_pending_run_limit(current_limit(600));
            },
        );
        rig.tick_on(engage_race, false, false).await.unwrap();
        assert_eq!(rig.committed(), 1_000);
        rig.tick().await.unwrap();
        assert_eq!(rig.committed(), 600);
        let first_write = position_of(&rig.log, (PID_TORQUE_FLUX_LIMITS.addr(), 600));
        rig.tick().await.unwrap();
        assert!(rig.runtime.offset_headroom_engaged());

        rig.tmc_state.accel_feedforward.store(false, Relaxed);
        let release_race = RuntimeMockSpi::default().race_pending_config_on_write(
            PID_TORQUE_FLUX_LIMITS.addr(),
            move || {
                tmc_state.store_pending_run_limit(current_limit(800));
            },
        );
        let _ = rig.tick_on(release_race, false, false).await;
        for _ in 0..2 {
            rig.tick().await.unwrap();
        }

        assert_eq!(rig.committed(), 800);
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert_eq!(rig.tmc_state.peek_pending_run_limit(), None);
        let second_write = position_of(&rig.log, (PID_TORQUE_FLUX_LIMITS.addr(), 800));
        for write in [first_write, second_write] {
            let last_offset = rig.log[..write]
                .iter()
                .rev()
                .find(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr())
                .map(|write| write.1);
            assert_eq!(last_offset, Some(0), "{:?}", rig.log);
        }
        assert_eq!(
            rig.log.last(),
            Some(&(PID_TORQUE_FLUX_LIMITS.addr(), 800)),
            "{:?}",
            rig.log
        );
        rig.assert_observed_invariant();
    });
}

#[test]
fn reservation_obligation_keeps_maintenance_alive() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tick().await.unwrap();
        rig.runtime.set_motor_enabled_for_test(false);
        let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr());
        let _ = rig.tick_on(failing, false, false).await;

        assert!(!rig.runtime.maintenance_obligations_clear());
        assert!(rig.runtime.feedforward_restoration_pending());

        rig.tick().await.unwrap();

        assert!(rig.runtime.maintenance_obligations_clear());
        assert!(!rig.runtime.feedforward_restoration_pending());
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

#[test]
fn requested_release_completes_from_a_held_reservation() {
    pollster::block_on(async {
        let mut rig = ReservationRig::eligible(1_000);
        rig.tick().await.unwrap();
        let held_len = rig.log.len();

        let tmc_state = rig.tmc_state;
        rig.runtime.request_offset_headroom_release(tmc_state);
        assert!(rig.runtime.offset_headroom_engaged());
        rig.tick().await.unwrap();

        assert_eq!(
            limit_offset_writes(&rig.log[held_len..]),
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 1_000)
            ]
        );
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        assert_eq!(
            rig.phase(),
            crate::tmc_control::state::OffsetHeadroomPhase::Released
        );
        assert_limit_plus_offset_within(&rig.log, 1_000);
    });
}

fn held_rig(limit_raw: u32) -> ReservationRig {
    let mut rig = ReservationRig::eligible(limit_raw);
    pollster::block_on(rig.tick()).unwrap();
    assert_eq!(
        rig.phase(),
        crate::tmc_control::state::OffsetHeadroomPhase::Held
    );
    rig
}

#[test]
fn downward_pending_limit_is_committed_only_once_the_chip_holds_it() {
    pollster::block_on(async {
        let mut rig = held_rig(1_000);
        rig.tmc_state.store_pending_run_limit(current_limit(300));

        let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr());
        assert_eq!(rig.tick_on(failing, false, false).await, Err(()));

        assert_eq!(rig.committed(), 1_000);
        assert!(rig.tmc_state.has_pending_run_limit());

        rig.tick().await.unwrap();

        assert_eq!(rig.committed(), 300);
        assert_eq!(applied_pair(&rig.log, 1_000), (300, 0));
        assert!(!rig.tmc_state.has_pending_run_limit());
        rig.assert_observed_invariant();
    });
}

#[test]
fn upward_pending_limit_is_committed_before_it_is_written() {
    pollster::block_on(async {
        let mut rig = held_rig(1_000);
        let held_len = rig.log.len();
        rig.tmc_state.store_pending_run_limit(current_limit(1_200));

        rig.tick().await.unwrap();

        assert_eq!(
            limit_offset_writes(&rig.log[held_len..]),
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 1_200)
            ]
        );
        let write = position_of(&rig.log, (PID_TORQUE_FLUX_LIMITS.addr(), 1_200));
        assert_eq!(rig.committed_at_write.borrow()[write], 1_200);
        assert_eq!(rig.committed(), 1_200);
        rig.assert_observed_invariant();
    });
}

#[test]
fn pending_limit_arriving_during_the_release_write_waits_for_the_next_tick() {
    pollster::block_on(async {
        let mut rig = held_rig(1_000);
        let tmc_state = rig.tmc_state;
        rig.tmc_state.accel_feedforward.store(false, Relaxed);
        let race = RuntimeMockSpi::default()
            .race_pending_config_on_write(PID_TORQUE_FLUX_LIMITS.addr(), move || {
                tmc_state.store_pending_run_limit(current_limit(300))
            });

        rig.tick_on(race, false, false).await.unwrap();

        assert_eq!(rig.committed(), 1_000);
        assert_eq!(
            rig.phase(),
            crate::tmc_control::state::OffsetHeadroomPhase::Releasing
        );
        assert!(rig.tmc_state.offset_headroom_reserved(Relaxed));

        rig.tick().await.unwrap();

        assert_eq!(rig.committed(), 300);
        assert_eq!(applied_pair(&rig.log, 1_000), (300, 0));
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        rig.assert_observed_invariant();
    });
}

#[test]
fn failed_pending_limit_write_keeps_the_request() {
    pollster::block_on(async {
        let mut rig = held_rig(1_000);
        rig.tmc_state.store_pending_run_limit(current_limit(1_200));

        let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr());
        assert_eq!(rig.tick_on(failing, false, false).await, Err(()));

        assert!(rig.tmc_state.has_pending_run_limit());

        rig.tick().await.unwrap();

        assert_eq!(rig.committed(), 1_200);
        assert_eq!(applied_pair(&rig.log, 1_000), (1_200, 0));
        assert!(!rig.tmc_state.has_pending_run_limit());
        rig.assert_observed_invariant();
    });
}

impl ReservationRig {
    async fn drain_set_current(
        &mut self,
        run_ma: u32,
    ) -> Result<(), crate::tmc_control::pending_config::PendingConfigItem> {
        self.tmc_state.run_current_ma_cfg.store(run_ma, Relaxed);
        self.tmc_state
            .config_apply_pending
            .fetch_or(crate::state::tmc::PENDING_CURRENT, Relaxed);
        let mut stored_inner_gains = None;
        let mut tmc = Tmc4671Spi::new(self.observed(RuntimeMockSpi::default()));
        let result = crate::tmc_control::pending_config::drain_pending_config(
            &mut tmc,
            self.tmc_state,
            self.config.tmc,
            crate::tmc_control::pending_config::PendingConfigSink {
                stored_inner_gains: &mut stored_inner_gains,
            },
        )
        .await;
        self.record(tmc.into_inner());
        result
    }

    fn run_limit_for(&self, run_ma: u32) -> u32 {
        crate::tmc_control::current_limit::current_limit_from_run_ma(
            run_ma,
            self.config.tmc.current_scale_ma_per_lsb,
        )
        .expect("current must be representable")
        .register_value()
    }
}

fn drained_set_current_while_held_converges(run_ma: u32) {
    pollster::block_on(async {
        let mut rig = held_rig(1_000);
        let requested = rig.run_limit_for(run_ma);
        let drain_start = rig.log.len();

        assert_eq!(rig.drain_set_current(run_ma).await, Ok(()));

        assert!(limit_offset_writes(&rig.log[drain_start..]).is_empty());
        assert_eq!(rig.committed(), 1_000);
        assert_eq!(
            rig.tmc_state.config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_CURRENT,
            0
        );
        assert_eq!(
            rig.tmc_state
                .peek_pending_run_limit()
                .map(|limit| limit.register_value()),
            Some(requested)
        );
        let release_start = rig.log.len();

        rig.tick().await.unwrap();

        assert_eq!(
            limit_offset_writes(&rig.log[release_start..]),
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), requested)
            ]
        );
        let write = release_start
            + rig.log[release_start..]
                .iter()
                .position(|entry| *entry == (PID_TORQUE_FLUX_LIMITS.addr(), requested))
                .expect("the release writes the requested limit");
        let committed_at_write = rig.committed_at_write.borrow()[write];
        if requested < 1_000 {
            assert_eq!(committed_at_write, 1_000);
        } else {
            assert_eq!(committed_at_write, requested);
        }
        assert_eq!(rig.committed(), requested);
        assert!(!rig.tmc_state.has_pending_run_limit());
        assert!(!rig.tmc_state.offset_headroom_reserved(Relaxed));
        rig.assert_observed_invariant();
    });
}

#[test]
fn firmware_defers_a_lower_set_current_drained_while_held() {
    drained_set_current_while_held_converges(400);
}

#[test]
fn firmware_defers_a_higher_set_current_drained_while_held() {
    drained_set_current_while_held_converges(1_000);
}

#[test]
fn set_current_while_held_keeps_the_committed_limit_until_the_release_commits_it() {
    pollster::block_on(async {
        let mut rig = held_rig(1_000);

        assert_eq!(rig.drain_set_current(400).await, Ok(()));
        assert_eq!(rig.committed(), 1_000);
        for _ in 0..3 {
            let failing = RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr());
            let _ = rig.tick_on(failing, false, false).await;
            assert_eq!(rig.committed(), 1_000);
        }

        rig.tick().await.unwrap();

        assert_eq!(rig.committed(), rig.run_limit_for(400));
        rig.assert_observed_invariant();
    });
}

#[test]
fn release_with_an_unknown_offset_writes_only_the_offset_zero() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.store_current_limit(current_limit(1_000), Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime.release_offset_headroom(&mut tmc, &tmc_state).await,
            Ok(())
        );
        assert_eq!(
            runtime.release_offset_headroom(&mut tmc, &tmc_state).await,
            Ok(())
        );

        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_TORQUE_FLUX_OFFSET.addr(), 0)],
            "an unknown offset is cleared once and the limit is left to its owner"
        );
        assert!(!runtime.offset_headroom_engaged());
        assert!(!tmc_state.offset_headroom_reserved(Relaxed));
    });
}

#[test]
fn failed_unknown_offset_clear_refuses_the_exclusive_activity() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr()),
        );

        let first = runtime.release_offset_headroom(&mut tmc, &tmc_state).await;
        let second = runtime.release_offset_headroom(&mut tmc, &tmc_state).await;

        assert_eq!((first, second), (Err(()), Err(())));
        let spi = tmc.into_inner();
        assert!(!spi.attempted_write_to(PID_TORQUE_FLUX_LIMITS));
        assert_eq!(
            spi.write_attempts,
            std::vec![(PID_TORQUE_FLUX_OFFSET.addr(), 0); 2],
            "the unknown offset stays unknown and is retried"
        );
    });
}
