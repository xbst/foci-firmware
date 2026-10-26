//! Stepper command handlers.

use core::sync::atomic::Ordering::Relaxed;

use ankyra::prelude::klipper_command;

use crate::context::{BoardChannels, ShutdownRecovery, StateAccess, StepperHost, TrsyncHost};
use crate::shutdown::ShutdownReason;
use crate::state::stepper::stepper_channel;
use crate::state::trsync::begin_homing_preparation;

/// Register a stepper object.
/// STEP/DIR pins must be FOCI virtual pins mapping to the same board-local TMC4671 channel.
#[klipper_command]
pub fn config_stepper(
    ctx: &mut dyn StepperConfigHost,
    oid: u8,
    step_pin: u8,
    dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    let step_channel = match ctx.step_pin_channel(step_pin) {
        Some(channel) => channel,
        None => {
            defmt::error!("config_stepper: unknown STEP pin {}", step_pin);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
            }
            return;
        }
    };
    let dir_channel = match ctx.dir_pin_channel(dir_pin) {
        Some(channel) => channel,
        None => {
            defmt::error!("config_stepper: unknown DIR pin {}", dir_pin);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
            }
            return;
        }
    };
    if step_channel != dir_channel {
        defmt::error!(
            "config_stepper: STEP/DIR channel mismatch step={} dir={}",
            step_channel,
            dir_channel
        );
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
        }
        return;
    }

    if let Err(e) =
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), oid, step_channel)
    {
        defmt::error!("config_stepper: {}", e);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigStepperFailed, clock);
        }
        return;
    }
    defmt::info!(
        "config_stepper: oid={} channel={} (TMC4671 mode)",
        oid,
        step_channel
    );
}

/// Enqueue a trapezoidal move segment for a stepper.
#[klipper_command]
pub fn queue_step(ctx: &mut dyn StepperQueue, oid: u8, interval: u32, count: u16, add: i16) {
    let Some(channel) = stepper_channel_or_shutdown(ctx, oid, || {
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociDiagnostic,
            fault: u8 = crate::diagnostic::FaultClass::UnknownOid as u8,
            operation: u8 = crate::diagnostic::operation::STEPPER_LOOKUP,
            oid: u8 = oid,
            channel: u8 = 255,
            data0: u32 = 0,
            data1: u32 = 0,
        );
    }) else {
        return;
    };
    if refuse_if_commissioning_owned(ctx, oid, channel) {
        return;
    }
    let move_ = foci_klipper::stepper::StepMove::new(interval, count, add);
    if let Err(error) = ctx.queue_step_move(channel, move_) {
        defmt::error!("queue_step: oid={} channel={} {}", oid, channel, error);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::QueueStepFailed, clock);
        }
    }
}

/// Set the direction for subsequent steps on a stepper.
#[klipper_command]
pub fn set_next_step_dir(ctx: &mut dyn StepperDirection, oid: u8, dir: u8) {
    let Some(channel) = stepper_channel_or_shutdown(ctx, oid, || {
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociDiagnostic,
            fault: u8 = crate::diagnostic::FaultClass::UnknownOid as u8,
            operation: u8 = crate::diagnostic::operation::STEPPER_LOOKUP,
            oid: u8 = oid,
            channel: u8 = 255,
            data0: u32 = 0,
            data1: u32 = 0,
        );
    }) else {
        return;
    };
    if refuse_if_commissioning_owned(ctx, oid, channel) {
        return;
    }
    ctx.set_next_step_dir(channel, dir != 0);
}

/// Reset the step clock reference. Only valid while the stepper is inactive; shuts down otherwise.
#[klipper_command]
pub fn reset_step_clock(ctx: &mut dyn StepperReset, oid: u8, clock: u32) {
    let Some(channel) = stepper_channel_or_shutdown(ctx, oid, || {
        ::ankyra::klipper_reply!(
            crate::ankyra::replies::FociDiagnostic,
            fault: u8 = crate::diagnostic::FaultClass::UnknownOid as u8,
            operation: u8 = crate::diagnostic::operation::STEPPER_LOOKUP,
            oid: u8 = oid,
            channel: u8 = 255,
            data0: u32 = 0,
            data1: u32 = 0,
        );
    }) else {
        return;
    };
    if refuse_if_commissioning_owned(ctx, oid, channel) {
        return;
    }
    if let Err(e) = ctx.reset_step_clock(channel, clock) {
        defmt::error!("reset_step_clock: oid={} channel={} {}", oid, channel, e);
        if ctx.enter_shutdown() {
            let shutdown_clock = ctx.now();
            ctx.send_shutdown(
                ShutdownReason::CannotResetTimeWhenStepperActive,
                shutdown_clock,
            );
        }
    }
}

fn stepper_channel_or_shutdown(
    ctx: &mut dyn ShutdownRecovery,
    oid: u8,
    emit_diagnostic: impl FnOnce(),
) -> Option<u8> {
    match ctx.state().stepper_channel(oid) {
        Some(channel) => Some(channel),
        None => {
            defmt::error!("stepper command: unknown stepper oid={}", oid);
            emit_diagnostic();
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::UnknownStepperOid, clock);
            }
            None
        }
    }
}

fn refuse_if_commissioning_owned(ctx: &mut dyn ShutdownRecovery, oid: u8, channel: u8) -> bool {
    let owned = crate::state::tmc::tmc_channel(channel)
        .is_some_and(|tmc| tmc.commission_in_flight.load(Relaxed));
    if owned {
        defmt::error!(
            "stepper command on commissioning-owned channel: oid={} channel={}",
            oid,
            channel
        );
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::StepperOwnedByCommissioning, clock);
        }
    }
    owned
}

/// Report the current position of a stepper.
#[klipper_command]
pub fn stepper_get_position(ctx: &mut dyn StateAccess, oid: u8) {
    let pos = stepper_position(ctx, oid);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::StepperPosition,
        oid: u8 = oid,
        pos: i32 = pos
    );
}

/// Report the commanded position; equals `stepper_get_position` for hardware step/dir.
#[klipper_command]
pub fn stepper_get_commanded_position(ctx: &mut dyn StateAccess, oid: u8) {
    let pos = stepper_position(ctx, oid);
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::StepperPosition,
        oid: u8 = oid,
        pos: i32 = pos
    );
}

fn stepper_position(ctx: &mut dyn StateAccess, oid: u8) -> i32 {
    let channel = match ctx.state().stepper_channel(oid) {
        Some(channel) => channel,
        None => {
            defmt::error!("stepper_get_position: unknown stepper oid={}", oid);
            return 0;
        }
    };
    let state = match stepper_channel(channel) {
        Some(state) => state,
        None => {
            defmt::error!("stepper_get_position: invalid channel={}", channel);
            return 0;
        }
    };
    state.position.load(core::sync::atomic::Ordering::Relaxed)
}

/// Link a stepper to a TrSync so it stops when the TrSync triggers.
#[klipper_command]
pub fn stepper_stop_on_trigger(ctx: &mut dyn StepperStopOnTrigger, oid: u8, trsync_oid: u8) {
    let stepper_idx = match ctx.state().steppers_by_oid.get(&oid).copied() {
        Some(idx) => idx,
        None => {
            defmt::error!("stepper_stop_on_trigger: unknown stepper oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::StepperStopOnTriggerUnknownStepper, clock);
            }
            return;
        }
    };

    match ctx.link_stepper_stop(stepper_idx, trsync_oid) {
        Err(()) => {
            defmt::error!(
                "stepper_stop_on_trigger: oid={} trsync_oid={} failed",
                oid,
                trsync_oid,
            );
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::StepperStopOnTriggerFailed, clock);
            }
        }
        Ok(start_clock) => {
            if begin_homing_preparation(start_clock) && ctx.notify_homing_start().is_err() {
                defmt::error!("stepper_stop_on_trigger: homing-start notification failed");
                if ctx.enter_shutdown() {
                    let clock = ctx.now();
                    ctx.send_shutdown(ShutdownReason::StepperStopOnTriggerFailed, clock);
                }
            }
        }
    }
}

/// Composite bound for `queue_step`: shutdown recovery plus the stepper host.
pub trait StepperQueue: ShutdownRecovery + StepperHost {}
impl<T: ?Sized + ShutdownRecovery + StepperHost> StepperQueue for T {}

/// Composite bound for `set_next_step_dir`: needs shutdown recovery (to
/// resolve unknown stepper OIDs through the common shutdown path) and the
/// stepper host (to delegate direction capture).
pub trait StepperDirection: ShutdownRecovery + StepperHost {}
impl<T: ?Sized + ShutdownRecovery + StepperHost> StepperDirection for T {}

/// Composite bound for `config_stepper`: needs shutdown recovery and the
/// board's virtual STEP/DIR pin-to-channel mapping.
pub trait StepperConfigHost: ShutdownRecovery + BoardChannels {}
impl<T: ?Sized + ShutdownRecovery + BoardChannels> StepperConfigHost for T {}

/// Composite bound for `reset_step_clock`: needs shutdown recovery (to
/// enter shutdown on error) and the stepper host (to delegate the
/// clock-reset operation).
pub trait StepperReset: ShutdownRecovery + StepperHost {}
impl<T: ?Sized + ShutdownRecovery + StepperHost> StepperReset for T {}

/// Composite bound for `stepper_stop_on_trigger`: needs shutdown recovery
/// (to enter shutdown on error, implying `StateAccess` and `ClockSource`
/// via its supertraits) and the trsync host (to call `link_stepper_stop`
pub trait StepperStopOnTrigger: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> StepperStopOnTrigger for T {}

#[cfg(test)]
mod tests {
    use core::sync::atomic::Ordering::Relaxed;

    use super::*;
    use crate::context::{BoardChannels, ClockSource, StateAccess};
    use crate::shutdown::ShutdownReason;
    use crate::state::trsync::{HOMING_START_CLOCK, HOMING_STATE};
    use foci_firmware_macros::fixture_test;
    use foci_klipper::state::State;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum StepperCall {
        SetDir { channel: u8, forward: bool },
        Queue { channel: u8, count: u16 },
        Reset { channel: u8, clock: u32 },
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct DiagnosticReply {
        fault: u8,
        operation: u8,
        oid: u8,
        channel: u8,
        data0: u32,
        data1: u32,
    }

    struct MockSender {
        diagnostics: std::vec::Vec<DiagnosticReply>,
    }

    impl MockSender {
        fn new() -> Self {
            Self {
                diagnostics: std::vec::Vec::new(),
            }
        }
    }

    impl ::ankyra::SendReply<crate::ankyra::replies::FociDiagnostic> for MockSender {
        fn send(&mut self, payload: crate::ankyra::replies::FociDiagnostic) {
            self.diagnostics.push(DiagnosticReply {
                fault: payload.fault,
                operation: payload.operation,
                oid: payload.oid,
                channel: payload.channel,
                data0: payload.data0,
                data1: payload.data1,
            });
        }
    }

    struct MockCtx {
        state: State,
        last_shutdown: Option<ShutdownReason>,
        stepper_calls: std::cell::RefCell<std::vec::Vec<StepperCall>>,
        queue_result: Result<(), crate::context::StepperEnqueueError>,
        link_result: Result<u32, ()>,
        link_calls: usize,
        notification_result: Result<(), ()>,
        notification_calls: usize,
        homing_state_at_notification: Option<u8>,
        trigger_result: Result<u8, ()>,
    }

    impl MockCtx {
        fn new() -> Self {
            Self {
                state: State::new(),
                last_shutdown: None,
                stepper_calls: std::cell::RefCell::new(std::vec::Vec::new()),
                queue_result: Ok(()),
                link_result: Ok(0),
                link_calls: 0,
                notification_result: Ok(()),
                notification_calls: 0,
                homing_state_at_notification: None,
                trigger_result: Ok(1),
            }
        }
    }

    impl StateAccess for MockCtx {
        fn state(&mut self) -> &mut State {
            &mut self.state
        }
    }

    impl ClockSource for MockCtx {
        fn now(&self) -> u32 {
            123
        }
    }

    impl ShutdownRecovery for MockCtx {
        fn enter_shutdown(&mut self) -> bool {
            true
        }

        fn reset_to_bootloader(&mut self) -> ! {
            panic!("reset_to_bootloader not used in tests")
        }

        fn send_shutdown(&mut self, reason: ShutdownReason, _clock: u32) {
            self.last_shutdown = Some(reason);
        }
    }

    impl TrsyncHost for MockCtx {
        fn reset_all(&mut self) {
            panic!("reset_all not used in this test")
        }

        fn register_trsync(&mut self, _oid: u8) -> Result<(), ()> {
            panic!("register_trsync not used in this test")
        }

        fn trsync_arm(
            &mut self,
            _oid: u8,
            _report_clock: u32,
            _report_ticks: u32,
            _expire_reason: u8,
        ) -> Result<(), ()> {
            panic!("trsync_arm not used in this test")
        }

        fn trsync_set_timeout(&mut self, _oid: u8, _clock: u32) -> Result<(), ()> {
            panic!("trsync_set_timeout not used in this test")
        }

        fn trsync_trigger(&mut self, _oid: u8, _reason: u8) -> Result<u8, ()> {
            self.trigger_result
        }

        fn register_endstop(&mut self, _oid: u8) -> Result<usize, ()> {
            panic!("register_endstop not used in this test")
        }

        fn endstop_start_homing(
            &mut self,
            _oid: u8,
            _request: crate::context::trsync::HomingRequest,
        ) -> Result<(), ()> {
            panic!("endstop_start_homing not used in this test")
        }

        fn endstop_query(&mut self, _oid: u8) -> Result<(bool, u32, u8), ()> {
            panic!("endstop_query not used in this test")
        }

        fn link_stepper_stop(&mut self, _stepper_idx: usize, _trsync_oid: u8) -> Result<u32, ()> {
            self.link_calls += 1;
            self.link_result
        }

        fn notify_homing_start(&mut self) -> Result<(), ()> {
            self.notification_calls += 1;
            self.homing_state_at_notification = Some(HOMING_STATE.load(Relaxed));
            self.notification_result
        }
    }

    impl BoardChannels for MockCtx {
        fn tmc_channel_count(&self) -> u8 {
            2
        }

        fn step_pin_channel(&self, pin: u8) -> Option<u8> {
            match pin {
                1 => Some(0),
                3 => Some(1),
                _ => None,
            }
        }

        fn dir_pin_channel(&self, pin: u8) -> Option<u8> {
            match pin {
                2 => Some(0),
                4 => Some(1),
                _ => None,
            }
        }

        fn enable_pin_channel(&self, _pin: u32) -> Option<u8> {
            None
        }
    }

    impl StepperHost for MockCtx {
        fn set_next_step_dir(&mut self, channel: u8, forward: bool) {
            self.stepper_calls
                .borrow_mut()
                .push(StepperCall::SetDir { channel, forward });
        }

        fn queue_step_move(
            &mut self,
            channel: u8,
            move_: foci_klipper::stepper::StepMove,
        ) -> Result<(), crate::context::StepperEnqueueError> {
            self.stepper_calls.borrow_mut().push(StepperCall::Queue {
                channel,
                count: move_.count,
            });
            self.queue_result.clone()
        }

        fn reset_step_clock(
            &mut self,
            channel: u8,
            clock: u32,
        ) -> Result<(), foci_klipper::commands::CommandError> {
            self.stepper_calls
                .borrow_mut()
                .push(StepperCall::Reset { channel, clock });
            Ok(())
        }
    }

    #[test]
    fn config_stepper_registers_matching_step_dir_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();

        config_stepper(&mut ctx, &mut sender, 10, 1, 2, 0, 0);

        assert_eq!(ctx.state.stepper_channel(10), Some(0));
        assert_eq!(ctx.last_shutdown, None);
        assert_eq!(sender.diagnostics.as_slice(), &[]);
    }

    #[test]
    fn config_stepper_rejects_step_dir_channel_mismatch() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();

        config_stepper(&mut ctx, &mut sender, 10, 1, 4, 0, 0);

        assert_eq!(ctx.state.stepper_channel(10), None);
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::ConfigStepperFailed));
    }

    #[test]
    fn queue_step_routes_by_stepper_oid_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        queue_step(&mut ctx, &mut sender, 10, 100, 7, 0);

        assert_eq!(
            ctx.stepper_calls.borrow().as_slice(),
            &[StepperCall::Queue {
                channel: 1,
                count: 7
            }]
        );
        assert_eq!(ctx.last_shutdown, None);
        assert_eq!(sender.diagnostics.as_slice(), &[]);
    }

    fn commissioning_owned_channel0() -> &'static crate::state::tmc::TmcChannelState {
        let tmc = crate::state::tmc::tmc_channel0();
        tmc.commission_in_flight.store(true, Relaxed);
        tmc
    }

    #[fixture_test]
    fn queue_step_on_a_commissioning_owned_channel_shuts_down_without_queuing() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 0).unwrap();
        let tmc = commissioning_owned_channel0();

        queue_step(&mut ctx, &mut sender, 10, 100, 7, 0);

        assert!(ctx.stepper_calls.borrow().is_empty());
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::StepperOwnedByCommissioning)
        );
        tmc.commission_in_flight.store(false, Relaxed);
    }

    #[fixture_test]
    fn set_next_step_dir_on_a_commissioning_owned_channel_shuts_down() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 0).unwrap();
        let tmc = commissioning_owned_channel0();

        set_next_step_dir(&mut ctx, &mut sender, 10, 1);

        assert!(ctx.stepper_calls.borrow().is_empty());
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::StepperOwnedByCommissioning)
        );
        tmc.commission_in_flight.store(false, Relaxed);
    }

    #[fixture_test]
    fn reset_step_clock_on_a_commissioning_owned_channel_shuts_down() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 0).unwrap();
        let tmc = commissioning_owned_channel0();

        reset_step_clock(&mut ctx, &mut sender, 10, 1_234);

        assert!(ctx.stepper_calls.borrow().is_empty());
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::StepperOwnedByCommissioning)
        );
        tmc.commission_in_flight.store(false, Relaxed);
    }

    #[fixture_test]
    fn host_step_commands_route_normally_once_commissioning_releases_the_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 0).unwrap();
        let tmc = commissioning_owned_channel0();
        tmc.commission_in_flight.store(false, Relaxed);

        queue_step(&mut ctx, &mut sender, 10, 100, 7, 0);

        assert_eq!(
            ctx.stepper_calls.borrow().as_slice(),
            &[StepperCall::Queue {
                channel: 0,
                count: 7
            }]
        );
        assert_eq!(ctx.last_shutdown, None);
    }

    #[test]
    fn set_next_step_dir_routes_by_stepper_oid_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        set_next_step_dir(&mut ctx, &mut sender, 10, 0);

        assert_eq!(
            ctx.stepper_calls.borrow().as_slice(),
            &[StepperCall::SetDir {
                channel: 1,
                forward: false
            }]
        );
        assert_eq!(sender.diagnostics.as_slice(), &[]);
    }

    #[test]
    fn reset_step_clock_routes_by_stepper_oid_channel() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        reset_step_clock(&mut ctx, &mut sender, 10, 1234);

        assert_eq!(
            ctx.stepper_calls.borrow().as_slice(),
            &[StepperCall::Reset {
                channel: 1,
                clock: 1234
            }]
        );
        assert_eq!(sender.diagnostics.as_slice(), &[]);
    }

    #[test]
    fn queue_step_unknown_stepper_oid_uses_stepper_shutdown_reason() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();

        queue_step(&mut ctx, &mut sender, 99, 100, 7, 0);

        assert_eq!(ctx.stepper_calls.borrow().as_slice(), &[]);
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::UnknownStepperOid));
        assert_eq!(
            sender.diagnostics.as_slice(),
            &[DiagnosticReply {
                fault: crate::diagnostic::FaultClass::UnknownOid as u8,
                operation: crate::diagnostic::operation::STEPPER_LOOKUP,
                oid: 99,
                channel: 255,
                data0: 0,
                data1: 0,
            }]
        );
    }

    #[test]
    fn set_next_step_dir_unknown_stepper_oid_uses_stepper_shutdown_reason() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();

        set_next_step_dir(&mut ctx, &mut sender, 99, 0);

        assert_eq!(ctx.stepper_calls.borrow().as_slice(), &[]);
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::UnknownStepperOid));
        assert_eq!(
            sender.diagnostics.as_slice(),
            &[DiagnosticReply {
                fault: crate::diagnostic::FaultClass::UnknownOid as u8,
                operation: crate::diagnostic::operation::STEPPER_LOOKUP,
                oid: 99,
                channel: 255,
                data0: 0,
                data1: 0,
            }]
        );
    }

    #[test]
    fn reset_step_clock_unknown_stepper_oid_uses_stepper_shutdown_reason() {
        let mut ctx = MockCtx::new();
        let mut sender = MockSender::new();

        reset_step_clock(&mut ctx, &mut sender, 99, 1234);

        assert_eq!(ctx.stepper_calls.borrow().as_slice(), &[]);
        assert_eq!(ctx.last_shutdown, Some(ShutdownReason::UnknownStepperOid));
        assert_eq!(
            sender.diagnostics.as_slice(),
            &[DiagnosticReply {
                fault: crate::diagnostic::FaultClass::UnknownOid as u8,
                operation: crate::diagnostic::operation::STEPPER_LOOKUP,
                oid: 99,
                channel: 255,
                data0: 0,
                data1: 0,
            }]
        );
    }

    #[test]
    fn stepper_stop_on_trigger_transitions_before_successful_notification() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_STATE.store(0, Relaxed);
        let mut ctx = MockCtx::new();
        let mut sender = ();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        stepper_stop_on_trigger(&mut ctx, &mut sender, 10, 7);

        assert_eq!(HOMING_STATE.load(Relaxed), 1);
        assert_eq!(ctx.homing_state_at_notification, Some(1));
        assert_eq!(ctx.notification_calls, 1);
        assert_eq!(ctx.last_shutdown, None);
        HOMING_STATE.store(0, Relaxed);
    }

    struct TrsyncSender {
        replies: std::vec::Vec<(u8, u8, u8)>,
    }

    impl ::ankyra::SendReply<crate::ankyra::replies::TrsyncState> for TrsyncSender {
        fn send(&mut self, payload: crate::ankyra::replies::TrsyncState) {
            self.replies
                .push((payload.oid, payload.can_trigger, payload.trigger_reason));
        }
    }

    #[test]
    fn trsync_trigger_ends_a_move_whose_preparation_has_not_finished() {
        use crate::ankyra::handlers::trsync::trsync_trigger;
        use crate::state::trsync::HOMING_TRIGGERED;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        // An endstop that is already pressed fires within milliseconds of the
        // start clock, often before the maintenance tick has advanced state 1 to
        // 2; the host then ends the move while preparation is still pending.
        for (initial, expected) in [(1u8, 3u8), (2, 3), (3, 3), (0, 0)] {
            HOMING_STATE.store(initial, Relaxed);
            HOMING_TRIGGERED.store(5, Relaxed);
            let mut ctx = MockCtx::new();
            ctx.trigger_result = Ok(2);
            let mut sender = TrsyncSender {
                replies: std::vec::Vec::new(),
            };

            trsync_trigger(&mut ctx, &mut sender, 5, 2);

            assert_eq!(
                HOMING_STATE.load(Relaxed),
                expected,
                "homing state {initial} after the host ended the move"
            );
            assert_eq!(HOMING_TRIGGERED.load(Relaxed), 0xFF);
            assert_eq!(sender.replies, std::vec![(5, 0, 2)]);
            assert_eq!(ctx.last_shutdown, None);
        }
        HOMING_STATE.store(0, Relaxed);
    }

    #[test]
    fn stepper_stop_on_trigger_records_the_homing_start_clock() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_STATE.store(0, Relaxed);
        HOMING_START_CLOCK.store(0, Relaxed);
        let mut ctx = MockCtx::new();
        let mut sender = ();
        ctx.link_result = Ok(0x1234_5678);
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        stepper_stop_on_trigger(&mut ctx, &mut sender, 10, 7);

        assert_eq!(HOMING_STATE.load(Relaxed), 1);
        assert_eq!(HOMING_START_CLOCK.load(Relaxed), 0x1234_5678);
        HOMING_STATE.store(0, Relaxed);
    }

    #[test]
    fn stepper_stop_on_trigger_notification_unavailable_enters_shutdown() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_STATE.store(0, Relaxed);
        let mut ctx = MockCtx::new();
        let mut sender = ();
        ctx.notification_result = Err(());
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        stepper_stop_on_trigger(&mut ctx, &mut sender, 10, 7);

        assert_eq!(HOMING_STATE.load(Relaxed), 1);
        assert_eq!(ctx.notification_calls, 1);
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::StepperStopOnTriggerFailed)
        );
        HOMING_STATE.store(0, Relaxed);
    }

    #[test]
    fn stepper_stop_on_trigger_failed_link_does_not_transition_or_notify() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_STATE.store(0, Relaxed);
        let mut ctx = MockCtx::new();
        let mut sender = ();
        ctx.link_result = Err(());
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        stepper_stop_on_trigger(&mut ctx, &mut sender, 10, 7);

        assert_eq!(HOMING_STATE.load(Relaxed), 0);
        assert_eq!(ctx.notification_calls, 0);
        assert_eq!(
            ctx.last_shutdown,
            Some(ShutdownReason::StepperStopOnTriggerFailed)
        );
        HOMING_STATE.store(0, Relaxed);
    }

    #[test]
    fn stepper_stop_on_trigger_already_active_coalesces_notification() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_STATE.store(0, Relaxed);
        let mut ctx = MockCtx::new();
        let mut sender = ();
        foci_klipper::commands::stepper::config_stepper_channel(ctx.state(), 10, 1).unwrap();

        stepper_stop_on_trigger(&mut ctx, &mut sender, 10, 7);
        stepper_stop_on_trigger(&mut ctx, &mut sender, 10, 7);

        assert_eq!(HOMING_STATE.load(Relaxed), 1);
        assert_eq!(ctx.link_calls, 2);
        assert_eq!(ctx.notification_calls, 1);
        assert_eq!(ctx.last_shutdown, None);
        HOMING_STATE.store(0, Relaxed);
    }
}
