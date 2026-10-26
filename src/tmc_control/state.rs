//! Persistent state for the shared TMC control runtime.

use core::sync::atomic::Ordering::Relaxed;

use crate::autotune::CommissionError;
use crate::commission_dispatch::{DispatchState, TerminalRestoreMachine};
use crate::commissioning::CommissioningEngine;
use crate::commissioning::schedule::{CommissionPoll, CommissionTerminal, adapt_step_result};
use crate::commissioning::types::{CommissioningBackend, EngineKind};
use crate::pid::InnerLoopGains;
use crate::state::stepper::StepperChannelState;
use crate::state::tmc::TmcChannelState;
use crate::tmc_control::actions::{
    BoardAction, BoardActions, ScheduledCommissionTickAction, StepperAction, push_action,
};
use crate::tmc_control::config::{TmcControlConfig, TmcControlHomingRefs, TmcControlStateRefs};
use crate::tmc_control::signals::TmcControlSignals;
use foci_tmc4671::pid::{PidGainPair, PidIGain};

pub(crate) const HOMING_CLAMP_MAX_RETRIES: u8 = 3;

pub(crate) const HOMING_RESTORE_MAX_RETRIES: u8 = 2;

/// Trace-only cached state. Core safety state stays in `TmcControlRuntime`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcControlTraceState {
    /// Trace control tick counter.
    pub control_tick: u32,
    /// Last command rate in TMC position counts per PWM cycle.
    pub cached_velocity_reference: i32,
    /// Last velocity feedforward offset emitted to trace sampling.
    pub cached_velocity_offset: i32,
    /// Velocity feedforward offset was clamped on the last tick.
    pub cached_velocity_clamped: bool,
    /// Last acceleration torque offset emitted to trace sampling.
    pub cached_accel_torque_offset: i32,
    /// Last acceleration gain emitted to trace sampling.
    pub cached_accel_gain_permille: u16,
    /// Last acceleration trace flags.
    pub cached_accel_flags: u8,
}

impl TmcControlTraceState {
    /// Create an empty trace state.
    pub const fn new() -> Self {
        Self {
            control_tick: 0,
            cached_velocity_reference: 0,
            cached_velocity_offset: 0,
            cached_velocity_clamped: false,
            cached_accel_torque_offset: 0,
            cached_accel_gain_permille: 0,
            cached_accel_flags: 0,
        }
    }
}

impl Default for TmcControlTraceState {
    fn default() -> Self {
        Self::new()
    }
}

/// Trace timing decisions for one USB/FTRC slow-control emission pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcControlTraceTick {
    /// Tick value to pass to `sample_tick`.
    pub sample_tick: u32,
    /// Incremented trace control tick for status frames and metadata cadence.
    pub control_tick: u32,
    /// Whether detailed status frames are due.
    pub status_detail_due: bool,
    /// Whether trace metadata is due.
    pub metadata_due: bool,
}

/// Result of applying one channel's homing maintenance for a frozen state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum HomingMaintenanceResult {
    /// This channel completed or did not need work for the frozen state.
    ReadyToAdvance,
    /// This channel could not complete work; keep the global homing state.
    RetryCurrentState,
    /// Homing cannot proceed safely; the board must disable the motor.
    FailClosed,
    /// The clamp window has not opened yet; keep the global homing state and
    /// retry on the next maintenance group without consuming the retry budget.
    Deferred,
}

/// Board clock context for the homing preparation window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct HomingClockWindow {
    /// Current Klipper clock.
    pub now: u32,
    /// Clock at which the homing move's steps begin (`HOMING_START_CLOCK`).
    pub start: u32,
}

/// Lead in Klipper clock ticks (4 ms) by which homing preparation precedes the
/// homing move's start clock.
pub const fn homing_clamp_lead_ticks(clock_freq: u32) -> u32 {
    clock_freq / 250
}

/// Whether the homing preparation window has opened for `window`.
///
/// Uses wrapping clock arithmetic.
pub fn homing_clamp_due(window: HomingClockWindow, clock_freq: u32) -> bool {
    let opens = window
        .start
        .wrapping_sub(homing_clamp_lead_ticks(clock_freq));
    (window.now.wrapping_sub(opens) as i32) >= 0
}

/// Where the torque/flux limit stands relative to the acceleration-offset headroom reservation.
///
/// The torque offset written by the feedforward tick is non-zero only in [`Self::Held`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum OffsetHeadroomPhase {
    /// The reservation flag is clear and the limit runs at the committed value.
    Released,
    /// The reservation flag is set; the reduced limit write has not yet succeeded.
    Engaging,
    /// The reduced limit is applied; the offset may use the reserved headroom.
    Held,
    /// Returning to the committed limit: offset zero, then the limit write, then flag clear, each
    /// retried until it succeeds. A pending lower limit is committed after its write succeeds and a
    /// pending higher limit before it is written.
    Releasing,
}

/// Long-lived board-agnostic TMC control state.
pub struct TmcControlRuntime {
    channel: u8,
    motor_enabled: bool,
    calibrated: bool,
    stored_gains: Option<InnerLoopGains>,
    commission_engine: Option<CommissioningEngine>,
    restore_machine: Option<TerminalRestoreMachine>,
    restore_cancel_requested: bool,
    planner_velocity_scale: Option<crate::motion_scale::PlannerVelocityScale>,
    planner_rpm_scale: Option<crate::motion_scale::PlannerRpmScale>,
    feedforward_was_active: bool,
    torque_flux_offset_was_active: bool,
    offset_headroom: OffsetHeadroomPhase,
    applied_velocity_offset: Option<u32>,
    applied_torque_flux_offset: Option<u32>,
    pid_error_clear_countdown_ms: u8,
    stall_detector: Option<crate::tmc_control::stall_detector::StallDetector>,
    homing_state_1_retries: u8,
    homing_state_3_retries: u8,
    #[cfg(feature = "trace")]
    trace: TmcControlTraceState,
}

impl TmcControlRuntime {
    /// Create a runtime for one board-local TMC channel.
    pub const fn new_for_channel(channel: u8) -> Self {
        Self {
            channel,
            motor_enabled: false,
            calibrated: false,
            stored_gains: None,
            commission_engine: None,
            restore_machine: None,
            restore_cancel_requested: false,
            planner_velocity_scale: None,
            planner_rpm_scale: None,
            feedforward_was_active: false,
            torque_flux_offset_was_active: false,
            offset_headroom: OffsetHeadroomPhase::Released,
            applied_velocity_offset: None,
            applied_torque_flux_offset: None,
            pid_error_clear_countdown_ms: 0,
            stall_detector: None,
            homing_state_1_retries: 0,
            homing_state_3_retries: 0,
            #[cfg(feature = "trace")]
            trace: TmcControlTraceState::new(),
        }
    }

    /// Return this runtime's board-local TMC channel.
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    pub(super) const fn maintenance_obligations_clear(&self) -> bool {
        !self.motor_enabled
            && self.commission_engine.is_none()
            && self.restore_machine.is_none()
            && !self.feedforward_was_active
            && !self.torque_flux_offset_was_active
            && matches!(self.offset_headroom, OffsetHeadroomPhase::Released)
            && self.stall_detector.is_none()
            && self.homing_state_1_retries == 0
            && self.homing_state_3_retries == 0
    }

    /// Return whether the motor is currently electrically enabled.
    pub const fn is_motor_enabled(&self) -> bool {
        self.motor_enabled
    }

    /// Return whether calibration has completed.
    pub const fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Return stored inner-loop gains.
    pub const fn stored_gains(&self) -> Option<InnerLoopGains> {
        self.stored_gains
    }

    /// Store inner-loop gains for future calibration/commissioning.
    #[cfg(test)]
    pub(crate) fn set_stored_gains(&mut self, gains: InnerLoopGains) {
        self.stored_gains = Some(gains);
    }

    /// Mutably borrow the stored inner-loop gains slot.
    pub fn stored_gains_mut(&mut self) -> &mut Option<InnerLoopGains> {
        &mut self.stored_gains
    }

    /// Return the active commissioning kind when an engine exists.
    pub fn pending_commission_kind(&self) -> Option<crate::commissioning::types::EngineKind> {
        self.commission_engine
            .as_ref()
            .map(CommissioningEngine::kind)
    }

    /// Return trace-only cached state when trace support is enabled.
    #[cfg(feature = "trace")]
    pub const fn trace_state(&self) -> &TmcControlTraceState {
        &self.trace
    }

    /// Mutably borrow trace-only cached state when trace support is enabled.
    #[cfg(feature = "trace")]
    pub fn trace_state_mut(&mut self) -> &mut TmcControlTraceState {
        &mut self.trace
    }

    /// Build a borrowed dispatch view over runtime-owned state.
    ///
    /// `tmc` and `stepper` must be `'static`.
    pub fn dispatch_state<'a>(
        &'a mut self,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> DispatchState<'a> {
        DispatchState::new(
            self.channel,
            tmc,
            stepper,
            &mut self.calibrated,
            &mut self.motor_enabled,
            &mut self.stored_gains,
        )
    }

    /// Reset runtime-local state after a config reset or shutdown clear.
    ///
    /// The offset-headroom reservation flag on the channel's `TmcChannelState` is cleared together
    /// with the runtime phase so the two cannot diverge. All other channel atomics
    /// (`TmcChannelState`, `StepperChannelState`) are still reset by `reset_channel_state`.
    pub fn reset_runtime_state(&mut self) {
        self.motor_enabled = false;
        self.calibrated = false;
        self.stored_gains = None;
        self.clear_or_cancel_commission();
        self.planner_velocity_scale = None;
        self.planner_rpm_scale = None;
        self.feedforward_was_active = false;
        self.torque_flux_offset_was_active = false;
        self.offset_headroom = OffsetHeadroomPhase::Released;
        if let Some(tmc_state) = crate::state::tmc::tmc_channel(self.channel) {
            tmc_state.set_offset_headroom_reserved(false, Relaxed);
        }
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.pid_error_clear_countdown_ms = 0;
        self.clear_homing_runtime();
        #[cfg(feature = "trace")]
        {
            self.trace = TmcControlTraceState::new();
        }
    }

    /// Drop runtime-owned homing state without touching channel atomics.
    pub fn clear_homing_runtime(&mut self) {
        self.stall_detector = None;
        self.homing_state_1_retries = 0;
        self.homing_state_3_retries = 0;
    }

    /// Reset channel atomics to firmware defaults after a config reset.
    pub fn reset_channel_state(tmc: &TmcChannelState, stepper: &StepperChannelState) {
        tmc.reset();
        stepper.reset();
    }

    /// Consume a config-reset queue flush request and return board actions.
    ///
    /// Returns an empty list when no flush was requested; otherwise resets runtime-local and
    /// channel-local state and emits `StopNow`. The fault latch is retired only when neither
    /// `tmc.status_pin_asserted` nor a pending emergency disable reports a fault.
    pub fn poll_queue_flush(
        &mut self,
        signals: &TmcControlSignals,
        tmc: &TmcChannelState,
        stepper: &StepperChannelState,
    ) -> BoardActions {
        let mut actions = BoardActions::new();
        if !signals.queue_flush.swap(false, Relaxed) {
            return actions;
        }

        let contained = tmc.status_pin_asserted.load(Relaxed) || signals.disable_signal.signaled();

        self.reset_runtime_state();
        Self::reset_channel_state(tmc, stepper);
        if !contained {
            signals.fault_active.store(false, Relaxed);
        }
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        actions
    }

    /// Poll host request to clear PID error sums after hold settles.
    pub fn poll_pid_error_clear_request(&mut self, tmc: &TmcChannelState) {
        if tmc.pid_error_sums_clear.swap(false, Relaxed) {
            self.pid_error_clear_countdown_ms = crate::tmc_control::HOLD_SETTLE_DELAY_MS;
        }
    }

    /// Apply host-requested PID error-sum settle and clear maintenance.
    ///
    /// Consumes the request latch, waits for the hold timer to become inactive,
    /// then snaps the hold target and clears PID error accumulators when the
    /// runtime countdown expires. SPI failures are best-effort and logged.
    pub async fn apply_pid_error_settle_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
    ) where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        self.poll_pid_error_clear_request(refs.tmc);
        let timer_active = refs.stepper.timer_active_mirror.load(Relaxed);
        if self.motor_enabled
            && self.commission_engine.is_none()
            && !timer_active
            && self.pid_error_clear_countdown_ms > 0
        {
            self.pid_error_clear_countdown_ms -= 1;
            if self.pid_error_clear_countdown_ms == 0 {
                if crate::tmc_control::settle::settle_hold_and_clear_pid_errors(tmc_spi, refs.tmc)
                    .await
                    .is_err()
                {
                    defmt::warn!("TMC4671 PID settle/clear failed");
                } else {
                    defmt::info!("TMC4671 PID settled and cleared after motion");
                }
            }
        }
    }

    /// Prepare immediate shutdown/fault quiesce for one channel.
    ///
    /// Clears runtime-owned energized and transient maintenance state without
    /// resetting configuration such as stored gains or velocity scale. The
    /// board shell must apply the returned stepper action and lower enable
    /// pins before any awaited SPI safe-state cleanup.
    pub fn prepare_shutdown_quiesce(&mut self) -> BoardActions {
        let mut actions = BoardActions::new();
        self.clear_or_cancel_commission();
        self.motor_enabled = false;
        self.calibrated = false;
        self.feedforward_was_active = false;
        self.torque_flux_offset_was_active = false;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.pid_error_clear_countdown_ms = 0;
        self.clear_homing_runtime();
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        actions
    }

    /// Returns true when a commissioning engine is currently active.
    pub fn is_commission_active(&self) -> bool {
        self.commission_engine.is_some()
    }

    pub(crate) fn planner_paths_allowed(&self, refs: TmcControlStateRefs<'_>) -> bool {
        self.commission_engine.is_none() || refs.tmc.commission_stimulus_active.load(Relaxed)
    }

    pub(crate) fn feedforward_restoration_pending(&self) -> bool {
        self.feedforward_was_active
            || self.torque_flux_offset_was_active
            || self.offset_headroom_engaged()
    }

    /// Return whether the offset-headroom reservation is anywhere but released.
    pub const fn offset_headroom_engaged(&self) -> bool {
        !matches!(self.offset_headroom, OffsetHeadroomPhase::Released)
    }

    /// Force the offset-headroom reservation onto its release path.
    ///
    /// Sets the reservation flag so every operational-limit writer keeps the reduced limit, then
    /// lets the next slow feedforward tick write the offset to zero, commit any pending run
    /// limit, write the committed limit, and clear the flag, retrying each step until it
    /// succeeds.
    pub fn request_offset_headroom_release(&mut self, tmc_state: &TmcChannelState) {
        tmc_state.set_offset_headroom_reserved(true, Relaxed);
        self.offset_headroom = OffsetHeadroomPhase::Releasing;
    }

    fn offset_headroom_eligible(
        &self,
        refs: TmcControlStateRefs<'_>,
        homing_active: bool,
        reset: bool,
    ) -> bool {
        self.motor_enabled
            && self.planner_velocity_scale.is_some()
            && refs.tmc.accel_feedforward.load(Relaxed)
            && (refs.tmc.accel_feedforward_accel_gain_permille.load(Relaxed) != 0
                || refs.tmc.accel_feedforward_decel_gain_permille.load(Relaxed) != 0)
            && self.commission_engine.is_none()
            && self.restore_machine.is_none()
            && !homing_active
            && !refs.tmc.homing_clamp_active.load(Relaxed)
            && !reset
            && !refs.tmc.has_pending_run_limit()
    }

    async fn advance_offset_headroom<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        eligible: bool,
    ) -> Result<(), ()>
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        use OffsetHeadroomPhase::{Engaging, Held, Released, Releasing};
        match (self.offset_headroom, eligible) {
            (Released, false) | (Held, true) => Ok(()),
            (Released | Engaging, true) => {
                refs.tmc.set_offset_headroom_reserved(true, Relaxed);
                self.offset_headroom = Engaging;
                let reserved = crate::tmc_control::current_limit::reserved_limit(
                    refs.tmc.load_current_limit(Relaxed),
                );
                if tmc_spi
                    .write_register(
                        foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                        reserved.register_value(),
                    )
                    .await
                    .is_ok()
                {
                    self.offset_headroom = Held;
                }
                Ok(())
            }
            (Engaging | Held, false) | (Releasing, _) => {
                self.offset_headroom = Releasing;
                self.release_offset_headroom_step(tmc_spi, refs.tmc).await
            }
        }
    }

    /// Release the offset-headroom reservation before an exclusive activity writes the TMC.
    ///
    /// Runs the feedforward tick's release sequence once: offset zero, the committed (or pending)
    /// limit, then the flag clear. With the reservation released and the applied offset unknown,
    /// writes only offset zero and leaves the limit to its current owner. Does nothing when the
    /// reservation is released and the applied offset is known to be zero.
    ///
    /// # Errors
    ///
    /// Returns `Err(())` when a write failed or the reservation is still not released; the caller
    /// must not issue its own register writes and may retry later.
    pub(crate) async fn release_offset_headroom<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        tmc_state: &TmcChannelState,
    ) -> Result<(), ()>
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        if !self.offset_headroom_engaged() {
            match self.applied_torque_flux_offset {
                Some(0) => return Ok(()),
                None => {
                    tmc_spi
                        .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, 0)
                        .await
                        .map_err(|_| ())?;
                    self.applied_torque_flux_offset = Some(0);
                    return Ok(());
                }
                Some(_) => {}
            }
        }
        self.request_offset_headroom_release(tmc_state);
        self.release_offset_headroom_step(tmc_spi, tmc_state)
            .await?;
        if self.offset_headroom_engaged() {
            return Err(());
        }
        Ok(())
    }

    async fn release_offset_headroom_step<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        tmc_state: &TmcChannelState,
    ) -> Result<(), ()>
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        if self.applied_torque_flux_offset != Some(0) {
            tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, 0)
                .await
                .map_err(|_| ())?;
            self.applied_torque_flux_offset = Some(0);
            self.torque_flux_offset_was_active = false;
            self.clear_accel_trace();
        }
        let committed = tmc_state.load_current_limit(Relaxed);
        let pending = tmc_state.peek_pending_run_limit();
        let target = pending.unwrap_or(committed);
        if target.raw() > committed.raw() {
            tmc_state.store_current_limit(target, Relaxed);
        }
        tmc_spi
            .write_register(
                foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                target.register_value(),
            )
            .await
            .map_err(|_| ())?;
        tmc_state.store_current_limit(target, Relaxed);
        if let Some(limit) = pending {
            tmc_state.retire_pending_run_limit(limit);
        }
        if tmc_state.has_pending_run_limit() {
            return Ok(());
        }
        tmc_state.set_offset_headroom_reserved(false, Relaxed);
        self.offset_headroom = OffsetHeadroomPhase::Released;
        Ok(())
    }

    fn clear_accel_trace(&mut self) {
        #[cfg(feature = "trace")]
        {
            let trace = self.trace_state_mut();
            trace.cached_accel_torque_offset = 0;
            trace.cached_accel_gain_permille = 0;
            trace.cached_accel_flags = 0;
        }
    }

    pub(super) fn publish_feedforward_surfaces_idle(&self, refs: TmcControlStateRefs<'_>) {
        refs.tmc
            .feedforward_surfaces_idle
            .store(!self.feedforward_restoration_pending(), Relaxed);
    }

    /// Force the pending terminal restore onto its safe-disarm path with the
    /// safe-state-incomplete outcome.
    pub(crate) async fn force_restore_safe_state_incomplete<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> Option<crate::commission_dispatch::CommissionOutcome>
    where
        B: CommissioningBackend,
    {
        self.restore_machine.as_ref()?;
        self.request_restore_cancel();
        match self
            .drive_restore_cancel(backend, tmc, stepper, CommissionError::SafeStateIncomplete)
            .await
        {
            crate::commission_dispatch::RestoreStep::Done(outcome) => Some(outcome),
            crate::commission_dispatch::RestoreStep::Pending => {
                unreachable!("force_safe_terminal completes in one step")
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn restore_status_error_for_test(&self) -> Option<CommissionError> {
        self.restore_machine
            .as_ref()
            .map(|machine| machine.status_error())
    }

    /// Install a new commissioning engine, replacing any existing one.
    pub fn set_commission_engine(&mut self, engine: CommissioningEngine) {
        self.commission_engine = Some(engine);
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
    }

    /// Remove and return the active commissioning engine without completing a
    /// breakaway-campaign breadcrumb.
    pub fn take_commission_engine(&mut self) -> Option<CommissioningEngine> {
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.commission_engine.take()
    }

    /// Complete a retained active breakaway cancellation, then remove its engine.
    ///
    /// Callers must first restore their path-specific safe state.
    pub fn take_commission_engine_after_handled_stop(&mut self) -> Option<CommissioningEngine> {
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        CommissioningEngine::take_after_handled_stop(&mut self.commission_engine)
    }

    /// Clear the calibrated flag before launching a (re)calibration run.
    pub fn clear_calibrated(&mut self) {
        self.calibrated = false;
    }

    /// Advance the engine through explicit scheduling, or `None` when idle.
    /// Legacy results are adapted to absolute scheduling at this boundary.
    pub async fn step_commissioning_scheduled<B>(
        &mut self,
        backend: &mut B,
    ) -> Option<CommissionPoll>
    where
        B: CommissioningBackend,
    {
        let (oid, engine_kind) = self
            .commission_engine
            .as_ref()
            .map(|engine| (engine.oid(), engine.kind()))?;
        let result = self
            .commission_engine
            .as_mut()
            .expect("commission engine disappeared during scheduled step")
            .step_scheduled(backend)
            .await;
        let now_us = crate::commissioning::CommissionTiming::elapsed_us(backend);

        Some(
            adapt_step_result(result, now_us, Some((oid, engine_kind))).unwrap_or_else(|error| {
                CommissionPoll::Terminal {
                    result: if matches!(engine_kind, EngineKind::OuterTune) {
                        CommissionTerminal::OuterFailed(error)
                    } else {
                        CommissionTerminal::InnerFailed(error)
                    },
                }
            }),
        )
    }

    /// Convert a nonterminal explicit poll into the board-facing action.
    #[must_use]
    pub fn classify_commission_poll(
        &self,
        poll: CommissionPoll,
    ) -> Option<ScheduledCommissionTickAction> {
        self.commission_engine.as_ref()?;
        match poll {
            CommissionPoll::ReadyNow { event } => {
                Some(ScheduledCommissionTickAction::ReadyNow { event })
            }
            CommissionPoll::At {
                timestamp_us,
                event,
            } => Some(ScheduledCommissionTickAction::At {
                timestamp_us,
                event,
            }),
            CommissionPoll::Terminal { .. } => None,
        }
    }

    fn clear_or_cancel_commission(&mut self) {
        if self.restore_machine.is_some() {
            self.restore_cancel_requested = true;
        } else {
            self.commission_engine = None;
        }
    }

    /// Begin a poll-driven terminal restore for one commissioning terminal.
    ///
    /// The restore is advanced by [`Self::poll_commissioning_terminal`], one
    /// backend operation per call, until it reports its terminal action.
    pub fn begin_commissioning_terminal(
        &mut self,
        result: CommissionTerminal,
        config: &TmcControlConfig,
    ) {
        debug_assert!(
            self.restore_machine.is_none(),
            "a terminal restore is already in flight on this channel"
        );
        let status_error = match &result {
            CommissionTerminal::InnerFailed(error) | CommissionTerminal::OuterFailed(error) => {
                Some(*error)
            }
            _ => None,
        };
        let mut machine =
            TerminalRestoreMachine::start_staged(result, self.channel, config.tmc.pwm_maxcnt);
        if let Some(error) = status_error {
            machine.set_status_error(error);
        }
        self.restore_machine = Some(machine);
    }

    /// Returns true while a terminal restore still owns this channel.
    pub const fn is_restore_pending(&self) -> bool {
        self.restore_machine.is_some()
    }

    /// Latch a cancel request for a pending terminal restore.
    pub fn request_restore_cancel(&mut self) {
        if self.restore_machine.is_some() {
            self.restore_cancel_requested = true;
        }
    }

    /// Returns true while a latched cancel still needs draining.
    pub const fn restore_cancel_pending(&self) -> bool {
        self.restore_cancel_requested
    }

    async fn poll_restore_outcome<B>(
        &mut self,
        backend: &mut B,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> Option<crate::commission_dispatch::CommissionOutcome>
    where
        B: CommissioningBackend,
    {
        let step = {
            let machine = self.restore_machine.as_mut()?;
            let mut dispatch = crate::commission_dispatch::DispatchState::new(
                self.channel,
                tmc,
                stepper,
                &mut self.calibrated,
                &mut self.motor_enabled,
                &mut self.stored_gains,
            );
            machine
                .poll_step(
                    &mut self.commission_engine,
                    backend,
                    &mut dispatch,
                    Some(retention),
                )
                .await
        };
        match step {
            crate::commission_dispatch::RestoreStep::Pending => None,
            crate::commission_dispatch::RestoreStep::Done(outcome) => {
                self.restore_machine = None;
                self.applied_velocity_offset = None;
                self.applied_torque_flux_offset = None;
                Some(outcome)
            }
        }
    }

    /// Advance a pending terminal restore and map a completed one to the
    /// board-facing action; `None` while it is still in flight.
    pub async fn poll_commissioning_terminal<B>(
        &mut self,
        backend: &mut B,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
    ) -> Option<ScheduledCommissionTickAction>
    where
        B: CommissioningBackend,
    {
        let outcome = self
            .poll_restore_outcome(backend, retention, tmc, stepper)
            .await?;
        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, actions } => {
                Some(ScheduledCommissionTickAction::Terminal { reply, actions })
            }
            crate::commission_dispatch::CommissionOutcome::Continue
            | crate::commission_dispatch::CommissionOutcome::Evidence { .. }
            | crate::commission_dispatch::CommissionOutcome::PhaseChanged { .. } => {
                unreachable!("terminal restore produced a nonterminal dispatch action")
            }
        }
    }

    /// Drive a latched cancel to safe state, then retire both slots.
    pub async fn drive_restore_cancel<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        status_error: crate::autotune::CommissionError,
    ) -> crate::commission_dispatch::RestoreStep
    where
        B: CommissioningBackend,
    {
        if !self.restore_cancel_requested {
            return crate::commission_dispatch::RestoreStep::Done(
                crate::commission_dispatch::CommissionOutcome::Continue,
            );
        }
        if self.restore_machine.is_none() {
            self.commission_engine = None;
            self.restore_cancel_requested = false;
            return crate::commission_dispatch::RestoreStep::Done(
                crate::commission_dispatch::CommissionOutcome::Continue,
            );
        }
        let outcome = {
            let machine = self
                .restore_machine
                .as_mut()
                .expect("restore machine checked present above");
            let mut dispatch = crate::commission_dispatch::DispatchState::new(
                self.channel,
                tmc,
                stepper,
                &mut self.calibrated,
                &mut self.motor_enabled,
                &mut self.stored_gains,
            );
            machine.set_status_error(status_error);
            machine
                .force_safe_terminal(&mut self.commission_engine, backend, &mut dispatch)
                .await
        };
        self.restore_machine = None;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        self.restore_cancel_requested = false;
        crate::commission_dispatch::RestoreStep::Done(outcome)
    }

    /// Cancel this channel's in-flight commissioning run on operator request.
    ///
    /// Returns `None` when `tmc.cancel_requested` was not set or nothing needs cancelling
    /// (the flag is consumed either way). The caller must emit the returned reply.
    pub async fn drive_operator_cancel<B>(
        &mut self,
        backend: &mut B,
        tmc: &'static TmcChannelState,
        stepper: &'static StepperChannelState,
        pwm_maxcnt: u16,
    ) -> Option<crate::commission_dispatch::CommissionOutcome>
    where
        B: CommissioningBackend,
    {
        use core::sync::atomic::Ordering::Relaxed;
        if !tmc.cancel_requested.swap(false, Relaxed) {
            return None;
        }
        if self.restore_machine.is_some() {
            crate::commissioning::outer::position_tune::abort_stimulus(backend);
            self.request_restore_cancel();
            match self
                .drive_restore_cancel(
                    backend,
                    tmc,
                    stepper,
                    crate::autotune::CommissionError::Cancelled,
                )
                .await
            {
                crate::commission_dispatch::RestoreStep::Done(outcome) => Some(outcome),
                crate::commission_dispatch::RestoreStep::Pending => {
                    unreachable!("force_safe_terminal completes in one step")
                }
            }
        } else if let Some(engine) = self.commission_engine.as_ref() {
            crate::commissioning::outer::position_tune::abort_stimulus(backend);
            let kind = engine.kind();
            let mut machine = crate::commission_dispatch::TerminalRestoreMachine::start_cancelled(
                kind,
                self.channel,
                pwm_maxcnt,
            );
            let mut dispatch = crate::commission_dispatch::DispatchState::new(
                self.channel,
                tmc,
                stepper,
                &mut self.calibrated,
                &mut self.motor_enabled,
                &mut self.stored_gains,
            );
            match machine
                .poll_step(&mut self.commission_engine, backend, &mut dispatch, None)
                .await
            {
                crate::commission_dispatch::RestoreStep::Done(outcome) => Some(outcome),
                crate::commission_dispatch::RestoreStep::Pending => {
                    unreachable!("ForceSafeDisarm completes in one step")
                }
            }
        } else {
            None
        }
    }

    /// Prepare an enable request up to the point where SPI or backend work begins.
    ///
    /// Returns no action when the motor is already enabled or uncalibrated without
    /// `auto_calibrate_on_enable_allowed`; otherwise the required stepper post-action.
    /// Does not energize the motor.
    pub fn prepare_enable_request(
        &mut self,
        tmc: &TmcChannelState,
    ) -> Option<crate::commission_dispatch::StepperPostAction> {
        if self.motor_enabled {
            return None;
        }
        if !self.calibrated {
            if !tmc
                .auto_calibrate_on_enable_allowed
                .load(core::sync::atomic::Ordering::Relaxed)
            {
                return None;
            }
            return None;
        }
        Some(crate::commission_dispatch::StepperPostAction::ArmPreservePosition)
    }

    /// Prepare a disable request, emitting an immediate board stop action.
    ///
    /// Clears `motor_enabled` and `calibrated`; the board must apply the returned
    /// [`StepperAction::StopNow`] before issuing further SPI commands.
    pub fn prepare_disable_request(&mut self) -> BoardActions {
        let mut actions = BoardActions::new();
        push_action(&mut actions, BoardAction::Stepper(StepperAction::StopNow));
        self.motor_enabled = false;
        self.calibrated = false;
        actions
    }

    /// Clear runtime and shared step-acceptance state after a diagnostic trip.
    ///
    /// The board must lower the enable GPIO and stop its step timer first.
    pub fn contain_diagnostic_trip(&mut self) {
        self.motor_enabled = false;
        self.calibrated = false;
        self.applied_velocity_offset = None;
        self.applied_torque_flux_offset = None;
        if let Some(tmc) = crate::state::tmc::tmc_channel(self.channel) {
            tmc.motor_armed.store(false, Relaxed);
        }
        if let Some(stepper) = crate::state::stepper::stepper_channel(self.channel) {
            stepper.timer_active_mirror.store(false, Relaxed);
        }
    }

    /// Return the exact planner-timing velocity ratio cached for later consumers.
    pub const fn planner_velocity_scale(
        &self,
    ) -> Option<crate::motion_scale::PlannerVelocityScale> {
        self.planner_velocity_scale
    }

    /// Return the exact planner-timing mechanical-RPM ratio cached for feedforward.
    pub const fn planner_rpm_scale(&self) -> Option<crate::motion_scale::PlannerRpmScale> {
        self.planner_rpm_scale
    }

    /// Cache both exact planner-timing ratios derived from one planner resolution.
    pub fn set_planner_scales(
        &mut self,
        velocity: Option<crate::motion_scale::PlannerVelocityScale>,
        rpm: Option<crate::motion_scale::PlannerRpmScale>,
    ) {
        self.planner_velocity_scale = velocity;
        self.planner_rpm_scale = rpm;
    }

    /// Return whether velocity feedforward was active on the previous control tick.
    pub const fn feedforward_was_active(&self) -> bool {
        self.feedforward_was_active
    }

    /// Store whether velocity feedforward is active for the current control tick.
    #[cfg(test)]
    pub(crate) fn set_feedforward_was_active(&mut self, active: bool) {
        self.feedforward_was_active = active;
    }

    /// Invalidate the cached applied `PID_TORQUE_FLUX_OFFSET` value.
    ///
    /// Call after any write to that register outside the feedforward path.
    pub fn invalidate_torque_flux_offset_cache(&mut self) {
        self.applied_torque_flux_offset = None;
    }

    /// Return the remaining PID error-clear settle countdown in milliseconds.
    pub const fn pid_error_clear_countdown_ms(&self) -> u8 {
        self.pid_error_clear_countdown_ms
    }

    /// Store the remaining PID error-clear settle countdown in milliseconds.
    #[cfg(test)]
    pub(crate) fn set_pid_error_clear_countdown_ms(&mut self, value: u8) {
        self.pid_error_clear_countdown_ms = value;
    }

    /// Compute current command velocity reference for feedforward surfaces.
    ///
    /// Returns zero while the motor is disabled, velocity scaling has not been
    /// configured, commissioning is active, homing is active, or the caller has
    /// consumed a one-tick feedforward reset latch.
    pub fn command_velocity_reference(
        &mut self,
        refs: TmcControlStateRefs<'_>,
        homing_active: bool,
        reset: bool,
    ) -> i32 {
        let reference = match self.planner_velocity_scale {
            Some(scale)
                if self.motor_enabled
                    && self.planner_paths_allowed(refs)
                    && !homing_active
                    && !reset =>
            {
                let interval = refs.stepper.interval.load(Relaxed);
                let forward = refs.stepper.direction.load(Relaxed);
                crate::tmc::compute_planner_command_velocity_reference(scale, interval, forward)
            }
            Some(_) | None => 0,
        };
        #[cfg(feature = "trace")]
        {
            self.trace_state_mut().cached_velocity_reference = reference;
        }
        reference
    }

    /// Apply velocity and acceleration feedforward.
    ///
    /// The acceleration offset runs only when `slow_control_tick` is true, and is non-zero only
    /// while the offset-headroom reservation holds the torque/flux limit at the reduced value.
    /// Active feedforward SPI failures are best-effort; failed inactive-offset clears and failed
    /// reservation-release writes are propagated so they are retried.
    pub(crate) async fn apply_feedforward_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing_active: bool,
        reset: bool,
        slow_control_tick: bool,
    ) -> Result<(), ()>
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        let ff_active = self.motor_enabled
            && self.planner_rpm_scale.is_some()
            && refs.tmc.velocity_feedforward.load(Relaxed)
            && self.planner_paths_allowed(refs);
        let accel_ff_active = self.motor_enabled
            && self.planner_velocity_scale.is_some()
            && refs.tmc.accel_feedforward.load(Relaxed)
            && self.planner_paths_allowed(refs);

        if ff_active {
            let interval = refs.stepper.interval.load(Relaxed);
            let forward = refs.stepper.direction.load(Relaxed);
            let rpm_scale = self
                .planner_rpm_scale
                .expect("active feedforward has planner rpm scale");
            let base_offset = if !homing_active && !reset {
                let gain_permille = refs.tmc.velocity_feedforward_gain_permille.load(Relaxed);
                crate::tmc::compute_planner_velocity_offset_rpm(
                    rpm_scale,
                    interval,
                    forward,
                    gain_permille,
                )
            } else {
                0
            };
            let command_rpm =
                crate::tmc::compute_planner_command_rpm_reference(rpm_scale, interval, forward);
            let (offset, clamp_engaged) = crate::tmc::clamp_velocity_offset_rpm(
                base_offset,
                command_rpm,
                crate::tmc::VELOCITY_FF_CLAMP_K,
            );
            if clamp_engaged {
                refs.tmc.record_velocity_clamp_engagement();
            }
            let desired = offset as u32;

            if self.applied_velocity_offset != Some(desired) {
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, desired)
                    .await
                    .inspect(|_| self.applied_velocity_offset = Some(desired));
            }
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_velocity_offset = offset;
                trace.cached_velocity_clamped = clamp_engaged;
            }
        } else if self.feedforward_was_active {
            tmc_spi
                .write_register(foci_tmc4671::registers::PID_VELOCITY_OFFSET, 0)
                .await
                .map_err(|_| ())?;
            self.applied_velocity_offset = Some(0);
            #[cfg(feature = "trace")]
            {
                let trace = self.trace_state_mut();
                trace.cached_velocity_offset = 0;
                trace.cached_velocity_clamped = false;
            }
        }
        self.feedforward_was_active = ff_active;

        if !slow_control_tick {
            self.publish_feedforward_surfaces_idle(refs);
            return Ok(());
        }

        let eligible = self.offset_headroom_eligible(refs, homing_active, reset);
        self.advance_offset_headroom(tmc_spi, refs, eligible)
            .await?;
        let offset_allowed = self.offset_headroom == OffsetHeadroomPhase::Held;

        if accel_ff_active {
            let accel_gain_permille = refs.tmc.accel_feedforward_accel_gain_permille.load(Relaxed);
            let decel_gain_permille = refs.tmc.accel_feedforward_decel_gain_permille.load(Relaxed);
            let torque_limit = u32::from(crate::tmc_control::current_limit::offset_headroom(
                refs.tmc.load_current_limit(Relaxed),
            ));
            let interval = refs.stepper.interval.load(Relaxed);
            let interval_add = refs
                .stepper
                .interval_add
                .load(Relaxed)
                .clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            #[cfg(feature = "trace")]
            let selected_gain_permille = if interval_add < 0 {
                accel_gain_permille
            } else if interval_add > 0 {
                decel_gain_permille
            } else {
                0
            };
            let offset = if !offset_allowed {
                0i16
            } else {
                let forward = refs.stepper.direction.load(Relaxed);
                crate::tmc::compute_planner_accel_torque_offset_split(
                    self.planner_velocity_scale
                        .expect("active acceleration feedforward has planner velocity scale"),
                    config.clock_freq,
                    interval,
                    interval_add,
                    forward,
                    crate::tmc::AccelFeedforwardGains {
                        accel_permille: accel_gain_permille,
                        decel_permille: decel_gain_permille,
                    },
                    torque_limit,
                )
            };
            #[cfg(feature = "trace")]
            {
                let mut flags = if !offset_allowed {
                    0
                } else {
                    crate::trace::stream::ACCEL_FF_FLAG_ENABLED
                };
                if offset != 0 {
                    flags |= crate::trace::stream::ACCEL_FF_FLAG_NONZERO;
                }
                if offset != 0 && offset.unsigned_abs() as u32 >= torque_limit.min(i16::MAX as u32)
                {
                    flags |= crate::trace::stream::ACCEL_FF_FLAG_CLAMPED;
                }
                let trace = self.trace_state_mut();
                trace.cached_accel_torque_offset = offset as i32;
                trace.cached_accel_gain_permille = if !offset_allowed {
                    0
                } else {
                    selected_gain_permille
                };
                trace.cached_accel_flags = flags;
            }
            let offset_reg = u32::from(offset as u16) << 16;
            if self.applied_torque_flux_offset != Some(offset_reg) {
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, offset_reg)
                    .await
                    .inspect(|_| self.applied_torque_flux_offset = Some(offset_reg));
            }
        } else if self.torque_flux_offset_was_active {
            tmc_spi
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, 0)
                .await
                .map_err(|_| ())?;
            self.applied_torque_flux_offset = Some(0);
            self.clear_accel_trace();
        }
        self.torque_flux_offset_was_active = accel_ff_active;

        self.publish_feedforward_surfaces_idle(refs);
        Ok(())
    }

    /// Apply one channel's homing maintenance for a frozen homing state.
    ///
    /// State 1 suppresses the position integrator, clamps the torque/flux limit to the homing
    /// current, and arms the stall detector. State 2 feeds the detector one sample. State 3
    /// restores the stored run current and publishes the detector result. A missing
    /// precondition or exhausted retry budget returns [`HomingMaintenanceResult::FailClosed`],
    /// which the board must translate into an emergency disable. The stored run-current limit
    /// is never mutated.
    pub async fn apply_homing_channel_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing_state: u8,
        window: HomingClockWindow,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        match homing_state {
            1 => {
                if !homing_clamp_due(window, config.clock_freq) {
                    return HomingMaintenanceResult::Deferred;
                }
                if self.offset_headroom_engaged() {
                    return self.retry_homing_state_1();
                }
                let gains = match refs.tmc.load_outer_gains(Relaxed) {
                    Ok(gains) => gains,
                    Err(_) => return self.retry_homing_state_1(),
                };
                let reg = PidGainPair::new(gains.position_p, PidIGain::ZERO).register_value();
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                    .await;
                defmt::info!("homing: position I suppressed");

                match self.apply_homing_current_clamp(tmc_spi, refs, config).await {
                    HomingMaintenanceResult::ReadyToAdvance => {
                        self.homing_state_1_retries = 0;
                        HomingMaintenanceResult::ReadyToAdvance
                    }
                    HomingMaintenanceResult::RetryCurrentState => self.retry_homing_state_1(),
                    HomingMaintenanceResult::FailClosed => HomingMaintenanceResult::FailClosed,
                    HomingMaintenanceResult::Deferred => HomingMaintenanceResult::Deferred,
                }
            }
            2 => {
                if let Some(detector) = self.stall_detector.as_mut() {
                    let target = tmc_spi
                        .read_register(foci_tmc4671::registers::PID_POSITION_TARGET)
                        .await;
                    let actual = tmc_spi
                        .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                        .await;
                    if let (Ok(target), Ok(actual)) = (target, actual)
                        && detector.observe(target, actual)
                    {
                        refs.tmc.stall_latched.store(true, Relaxed);
                    }
                }
                HomingMaintenanceResult::ReadyToAdvance
            }
            3 => {
                let gains = match refs.tmc.load_outer_gains(Relaxed) {
                    Ok(gains) => gains,
                    Err(_) => return HomingMaintenanceResult::RetryCurrentState,
                };
                let reg = PidGainPair::new(gains.position_p, gains.position_i).register_value();
                let _ = tmc_spi
                    .write_register(foci_tmc4671::registers::PID_POSITION_P_POSITION_I, reg)
                    .await;
                let _ = tmc_spi
                    .write_sub_register(
                        foci_tmc4671::registers::PID_ERROR_PID_POSITION_ERROR_SUM,
                        0,
                    )
                    .await;
                defmt::info!("homing: position I restored, integrator cleared");

                self.restore_run_current_after_homing(tmc_spi, refs).await
            }
            _ => {
                self.homing_state_1_retries = 0;
                self.homing_state_3_retries = 0;
                if self.stall_detector.take().is_some()
                    || refs.tmc.homing_clamp_active.load(Relaxed)
                {
                    refs.tmc.homing_clamp_active.store(false, Relaxed);
                    refs.tmc.stall_latched.store(false, Relaxed);
                    defmt::warn!("homing: state forced idle outside the tick, detector dropped");
                }
                HomingMaintenanceResult::ReadyToAdvance
            }
        }
    }

    fn retry_homing_state_1(&mut self) -> HomingMaintenanceResult {
        self.homing_state_1_retries = self.homing_state_1_retries.saturating_add(1);
        if self.homing_state_1_retries >= HOMING_CLAMP_MAX_RETRIES {
            defmt::error!(
                "homing: state 1 failed {} times",
                self.homing_state_1_retries
            );
            self.homing_state_1_retries = 0;
            return HomingMaintenanceResult::FailClosed;
        }
        HomingMaintenanceResult::RetryCurrentState
    }

    /// Clamp the torque/flux limit to the homing current and arm the detector.
    async fn apply_homing_current_clamp<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        let homing_ma = refs.tmc.homing_current_ma.load(Relaxed);
        if homing_ma == 0 {
            if refs.tmc.stall_endstop_configured.load(Relaxed) {
                defmt::error!("homing: stall endstop configured without homing current");
                return HomingMaintenanceResult::FailClosed;
            }
            return HomingMaintenanceResult::ReadyToAdvance;
        }
        if !(self.motor_enabled && self.calibrated) || self.is_commission_active() {
            defmt::error!("homing: channel not armed for steps, refusing clamp");
            return HomingMaintenanceResult::FailClosed;
        }
        let Some(encoder_counts_per_rev) = refs.tmc.encoder_counts_per_rev() else {
            defmt::error!("homing: no motion scale published, refusing clamp");
            return HomingMaintenanceResult::FailClosed;
        };
        let limit = match crate::tmc_control::current_limit::current_limit_from_run_ma(
            homing_ma,
            config.current_scale_ma_per_lsb,
        ) {
            Ok(limit) => limit,
            Err(_) => {
                defmt::error!("homing: invalid homing current {} mA", homing_ma);
                return HomingMaintenanceResult::FailClosed;
            }
        };
        if limit.raw() >= refs.tmc.load_current_limit(Relaxed).raw() {
            defmt::error!(
                "homing: homing current {} mA is not below run current",
                homing_ma
            );
            return HomingMaintenanceResult::FailClosed;
        }
        if tmc_spi
            .write_register(
                foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                limit.register_value(),
            )
            .await
            .is_err()
        {
            return HomingMaintenanceResult::RetryCurrentState;
        }
        refs.tmc.stall_latched.store(false, Relaxed);
        refs.tmc.homing_clamp_active.store(true, Relaxed);
        self.stall_detector = Some(crate::tmc_control::stall_detector::StallDetector::new(
            refs.tmc.stall_ceiling_units.load(Relaxed),
            refs.tmc.stall_margin_units.load(Relaxed),
            refs.tmc.stall_persistence.load(Relaxed),
            encoder_counts_per_rev,
        ));
        defmt::info!("homing: clamp {} mA, detector armed", homing_ma);
        HomingMaintenanceResult::ReadyToAdvance
    }

    /// Rewrite the operational limit and publish the stall-detector result.
    async fn restore_run_current_after_homing<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        if refs.tmc.homing_clamp_active.load(Relaxed) {
            if tmc_spi
                .write_register(
                    foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                    crate::tmc_control::current_limit::operational_limit_register(refs.tmc),
                )
                .await
                .is_err()
            {
                self.homing_state_3_retries = self.homing_state_3_retries.saturating_add(1);
                if self.homing_state_3_retries >= HOMING_RESTORE_MAX_RETRIES {
                    defmt::error!(
                        "homing: run-current restore failed {} times",
                        self.homing_state_3_retries
                    );
                    self.homing_state_3_retries = 0;
                    return HomingMaintenanceResult::FailClosed;
                }
                return HomingMaintenanceResult::RetryCurrentState;
            }
            self.homing_state_3_retries = 0;
            refs.tmc.homing_clamp_active.store(false, Relaxed);
        }
        if let Some(detector) = self.stall_detector.take() {
            refs.tmc
                .stall_result_latched
                .store(detector.latched(), Relaxed);
            refs.tmc
                .stall_peak_error_units
                .store(detector.peak_error_units(), Relaxed);
            refs.tmc
                .stall_trigger_tick
                .store(detector.trigger_tick(), Relaxed);
            refs.tmc
                .stall_trigger_path
                .store(detector.trigger_path().as_u8(), Relaxed);
            refs.tmc
                .stall_peak_margin_delta_units
                .store(detector.peak_margin_delta_units(), Relaxed);
        }
        refs.tmc.stall_latched.store(false, Relaxed);
        defmt::info!("homing: run current restored");
        HomingMaintenanceResult::ReadyToAdvance
    }

    /// Advance the global homing state after every channel observed a snapshot.
    pub fn advance_homing_state_after_channels(
        homing: TmcControlHomingRefs<'_>,
        snapshot: u8,
        all_channels_ready: bool,
    ) {
        if !all_channels_ready {
            return;
        }

        // `snapshot` was taken when the maintenance group opened. The host's
        // `trsync_trigger` can end the homing move in the meantime and move the
        // state to 3 on its own, so only advance from the state the snapshot
        // described; a blind store would drag a finished move back to 2 and
        // strand it there with its trigger already cleared.
        match snapshot {
            1 => {
                let _ = homing.state.compare_exchange(1, 2, Relaxed, Relaxed);
            }
            2 if homing.triggered.load(Relaxed) != 0xFF => {
                let _ = homing.state.compare_exchange(2, 3, Relaxed, Relaxed);
            }
            3 => Self::leave_restore_pending(&homing),
            _ => {}
        }
    }

    fn leave_restore_pending(homing: &TmcControlHomingRefs<'_>) {
        if homing
            .state
            .compare_exchange(3, 0, Relaxed, Relaxed)
            .is_ok()
            && homing.reenter.swap(false, Relaxed)
        {
            let _ = homing.state.compare_exchange(0, 1, Relaxed, Relaxed);
        }
    }

    /// Apply the shared per-channel slow PID and homing policy.
    ///
    /// A non-ready homing result means the caller should skip later slow-maintenance work
    /// for this channel until the next slow tick.
    pub async fn apply_slow_channel_maintenance_tick<B>(
        &mut self,
        tmc_spi: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
        refs: TmcControlStateRefs<'_>,
        config: &TmcControlConfig,
        homing_snapshot: u8,
        window: HomingClockWindow,
    ) -> HomingMaintenanceResult
    where
        B: embedded_hal_async::spi::SpiDevice<u8>,
    {
        self.apply_pid_error_settle_tick(tmc_spi, refs).await;
        self.apply_homing_channel_tick(tmc_spi, refs, config, homing_snapshot, window)
            .await
    }

    /// Advance the trace control tick for a board-local trace sample.
    #[cfg(feature = "trace")]
    pub fn advance_trace_sample_tick(&mut self) -> u32 {
        let sample_tick = self.trace.control_tick;
        self.trace.control_tick = self.trace.control_tick.wrapping_add(1);
        sample_tick
    }

    /// Advance trace timing and return transport emission decisions.
    #[cfg(feature = "trace")]
    pub fn advance_trace_tick(&mut self) -> TmcControlTraceTick {
        let sample_tick = self.advance_trace_sample_tick();
        let control_tick = self.trace.control_tick;

        TmcControlTraceTick {
            sample_tick,
            control_tick,
            status_detail_due: control_tick.is_multiple_of(100),
            metadata_due: control_tick.is_multiple_of(1000),
        }
    }

    /// Compute the requested control period in microseconds from current state.
    ///
    /// Returns [`crate::tmc::CONTROL_PERIOD_IDLE_US`] when the motor is disabled or
    /// `Self::planner_paths_allowed` is false, otherwise [`crate::tmc::CONTROL_PERIOD_ACTIVE_US`].
    pub fn control_period_us(&self, refs: TmcControlStateRefs<'_>) -> u32 {
        if !self.motor_enabled || !self.planner_paths_allowed(refs) {
            return crate::tmc::CONTROL_PERIOD_IDLE_US;
        }
        crate::tmc::CONTROL_PERIOD_ACTIVE_US
    }

    #[cfg(test)]
    pub(crate) async fn drive_commissioning_terminal_for_test<B>(
        &mut self,
        result: CommissionTerminal,
        backend: &mut B,
        retention: &mut crate::commission_dispatch::CommissionRetention<'_>,
        config: &TmcControlConfig,
    ) -> ScheduledCommissionTickAction
    where
        B: CommissioningBackend,
    {
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        self.begin_commissioning_terminal(result, config);
        loop {
            if let Some(action) = self
                .poll_commissioning_terminal(backend, retention, tmc, stepper)
                .await
            {
                return action;
            }
        }
    }

    #[cfg(test)]
    pub(crate) const fn offset_headroom_phase_for_test(&self) -> OffsetHeadroomPhase {
        self.offset_headroom
    }

    #[cfg(test)]
    pub(crate) fn hold_offset_headroom_for_test(
        &mut self,
        tmc_state: &TmcChannelState,
        offset: u32,
    ) {
        tmc_state.set_offset_headroom_reserved(true, Relaxed);
        self.offset_headroom = OffsetHeadroomPhase::Held;
        self.torque_flux_offset_was_active = true;
        self.applied_torque_flux_offset = Some(offset);
    }

    #[cfg(test)]
    pub(crate) fn set_calibrated_for_test(&mut self, value: bool) {
        self.calibrated = value;
    }

    #[cfg(test)]
    pub(crate) fn set_motor_enabled_for_test(&mut self, value: bool) {
        self.motor_enabled = value;
    }

    #[cfg(test)]
    pub(crate) fn set_torque_flux_offset_was_active_for_test(&mut self, value: bool) {
        self.torque_flux_offset_was_active = value;
    }

    #[cfg(test)]
    pub(crate) fn set_homing_state_1_retries_for_test(&mut self, value: u8) {
        self.homing_state_1_retries = value;
    }

    #[cfg(test)]
    pub(crate) fn set_homing_state_3_retries_for_test(&mut self, value: u8) {
        self.homing_state_3_retries = value;
    }

    #[cfg(test)]
    pub(crate) const fn homing_retry_counts_for_test(&self) -> (u8, u8) {
        (self.homing_state_1_retries, self.homing_state_3_retries)
    }

    #[cfg(test)]
    pub(crate) fn set_applied_velocity_offset_for_test(&mut self, value: Option<u32>) {
        self.applied_velocity_offset = value;
    }

    #[cfg(test)]
    pub(crate) fn set_applied_torque_flux_offset_for_test(&mut self, value: Option<u32>) {
        self.applied_torque_flux_offset = value;
    }

    #[cfg(test)]
    pub(crate) fn stall_detector_armed_for_test(&self) -> bool {
        self.stall_detector.is_some()
    }

    #[cfg(test)]
    pub(crate) fn homing_state_1_retries_for_test(&self) -> u8 {
        self.homing_state_1_retries
    }

    #[cfg(test)]
    pub(crate) fn arm_stall_detector_for_test(
        &mut self,
        ceiling: u32,
        margin: u32,
        persistence: u8,
    ) {
        self.stall_detector = Some(crate::tmc_control::stall_detector::StallDetector::new(
            ceiling,
            margin,
            persistence,
            4_000,
        ));
    }

    #[cfg(test)]
    pub(crate) fn set_planner_velocity_scale_for_test(&mut self, numerator: u64) {
        self.planner_velocity_scale = Some(
            crate::motion_scale::PlannerVelocityScale::from_reduced(numerator, 1),
        );
        self.planner_rpm_scale = Some(crate::motion_scale::PlannerRpmScale::from_reduced(
            numerator, 1,
        ));
    }

    #[cfg(test)]
    pub(crate) fn pid_error_clear_countdown_for_test(&self) -> u8 {
        self.pid_error_clear_countdown_ms
    }

    #[cfg(test)]
    pub(crate) fn pending_calibrate_inner_gains_for_test(
        &self,
    ) -> Option<crate::pid::InnerLoopGains> {
        self.commission_engine.as_ref().and_then(
            crate::commissioning::CommissioningEngine::calibrate_entry_inner_gains_for_test,
        )
    }

    #[cfg(test)]
    pub(crate) fn torque_flux_offset_was_active_for_test(&self) -> bool {
        self.torque_flux_offset_was_active
    }

    #[cfg(all(test, feature = "trace"))]
    pub(crate) fn set_trace_control_tick_for_test(&mut self, value: u32) {
        self.trace.control_tick = value;
    }
}

const STATUS_ADC_I_CLIPPED: u32 = 1 << 26;

/// Whether a STATUS_FLAGS read indicates ADC current clipping; a failed read reports `false`.
pub fn is_adc_current_clipped(status_flags: Result<u32, ()>) -> bool {
    matches!(status_flags, Ok(flags) if flags & STATUS_ADC_I_CLIPPED != 0)
}

/// Which emergency-disable shutdown message a STATUS_FLAGS poll selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum EmergencyDisableFault {
    /// ADC current clipped (overcurrent); wins over the other variants.
    AdcCurrentClipped,
    /// The STATUS_FLAGS read itself failed.
    StatusReadFailed,
    /// A non-ADC-clip fault bit was set, or no bit was set.
    Generic,
}

/// Classifies one STATUS_FLAGS poll outcome.
pub fn classify_emergency_disable_fault(status_flags: Result<u32, ()>) -> EmergencyDisableFault {
    if is_adc_current_clipped(status_flags) {
        EmergencyDisableFault::AdcCurrentClipped
    } else if status_flags.is_err() {
        EmergencyDisableFault::StatusReadFailed
    } else {
        EmergencyDisableFault::Generic
    }
}

/// Fold one channel's classification into a multi-channel aggregate; priority is
/// [`EmergencyDisableFault::AdcCurrentClipped`] over [`EmergencyDisableFault::StatusReadFailed`]
/// over [`EmergencyDisableFault::Generic`].
pub fn combine_emergency_disable_fault(
    so_far: EmergencyDisableFault,
    channel: EmergencyDisableFault,
) -> EmergencyDisableFault {
    use EmergencyDisableFault::{AdcCurrentClipped, Generic, StatusReadFailed};
    match (so_far, channel) {
        (AdcCurrentClipped, _) | (_, AdcCurrentClipped) => AdcCurrentClipped,
        (StatusReadFailed, _) | (_, StatusReadFailed) => StatusReadFailed,
        (Generic, Generic) => Generic,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_period_constants_are_pinned() {
        assert_eq!(crate::tmc::CONTROL_PERIOD_IDLE_US, 10_000);
        assert_eq!(crate::tmc::CONTROL_PERIOD_ACTIVE_US, 1_000);
    }

    #[test]
    fn control_period_us_returns_idle_period_when_disabled() {
        let runtime = TmcControlRuntime::new_for_channel(0);
        let tmc = crate::state::tmc::TmcChannelState::new();
        let stepper = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc, &stepper);

        assert_eq!(
            runtime.control_period_us(refs),
            crate::tmc::CONTROL_PERIOD_IDLE_US
        );
    }

    #[test]
    fn control_period_us_returns_active_period_when_enabled() {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        let tmc = crate::state::tmc::TmcChannelState::new();
        let stepper = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc, &stepper);

        assert_eq!(
            runtime.control_period_us(refs),
            crate::tmc::CONTROL_PERIOD_ACTIVE_US
        );
    }

    #[test]
    fn an_idle_operator_cancel_does_not_touch_the_stimulus() {
        pollster::block_on(async {
            let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
                .lock()
                .await;
            crate::state::tmc::reset_all_tmc_channels();
            crate::state::stepper::reset_all_stepper_channels();

            let tmc = crate::state::tmc::tmc_channel0();
            let stepper = crate::state::stepper::stepper_channel0();
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
            tmc.cancel_requested.store(true, Relaxed);

            assert!(!runtime.is_restore_pending());
            assert!(!runtime.is_commission_active());

            let outcome = runtime
                .drive_operator_cancel(&mut backend, tmc, stepper, 4_095)
                .await;

            assert!(
                outcome.is_none(),
                "a cancel with nothing in flight must be a no-op"
            );
            assert!(
                !tmc.cancel_requested.load(Relaxed),
                "the cancel flag must still be consumed"
            );
            assert!(
                backend.stimulus_log.is_empty(),
                "an idle cancel must not touch the stimulus, got {:?}",
                backend.stimulus_log
            );
            assert!(
                backend.event_log.is_empty(),
                "an idle cancel must not issue any backend operation, got {:?}",
                backend.event_log
            );
        });
    }
}
