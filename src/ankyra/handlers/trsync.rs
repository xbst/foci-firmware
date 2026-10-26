//! Trsync command handlers.

use ankyra::prelude::klipper_command;

use crate::context::{ShutdownRecovery, TrsyncHost};
use crate::shutdown::ShutdownReason;
use crate::state::trsync::{HOMING_STATE, HOMING_TRIGGERED};

/// Register a TrSync (trigger synchronization) object.
#[klipper_command]
pub fn config_trsync(ctx: &mut dyn TrsyncLifecycleHost, oid: u8) {
    if ctx.register_trsync(oid).is_err() {
        defmt::error!("config_trsync: oid={}", oid);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::ConfigTrsyncFailed, clock);
        }
        return;
    }
    defmt::info!("config_trsync: oid={}", oid);
}

/// Arm a TrSync: set report interval and timeout reason, and start reporting.
#[klipper_command]
pub fn trsync_start(
    ctx: &mut dyn TrsyncLifecycleHost,
    oid: u8,
    report_clock: u32,
    report_ticks: u32,
    expire_reason: u8,
) {
    if ctx
        .trsync_arm(oid, report_clock, report_ticks, expire_reason)
        .is_err()
    {
        defmt::error!("trsync_start: oid={}", oid);
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TrsyncStartFailed, clock);
        }
    }
}

/// Set the TrSync expiry deadline; it auto-triggers with `expire_reason` if the clock reaches it first.
#[klipper_command]
pub fn trsync_set_timeout(ctx: &mut dyn TrsyncLifecycleHost, oid: u8, clock: u32) {
    if ctx.trsync_set_timeout(oid, clock).is_err() {
        defmt::error!("trsync_set_timeout: oid={}", oid);
        if ctx.enter_shutdown() {
            let shutdown_clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::TrsyncSetTimeoutFailed, shutdown_clock);
        }
    }
}

/// Manually trigger a TrSync (no-op if already triggered), stop its timers, and send a final `trsync_state` with `can_trigger=0`.
#[klipper_command]
pub fn trsync_trigger(ctx: &mut dyn TrsyncLifecycleHost, oid: u8, reason: u8) {
    match ctx.trsync_trigger(oid, reason) {
        Ok(trigger_reason) => {
            use core::sync::atomic::Ordering::Relaxed;
            HOMING_TRIGGERED.store(0xFF, Relaxed);
            // The host ends every homing move with this command. The move is
            // over whether preparation (state 1) finished or not, so hand both
            // the armed state and a still-preparing state to the restore step;
            // leaving state 1 to advance to 2 afterwards would strand the state
            // machine in 2 with its trigger already cleared, and the next homing
            // would park its steps until the segment queue overflowed.
            if HOMING_STATE
                .compare_exchange(2, 3, Relaxed, Relaxed)
                .is_err()
            {
                let _ = HOMING_STATE.compare_exchange(1, 3, Relaxed, Relaxed);
            }
            ::ankyra::klipper_reply!(
                crate::ankyra::replies::TrsyncState,
                oid: u8 = oid,
                can_trigger: u8 = 0,
                trigger_reason: u8 = trigger_reason,
                clock: u32 = 0
            );
        }
        Err(()) => {
            defmt::error!("trsync_trigger: unknown oid={}", oid);
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::TrsyncTriggerFailed, clock);
            }
        }
    }
}

/// Composite bound for all four trsync handlers: shutdown recovery plus the
/// trigger-domain bridge.
pub trait TrsyncLifecycleHost: ShutdownRecovery + TrsyncHost {}
impl<T: ?Sized + ShutdownRecovery + TrsyncHost> TrsyncLifecycleHost for T {}
