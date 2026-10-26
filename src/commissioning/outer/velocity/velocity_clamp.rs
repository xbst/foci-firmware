//! Shared velocity-current prevention and evidence classification.

use foci_tmc4671::registers::{
    INTERIM_PIDIN_TARGET_TORQUE, INTERIM_PIDOUT_TARGET_TORQUE, PID_TORQUE_FLUX_LIMITS, STATUS_FLAGS,
};
use foci_tmc4671::{current_limit::PidTorqueFluxLimit, registers::StatusFlags};

use super::sweep_plan::{CURRENT_HEADROOM_DEN, CURRENT_HEADROOM_NUM};
use crate::commissioning::backend::{CommissionRegisterIo, CommissioningBackend};

/// Firmware's ability to prove whether the velocity-output clamp engaged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum VelocityLimitEvidenceCapability {
    /// Periodic reads can confirm observed contacts but cannot prove absence.
    BestEffortPolling = 0,
    /// The controller preserves a contact until firmware reads the status flag.
    LatchedFlag = 1,
}

/// Primitive phase in which the velocity-output clamp first made contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum VelocityCurrentContact {
    /// No clamp contact was observed.
    None = 0,
    /// Contact occurred while collecting moving evidence.
    Moving = 1,
    /// Contact occurred after commanding zero for braking and settle.
    Braking = 2,
    /// Contact occurred during the optional return-to-origin motion.
    Recovery = 3,
}

/// Zero-target phase represented by the second observation window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum VelocityCurrentZeroPhase {
    /// The moving primitive's braking and settle phase.
    Braking,
    /// The optional return-to-origin primitive.
    Recovery,
}

/// Current-limit evidence collected across one velocity primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityCurrentPhaseEvidence {
    /// Temporary probe clamp applied to the velocity PID output.
    pub(crate) clamp_limit: u16,
    /// Largest sampled absolute velocity-PID output during moving evidence.
    pub(crate) moving_pid_output_peak: u16,
    /// Largest sampled absolute velocity-PID output after commanding zero.
    pub(crate) zero_pid_output_peak: u16,
    /// Complete `STATUS_FLAGS` word observed during moving evidence.
    pub(crate) moving_status_flags: u32,
    /// Complete `STATUS_FLAGS` word observed after commanding zero.
    pub(crate) zero_status_flags: u32,
    /// Available mechanism for proving absence of an unsampled contact.
    pub(crate) capability: VelocityLimitEvidenceCapability,
    /// First phase in which clamp contact was observed.
    pub(crate) contact: VelocityCurrentContact,
}

impl VelocityCurrentPhaseEvidence {
    /// Return whether the moving phase is proven free of clamp contact.
    #[cfg(test)]
    pub(crate) fn moving_clip_absence_proven(self) -> bool {
        self.capability == VelocityLimitEvidenceCapability::LatchedFlag
            && self.contact != VelocityCurrentContact::Moving
            && self.moving_pid_output_peak < self.clamp_limit
    }

    /// Return whether an unobserved moving-phase contact remains possible.
    #[cfg(test)]
    pub(crate) fn moving_clip_uncertain(self) -> bool {
        self.capability == VelocityLimitEvidenceCapability::BestEffortPolling
            && self.contact != VelocityCurrentContact::Moving
            && self.moving_pid_output_peak < self.clamp_limit
    }
}

/// Invalid operational current limit for a nonzero commissioning clamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum VelocityCurrentGuardError {
    /// A configured limit cannot represent a nonzero commissioning clamp.
    InvalidLimit,
    /// A register read, write, or verified write failed.
    RegisterIo,
    /// A non-velocity status bit requires the existing fail-closed consequence.
    ActionableStatus(u32),
    /// Only limiter bits were set at a phase boundary: the stroke drove the
    /// current loops or the voltage output into a configured limit. Phases treat
    /// this as current-authority contact at the active gain, not a fault.
    LimiterContact(u32),
}

/// Classify the actionable status bits observed at a velocity-phase boundary.
///
/// A word made only of [`crate::autotune::current_loop_limiter_status_mask`]
/// bits is [`VelocityCurrentGuardError::LimiterContact`]; any other actionable
/// bit keeps the fail-closed [`VelocityCurrentGuardError::ActionableStatus`].
fn boundary_status_error(actionable: u32) -> VelocityCurrentGuardError {
    if actionable & !crate::autotune::current_loop_limiter_status_mask() == 0 {
        VelocityCurrentGuardError::LimiterContact(actionable)
    } else {
        VelocityCurrentGuardError::ActionableStatus(actionable)
    }
}

/// Derive the exact nine-tenths probe limit from the operational limit.
pub(crate) fn derive_probe_limit(
    operational_limit: PidTorqueFluxLimit,
) -> Result<PidTorqueFluxLimit, VelocityCurrentGuardError> {
    let probe_raw =
        u32::from(operational_limit.raw()) * CURRENT_HEADROOM_NUM / CURRENT_HEADROOM_DEN;
    if probe_raw == 0 {
        return Err(VelocityCurrentGuardError::InvalidLimit);
    }

    PidTorqueFluxLimit::new(probe_raw).map_err(|_| VelocityCurrentGuardError::InvalidLimit)
}

/// Primitive-wide accumulator for velocity-output peaks and status flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityCurrentObserver {
    moving_status_flags: u32,
    zero_status_flags: u32,
    clamp_limit: u16,
    moving_pid_output_peak: u16,
    zero_pid_output_peak: u16,
    capability: VelocityLimitEvidenceCapability,
}

impl VelocityCurrentObserver {
    /// Empty sentinel used before command admission installs a valid plan.
    pub(crate) const EMPTY: Self = Self::new(
        PidTorqueFluxLimit::ZERO,
        VelocityLimitEvidenceCapability::BestEffortPolling,
    );

    /// Start an empty observer under one fixed clamp and evidence capability.
    pub(crate) const fn new(
        clamp_limit: PidTorqueFluxLimit,
        capability: VelocityLimitEvidenceCapability,
    ) -> Self {
        Self {
            moving_status_flags: 0,
            zero_status_flags: 0,
            clamp_limit: clamp_limit.raw(),
            moving_pid_output_peak: 0,
            zero_pid_output_peak: 0,
            capability,
        }
    }

    /// Clear phase evidence before the next signed-plus-zero primitive.
    pub(crate) fn reset(
        &mut self,
        clamp_limit: PidTorqueFluxLimit,
        capability: VelocityLimitEvidenceCapability,
    ) {
        *self = Self::new(clamp_limit, capability);
    }

    /// Clear observations while retaining the active clamp and capability.
    pub(crate) fn reset_existing_contract(&mut self) {
        self.moving_status_flags = 0;
        self.zero_status_flags = 0;
        self.moving_pid_output_peak = 0;
        self.zero_pid_output_peak = 0;
    }

    /// Accumulate one absolute velocity-PID output sample from the signed phase.
    pub(crate) fn observe_moving_output(&mut self, output_abs: u16) {
        self.moving_pid_output_peak = self.moving_pid_output_peak.max(output_abs);
    }

    /// Accumulate one absolute velocity-PID output sample after commanding zero.
    pub(crate) fn observe_zero_output(&mut self, output_abs: u16) {
        self.zero_pid_output_peak = self.zero_pid_output_peak.max(output_abs);
    }

    /// Preserve all status bits observed at the moving-phase boundary.
    pub(crate) fn observe_moving_flags(&mut self, flags: u32) {
        self.moving_status_flags |= flags;
        if StatusFlags::from(flags).pid_v_output_limit() {
            self.moving_pid_output_peak = self.moving_pid_output_peak.max(self.clamp_limit);
        }
    }

    /// Preserve all status bits observed after the zero-target phase.
    pub(crate) fn observe_zero_flags(&mut self, flags: u32) {
        self.zero_status_flags |= flags;
        if StatusFlags::from(flags).pid_v_output_limit() {
            self.zero_pid_output_peak = self.zero_pid_output_peak.max(self.clamp_limit);
        }
    }

    /// Active clamp magnitude used by this primitive.
    pub(crate) const fn clamp_limit(self) -> u16 {
        self.clamp_limit
    }

    /// Largest sampled signed-phase output magnitude.
    pub(crate) const fn moving_pid_output_peak(self) -> u16 {
        self.moving_pid_output_peak
    }

    /// Largest sampled zero-phase output magnitude.
    pub(crate) const fn zero_pid_output_peak(self) -> u16 {
        self.zero_pid_output_peak
    }

    /// Largest sampled output magnitude across the complete primitive.
    pub(crate) const fn peak_pid_output_abs(self) -> u16 {
        if self.moving_pid_output_peak > self.zero_pid_output_peak {
            self.moving_pid_output_peak
        } else {
            self.zero_pid_output_peak
        }
    }

    /// Classify the accumulated evidence without discarding its raw inputs.
    pub(crate) fn evidence(
        self,
        zero_phase: VelocityCurrentZeroPhase,
    ) -> Result<VelocityCurrentPhaseEvidence, VelocityCurrentGuardError> {
        classify_phase_evidence(
            PidTorqueFluxLimit::new(u32::from(self.clamp_limit))
                .map_err(|_| VelocityCurrentGuardError::InvalidLimit)?,
            PhaseObservation {
                pid_output_peak: self.moving_pid_output_peak,
                status_flags: self.moving_status_flags,
            },
            PhaseObservation {
                pid_output_peak: self.zero_pid_output_peak,
                status_flags: self.zero_status_flags,
            },
            self.capability,
            zero_phase,
        )
    }
}

const _: () = assert!(core::mem::size_of::<VelocityCurrentObserver>() == 16);

/// Verified temporary velocity-current clamp state for one commissioning workflow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityCurrentClampSession {
    operational_limit: PidTorqueFluxLimit,
    probe_limit: PidTorqueFluxLimit,
    probe_installed: bool,
}

impl VelocityCurrentClampSession {
    /// Validate the operational limit and derive its exact probe clamp.
    pub(crate) fn try_new(operational_limit_raw: u32) -> Result<Self, VelocityCurrentGuardError> {
        let operational_limit = PidTorqueFluxLimit::new(operational_limit_raw)
            .map_err(|_| VelocityCurrentGuardError::InvalidLimit)?;
        let probe_limit = derive_probe_limit(operational_limit)?;
        Ok(Self {
            operational_limit,
            probe_limit,
            probe_installed: false,
        })
    }

    /// Validate an explicit commissioning clamp against the operational limit.
    pub(crate) fn try_new_with_limit(
        operational_limit_raw: u32,
        probe_limit_raw: u32,
    ) -> Result<Self, VelocityCurrentGuardError> {
        let operational_limit = PidTorqueFluxLimit::new(operational_limit_raw)
            .map_err(|_| VelocityCurrentGuardError::InvalidLimit)?;
        let probe_limit = PidTorqueFluxLimit::new(probe_limit_raw)
            .map_err(|_| VelocityCurrentGuardError::InvalidLimit)?;
        if operational_limit.raw() == 0
            || probe_limit.raw() == 0
            || probe_limit.raw() > operational_limit.raw()
        {
            return Err(VelocityCurrentGuardError::InvalidLimit);
        }
        Ok(Self {
            operational_limit,
            probe_limit,
            probe_installed: false,
        })
    }

    /// Return the limit that normal controller operation must regain on exit.
    #[cfg(test)]
    pub(crate) const fn operational_limit(self) -> PidTorqueFluxLimit {
        self.operational_limit
    }

    /// Return the temporary commissioning clamp.
    pub(crate) const fn probe_limit(self) -> PidTorqueFluxLimit {
        self.probe_limit
    }

    /// Return whether the latest verified session write installed the probe clamp.
    #[cfg(test)]
    pub(crate) const fn probe_is_installed(self) -> bool {
        self.probe_installed
    }

    /// Install or reapply the probe clamp with complete-word readback verification.
    pub(crate) async fn install_probe_limit<B>(
        &mut self,
        backend: &mut B,
    ) -> Result<(), VelocityCurrentGuardError>
    where
        B: CommissionRegisterIo,
    {
        backend
            .write_register_verified(PID_TORQUE_FLUX_LIMITS, self.probe_limit.register_value())
            .await
            .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
        self.probe_installed = true;
        Ok(())
    }

    /// Restore the operational clamp with complete-word readback verification.
    pub(crate) async fn restore_operational_limit<B>(
        &mut self,
        backend: &mut B,
    ) -> Result<(), VelocityCurrentGuardError>
    where
        B: CommissionRegisterIo,
    {
        backend
            .write_register_verified(
                PID_TORQUE_FLUX_LIMITS,
                self.operational_limit.register_value(),
            )
            .await
            .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
        self.probe_installed = false;
        Ok(())
    }
}

/// Peak PID output and raw status flags observed during one phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PhaseObservation {
    pub(crate) pid_output_peak: u16,
    pub(crate) status_flags: u32,
}

/// Classify current-limit evidence without changing the underlying raw flags.
pub(crate) fn classify_phase_evidence(
    clamp_limit: PidTorqueFluxLimit,
    moving: PhaseObservation,
    zero: PhaseObservation,
    capability: VelocityLimitEvidenceCapability,
    zero_phase: VelocityCurrentZeroPhase,
) -> Result<VelocityCurrentPhaseEvidence, VelocityCurrentGuardError> {
    if clamp_limit == PidTorqueFluxLimit::ZERO {
        return Err(VelocityCurrentGuardError::InvalidLimit);
    }

    let moving_contact = moving.pid_output_peak >= clamp_limit.raw()
        || StatusFlags::from(moving.status_flags).pid_v_output_limit();
    let zero_contact = zero.pid_output_peak >= clamp_limit.raw()
        || StatusFlags::from(zero.status_flags).pid_v_output_limit();
    let contact = if moving_contact {
        VelocityCurrentContact::Moving
    } else if zero_contact {
        match zero_phase {
            VelocityCurrentZeroPhase::Braking => VelocityCurrentContact::Braking,
            VelocityCurrentZeroPhase::Recovery => VelocityCurrentContact::Recovery,
        }
    } else {
        VelocityCurrentContact::None
    };

    Ok(VelocityCurrentPhaseEvidence {
        clamp_limit: clamp_limit.raw(),
        moving_pid_output_peak: moving.pid_output_peak,
        zero_pid_output_peak: zero.pid_output_peak,
        moving_status_flags: moving.status_flags,
        zero_status_flags: zero.status_flags,
        capability,
        contact,
    })
}

/// Reset one observer and clear stale flags immediately before a signed target.
pub(crate) async fn begin_velocity_current_primitive<B>(
    observer: &mut VelocityCurrentObserver,
    backend: &mut B,
    clamp_limit: PidTorqueFluxLimit,
    capability: VelocityLimitEvidenceCapability,
) -> Result<(), VelocityCurrentGuardError>
where
    B: CommissioningBackend,
{
    observer.reset(clamp_limit, capability);
    let flags = backend
        .read_register(STATUS_FLAGS)
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
    let actionable_mask =
        crate::autotune::current_loop_entry_actionable_status_mask(backend.feedback_config());
    if crate::autotune::is_actionable_status(flags, actionable_mask) {
        return Err(VelocityCurrentGuardError::ActionableStatus(
            flags & actionable_mask,
        ));
    }
    crate::tmc_control::interlock::clear_status_flags_via_backend(backend)
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)
}

/// Clear stale flags before recovery while retaining its inherited contract.
pub(crate) async fn restart_velocity_current_primitive<B>(
    observer: &mut VelocityCurrentObserver,
    backend: &mut B,
) -> Result<(), VelocityCurrentGuardError>
where
    B: CommissioningBackend,
{
    observer.reset_existing_contract();
    let flags = backend
        .read_register(STATUS_FLAGS)
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
    let actionable_mask =
        crate::autotune::current_loop_entry_actionable_status_mask(backend.feedback_config());
    if crate::autotune::is_actionable_status(flags, actionable_mask) {
        return Err(VelocityCurrentGuardError::ActionableStatus(
            flags & actionable_mask,
        ));
    }
    crate::tmc_control::interlock::clear_status_flags_via_backend(backend)
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)
}

/// Read and retain the complete status word at the signed-phase boundary.
pub(crate) async fn finish_velocity_current_moving_phase<B>(
    observer: &mut VelocityCurrentObserver,
    backend: &mut B,
) -> Result<(), VelocityCurrentGuardError>
where
    B: CommissioningBackend,
{
    let flags = backend
        .read_register(STATUS_FLAGS)
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
    observer.observe_moving_flags(flags);
    let actionable_mask =
        crate::autotune::current_loop_actionable_status_mask(backend.feedback_config());
    if crate::autotune::is_actionable_status(flags, actionable_mask) {
        return Err(boundary_status_error(flags & actionable_mask));
    }
    Ok(())
}

/// Read and retain the complete status word after confirmed zero-target rest.
pub(crate) async fn finish_velocity_current_zero_phase<B>(
    observer: &mut VelocityCurrentObserver,
    backend: &mut B,
) -> Result<(), VelocityCurrentGuardError>
where
    B: CommissioningBackend,
{
    let flags = backend
        .read_register(STATUS_FLAGS)
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
    observer.observe_zero_flags(flags);
    let actionable_mask =
        crate::autotune::current_loop_actionable_status_mask(backend.feedback_config());
    if crate::autotune::is_actionable_status(flags, actionable_mask) {
        return Err(boundary_status_error(flags & actionable_mask));
    }
    Ok(())
}

/// Read one absolute velocity-PID output sample through the interim selector.
pub(crate) async fn read_velocity_pid_output_abs<B>(
    backend: &mut B,
) -> Result<u16, VelocityCurrentGuardError>
where
    B: CommissionRegisterIo,
{
    backend
        .write_register(
            INTERIM_PIDOUT_TARGET_TORQUE.addr,
            u32::from(INTERIM_PIDOUT_TARGET_TORQUE.sub_addr),
        )
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
    backend
        .read_register(INTERIM_PIDOUT_TARGET_TORQUE.data)
        .await
        .map(|value| (value as u16 as i16).unsigned_abs())
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)
}

/// Read one absolute velocity-PID input sample through the interim selector.
pub(crate) async fn read_velocity_pid_input_abs<B>(
    backend: &mut B,
) -> Result<u16, VelocityCurrentGuardError>
where
    B: CommissionRegisterIo,
{
    backend
        .write_register(
            INTERIM_PIDIN_TARGET_TORQUE.addr,
            u32::from(INTERIM_PIDIN_TARGET_TORQUE.sub_addr),
        )
        .await
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)?;
    backend
        .read_register(INTERIM_PIDIN_TARGET_TORQUE.data)
        .await
        .map(|value| (value as u16 as i16).unsigned_abs())
        .map_err(|_| VelocityCurrentGuardError::RegisterIo)
}
