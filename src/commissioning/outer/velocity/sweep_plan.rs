use crate::autotune::TmcMechanicalRpm;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::pid::{PidPGain, RegisterGain};
use heapless::Vec;

use super::recovery::{
    RECOVERY_STABILIZATION_US, RungOriginRecoveryPlanError, STABILIZED_RECOVERY_SCHEDULE_CODE,
    plan_rung_origin_recovery,
};
use super::velocity_clamp::{VelocityLimitEvidenceCapability, derive_probe_limit};

/// Numerator of the runtime velocity-envelope proportional margin.
pub const ENVELOPE_PROPORTIONAL_NUM: u32 = 3;
/// Denominator of the runtime velocity-envelope proportional margin.
pub const ENVELOPE_PROPORTIONAL_DEN: u32 = 2;
/// Absolute runtime velocity-envelope margin in mechanical millirevolutions per second.
pub const ENVELOPE_ABSOLUTE_MARGIN_MREV_S: u32 = 2_000;
/// Window after a primitive starts during which the transient velocity cap applies, in
/// microseconds.
pub const TRANSIENT_ALLOWANCE_US: u64 = 24_000;
/// Numerator of the transient velocity-cap proportional margin.
pub const TRANSIENT_PROPORTIONAL_NUM: u32 = 5;
/// Denominator of the transient velocity-cap proportional margin.
pub const TRANSIENT_PROPORTIONAL_DEN: u32 = 2;
/// Absolute transient velocity-cap margin in mechanical millirevolutions per second.
pub const TRANSIENT_ABSOLUTE_MARGIN_MREV_S: u32 = 4_000;
/// Fixed duration of one moving observation.
pub const MOVING_STROKE_US: u32 = 512_000;
/// Proportional schema for the shared Proportional velocity sweep planner.
pub const PROPORTIONAL_PLAN_SCHEMA_REVISION: u16 = 14;
/// First Proportional schema with cap-safe target quantization.
pub const FIRST_CAP_SAFE_PROPORTIONAL_SCHEMA_REVISION: u16 = 10;

/// Where one moving stroke sits relative to the position it started from.
///
/// The travel envelope bounds distance from the start, which equals displacement only when
/// a stroke travels one way.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum StrokeGeometry {
    /// The stroke starts at the origin and runs one way; peak excursion equals displacement.
    OriginAnchored = 0,
    /// The stroke runs symmetrically about the origin; peak excursion is half the
    /// displacement, and each direction settles at symmetric positions.
    OriginCentred = 1,
}

impl StrokeGeometry {
    /// Furthest the axis gets from its start during one stroke.
    pub const fn peak_excursion_mrev(self, stroke_displacement_mrev: u32) -> u32 {
        match self {
            Self::OriginAnchored => stroke_displacement_mrev,
            Self::OriginCentred => stroke_displacement_mrev / 2,
        }
    }

    const fn displacement_for_excursion(self, excursion_mrev: u64) -> u64 {
        match self {
            Self::OriginAnchored => excursion_mrev,
            Self::OriginCentred => excursion_mrev * 2,
        }
    }

    /// Stable wire code.
    pub const fn report_code(self) -> u8 {
        self as u8
    }
}
/// Fixed duration of one zero-target settle.
pub const ZERO_SETTLE_US: u32 = 500_000;
/// Sampling period used by moving and zero-target velocity observations.
pub const VELOCITY_SWEEP_SAMPLE_PERIOD_US: u32 = 1_000;
/// Minimum zero-target capture retained for the shortest stationarity suffix.
pub const MIN_SETTLE_CAPTURE_US: u32 =
    crate::autotune::velocity_measurement::VELOCITY_SUFFIX_LENGTHS[0] as u32
        * VELOCITY_SWEEP_SAMPLE_PERIOD_US;
/// Maximum cooperative analysis budget validated independently of the rest window.
pub const SETTLE_ANALYSIS_BUDGET_US: u32 = 236_000;
/// Minimum number of distinct gains that can support the required evidence geometry.
pub const MIN_STRUCTURAL_RUNGS: usize = 5;
/// Maximum audited gain count in one deterministic sweep.
pub const MAX_SWEEP_RUNGS: usize = 32;
/// Independent observations collected in each direction at every rung.
pub const OBSERVATIONS_PER_DIRECTION_PER_RUNG: u16 = 4;
/// Signed directions collected at every gain rung.
pub const SWEEP_DIRECTION_COUNT: u16 = 2;
/// Total moving observations collected at every gain rung.
pub const SWEEP_SLOT_COUNT: u8 =
    (OBSERVATIONS_PER_DIRECTION_PER_RUNG * SWEEP_DIRECTION_COUNT) as u8;
/// Numerator of the usable current-headroom boundary.
pub const CURRENT_HEADROOM_NUM: u32 = 9;
/// Denominator of the usable current-headroom boundary.
pub const CURRENT_HEADROOM_DEN: u32 = 10;

const MICROS_PER_SECOND: u64 = 1_000_000;
/// Maximum measured time between encoder reads before wrap-safe interpretation is refused.
///
/// Twenty milliseconds is the largest 5 ms-aligned bound that keeps the 10,000 mrev/s host
/// ceiling below encoder observability.
pub const MAX_SAFETY_OBSERVATION_GAP_US: u64 = 20_000;
const VELOCITY_P_DIVISOR: u32 = 256;
const HALF_OCTAVE_NUM: u32 = 181;
const HALF_OCTAVE_DEN_SHIFT: u32 = 7;

/// The effective ceiling is the host-declared machine ceiling.
pub const EFFECTIVE_CEILING_MACHINE: u8 = 0;
/// The effective ceiling is limited by wrap-safe encoder observability.
pub const EFFECTIVE_CEILING_ENCODER: u8 = 1;

/// `p_top` is bound by the current-limit-derived candidate
/// (`VELOCITY_P_DIVISOR * usable_torque_limit / target`).
pub(crate) const CEILING_BINDING_CURRENT_LIMIT: u8 = 0;
/// `p_top` is bound by the [`PidPGain::MAX`] representability clamp instead of
/// the current-limit-derived candidate.
pub(crate) const CEILING_BINDING_REPRESENTABILITY_CLAMP: u8 = 1;
/// The requested amplitude exceeded the normal envelope-compatible amplitude.
pub const CLAMP_ENVELOPE: u16 = 1 << 0;
/// The requested amplitude exceeded the declared moving-travel budget.
pub const CLAMP_TRAVEL: u16 = 1 << 1;
/// A firmware-owned observability or representation ceiling reduced the machine ceiling.
pub const CLAMP_EFFECTIVE_CEILING: u16 = 1 << 2;

/// Origin of the host's requested velocity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RequestedVelocitySource {
    /// The operator supplied `TUNE_VELOCITY`.
    Explicit,
    /// The host derived the envelope-compatible default.
    Defaulted,
}

/// Host-planned motion and geometry inputs for one velocity sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepRequest {
    /// Requested mechanical velocity amplitude.
    pub requested_velocity_mrev_s: u32,
    /// Kinematics-derived absolute machine velocity ceiling.
    pub machine_velocity_ceiling_mrev_s: u32,
    /// Whether the requested amplitude was explicit or defaulted.
    pub requested_velocity_source: RequestedVelocitySource,
    /// Complete displacement window for one moving stroke and its settle.
    pub max_stroke_travel_mrev: u32,
    /// Portion of the displacement window reserved for braking and settle drift.
    pub settle_travel_reserve_mrev: u32,
    /// Negative unwrapped position headroom from the installed-tuning anchor.
    pub negative_position_headroom_mrev: u32,
    /// Positive unwrapped position headroom from the installed-tuning anchor.
    pub positive_position_headroom_mrev: u32,
    /// Host-declared per-primitive safety cap.
    ///
    /// Bounds one moving-plus-settle primitive, not the aggregate campaign duration.
    pub max_duration_ms: u16,
    /// Nominate only rungs with P strictly below this raw gain; 0 is unconstrained.
    pub nomination_p_below: u16,
}

/// Complete immutable motion plan produced before the first sweep stroke.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepPlan {
    /// Evidence schema governing this plan's terminal semantics.
    pub schema_revision: u16,
    /// Host-requested velocity amplitude.
    pub requested_velocity_mrev_s: u32,
    /// Frozen velocity amplitude used for every rung.
    pub planned_velocity_mrev_s: u32,
    /// Origin of the requested amplitude.
    pub requested_velocity_source: RequestedVelocitySource,
    /// Bitset describing every amplitude reduction.
    pub clamp_flags: u16,
    /// Lowest machine, observability, or representation ceiling.
    pub effective_ceiling_mrev_s: u32,
    /// Source code for the binding effective ceiling.
    pub effective_ceiling_source: u8,
    /// Frozen positive TMC velocity-target register magnitude.
    pub target_velocity_rpm: TmcMechanicalRpm,
    /// Smallest gain producing one nonzero controller-output quantum.
    pub p_start: PidPGain,
    /// Largest gain at the inclusive usable-current boundary.
    pub p_top: PidPGain,
    /// Post-dedupe anchored half-octave gains; unused entries are zero.
    pub gains: [PidPGain; MAX_SWEEP_RUNGS],
    /// Number of valid entries in `gains`.
    pub rung_count: u8,
    /// Independent observations planned in each direction per rung.
    pub observations_per_direction: u16,
    /// Fixed moving-stroke duration.
    pub moving_stroke_us: u32,
    /// Fixed zero-target settle duration.
    pub zero_settle_us: u32,
    /// Exact nominal workflow duration for the fixed schedule.
    pub nominal_workflow_ms: u32,
    /// Maximum planned duration; equal to nominal while the schedule has no extension.
    pub maximum_workflow_ms: u32,
    /// Inclusive reserve-derived origin band used after fully acquired rungs.
    pub recovery_origin_band_counts: u32,
    /// Conservative nominal duration reserved for one recovery slot.
    pub recovery_nominal_slot_us: u32,
    /// Host-declared maximum duration reserved for one recovery slot.
    pub recovery_maximum_slot_us: u32,
    /// Maximum number of recovery slots reserved by this plan.
    pub recovery_slot_count: u8,
    /// Complete per-primitive displacement window.
    pub max_stroke_travel_mrev: u32,
    /// Reserved braking and settle displacement.
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom.
    pub negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom.
    pub positive_position_headroom_mrev: u32,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive nine-tenths torque-target boundary.
    pub usable_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
}

/// Compact immutable plan retained after the complete pre-motion report is emitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct VelocitySweepExecutionPlan {
    /// Digest of every frozen request, bound, gain, duration, and geometry field.
    pub plan_digest: u64,
    /// Evidence schema governing this plan's terminal semantics.
    pub schema_revision: u16,
    /// Frozen encoder-domain velocity amplitude.
    pub planned_velocity_mrev_s: u32,
    /// Frozen positive TMC mechanical-RPM target.
    pub target_velocity_rpm: TmcMechanicalRpm,
    /// Structural ladder anchor.
    pub p_start: PidPGain,
    /// Explicit structural ladder top.
    pub p_top: PidPGain,
    /// Number of post-dedupe rungs.
    pub rung_count: u8,
    /// Independent observations per direction and rung.
    pub observations_per_direction: u16,
    /// Configured hard torque-target limit.
    pub hard_torque_limit: u16,
    /// Inclusive usable torque-target boundary.
    pub usable_torque_limit: u16,
    /// Available authority for proving absence of transient clamp contact.
    pub(crate) limit_evidence_capability: VelocityLimitEvidenceCapability,
}

impl VelocitySweepExecutionPlan {
    /// Sentinel used before a trace-gated sweep has an admitted immutable plan.
    pub const EMPTY: Self = Self {
        plan_digest: 0,
        schema_revision: 0,
        planned_velocity_mrev_s: 0,
        target_velocity_rpm: TmcMechanicalRpm::from_register(0),
        p_start: PidPGain::ZERO,
        p_top: PidPGain::ZERO,
        rung_count: 0,
        observations_per_direction: 0,
        hard_torque_limit: 0,
        usable_torque_limit: 0,
        limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
    };

    /// Whether command admission has installed a usable plan.
    pub const fn is_present(self) -> bool {
        self.rung_count != 0
    }

    /// Regenerate one exact post-dedupe gain without retaining the ladder array.
    pub fn gain_at(self, rung_index: u8) -> Option<PidPGain> {
        gain_at(
            self.p_start.raw(),
            self.p_top.raw(),
            self.rung_count,
            rung_index,
        )
    }
}

const _: () = assert!(core::mem::size_of::<VelocitySweepExecutionPlan>() == 32);

impl VelocitySweepPlan {
    /// Hash every frozen plan field and emitted post-dedupe gain exactly once.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        macro_rules! hash_value {
            ($value:expr) => {
                for byte in $value.to_le_bytes() {
                    hash = (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
                }
            };
        }
        hash_value!(self.requested_velocity_mrev_s);
        hash_value!(self.schema_revision);
        hash_value!(self.planned_velocity_mrev_s);
        hash_value!(self.requested_velocity_source as u8);
        hash_value!(self.clamp_flags);
        hash_value!(self.effective_ceiling_mrev_s);
        hash_value!(self.effective_ceiling_source);
        hash_value!(self.target_velocity_rpm.raw());
        hash_value!(self.p_start.raw());
        hash_value!(self.p_top.raw());
        hash_value!(self.rung_count);
        for gain in self.gains.iter().take(usize::from(self.rung_count)) {
            hash_value!(gain.raw());
        }
        hash_value!(self.observations_per_direction);
        hash_value!(self.moving_stroke_us);
        hash_value!(self.zero_settle_us);
        hash_value!(self.nominal_workflow_ms);
        hash_value!(self.maximum_workflow_ms);
        hash_value!(self.recovery_origin_band_counts);
        if self.schema_revision == PROPORTIONAL_PLAN_SCHEMA_REVISION {
            hash_value!(STABILIZED_RECOVERY_SCHEDULE_CODE);
        }
        hash_value!(self.recovery_nominal_slot_us);
        hash_value!(self.recovery_maximum_slot_us);
        hash_value!(self.recovery_slot_count);
        hash_value!(self.max_stroke_travel_mrev);
        hash_value!(self.settle_travel_reserve_mrev);
        hash_value!(self.negative_position_headroom_mrev);
        hash_value!(self.positive_position_headroom_mrev);
        hash_value!(self.hard_torque_limit);
        hash_value!(self.usable_torque_limit);
        hash_value!(self.limit_evidence_capability as u8);
        hash
    }

    /// Compact this complete report plan into the state needed after emission.
    pub const fn execution_plan(self, plan_digest: u64) -> VelocitySweepExecutionPlan {
        VelocitySweepExecutionPlan {
            plan_digest,
            schema_revision: self.schema_revision,
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            target_velocity_rpm: self.target_velocity_rpm,
            p_start: self.p_start,
            p_top: self.p_top,
            rung_count: self.rung_count,
            observations_per_direction: self.observations_per_direction,
            hard_torque_limit: self.hard_torque_limit,
            usable_torque_limit: self.usable_torque_limit,
            limit_evidence_capability: self.limit_evidence_capability,
        }
    }

    /// Build the exact pre-motion evidence record for this immutable plan.
    pub fn report(
        self,
        run_sequence: u32,
        evidence_sequence: u16,
    ) -> crate::commissioning::types::VelocitySweepPlanned {
        crate::commissioning::types::VelocitySweepPlanned {
            run_sequence,
            evidence_sequence,
            plan_digest: self.digest(),
            requested_velocity_mrev_s: self.requested_velocity_mrev_s,
            requested_velocity_source: match self.requested_velocity_source {
                RequestedVelocitySource::Explicit => 0,
                RequestedVelocitySource::Defaulted => 1,
            },
            planned_velocity_mrev_s: self.planned_velocity_mrev_s,
            effective_ceiling_mrev_s: self.effective_ceiling_mrev_s,
            clamp_flags: self.clamp_flags,
            effective_ceiling_source: self.effective_ceiling_source,
            target_velocity_rpm: self.target_velocity_rpm.raw(),
            p_start: self.p_start.raw(),
            p_top: self.p_top.raw(),
            rung_count: self.rung_count,
            observations_per_direction: self.observations_per_direction,
            moving_stroke_us: self.moving_stroke_us,
            zero_settle_us: self.zero_settle_us,
            nominal_workflow_ms: self.nominal_workflow_ms,
            maximum_workflow_ms: self.maximum_workflow_ms,
            recovery_origin_band_counts: self.recovery_origin_band_counts,
            recovery_nominal_slot_us: self.recovery_nominal_slot_us,
            recovery_maximum_slot_us: self.recovery_maximum_slot_us,
            recovery_slot_count: self.recovery_slot_count,
            max_stroke_travel_mrev: self.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: self.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: self.negative_position_headroom_mrev,
            positive_position_headroom_mrev: self.positive_position_headroom_mrev,
            hard_torque_limit: self.hard_torque_limit,
            usable_torque_limit: self.usable_torque_limit,
        }
    }
}

/// Failure to construct a safe deterministic sweep before motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum VelocityPlanError {
    /// A required request or hardware scale was zero or inconsistent.
    InvalidRequest,
    /// Positive travel remained after the reserve, but its rate floored to zero.
    ZeroMovingTravelAmplitude,
    /// The current-limited gain range cannot hold five distinct rungs.
    InsufficientStructuralSpan,
    /// The planned observation family exceeds the audited confidence table.
    ConfidenceFamilyTooLarge,
    /// Checked planner arithmetic exceeded its representation.
    Arithmetic,
}

/// Convert a positive millirevolutions-per-second cap to a nonexceeding RPM target.
pub(crate) fn cap_safe_target_rpm(planned_velocity_mrev_s: u32) -> Result<i32, VelocityPlanError> {
    let target_rpm = planned_velocity_mrev_s
        .checked_mul(60)
        .ok_or(VelocityPlanError::Arithmetic)?
        / 1_000;
    if target_rpm == 0 {
        return Err(VelocityPlanError::ZeroMovingTravelAmplitude);
    }
    i32::try_from(target_rpm).map_err(|_| VelocityPlanError::Arithmetic)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RepresentabilityCeilings {
    encoder_observable_mrev_s: u32,
}

/// Construct one immutable velocity sweep from host geometry and firmware limits.
pub fn plan_velocity_sweep(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_flux_limit: u16,
    limit_evidence_capability: u32,
) -> Result<VelocitySweepPlan, VelocityPlanError> {
    plan_velocity_sweep_with_limit(
        request,
        encoder_counts_per_rev,
        torque_flux_limit,
        limit_evidence_capability,
        super::reproduction::PROPORTIONAL_EVIDENCE_SCHEMA_REVISION,
        false,
    )
}

/// Construct the nonretained generic Proportional P search under operational authority.
pub fn plan_proportional_velocity_sweep(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_flux_limit: u16,
    limit_evidence_capability: u32,
) -> Result<VelocitySweepPlan, VelocityPlanError> {
    plan_velocity_sweep_with_limit(
        request,
        encoder_counts_per_rev,
        torque_flux_limit,
        limit_evidence_capability,
        PROPORTIONAL_PLAN_SCHEMA_REVISION,
        true,
    )
}

fn plan_velocity_sweep_with_limit(
    request: VelocitySweepRequest,
    encoder_counts_per_rev: u32,
    torque_flux_limit: u16,
    limit_evidence_capability: u32,
    schema_revision: u16,
    use_operational_limit: bool,
) -> Result<VelocitySweepPlan, VelocityPlanError> {
    if request.requested_velocity_mrev_s == 0
        || request.machine_velocity_ceiling_mrev_s == 0
        || request.negative_position_headroom_mrev == 0
        || request.positive_position_headroom_mrev == 0
        || request.max_duration_ms == 0
        || torque_flux_limit == 0
    {
        return Err(VelocityPlanError::InvalidRequest);
    }

    let ceilings = representability_ceilings(encoder_counts_per_rev)?;
    let candidates = [
        (
            request.machine_velocity_ceiling_mrev_s,
            EFFECTIVE_CEILING_MACHINE,
        ),
        (
            ceilings.encoder_observable_mrev_s,
            EFFECTIVE_CEILING_ENCODER,
        ),
    ];
    let (effective_ceiling_mrev_s, effective_ceiling_source) = candidates
        .into_iter()
        .min_by_key(|candidate| candidate.0)
        .ok_or(VelocityPlanError::InvalidRequest)?;
    let envelope_velocity_cap = envelope_cap(effective_ceiling_mrev_s)?;
    let travel_velocity_cap = travel_cap(
        StrokeGeometry::OriginAnchored,
        request.max_stroke_travel_mrev,
        request.settle_travel_reserve_mrev,
        MOVING_STROKE_US,
    )?;
    let planned_velocity_mrev_s = request
        .requested_velocity_mrev_s
        .min(envelope_velocity_cap)
        .min(travel_velocity_cap);
    if planned_velocity_mrev_s == 0 {
        return Err(VelocityPlanError::ZeroMovingTravelAmplitude);
    }

    let mut clamp_flags = 0;
    if request.requested_velocity_mrev_s > envelope_velocity_cap {
        clamp_flags |= CLAMP_ENVELOPE;
    }
    if request.requested_velocity_mrev_s > travel_velocity_cap {
        clamp_flags |= CLAMP_TRAVEL;
    }
    if effective_ceiling_mrev_s < request.machine_velocity_ceiling_mrev_s {
        clamp_flags |= CLAMP_EFFECTIVE_CEILING;
    }

    let cap_safe_schemas =
        FIRST_CAP_SAFE_PROPORTIONAL_SCHEMA_REVISION..=PROPORTIONAL_PLAN_SCHEMA_REVISION;
    let target_velocity_rpm = if cap_safe_schemas.contains(&schema_revision) {
        let raw = cap_safe_target_rpm(planned_velocity_mrev_s)?;
        let executed_mrev_numerator = u64::try_from(raw)
            .map_err(|_| VelocityPlanError::Arithmetic)?
            .checked_mul(1_000)
            .ok_or(VelocityPlanError::Arithmetic)?;
        let planned_mrev_numerator = u64::from(planned_velocity_mrev_s)
            .checked_mul(60)
            .ok_or(VelocityPlanError::Arithmetic)?;
        if executed_mrev_numerator > planned_mrev_numerator {
            return Err(VelocityPlanError::InvalidRequest);
        }
        TmcMechanicalRpm::from_register(raw)
    } else {
        TmcMechanicalRpm::from_mrev_s_nearest(planned_velocity_mrev_s)
            .map_err(|_| VelocityPlanError::ZeroMovingTravelAmplitude)?
    };
    let recovery = plan_rung_origin_recovery(
        crate::commissioning::outer::OuterMotionBudget {
            requested_velocity_mrev_s: request.requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s: request.machine_velocity_ceiling_mrev_s,
            requested_velocity_source: request.requested_velocity_source,
            max_stroke_travel_mrev: request.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: request.negative_position_headroom_mrev,
            positive_position_headroom_mrev: request.positive_position_headroom_mrev,
            max_duration_ms: request.max_duration_ms,
            homing_speed_mrev_s: 0,
            max_accel_mrev_s2: 0,
        },
        encoder_counts_per_rev,
        planned_velocity_mrev_s,
    )
    .map_err(map_recovery_plan_error)?;
    let (p_start, p_top, usable_torque_limit) = if use_operational_limit {
        let (p_start, p_top, _ceiling_binding_source) =
            current_gain_span_with_limit(target_velocity_rpm, torque_flux_limit)?;
        (p_start, p_top, torque_flux_limit)
    } else {
        let (p_start, p_top, usable_torque_limit, _ceiling_binding_source) =
            current_gain_span(target_velocity_rpm, torque_flux_limit)?;
        (p_start, p_top, usable_torque_limit)
    };
    let ladder = build_gain_ladder(p_start.raw(), p_top.raw())?;
    let rung_count = u8::try_from(ladder.len()).map_err(|_| VelocityPlanError::Arithmetic)?;
    let family_size = usize::from(rung_count)
        .checked_mul(usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG))
        .ok_or(VelocityPlanError::Arithmetic)?;
    if family_size > usize::from(crate::autotune::velocity_measurement::MAX_CONFIDENCE_FAMILY_SIZE)
    {
        return Err(VelocityPlanError::ConfidenceFamilyTooLarge);
    }
    let primitive_us = MOVING_STROKE_US
        .checked_add(ZERO_SETTLE_US)
        .and_then(|value| value.checked_add(SETTLE_ANALYSIS_BUDGET_US))
        .ok_or(VelocityPlanError::Arithmetic)?;
    if primitive_us.div_ceil(1_000) > u32::from(request.max_duration_ms) {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let workflow_us = u32::from(rung_count)
        .checked_mul(u32::from(SWEEP_SLOT_COUNT))
        .and_then(|value| value.checked_mul(primitive_us))
        .ok_or(VelocityPlanError::Arithmetic)?;
    let recovery_nominal_slot_us = if schema_revision == PROPORTIONAL_PLAN_SCHEMA_REVISION {
        recovery
            .nominal_slot_us
            .checked_add(RECOVERY_STABILIZATION_US)
            .ok_or(VelocityPlanError::Arithmetic)?
    } else {
        recovery.nominal_slot_us
    };
    let nominal_workflow_us = workflow_us
        .checked_add(
            u32::from(rung_count)
                .checked_mul(recovery_nominal_slot_us)
                .ok_or(VelocityPlanError::Arithmetic)?,
        )
        .ok_or(VelocityPlanError::Arithmetic)?;
    let maximum_workflow_us = workflow_us
        .checked_add(
            u32::from(rung_count)
                .checked_mul(recovery.maximum_slot_us)
                .ok_or(VelocityPlanError::Arithmetic)?,
        )
        .ok_or(VelocityPlanError::Arithmetic)?;
    let nominal_workflow_ms = nominal_workflow_us.div_ceil(1_000);
    let maximum_workflow_ms = maximum_workflow_us.div_ceil(1_000);
    let mut gains = [PidPGain::ZERO; MAX_SWEEP_RUNGS];
    gains[..ladder.len()].copy_from_slice(ladder.as_slice());

    let limit_evidence_capability = match limit_evidence_capability {
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT => {
            VelocityLimitEvidenceCapability::BestEffortPolling
        }
        super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_LATCHED_FLAG => {
            VelocityLimitEvidenceCapability::LatchedFlag
        }
        _ => return Err(VelocityPlanError::InvalidRequest),
    };

    Ok(VelocitySweepPlan {
        schema_revision,
        requested_velocity_mrev_s: request.requested_velocity_mrev_s,
        planned_velocity_mrev_s,
        requested_velocity_source: request.requested_velocity_source,
        clamp_flags,
        effective_ceiling_mrev_s,
        effective_ceiling_source,
        target_velocity_rpm,
        p_start,
        p_top,
        gains,
        rung_count,
        observations_per_direction: OBSERVATIONS_PER_DIRECTION_PER_RUNG,
        moving_stroke_us: MOVING_STROKE_US,
        zero_settle_us: ZERO_SETTLE_US,
        nominal_workflow_ms,
        maximum_workflow_ms,
        recovery_origin_band_counts: recovery.origin_band_counts,
        recovery_nominal_slot_us,
        recovery_maximum_slot_us: recovery.maximum_slot_us,
        recovery_slot_count: rung_count,
        max_stroke_travel_mrev: request.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: request.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: request.negative_position_headroom_mrev,
        positive_position_headroom_mrev: request.positive_position_headroom_mrev,
        hard_torque_limit: torque_flux_limit,
        usable_torque_limit,
        limit_evidence_capability,
    })
}

fn map_recovery_plan_error(error: RungOriginRecoveryPlanError) -> VelocityPlanError {
    match error {
        RungOriginRecoveryPlanError::InvalidRequest
        | RungOriginRecoveryPlanError::ExceedsPrimitiveCap => VelocityPlanError::InvalidRequest,
        RungOriginRecoveryPlanError::Arithmetic => VelocityPlanError::Arithmetic,
    }
}

fn envelope_cap(effective_ceiling_mrev_s: u32) -> Result<u32, VelocityPlanError> {
    if effective_ceiling_mrev_s <= ENVELOPE_ABSOLUTE_MARGIN_MREV_S {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let proportional = u64::from(effective_ceiling_mrev_s)
        .checked_mul(u64::from(ENVELOPE_PROPORTIONAL_DEN))
        .ok_or(VelocityPlanError::Arithmetic)?
        / u64::from(ENVELOPE_PROPORTIONAL_NUM);
    let absolute = effective_ceiling_mrev_s - ENVELOPE_ABSOLUTE_MARGIN_MREV_S;
    Ok(u32::try_from(proportional)
        .map_err(|_| VelocityPlanError::Arithmetic)?
        .min(absolute))
}

fn travel_cap(
    geometry: StrokeGeometry,
    full_window_mrev: u32,
    settle_reserve_mrev: u32,
    moving_stroke_us: u32,
) -> Result<u32, VelocityPlanError> {
    if moving_stroke_us == 0 || settle_reserve_mrev >= full_window_mrev {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let moving_window =
        geometry.displacement_for_excursion(u64::from(full_window_mrev - settle_reserve_mrev));
    let cap = moving_window
        .checked_mul(MICROS_PER_SECOND)
        .ok_or(VelocityPlanError::Arithmetic)?
        / u64::from(moving_stroke_us);
    if cap == 0 {
        return Err(VelocityPlanError::ZeroMovingTravelAmplitude);
    }
    u32::try_from(cap).map_err(|_| VelocityPlanError::Arithmetic)
}

fn representability_ceilings(
    encoder_counts_per_rev: u32,
) -> Result<RepresentabilityCeilings, VelocityPlanError> {
    if encoder_counts_per_rev == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let quarter_turn_fault_counts = u64::from(encoder_counts_per_rev).div_ceil(4);
    let max_nonfaulting_delta_counts = quarter_turn_fault_counts
        .checked_sub(1)
        .ok_or(VelocityPlanError::InvalidRequest)?;
    let encoder_denominator = u64::from(encoder_counts_per_rev)
        .checked_mul(MAX_SAFETY_OBSERVATION_GAP_US)
        .ok_or(VelocityPlanError::Arithmetic)?;
    let encoder_observable_mrev_s = max_nonfaulting_delta_counts
        .checked_mul(1_000_000_000)
        .ok_or(VelocityPlanError::Arithmetic)?
        / encoder_denominator;
    Ok(RepresentabilityCeilings {
        encoder_observable_mrev_s: u32::try_from(encoder_observable_mrev_s)
            .map_err(|_| VelocityPlanError::Arithmetic)?,
    })
}

/// Resolve the effective velocity ceiling that bounds every outer velocity
/// primitive: the lower of the host machine ceiling and the velocity the outer
/// safety sampler can observe without a quarter-turn fault.
///
/// # Errors
///
/// Returns [`VelocityPlanError::InvalidRequest`] when `encoder_counts_per_rev`
/// is zero and [`VelocityPlanError::Arithmetic`] when the encoder-observable
/// bound overflows.
pub(crate) fn effective_velocity_ceiling_mrev_s(
    machine_velocity_ceiling_mrev_s: u32,
    encoder_counts_per_rev: u32,
) -> Result<u32, VelocityPlanError> {
    let ceilings = representability_ceilings(encoder_counts_per_rev)?;
    Ok(machine_velocity_ceiling_mrev_s.min(ceilings.encoder_observable_mrev_s))
}

/// Resolve the current-representable `(p_start, p_top)` gain span and the
/// derived usable torque limit from the requested target and hard torque limit.
///
/// Shared by the velocity sweep and the breakaway campaign. Returns `(p_start, p_top)` and
/// the `CEILING_BINDING_*` discriminant naming which candidate bound `p_top`; see
/// [`current_gain_span_with_limit`].
pub(crate) fn current_gain_span(
    target_velocity_rpm: TmcMechanicalRpm,
    hard_torque_limit: u16,
) -> Result<(PidPGain, PidPGain, u16, u8), VelocityPlanError> {
    let target = target_velocity_rpm.raw().unsigned_abs();
    if target == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let operational_limit = PidTorqueFluxLimit::new(u32::from(hard_torque_limit))
        .map_err(|_| VelocityPlanError::InvalidRequest)?;
    let usable = u32::from(
        derive_probe_limit(operational_limit)
            .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?
            .raw(),
    );
    let (p_start, p_top, ceiling_binding_source) =
        current_gain_span_with_limit(target_velocity_rpm, usable as u16)?;
    Ok((p_start, p_top, usable as u16, ceiling_binding_source))
}

fn current_gain_span_with_limit(
    target_velocity_rpm: TmcMechanicalRpm,
    usable_torque_limit: u16,
) -> Result<(PidPGain, PidPGain, u8), VelocityPlanError> {
    let target = target_velocity_rpm.raw().unsigned_abs();
    if target == 0 || usable_torque_limit == 0 {
        return Err(VelocityPlanError::InvalidRequest);
    }
    let p_start_raw = VELOCITY_P_DIVISOR.div_ceil(target);
    let current_limit_p_top_raw = VELOCITY_P_DIVISOR
        .checked_mul(u32::from(usable_torque_limit))
        .ok_or(VelocityPlanError::Arithmetic)?
        / target;
    let representability_clamp_raw = u32::from(PidPGain::MAX.raw());
    let ceiling_binding_source = if current_limit_p_top_raw > representability_clamp_raw {
        CEILING_BINDING_REPRESENTABILITY_CLAMP
    } else {
        CEILING_BINDING_CURRENT_LIMIT
    };
    let p_top_raw = current_limit_p_top_raw.min(representability_clamp_raw);
    if p_start_raw > p_top_raw || p_start_raw > u32::from(PidPGain::MAX.raw()) {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    let p_start = PidPGain::new(p_start_raw as u16)
        .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
    let p_top = PidPGain::new(p_top_raw as u16)
        .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
    Ok((p_start, p_top, ceiling_binding_source))
}

/// Round a raw octave-aligned gain up to its geometric-mean half-octave
/// point (`raw * sqrt(2)`, approximated as `raw * HALF_OCTAVE_NUM /
/// 2^HALF_OCTAVE_DEN_SHIFT`, rounded up). Returns `None` on overflow.
///
/// Performs no structural-minimum or family-size validation.
pub(crate) fn half_octave_candidate(raw: u16) -> Option<u16> {
    let numerator = u64::from(raw).checked_mul(u64::from(HALF_OCTAVE_NUM))?;
    let rounding = (1_u64 << HALF_OCTAVE_DEN_SHIFT) - 1;
    let candidate = numerator.checked_add(rounding)? >> HALF_OCTAVE_DEN_SHIFT;
    u16::try_from(candidate).ok()
}

/// Build the anchored post-dedupe half-octave gain ladder between two raw gains.
pub(crate) fn build_gain_ladder(
    p_start_raw: u16,
    p_top_raw: u16,
) -> Result<Vec<PidPGain, MAX_SWEEP_RUNGS>, VelocityPlanError> {
    build_gain_ladder_generic(p_start_raw, p_top_raw, true)
}

/// Generic core of [`build_gain_ladder`], working over any [`RegisterGain`].
///
/// `include_exact_top` force-appends the exact `top_raw` when the half-octave progression
/// does not land on it.
pub(crate) fn build_gain_ladder_generic<G: RegisterGain>(
    start_raw: u16,
    top_raw: u16,
    include_exact_top: bool,
) -> Result<Vec<G, MAX_SWEEP_RUNGS>, VelocityPlanError> {
    if start_raw == 0 || start_raw > top_raw || top_raw > G::MAX.raw() {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    let start = u64::from(start_raw);
    let top = u64::from(top_raw);
    let mut gains = Vec::new();
    let mut previous = 0_u64;

    for octave in 0..16 {
        let Some(even_candidate) = start.checked_shl(octave) else {
            continue;
        };
        let Some(even_raw) = u16::try_from(even_candidate).ok() else {
            continue;
        };
        let Some(odd_candidate) = half_octave_candidate(even_raw) else {
            continue;
        };
        for candidate in [even_candidate, u64::from(odd_candidate)] {
            if candidate > top || candidate <= previous {
                continue;
            }
            let gain = G::new(candidate as u16)
                .map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
            gains
                .push(gain)
                .map_err(|_| VelocityPlanError::ConfidenceFamilyTooLarge)?;
            previous = candidate;
        }
    }
    if include_exact_top && previous != top {
        let top_gain =
            G::new(top_raw).map_err(|_| VelocityPlanError::InsufficientStructuralSpan)?;
        gains
            .push(top_gain)
            .map_err(|_| VelocityPlanError::ConfidenceFamilyTooLarge)?;
    }
    if gains.len() < MIN_STRUCTURAL_RUNGS {
        return Err(VelocityPlanError::InsufficientStructuralSpan);
    }
    if gains.len() * usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG)
        > usize::from(crate::autotune::velocity_measurement::MAX_CONFIDENCE_FAMILY_SIZE)
    {
        return Err(VelocityPlanError::ConfidenceFamilyTooLarge);
    }
    Ok(gains)
}

/// Regenerate one anchored post-dedupe ladder gain by index.
pub fn gain_at(
    p_start_raw: u16,
    p_top_raw: u16,
    rung_count: u8,
    rung_index: u8,
) -> Option<PidPGain> {
    gain_at_generic(p_start_raw, p_top_raw, rung_count, rung_index, true)
}

/// Generic core of [`gain_at`], working over any [`RegisterGain`].
///
/// See [`build_gain_ladder_generic`] for the meaning of `include_exact_top`.
pub(crate) fn gain_at_generic<G: RegisterGain>(
    start_raw: u16,
    top_raw: u16,
    rung_count: u8,
    rung_index: u8,
    include_exact_top: bool,
) -> Option<G> {
    if rung_index >= rung_count || start_raw == 0 || start_raw > top_raw || top_raw > G::MAX.raw() {
        return None;
    }
    let start = u64::from(start_raw);
    let top = u64::from(top_raw);
    let mut previous = 0_u64;
    let mut emitted = 0_u8;
    for octave in 0..16 {
        let Some(even_candidate) = start.checked_shl(octave) else {
            continue;
        };
        let Some(even_raw) = u16::try_from(even_candidate).ok() else {
            continue;
        };
        let Some(odd_candidate) = half_octave_candidate(even_raw) else {
            continue;
        };
        for candidate in [even_candidate, u64::from(odd_candidate)] {
            if candidate > top || candidate <= previous {
                continue;
            }
            if emitted == rung_index {
                return G::new(candidate as u16).ok();
            }
            previous = candidate;
            emitted = emitted.checked_add(1)?;
        }
    }
    if include_exact_top && previous != top && emitted == rung_index {
        G::new(top_raw).ok()
    } else {
        None
    }
}

#[cfg(test)]
#[path = "sweep_plan_tests.rs"]
mod tests;
