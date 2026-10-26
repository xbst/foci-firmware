use super::{
    AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC,
    CommissionProfile, CurrentGainSynthesis, CurrentGainSynthesisError, CurrentGainSynthesisInput,
    CurrentGains, CurrentLoopEvidence, ElectricalModel, Permille,
};
#[cfg(test)]
use super::{
    CURRENT_AXIS_TORQUE, CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, CURRENT_GAINS_SOURCE_DEFAULT,
    CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED, CURRENT_GAINS_TIER_DEFAULT,
    CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE,
    CURRENT_LOOP_FAILURE_FLUX_VALIDATION, CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS,
    CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN, CURRENT_LOOP_FAILURE_HOLD_SAMPLE,
    CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS, CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID,
    CURRENT_LOOP_FAILURE_MOTION, CURRENT_LOOP_FAILURE_NONE,
    CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED,
    CURRENT_LOOP_FAILURE_SATURATION, CURRENT_LOOP_FAILURE_SPI, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT,
    CURRENT_VALIDATION_AXIS_FLUX, CURRENT_VALIDATION_AXIS_TORQUE,
    CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, ELECTRICAL_MODEL_RESISTANCE_SCALE,
};
use foci_tmc4671::pid::PidPGain;

use super::integral_gain::{
    CURRENT_I_Q4_12_DETUNED_NUMERATOR, IntegralGainError, synthesize_i_gain,
};

/// Raw settled-flux current-validation reads retained for audit.
pub const CURRENT_VALIDATION_SETTLED_MAX_SAMPLES: usize = 8;

/// Axis sample passed validation.
pub const CURRENT_AXIS_VALIDATION_PASS: u8 = 0;
/// Same-axis current response had the opposite sign from the target.
pub const CURRENT_AXIS_VALIDATION_WRONG_SIGN: u8 = 1;
/// Same-axis current response was below the minimum expected response.
pub const CURRENT_AXIS_VALIDATION_LOW_RESPONSE: u8 = 2;
/// Same-axis current response exceeded the maximum expected response.
pub const CURRENT_AXIS_VALIDATION_HIGH_RESPONSE: u8 = 3;
/// Opposite-axis current response exceeded the allowed coupling limit.
pub const CURRENT_AXIS_VALIDATION_CROSS_AXIS: u8 = 4;
/// Voltage output exceeded the configured output limit.
pub const CURRENT_AXIS_VALIDATION_SATURATION: u8 = 5;
/// Encoder motion exceeded the allowed current-validation movement.
pub const CURRENT_AXIS_VALIDATION_MOTION: u8 = 6;
/// TMC4671 status flags contained an actionable fault bit.
pub const CURRENT_AXIS_VALIDATION_STATUS_FLAGS: u8 = 7;
/// Current target was zero, so response ratios cannot be classified.
pub const CURRENT_AXIS_VALIDATION_ZERO_TARGET: u8 = 8;

const CURRENT_AXIS_POSITION_VELOCITY_STATUS_MASK: u32 =
    (1 << 0) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 6) | (1 << 7);
const CURRENT_AXIS_PID_OUTPUT_STATUS_MASK: u32 = (1 << 11) | (1 << 15);
const CURRENT_AXIS_N_PULSE_STATUS_MASK: u32 = (1 << 28) | (1 << 29) | (1 << 30);
const CURRENT_AXIS_AENC_CLIPPED_STATUS_MASK: u32 = 1 << 27;
const CURRENT_AXIS_RESERVED_STATUS_MASK: u32 = 1 << 31;
const CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK_BASE: u32 = CURRENT_AXIS_POSITION_VELOCITY_STATUS_MASK
    | CURRENT_AXIS_PID_OUTPUT_STATUS_MASK
    | CURRENT_AXIS_N_PULSE_STATUS_MASK
    | CURRENT_AXIS_RESERVED_STATUS_MASK;

/// Status flags that fail current-loop validation and sustained-hold checks.
///
/// `aenc_clipped` counts only when analog feedback is selected as `phi_e_source` or
/// `position_source`; the N-pulse bits never count.
pub const fn current_loop_actionable_status_mask(
    feedback_config: crate::config::FeedbackConfig,
) -> u32 {
    let analog_selected = matches!(
        feedback_config.phi_e_source,
        crate::config::FeedbackSource::Analog
    ) || matches!(
        feedback_config.position_source,
        crate::config::FeedbackSource::Analog
    );
    if analog_selected {
        !CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK_BASE
    } else {
        !(CURRENT_AXIS_NON_ACTIONABLE_STATUS_MASK_BASE | CURRENT_AXIS_AENC_CLIPPED_STATUS_MASK)
    }
}

const CURRENT_AXIS_ENTRY_DEMOTED_STATUS_MASK: u32 = foci_tmc4671::registers::StatusFlags::new()
    .with_pid_id_target_limit(true)
    .with_pid_id_errsum_limit(true)
    .with_pid_iq_target_limit(true)
    .with_pid_iq_errsum_limit(true)
    .with_ipark_cirlim_limit_u_d(true)
    .with_ipark_cirlim_limit_u_q(true)
    .with_ipark_cirlim_limit_u_r(true)
    .with_ref_sw_r(true)
    .with_ref_sw_h(true)
    .with_ref_sw_l(true)
    .with_pwm_min(true)
    .with_pwm_max(true)
    .into_bits();

/// Actionable status the guard checks at primitive start, before commanding
/// Narrower than [`current_loop_actionable_status_mask`]: transition/limiter bits left by a
/// preceding phase are demoted.
pub const fn current_loop_entry_actionable_status_mask(
    feedback_config: crate::config::FeedbackConfig,
) -> u32 {
    current_loop_actionable_status_mask(feedback_config) & !CURRENT_AXIS_ENTRY_DEMOTED_STATUS_MASK
}

/// Limiter bits: the current-axis PI target and error-sum clamps, the inverse
/// Park circular limiter, and PWM duty at either rail. Reference-switch bits are
/// deliberately excluded; a switch closing mid-stroke is a motion event, not
/// limiter activity.
const CURRENT_AXIS_LIMITER_STATUS_MASK: u32 = foci_tmc4671::registers::StatusFlags::new()
    .with_pid_id_target_limit(true)
    .with_pid_id_errsum_limit(true)
    .with_pid_iq_target_limit(true)
    .with_pid_iq_errsum_limit(true)
    .with_ipark_cirlim_limit_u_d(true)
    .with_ipark_cirlim_limit_u_q(true)
    .with_ipark_cirlim_limit_u_r(true)
    .with_pwm_min(true)
    .with_pwm_max(true)
    .into_bits();

const _: () = assert!(
    CURRENT_AXIS_LIMITER_STATUS_MASK & !CURRENT_AXIS_ENTRY_DEMOTED_STATUS_MASK == 0,
    "limiter bits must be a subset of the entry-demoted bits"
);

/// Status bits a velocity stroke legitimately leaves set when it drives the
/// current loops or the voltage output into a configured limit.
///
/// A subset of the entry-demoted bits. Velocity-campaign phase boundaries
/// classify a status word made only of these bits as limiter contact (the axis
/// reached its current or voltage authority at that gain) instead of the
/// fail-closed consequence that genuine fault bits keep.
pub const fn current_loop_limiter_status_mask() -> u32 {
    CURRENT_AXIS_LIMITER_STATUS_MASK
}

/// Whether `flags` contains a bit that `mask` marks actionable.
pub const fn is_actionable_status(flags: u32, mask: u32) -> bool {
    flags & mask != 0
}

/// Retry action for one failed current-axis short-sample validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CurrentAxisRetryDecision {
    /// Retry with lower lambda and therefore higher bandwidth.
    RetryHigherBandwidth,
    /// Retry with higher lambda and therefore lower bandwidth.
    RetryLowerBandwidth,
    /// The axis failure is retryable, but the section retry has already been used.
    RetryBudgetExhausted,
    /// Do not retry this failure.
    Fail,
}

/// Raw current-loop sample for one current axis at one delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentAxisSample {
    /// Signed current target for the sampled axis.
    pub target: i16,
    /// Signed measured current on the commanded axis.
    pub same_axis_actual: i16,
    /// Signed measured current on the opposite current axis.
    pub cross_axis_actual: i16,
    /// Signed limited voltage output for the commanded axis.
    pub output_limited: i16,
    /// Configured voltage output limit used for normalization.
    pub output_limit: u16,
    /// Encoder movement observed during the sample window.
    pub encoder_delta_counts: i32,
    /// TMC4671 status flags captured at the sample.
    pub status_flags: u32,
}

/// Current-loop short-sample validation gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentAxisSampleCriteria {
    /// Minimum allowed same-axis response, in permille of target.
    pub min_response_permille: Permille,
    /// Maximum allowed same-axis response, in permille of target.
    pub max_response_permille: Permille,
    /// Minimum absolute same-axis current error before the relative response
    /// gate can become a hard fault.
    pub min_response_fault_delta_count: u16,
    /// Maximum allowed opposite-axis response, in permille of target.
    pub max_cross_axis_permille: Permille,
    /// Minimum absolute opposite-axis current count before the relative
    /// cross-axis gate can become a hard fault.
    pub min_cross_axis_fault_count: u16,
    /// Maximum allowed voltage output, in permille of output limit.
    pub max_voltage_output_permille: Permille,
    /// Maximum allowed encoder movement during the sample.
    pub max_encoder_delta_counts: u16,
    /// Status flags that should fail validation when present.
    pub actionable_status_mask: u32,
}

impl CurrentAxisSampleCriteria {
    /// Build a commissioning current-validation sample gate.
    pub const fn commissioning_short_sample(
        min_response_permille: u16,
        max_response_permille: u16,
        max_encoder_delta_counts: u16,
        feedback_config: crate::config::FeedbackConfig,
    ) -> Self {
        Self {
            min_response_permille: Permille::new(min_response_permille),
            max_response_permille: Permille::new(max_response_permille),
            min_response_fault_delta_count: 32,
            max_cross_axis_permille: Permille::new(150),
            min_cross_axis_fault_count: 32,
            max_voltage_output_permille: Permille::new(900),
            max_encoder_delta_counts,
            actionable_status_mask: current_loop_actionable_status_mask(feedback_config),
        }
    }
}

/// Classified current-loop short-sample result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CurrentAxisSampleClassification {
    /// `CURRENT_AXIS_VALIDATION_*` status code.
    pub status: u8,
    /// Same-axis current response normalized to target, in permille.
    pub response_permille: Permille,
    /// Opposite-axis current response normalized to target, in permille.
    pub cross_axis_permille: Permille,
    /// Voltage output normalized to output limit, in permille.
    pub voltage_output_permille: Permille,
    /// Absolute encoder movement observed during the sample.
    pub encoder_delta_counts: u16,
    /// Gate limit copied into evidence for host-side audit.
    pub max_cross_axis_permille: Permille,
}

/// Classify one current-axis short sample against a fixed gate.
pub fn classify_current_axis_sample(
    sample: &CurrentAxisSample,
    gate: &CurrentAxisSampleCriteria,
) -> CurrentAxisSampleClassification {
    if sample.target == 0 {
        return CurrentAxisSampleClassification {
            status: CURRENT_AXIS_VALIDATION_ZERO_TARGET,
            response_permille: Permille::ZERO,
            cross_axis_permille: Permille::ZERO,
            voltage_output_permille: Permille::ZERO,
            encoder_delta_counts: 0,
            max_cross_axis_permille: gate.max_cross_axis_permille,
        };
    }

    let target_abs = sample.target.unsigned_abs().max(1) as u32;
    let same_axis_count = sample.same_axis_actual.unsigned_abs();
    let response_permille =
        Permille::checked_ratio(u64::from(same_axis_count), u64::from(target_abs))
            .expect("nonzero current-axis target");
    let response_error_count = same_axis_count.abs_diff(sample.target.unsigned_abs());
    let cross_axis_count = sample.cross_axis_actual.unsigned_abs();
    let cross_axis_permille =
        Permille::checked_ratio(u64::from(cross_axis_count), u64::from(target_abs))
            .expect("nonzero current-axis target");
    let output_limit = sample.output_limit.max(1) as u32;
    let voltage_output_permille = Permille::checked_ratio(
        u64::from(sample.output_limited.unsigned_abs()),
        u64::from(output_limit),
    )
    .expect("nonzero current-axis output limit");
    let encoder_delta_counts = sample
        .encoder_delta_counts
        .unsigned_abs()
        .min(u16::MAX as u32) as u16;

    let status = if (sample.target > 0 && sample.same_axis_actual < 0)
        || (sample.target < 0 && sample.same_axis_actual > 0)
    {
        CURRENT_AXIS_VALIDATION_WRONG_SIGN
    } else if sample.status_flags & gate.actionable_status_mask != 0 {
        CURRENT_AXIS_VALIDATION_STATUS_FLAGS
    } else if voltage_output_permille > gate.max_voltage_output_permille {
        CURRENT_AXIS_VALIDATION_SATURATION
    } else if encoder_delta_counts > gate.max_encoder_delta_counts {
        CURRENT_AXIS_VALIDATION_MOTION
    } else if cross_axis_count > gate.min_cross_axis_fault_count
        && cross_axis_permille > gate.max_cross_axis_permille
    {
        CURRENT_AXIS_VALIDATION_CROSS_AXIS
    } else if response_error_count > gate.min_response_fault_delta_count
        && response_permille < gate.min_response_permille
    {
        CURRENT_AXIS_VALIDATION_LOW_RESPONSE
    } else if response_error_count > gate.min_response_fault_delta_count
        && response_permille > gate.max_response_permille
    {
        CURRENT_AXIS_VALIDATION_HIGH_RESPONSE
    } else {
        CURRENT_AXIS_VALIDATION_PASS
    };

    CurrentAxisSampleClassification {
        status,
        response_permille,
        cross_axis_permille,
        voltage_output_permille,
        encoder_delta_counts,
        max_cross_axis_permille: gate.max_cross_axis_permille,
    }
}

impl CurrentGainSynthesisInput {
    /// Build symmetric current-loop synthesis input from a measured electrical model.
    pub const fn symmetric(model: ElectricalModel, profile: CommissionProfile) -> Self {
        Self {
            model,
            profile,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_source: AXIS_SPLIT_SOURCE_NONE,
            high_inductance_axis: CURRENT_AXIS_FLUX,
        }
    }
}

const CURRENT_P_Q8_8_SCALE: f32 = 256.0;
const MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ: f32 = 2_000.0;

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn valid_physical_current_bandwidth_hz(value: f32) -> bool {
    finite_positive(value) && value <= MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ
}

fn pid_gain_from_float(value: f32) -> Result<PidPGain, CurrentGainSynthesisError> {
    if !finite_positive(value) {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }
    let rounded = (value + 0.5).max(1.0) as u64;
    if rounded > u64::from(PidPGain::MAX.raw()) {
        return Err(CurrentGainSynthesisError::GainOutOfRange {
            attempted_raw: rounded,
        });
    }
    PidPGain::new(rounded as u16).map_err(|error| CurrentGainSynthesisError::GainOutOfRange {
        attempted_raw: u64::from(error.raw()),
    })
}

fn physical_current_p_raw_from_l_count_micro(
    l_count_micro: u32,
    bandwidth_hz: f32,
) -> Result<PidPGain, CurrentGainSynthesisError> {
    if l_count_micro == 0 || !valid_physical_current_bandwidth_hz(bandwidth_hz) {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }
    let l_count_seconds = l_count_micro as f32 * 1e-6;
    let p_float =
        2.0 * core::f32::consts::PI * bandwidth_hz * l_count_seconds * CURRENT_P_Q8_8_SCALE;
    pid_gain_from_float(p_float)
}

#[cfg(test)]
fn physical_current_bandwidth_hz_for_p_raw(l_count_micro: u32, p_raw: u16) -> Option<f32> {
    if l_count_micro == 0 || p_raw == 0 {
        return None;
    }
    let l_count_seconds = l_count_micro as f32 * 1e-6;
    Some(p_raw as f32 / (2.0 * core::f32::consts::PI * l_count_seconds * CURRENT_P_Q8_8_SCALE))
}

fn bandwidth_period_us(bandwidth_hz: f32) -> Option<u32> {
    if !valid_physical_current_bandwidth_hz(bandwidth_hz) {
        return None;
    }
    let period_us = 1_000_000.0 / (2.0 * core::f32::consts::PI * bandwidth_hz);
    if period_us > u32::MAX as f32 {
        return None;
    }
    Some((period_us + 0.5).max(1.0) as u32)
}

fn physical_current_gains_checked(
    model: &ElectricalModel,
    bandwidth_hz: f32,
    config: &crate::config::TmcConfig,
) -> Result<CurrentGains, CurrentGainSynthesisError> {
    if model.r_count_milli == 0 || model.l_count_micro == 0 || model.tau_e_us == 0 {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }
    let p_reg = physical_current_p_raw_from_l_count_micro(model.l_count_micro, bandwidth_hz)?;
    let i_reg = synthesize_i_gain(
        CURRENT_I_Q4_12_DETUNED_NUMERATOR,
        u64::from(model.tau_e_us),
        config.pwm_freq_hz(),
    )
    .map_err(map_integral_gain_error)?;

    Ok(CurrentGains {
        flux_p: p_reg,
        flux_i: i_reg,
        torque_p: p_reg,
        torque_i: i_reg,
        effective_bandwidth_hz: (bandwidth_hz + 0.5).max(1.0) as u32,
        lambda_us: bandwidth_period_us(bandwidth_hz)
            .ok_or(CurrentGainSynthesisError::InvalidElectricalModel)?,
    })
}

fn map_integral_gain_error(error: IntegralGainError) -> CurrentGainSynthesisError {
    match error {
        IntegralGainError::InvalidInput | IntegralGainError::Arithmetic => {
            CurrentGainSynthesisError::InvalidElectricalModel
        }
        IntegralGainError::GainOutOfRange { attempted_raw } => {
            CurrentGainSynthesisError::GainOutOfRange {
                attempted_raw: attempted_raw.min(u128::from(u64::MAX)) as u64,
            }
        }
    }
}

/// Synthesize production current-loop gains and gain-source evidence.
pub fn synthesize_current_gains(
    input: &CurrentGainSynthesisInput,
    config: &crate::config::TmcConfig,
) -> Result<CurrentGainSynthesis, CurrentGainSynthesisError> {
    if input.model.r_count_milli == 0
        || input.model.l_count_micro == 0
        || input.model.theta_e_us == 0
    {
        return Err(CurrentGainSynthesisError::InvalidElectricalModel);
    }

    match input.axis_split_source {
        AXIS_SPLIT_SOURCE_NONE => {
            let gains = physical_current_gains_checked(
                &input.model,
                input.profile.current_loop_bandwidth_hz(),
                config,
            )?;
            Ok(CurrentGainSynthesis {
                gains,
                evidence: CurrentLoopEvidence {
                    gains_tier: CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC,
                    candidate_gains_tier: CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC,
                    ..CurrentLoopEvidence::measured_symmetric()
                },
                flux_tau_us: input.model.tau_e_us,
                torque_tau_us: input.model.tau_e_us,
            })
        }
        axis_split_source => {
            Err(CurrentGainSynthesisError::UnsupportedSynthesisPolicy { axis_split_source })
        }
    }
}

/// Return the retry policy decision for one current-axis validation failure.
pub fn current_axis_retry_decision(
    axis_status: u8,
    section_attempt: u8,
    status_fault: bool,
    saturated: bool,
    motion_fault: bool,
) -> CurrentAxisRetryDecision {
    if status_fault || saturated || motion_fault {
        return CurrentAxisRetryDecision::Fail;
    }

    let retryable = axis_status == CURRENT_AXIS_VALIDATION_LOW_RESPONSE
        || axis_status == CURRENT_AXIS_VALIDATION_HIGH_RESPONSE;
    if !retryable {
        return CurrentAxisRetryDecision::Fail;
    }

    if section_attempt > 0 {
        return CurrentAxisRetryDecision::RetryBudgetExhausted;
    }

    if axis_status == CURRENT_AXIS_VALIDATION_LOW_RESPONSE {
        CurrentAxisRetryDecision::RetryHigherBandwidth
    } else {
        CurrentAxisRetryDecision::RetryLowerBandwidth
    }
}

const RETRY_LOW_RESPONSE_LAMBDA_PERMILLE: u16 = 750;
const RETRY_HIGH_RESPONSE_LAMBDA_PERMILLE: u16 = 2000;
const RETRY_MIN_LAMBDA_OF_PREVIOUS_PERMILLE: u16 = 250;
const RETRY_MAX_LAMBDA_OF_PREVIOUS_PERMILLE: u16 = 4000;

fn retry_low_response_lambda_permille(_profile: CommissionProfile) -> u16 {
    RETRY_LOW_RESPONSE_LAMBDA_PERMILLE
}

fn retry_high_response_lambda_permille(_profile: CommissionProfile) -> u16 {
    RETRY_HIGH_RESPONSE_LAMBDA_PERMILLE
}

fn retry_min_lambda_permille(_profile: CommissionProfile) -> u16 {
    RETRY_MIN_LAMBDA_OF_PREVIOUS_PERMILLE
}

fn retry_max_lambda_permille(_profile: CommissionProfile) -> u16 {
    RETRY_MAX_LAMBDA_OF_PREVIOUS_PERMILLE
}

fn scale_lambda_permille(lambda_us: u32, permille: u16) -> u32 {
    ((lambda_us.max(1) as u64 * permille.max(1) as u64) / 1000).min(u32::MAX as u64) as u32
}

fn scale_gain_for_lambda(
    gain: PidPGain,
    previous_lambda_us: u32,
    next_lambda_us: u32,
) -> Result<PidPGain, CurrentGainSynthesisError> {
    if previous_lambda_us == 0 || next_lambda_us == 0 {
        return Ok(gain);
    }

    let raw = (u64::from(gain.raw()) * u64::from(previous_lambda_us)) / u64::from(next_lambda_us);
    if raw > u64::from(PidPGain::MAX.raw()) {
        return Err(CurrentGainSynthesisError::GainOutOfRange { attempted_raw: raw });
    }
    PidPGain::new(raw.max(1) as u16).map_err(|error| CurrentGainSynthesisError::GainOutOfRange {
        attempted_raw: u64::from(error.raw()),
    })
}

fn scale_bandwidth_for_lambda(
    bandwidth_hz: u32,
    previous_lambda_us: u32,
    next_lambda_us: u32,
) -> u32 {
    if previous_lambda_us == 0 || next_lambda_us == 0 || bandwidth_hz == 0 {
        return bandwidth_hz;
    }

    ((bandwidth_hz as u64 * previous_lambda_us as u64) / next_lambda_us as u64).min(u32::MAX as u64)
        as u32
}

fn cap_higher_retry_at_physical_limit(
    decision: CurrentAxisRetryDecision,
    bandwidth_hz: u32,
    lambda_us: u32,
) -> (u32, u32) {
    if decision != CurrentAxisRetryDecision::RetryHigherBandwidth
        || bandwidth_hz <= MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ as u32
    {
        return (bandwidth_hz, lambda_us);
    }

    let capped_bandwidth_hz = MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ as u32;
    let capped_lambda_us =
        bandwidth_period_us(MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ).unwrap_or(lambda_us);

    (capped_bandwidth_hz, capped_lambda_us)
}

/// Adjust current-loop gains according to the section retry policy.
///
/// P tracks the new bandwidth; I stays fixed.
pub fn retry_current_loop_gains(
    previous: CurrentGains,
    decision: CurrentAxisRetryDecision,
    profile: CommissionProfile,
) -> Result<CurrentGains, CurrentGainSynthesisError> {
    let lambda_permille = match decision {
        CurrentAxisRetryDecision::RetryHigherBandwidth => {
            retry_low_response_lambda_permille(profile)
        }
        CurrentAxisRetryDecision::RetryLowerBandwidth => {
            retry_high_response_lambda_permille(profile)
        }
        CurrentAxisRetryDecision::RetryBudgetExhausted | CurrentAxisRetryDecision::Fail => {
            return Ok(previous);
        }
    };
    let min_lambda = scale_lambda_permille(previous.lambda_us, retry_min_lambda_permille(profile));
    let max_lambda = scale_lambda_permille(previous.lambda_us, retry_max_lambda_permille(profile));
    let next_lambda = scale_lambda_permille(previous.lambda_us, lambda_permille)
        .clamp(min_lambda.min(max_lambda), min_lambda.max(max_lambda));
    let next_bandwidth = scale_bandwidth_for_lambda(
        previous.effective_bandwidth_hz,
        previous.lambda_us,
        next_lambda,
    );
    let (next_bandwidth, next_lambda) =
        cap_higher_retry_at_physical_limit(decision, next_bandwidth, next_lambda);
    let flux_p = scale_gain_for_lambda(previous.flux_p, previous.lambda_us, next_lambda)?;
    let torque_p = scale_gain_for_lambda(previous.torque_p, previous.lambda_us, next_lambda)?;
    let mut next = previous;

    next.lambda_us = next_lambda;
    next.effective_bandwidth_hz = next_bandwidth;
    next.flux_p = flux_p;
    next.torque_p = torque_p;

    Ok(next)
}

#[cfg(test)]
mod tests {
    use super::super::test_support::{autotune_test_config, test_electrical_model};
    use super::*;

    fn p_gain(raw: u16) -> PidPGain {
        PidPGain::new(raw).expect("test PID gain must be valid")
    }

    fn i_gain(raw: u16) -> foci_tmc4671::pid::PidIGain {
        foci_tmc4671::pid::PidIGain::new(raw).expect("test PID gain must be valid")
    }

    #[test]
    fn current_loop_evidence_constructors_set_gains_tier_and_failure_state() {
        let failed = CurrentLoopEvidence::failed(CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS);
        assert_eq!(failed.gains_source, CURRENT_GAINS_SOURCE_FAILED);
        assert_eq!(failed.gains_tier, CURRENT_GAINS_TIER_NONE);
        assert_eq!(failed.failure_reason, CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS);
        assert_eq!(failed.current_validation_axes, 0);
        assert_eq!(failed.retry_budget_exhausted, 0);

        let measured = CurrentLoopEvidence::measured_symmetric();
        assert_eq!(measured.gains_source, CURRENT_GAINS_SOURCE_MEASURED);
        assert_eq!(measured.gains_tier, CURRENT_GAINS_TIER_MEASURED_SYMMETRIC);
        assert_eq!(measured.failure_reason, CURRENT_LOOP_FAILURE_NONE);
        assert_eq!(measured.current_validation_axes, 0);
        assert_eq!(measured.retry_budget_exhausted, 0);
    }

    #[test]
    fn current_loop_evidence_pins_gain_source_split_and_tier_codes() {
        assert_eq!(CURRENT_GAINS_SOURCE_FAILED, 0);
        assert_eq!(CURRENT_GAINS_SOURCE_MEASURED, 1);
        assert_eq!(CURRENT_GAINS_SOURCE_DEFAULT, 2);

        assert_eq!(AXIS_SPLIT_SOURCE_NONE, 0);

        assert_eq!(CURRENT_GAINS_TIER_NONE, 0);
        assert_eq!(CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, 1);
        assert_eq!(CURRENT_GAINS_TIER_DEFAULT, 3);
        assert_eq!(CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC, 4);
    }

    #[test]
    fn current_loop_evidence_pins_axis_and_sample_count_codes() {
        assert_eq!(CURRENT_VALIDATION_AXIS_FLUX, 1);
        assert_eq!(CURRENT_VALIDATION_AXIS_TORQUE, 2);

        assert_eq!(CURRENT_AXIS_FLUX, 0);
        assert_eq!(CURRENT_AXIS_TORQUE, 1);

        assert_eq!(CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, 4);
        assert_eq!(CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT, 3);
        assert_eq!(CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, 4);
    }

    #[test]
    fn current_loop_evidence_pins_failure_reason_codes() {
        assert_eq!(CURRENT_LOOP_FAILURE_NONE, 0);
        assert_eq!(CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, 1);
        assert_eq!(CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID, 2);
        assert_eq!(CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS, 3);
        assert_eq!(CURRENT_LOOP_FAILURE_FLUX_VALIDATION, 4);
        assert_eq!(CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, 5);
        assert_eq!(CURRENT_LOOP_FAILURE_SATURATION, 6);
        assert_eq!(CURRENT_LOOP_FAILURE_MOTION, 7);
        assert_eq!(CURRENT_LOOP_FAILURE_STATUS_FLAGS, 8);
        assert_eq!(CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED, 9);
        assert_eq!(CURRENT_LOOP_FAILURE_SPI, 10);
        assert_eq!(CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN, 11);
        assert_eq!(CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS, 12);
        assert_eq!(CURRENT_LOOP_FAILURE_HOLD_SAMPLE, 13);
        assert_eq!(crate::autotune::CURRENT_LOOP_FAILURE_RESPONSE_MAGNITUDE, 14);
        assert_eq!(
            crate::autotune::CURRENT_LOOP_FAILURE_CROSS_AXIS_COUPLING,
            15
        );
        assert_eq!(crate::autotune::CURRENT_LOOP_FAILURE_WRONG_SIGN, 16);
        assert_eq!(
            crate::autotune::CURRENT_LOOP_FAILURE_CLOSED_LOOP_ACTIVATION,
            17
        );
    }

    #[test]
    fn physical_current_p_uses_count_space_l_and_bandwidth() {
        assert_eq!(
            physical_current_p_raw_from_l_count_micro(1_346, 600.0)
                .expect("valid physical gain")
                .raw(),
            1_299
        );
        assert_eq!(
            physical_current_p_raw_from_l_count_micro(1_346, 800.0)
                .expect("valid physical gain")
                .raw(),
            1_732
        );
        assert_eq!(
            physical_current_p_raw_from_l_count_micro(1_366, 650.0)
                .expect("valid physical gain")
                .raw(),
            1_428
        );
    }

    #[test]
    fn physical_current_p_anchor_matches_measured_axis_bandwidths() {
        let x_bw_for_p1422 =
            physical_current_bandwidth_hz_for_p_raw(1_346, 1_422).expect("valid X anchor");
        let y_bw_for_p1422 =
            physical_current_bandwidth_hz_for_p_raw(1_366, 1_422).expect("valid Y anchor");

        assert!(
            (656.0..=657.5).contains(&x_bw_for_p1422),
            "X anchor {x_bw_for_p1422}"
        );
        assert!(
            (646.0..=648.5).contains(&y_bw_for_p1422),
            "Y anchor {y_bw_for_p1422}"
        );
    }

    #[test]
    fn physical_current_p_rejects_invalid_inputs() {
        assert!(physical_current_p_raw_from_l_count_micro(0, 600.0).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 0.0).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, f32::NAN).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 2_000.0).is_ok());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 2_001.0).is_err());
        assert!(physical_current_p_raw_from_l_count_micro(1_346, 12_000.0).is_err());
    }

    #[test]
    fn retry_policy_allows_one_retry_for_response_magnitude_only() {
        let decision = current_axis_retry_decision(
            CURRENT_AXIS_VALIDATION_LOW_RESPONSE,
            0,
            false,
            false,
            false,
        );

        assert_eq!(decision, CurrentAxisRetryDecision::RetryHigherBandwidth);
    }

    #[test]
    fn retry_policy_rejects_saturation_without_retry() {
        let decision =
            current_axis_retry_decision(CURRENT_AXIS_VALIDATION_SATURATION, 0, false, true, false);

        assert_eq!(decision, CurrentAxisRetryDecision::Fail);
    }

    #[test]
    fn retry_policy_exhausts_section_budget() {
        let decision = current_axis_retry_decision(
            CURRENT_AXIS_VALIDATION_LOW_RESPONSE,
            1,
            false,
            false,
            false,
        );

        assert_eq!(decision, CurrentAxisRetryDecision::RetryBudgetExhausted);
    }

    #[test]
    fn gain_conversion_accepts_maximum_and_rejects_first_negative_encoding() {
        assert_eq!(
            pid_gain_from_float(32_767.0).unwrap(),
            foci_tmc4671::pid::PidPGain::MAX
        );
        assert!(matches!(
            pid_gain_from_float(32_768.0),
            Err(CurrentGainSynthesisError::GainOutOfRange {
                attempted_raw: 32_768,
            })
        ));
    }

    #[test]
    fn higher_bandwidth_retry_reports_gain_overflow_without_saturation() {
        let previous = CurrentGains {
            flux_p: foci_tmc4671::pid::PidPGain::MAX,
            flux_i: foci_tmc4671::pid::PidIGain::new(10).unwrap(),
            torque_p: foci_tmc4671::pid::PidPGain::MAX,
            torque_i: foci_tmc4671::pid::PidIGain::new(10).unwrap(),
            effective_bandwidth_hz: 100,
            lambda_us: 1_000,
        };

        assert!(matches!(
            retry_current_loop_gains(
                previous,
                CurrentAxisRetryDecision::RetryHigherBandwidth,
                CommissionProfile::Balanced,
            ),
            Err(CurrentGainSynthesisError::GainOutOfRange { .. })
        ));
    }

    #[test]
    fn retry_policy_scales_bandwidth_and_p_but_keeps_i() {
        let previous = CurrentGains {
            flux_p: p_gain(120),
            flux_i: i_gain(23),
            torque_p: p_gain(220),
            torque_i: i_gain(19),
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };

        let retry = retry_current_loop_gains(
            previous,
            CurrentAxisRetryDecision::RetryHigherBandwidth,
            CommissionProfile::Balanced,
        )
        .expect("retry gains must remain valid");

        assert!(retry.flux_p.raw() > previous.flux_p.raw());
        assert!(retry.torque_p.raw() > previous.torque_p.raw());
        assert_eq!(retry.flux_i, previous.flux_i);
        assert_eq!(retry.torque_i, previous.torque_i);
        assert_eq!(retry.effective_bandwidth_hz, 193);
        assert_eq!(retry.lambda_us, 700);
    }

    #[test]
    fn retry_policy_caps_higher_bandwidth_at_physical_limit() {
        let physical_limit_lambda =
            bandwidth_period_us(MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ).expect("valid limit");
        let previous = CurrentGains {
            flux_p: p_gain(1500),
            flux_i: i_gain(23),
            torque_p: p_gain(1600),
            torque_i: i_gain(19),
            effective_bandwidth_hz: MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ as u32,
            lambda_us: physical_limit_lambda,
        };

        let retry = retry_current_loop_gains(
            previous,
            CurrentAxisRetryDecision::RetryHigherBandwidth,
            CommissionProfile::Balanced,
        )
        .expect("capped retry gains must remain valid");

        assert_eq!(
            retry.effective_bandwidth_hz,
            previous.effective_bandwidth_hz
        );
        assert_eq!(retry.lambda_us, previous.lambda_us);
        assert_eq!(retry.flux_p, previous.flux_p);
        assert_eq!(retry.torque_p, previous.torque_p);
        assert_eq!(retry.flux_i, previous.flux_i);
        assert_eq!(retry.torque_i, previous.torque_i);
    }

    #[test]
    fn retry_policy_lower_bandwidth_keeps_i() {
        let previous = CurrentGains {
            flux_p: p_gain(120),
            flux_i: i_gain(23),
            torque_p: p_gain(220),
            torque_i: i_gain(19),
            effective_bandwidth_hz: 145,
            lambda_us: 934,
        };

        let retry = retry_current_loop_gains(
            previous,
            CurrentAxisRetryDecision::RetryLowerBandwidth,
            CommissionProfile::Balanced,
        )
        .expect("retry gains must remain valid");

        assert!(retry.flux_p.raw() < previous.flux_p.raw());
        assert!(retry.torque_p.raw() < previous.torque_p.raw());
        assert_eq!(retry.flux_i, previous.flux_i);
        assert_eq!(retry.torque_i, previous.torque_i);
        assert_eq!(retry.effective_bandwidth_hz, 72);
        assert_eq!(retry.lambda_us, 1868);
    }

    #[test]
    fn physical_synthesis_uses_the_profile_bandwidth_for_symmetric_p() {
        let model = ElectricalModel {
            r_count_milli: 888,
            l_count_micro: 1_346,
            tau_e_us: 1_515,
            theta_e_us: 160,
            theta_source: 0,
        };
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Conservative);

        let output = synthesize_current_gains(&input, &config).expect("physical gains");

        assert_eq!(output.gains.flux_p.raw(), 1_299);
        assert_eq!(output.gains.torque_p.raw(), 1_299);
        assert_eq!(output.gains.effective_bandwidth_hz, 600);
        assert_eq!(
            output.evidence.gains_tier,
            CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC
        );
        assert_eq!(
            output.evidence.candidate_gains_tier,
            CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC
        );
        assert_eq!(output.flux_tau_us, model.tau_e_us);
        assert_eq!(output.torque_tau_us, model.tau_e_us);
    }

    #[test]
    fn physical_synthesis_keeps_i_on_detuned_tau_path() {
        let model = ElectricalModel {
            r_count_milli: ELECTRICAL_MODEL_RESISTANCE_SCALE,
            l_count_micro: 1_366,
            tau_e_us: 1_366,
            theta_e_us: 160,
            theta_source: 0,
        };
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).expect("physical gains");

        let expected_i = synthesize_i_gain(
            CURRENT_I_Q4_12_DETUNED_NUMERATOR,
            u64::from(model.tau_e_us),
            config.pwm_freq_hz(),
        )
        .unwrap()
        .raw();

        assert_eq!(output.gains.flux_i.raw(), expected_i);
        assert_eq!(output.gains.torque_i.raw(), expected_i);
        assert_eq!(output.gains.flux_p.raw(), 2_197);
    }

    #[test]
    fn phase1_symmetric_synthesis_reports_physical_symmetric() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);

        let output = synthesize_current_gains(&input, &config).unwrap();

        assert_eq!(output.evidence.gains_source, CURRENT_GAINS_SOURCE_MEASURED);
        assert_eq!(output.evidence.axis_split_source, AXIS_SPLIT_SOURCE_NONE);
        assert_eq!(
            output.evidence.gains_tier,
            CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC
        );
        assert_eq!(output.evidence.failure_reason, CURRENT_LOOP_FAILURE_NONE);
        assert_eq!(output.gains.flux_p, output.gains.torque_p);
        assert_eq!(output.gains.flux_i, output.gains.torque_i);
        assert_eq!(output.flux_tau_us, model.tau_e_us);
        assert_eq!(output.torque_tau_us, model.tau_e_us);
    }

    #[test]
    fn physical_synthesis_rejects_unvalidated_axis_split() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let input = CurrentGainSynthesisInput {
            model,
            profile: CommissionProfile::Balanced,
            measured_axis_split_permille: 1_200,
            applied_axis_split_permille: 1_100,
            axis_split_source: 1,
            high_inductance_axis: CURRENT_AXIS_FLUX,
        };

        assert!(matches!(
            synthesize_current_gains(&input, &config),
            Err(CurrentGainSynthesisError::UnsupportedSynthesisPolicy {
                axis_split_source: 1
            })
        ));
    }

    #[test]
    fn split_synthesis_rejects_invalid_axis_split_source() {
        let model = test_electrical_model();
        let config = autotune_test_config();
        let mut input = CurrentGainSynthesisInput::symmetric(model, CommissionProfile::Balanced);
        input.axis_split_source = 0xff;

        assert!(matches!(
            synthesize_current_gains(&input, &config),
            Err(CurrentGainSynthesisError::UnsupportedSynthesisPolicy { .. })
        ));
    }

    #[test]
    fn current_sample_gate_accepts_good_flux_sample() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
        assert_eq!(gate.response_permille, Permille::new(960));
        assert!(gate.cross_axis_permille <= gate.max_cross_axis_permille);
    }

    #[test]
    fn current_sample_gate_rejects_zero_target() {
        let sample = CurrentAxisSample {
            target: 0,
            same_axis_actual: 0,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_ZERO_TARGET);
    }

    #[test]
    fn current_sample_gate_rejects_wrong_sign() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: -20,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_WRONG_SIGN);
    }

    #[test]
    fn current_sample_gate_saturates_response_permille_before_classifying() {
        let sample = CurrentAxisSample {
            target: 1,
            same_axis_actual: 132,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.response_permille, Permille::MAX);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE);
    }

    #[test]
    fn current_sample_gate_saturates_cross_axis_and_voltage_permille() {
        let sample = CurrentAxisSample {
            target: 1,
            same_axis_actual: 1,
            cross_axis_actual: 132,
            output_limited: 132,
            output_limit: 1,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.cross_axis_permille, Permille::MAX);
        assert_eq!(gate.voltage_output_permille, Permille::MAX);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_SATURATION);
    }

    #[test]
    fn current_sample_gate_accepts_low_target_cross_axis_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 128,
            cross_axis_actual: 20,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                850,
                1150,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.cross_axis_permille, Permille::new(156));
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_rejects_low_target_cross_axis_above_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 128,
            cross_axis_actual: 33,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                850,
                1150,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.cross_axis_permille, Permille::new(257));
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_CROSS_AXIS);
    }

    #[test]
    fn current_sample_gate_accepts_low_target_response_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 151,
            cross_axis_actual: 0,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                850,
                1150,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.response_permille, Permille::new(1179));
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_rejects_low_target_response_above_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 161,
            cross_axis_actual: 0,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                850,
                1150,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.response_permille, Permille::new(1257));
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE);
    }

    #[test]
    fn current_sample_gate_rejects_low_target_response_below_noise_floor() {
        let sample = CurrentAxisSample {
            target: 128,
            same_axis_actual: 95,
            cross_axis_actual: 0,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                850,
                1150,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.response_permille, Permille::new(742));
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
    }

    #[test]
    fn current_sample_gate_rejects_zero_actual_as_low_response() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 0,
            cross_axis_actual: 0,
            output_limited: 0,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.response_permille, Permille::ZERO);
        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_LOW_RESPONSE);
    }

    #[test]
    fn current_sample_gate_ignores_encoder_presence_status_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x7000_0000,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_ignores_unsupported_analog_encoder_clipping() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x7800_0000,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_analog_phi_e_source_makes_aenc_clipped_actionable() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x0800_0000,
        };
        let feedback_config = crate::config::FeedbackConfig {
            phi_e_source: crate::config::FeedbackSource::Analog,
            position_source: crate::config::FeedbackSource::Abn,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(700, 1250, 4, feedback_config),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
    }

    #[test]
    fn current_sample_gate_analog_position_source_alone_makes_aenc_clipped_actionable() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x0800_0000,
        };
        let feedback_config = crate::config::FeedbackConfig {
            phi_e_source: crate::config::FeedbackSource::Abn,
            position_source: crate::config::FeedbackSource::Analog,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(700, 1250, 4, feedback_config),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
    }

    #[test]
    fn current_sample_gate_analog_source_still_ignores_n_pulse_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x7000_0000,
        };
        let feedback_config = crate::config::FeedbackConfig {
            phi_e_source: crate::config::FeedbackSource::Analog,
            position_source: crate::config::FeedbackSource::Analog,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(700, 1250, 4, feedback_config),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_keeps_adc_current_clipping_actionable() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x0c00_0000,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
    }

    #[test]
    fn current_sample_gate_ignores_reserved_status_bit_31() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0xf000_0080,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_ignores_outer_cascade_status_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 0x0000_00dd,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn current_sample_gate_rejects_current_loop_status_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: 1 << 10,
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_STATUS_FLAGS);
    }

    #[test]
    fn current_sample_gate_ignores_latched_current_pid_output_bits() {
        let sample = CurrentAxisSample {
            target: 250,
            same_axis_actual: 240,
            cross_axis_actual: 8,
            output_limited: 500,
            output_limit: 2000,
            encoder_delta_counts: 0,
            status_flags: (1 << 11) | (1 << 15),
        };
        let gate = classify_current_axis_sample(
            &sample,
            &CurrentAxisSampleCriteria::commissioning_short_sample(
                700,
                1250,
                4,
                crate::config::FeedbackConfig::abn_abn(),
            ),
        );

        assert_eq!(gate.status, CURRENT_AXIS_VALIDATION_PASS);
    }

    #[test]
    fn is_actionable_status_true_only_when_flags_intersect_mask() {
        assert!(!is_actionable_status(0x0000_00ff, 0xffff_ff00));
        assert!(is_actionable_status(0x0000_0100, 0xffff_ff00));
    }

    #[test]
    fn entry_mask_demotes_transition_bits_but_keeps_faults() {
        use crate::config::FeedbackConfig;
        let full = current_loop_actionable_status_mask(FeedbackConfig::abn_abn());
        let entry = current_loop_entry_actionable_status_mask(FeedbackConfig::abn_abn());

        assert_eq!(entry & !full, 0, "entry mask must not add bits");

        for bit in [16u32, 17, 18, 20, 21, 22, 24, 25] {
            assert_eq!(entry & (1 << bit), 0, "bit {bit} must be demoted at entry");
            assert_ne!(full & (1 << bit), 0, "bit {bit} must stay in the full mask");
        }

        assert_ne!(
            entry & (1 << 26),
            0,
            "adc_i_clipped must remain entry-actionable"
        );
    }
}
