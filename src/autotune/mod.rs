//! Shared types and analysis helpers for the FOCI commissioning workflow.

mod buffer;
mod current;
mod equivalent_disturbance;
mod filters;
mod hold;
mod integral_gain;
mod metrics;
mod models;
#[cfg(test)]
mod test_support;
mod types;
mod velocity;
pub mod velocity_measurement;
mod velocity_units;

pub use buffer::{
    CaptureDirection, DelayTimestampOffsets, TRANSIENT_LEN, TransientBuffer,
    normalize_capture_sample,
};
pub use current::current_loop_actionable_status_mask;
pub use current::current_loop_entry_actionable_status_mask;
pub use current::current_loop_limiter_status_mask;
pub use current::is_actionable_status;
pub use current::{
    CURRENT_AXIS_VALIDATION_CROSS_AXIS, CURRENT_AXIS_VALIDATION_HIGH_RESPONSE,
    CURRENT_AXIS_VALIDATION_LOW_RESPONSE, CURRENT_AXIS_VALIDATION_MOTION,
    CURRENT_AXIS_VALIDATION_PASS, CURRENT_AXIS_VALIDATION_SATURATION,
    CURRENT_AXIS_VALIDATION_STATUS_FLAGS, CURRENT_AXIS_VALIDATION_WRONG_SIGN,
    CURRENT_AXIS_VALIDATION_ZERO_TARGET, CURRENT_VALIDATION_SETTLED_MAX_SAMPLES,
    CurrentAxisRetryDecision, CurrentAxisSample, CurrentAxisSampleClassification,
    CurrentAxisSampleCriteria, classify_current_axis_sample, current_axis_retry_decision,
    retry_current_loop_gains, synthesize_current_gains,
};
pub use equivalent_disturbance::{
    EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16, equivalent_disturbance_from_rate,
    equivalent_disturbance_variance_from_rate,
};
pub use filters::recommend_filters;
pub use hold::{
    HOLD_GATE_DWELL_US, HOLD_GATE_FINE_DWELL_US, HOLD_GATE_FINE_SAMPLE_PERIOD_US,
    HOLD_GATE_MAX_SAMPLES, HOLD_GATE_POSITION_SPAN_LIMIT_COUNTS, HOLD_GATE_SAMPLE_PERIOD_US,
    HOLD_GATE_STATUS_FAIL_POSITION_SPAN, HOLD_GATE_STATUS_FAIL_SAMPLE_ERROR,
    HOLD_GATE_STATUS_FAIL_STATUS_FLAGS, HOLD_GATE_STATUS_NOT_RUN, HOLD_GATE_STATUS_PASS,
    HOLD_GATE_WARNING_FLUX_CROSSINGS, HOLD_GATE_WARNING_FLUX_RMS, HOLD_GATE_WARNING_FLUX_SPAN,
    HOLD_GATE_WARNING_NON_ACTIONABLE_STATUS, HOLD_GATE_WARNING_POSITION_SPAN,
    HOLD_GATE_WARNING_SAMPLE_OVERRUN, HOLD_GATE_WARNING_TORQUE_CROSSINGS,
    HOLD_GATE_WARNING_TORQUE_RMS, HOLD_GATE_WARNING_TORQUE_SPAN, HoldGateAccumulator,
    HoldGateMetrics, HoldGateSample, hold_status_failure_reason,
};
pub use integral_gain::{
    ADVANCED_PI_I_Q4_12_NUMERATOR, CURRENT_I_Q4_12_DETUNED_NUMERATOR, IntegralGainError,
    integral_tau_us, round_positive_ratio, synthesize_i_gain, synthesize_i_gain_from_cycles,
};
pub use metrics::Permille;
pub use models::{
    AXIS_SPLIT_SOURCE_NONE, CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE,
    CURRENT_FLUX_VALIDATION_SAMPLE_COUNT, CURRENT_GAINS_SOURCE_DEFAULT,
    CURRENT_GAINS_SOURCE_FAILED, CURRENT_GAINS_SOURCE_MEASURED, CURRENT_GAINS_TIER_DEFAULT,
    CURRENT_GAINS_TIER_MEASURED_SYMMETRIC, CURRENT_GAINS_TIER_NONE,
    CURRENT_GAINS_TIER_PHYSICAL_SYMMETRIC, CURRENT_LOOP_FAILURE_CLOSED_LOOP_ACTIVATION,
    CURRENT_LOOP_FAILURE_CROSS_AXIS_COUPLING, CURRENT_LOOP_FAILURE_FLUX_VALIDATION,
    CURRENT_LOOP_FAILURE_GAIN_SYNTHESIS, CURRENT_LOOP_FAILURE_HOLD_POSITION_SPAN,
    CURRENT_LOOP_FAILURE_HOLD_SAMPLE, CURRENT_LOOP_FAILURE_HOLD_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_IMPEDANCE_INVALID, CURRENT_LOOP_FAILURE_MOTION, CURRENT_LOOP_FAILURE_NONE,
    CURRENT_LOOP_FAILURE_RESISTANCE_INVALID, CURRENT_LOOP_FAILURE_RESPONSE_MAGNITUDE,
    CURRENT_LOOP_FAILURE_RETRY_EXHAUSTED, CURRENT_LOOP_FAILURE_SATURATION,
    CURRENT_LOOP_FAILURE_SPI, CURRENT_LOOP_FAILURE_STATUS_FLAGS,
    CURRENT_LOOP_FAILURE_TORQUE_VALIDATION, CURRENT_LOOP_FAILURE_WRONG_SIGN,
    CURRENT_TORQUE_VALIDATION_SAMPLE_COUNT, CURRENT_VALIDATION_AXIS_FLUX,
    CURRENT_VALIDATION_AXIS_TORQUE, CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS, CurrentGainSynthesis,
    CurrentGainSynthesisError, CurrentGainSynthesisInput, CurrentGains, CurrentLoopEvidence,
    ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel, FilterConfig, PositionGains, VelocityGains,
};
pub use types::{
    CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT, CLOSED_LOOP_ACTIVATION_STATUS_FAIL_RUNAWAY,
    CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN, CLOSED_LOOP_ACTIVATION_STATUS_PASS,
    CLOSED_LOOP_ACTIVATION_STATUS_WARN_DRIFT, ClosedLoopActivationState, CommissionError,
    CommissionPhase, CommissionProfile, CommissionStatus, InnerSequence, OperatingMode,
    PositionTuneOutcome, SavedGains, ValidationResult,
};
pub use velocity_units::{TmcMechanicalRpm, VelocityUnitError};
