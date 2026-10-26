use super::super::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT;
use super::{
    CEILING_BINDING_CURRENT_LIMIT, CEILING_BINDING_REPRESENTABILITY_CLAMP, CLAMP_ENVELOPE,
    CLAMP_TRAVEL, CURRENT_HEADROOM_DEN, CURRENT_HEADROOM_NUM, EFFECTIVE_CEILING_ENCODER,
    EFFECTIVE_CEILING_MACHINE, ENVELOPE_ABSOLUTE_MARGIN_MREV_S, ENVELOPE_PROPORTIONAL_DEN,
    ENVELOPE_PROPORTIONAL_NUM, MAX_SAFETY_OBSERVATION_GAP_US, MAX_SWEEP_RUNGS,
    MIN_SETTLE_CAPTURE_US, MIN_STRUCTURAL_RUNGS, MOVING_STROKE_US,
    OBSERVATIONS_PER_DIRECTION_PER_RUNG, PROPORTIONAL_PLAN_SCHEMA_REVISION,
    RequestedVelocitySource, SETTLE_ANALYSIS_BUDGET_US, SWEEP_SLOT_COUNT, StrokeGeometry,
    VelocityPlanError, VelocitySweepRequest, ZERO_SETTLE_US, build_gain_ladder,
    build_gain_ladder_generic, cap_safe_target_rpm, current_gain_span,
    effective_velocity_ceiling_mrev_s, envelope_cap, gain_at, half_octave_candidate,
    plan_proportional_velocity_sweep, plan_velocity_sweep, representability_ceilings, travel_cap,
};
use crate::autotune::TmcMechanicalRpm;
use foci_tmc4671::pid::{PidIGain, PidPGain};

fn request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 10_000,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_000,
        settle_travel_reserve_mrev: 200,
        negative_position_headroom_mrev: 50_000,
        positive_position_headroom_mrev: 50_000,
        max_duration_ms: 60_000,
        nomination_p_below: 0,
    }
}

fn live_recovery_request() -> VelocitySweepRequest {
    VelocitySweepRequest {
        requested_velocity_mrev_s: 6_666,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source: RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 2_500,
        settle_travel_reserve_mrev: 250,
        negative_position_headroom_mrev: 7_025,
        positive_position_headroom_mrev: 7_025,
        max_duration_ms: 3_000,
        nomination_p_below: 0,
    }
}

#[test]
fn live_recovery_geometry_extends_both_workflow_durations() {
    let request = live_recovery_request();
    let primitive_cap_us = u32::from(request.max_duration_ms) * 1_000;

    let plan = plan_velocity_sweep(
        request,
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();

    assert_eq!(plan.planned_velocity_mrev_s, 4_394);
    assert_eq!(plan.rung_count, 23);
    assert_eq!(plan.recovery_origin_band_counts, 1_000);
    assert_eq!(plan.recovery_nominal_slot_us, 1_816_958);
    assert_eq!(plan.recovery_maximum_slot_us, primitive_cap_us);
    assert_eq!(plan.recovery_slot_count, 23);
    assert_eq!(plan.nominal_workflow_ms, 271_423);
    assert_eq!(plan.maximum_workflow_ms, 298_632);
}

#[test]
fn proportional_plan_uses_operational_current_authority_and_new_schemas() {
    let historical = plan_velocity_sweep(
        live_recovery_request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let combined = plan_proportional_velocity_sweep(
        live_recovery_request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();

    assert_eq!(historical.schema_revision, 8);
    assert_eq!(historical.target_velocity_rpm.raw(), 264);
    assert_eq!(historical.usable_torque_limit, 2_534);
    assert_eq!(historical.p_top.raw(), 2_457);
    assert_eq!(combined.schema_revision, PROPORTIONAL_PLAN_SCHEMA_REVISION);
    assert_eq!(PROPORTIONAL_PLAN_SCHEMA_REVISION, 14);
    assert_eq!(combined.recovery_nominal_slot_us, 2_316_958);
    assert_eq!(combined.nominal_workflow_ms, 282_923);
    assert_eq!(combined.target_velocity_rpm.raw(), 263);
    assert_eq!(combined.usable_torque_limit, 2_816);
    assert_eq!(combined.p_top.raw(), 2_741);
    assert!(combined.digest() != historical.digest());
    assert_eq!(
        combined.execution_plan(combined.digest()).schema_revision,
        PROPORTIONAL_PLAN_SCHEMA_REVISION
    );
}

#[test]
fn combined_target_execution_stays_inside_the_declared_travel_cap() {
    let plan = plan_proportional_velocity_sweep(
        live_recovery_request(),
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();

    assert_eq!(plan.planned_velocity_mrev_s, 4_394);
    assert_eq!(plan.target_velocity_rpm.raw(), 263);
    assert_eq!(
        i64::from(plan.target_velocity_rpm.raw()) * 4_000 * 1_000 / 60,
        17_533_333
    );
}

#[test]
fn cap_safe_targets_never_exceed_supported_positive_caps() {
    for cap_mrev_s in 1..17 {
        assert_eq!(
            cap_safe_target_rpm(cap_mrev_s),
            Err(VelocityPlanError::ZeroMovingTravelAmplitude)
        );
    }
    for cap_mrev_s in 17..=100_000 {
        let target_rpm = cap_safe_target_rpm(cap_mrev_s).unwrap();
        assert!(u64::try_from(target_rpm).unwrap() * 1_000 <= u64::from(cap_mrev_s) * 60);
    }
}

#[test]
fn live_geometry_uses_mechanical_rpm_and_the_corrected_ladder() {
    let mut live = request();
    live.requested_velocity_mrev_s = 2_929;
    live.max_stroke_travel_mrev = 2_000;
    live.settle_travel_reserve_mrev = 250;

    let plan = plan_velocity_sweep(
        live,
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let gains: heapless::Vec<u16, MAX_SWEEP_RUNGS> = plan
        .gains
        .iter()
        .take(usize::from(plan.rung_count))
        .map(|gain| gain.raw())
        .collect();

    assert_eq!(plan.planned_velocity_mrev_s, 2_929);
    assert_eq!(plan.target_velocity_rpm.raw(), 176);
    assert_eq!(plan.p_start.raw(), 2);
    assert_eq!(plan.p_top.raw(), 3_685);
    assert_eq!(plan.rung_count, 23);
    assert_eq!(plan.observations_per_direction, 4);
    assert_eq!(plan.nominal_workflow_ms, 274_042);
    assert_eq!(plan.maximum_workflow_ms, 1_609_632);
    assert_eq!(
        u16::from(plan.rung_count) * plan.observations_per_direction,
        92
    );
    assert_eq!(
        gains.as_slice(),
        &[
            2, 3, 4, 6, 8, 12, 16, 23, 32, 46, 64, 91, 128, 181, 256, 362, 512, 724, 1_024, 1_448,
            2_048, 2_896, 3_685,
        ]
    );
    assert_eq!(usize::from(plan.rung_count) * 2, 46);
    assert!(gains.contains(&362));
}

#[test]
fn inverse_envelope_uses_both_named_margins() {
    assert_eq!(ENVELOPE_PROPORTIONAL_NUM, 3);
    assert_eq!(ENVELOPE_PROPORTIONAL_DEN, 2);
    assert_eq!(ENVELOPE_ABSOLUTE_MARGIN_MREV_S, 2_000);
    assert_eq!(envelope_cap(10_000), Ok(6_666));
    assert_eq!(envelope_cap(4_000), Ok(2_000));
    assert_eq!(envelope_cap(2_000), Err(VelocityPlanError::InvalidRequest));
}

#[test]
fn plan_reports_machine_and_encoder_ceiling_provenance() {
    let machine_plan = plan_velocity_sweep(
        request(),
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    assert_eq!(machine_plan.effective_ceiling_mrev_s, 10_000);
    assert_eq!(
        machine_plan.effective_ceiling_source,
        EFFECTIVE_CEILING_MACHINE
    );
    assert_ne!(machine_plan.clamp_flags & CLAMP_ENVELOPE, 0);

    let mut encoder_limited = request();
    encoder_limited.machine_velocity_ceiling_mrev_s = 200_000;
    encoder_limited.requested_velocity_mrev_s = 200_000;
    let encoder_plan = plan_velocity_sweep(
        encoder_limited,
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    assert_eq!(
        encoder_plan.effective_ceiling_source,
        EFFECTIVE_CEILING_ENCODER
    );
    assert!(encoder_plan.effective_ceiling_mrev_s < 125_000);
}

#[test]
fn every_firmware_representability_term_is_enumerated() {
    let limits = representability_ceilings(4_000).unwrap();
    assert_eq!(limits.encoder_observable_mrev_s, 12_487);
    assert!(TmcMechanicalRpm::from_mrev_s_nearest(u32::MAX).is_ok());
}

#[test]
fn effective_velocity_ceiling_is_the_lower_of_machine_and_encoder_bounds() {
    assert_eq!(effective_velocity_ceiling_mrev_s(10_000, 4_000), Ok(10_000));
    assert_eq!(effective_velocity_ceiling_mrev_s(20_000, 4_000), Ok(12_487));
    assert_eq!(
        effective_velocity_ceiling_mrev_s(10_000, 0),
        Err(VelocityPlanError::InvalidRequest)
    );
}

#[test]
fn encoder_observability_tracks_the_stressed_twenty_ms_gap_bound() {
    assert_eq!(MAX_SAFETY_OBSERVATION_GAP_US, 20_000);
    let limits = representability_ceilings(4_000).unwrap();
    assert_eq!(limits.encoder_observable_mrev_s, 12_487);

    let ceiling_at = |gap_us: u64| 999_u64 * 1_000_000_000 / (4_000 * gap_us);
    assert_eq!(ceiling_at(24_975), 10_000);
    assert_eq!(ceiling_at(25_000), 9_990);
    assert_eq!(20_000 / 5_000, 4);
}

#[test]
fn travel_reserve_is_subtracted_before_the_active_duration_cap() {
    assert_eq!(
        travel_cap(StrokeGeometry::OriginAnchored, 1_000, 100, 256_000),
        Ok(3_515)
    );
    assert_eq!(
        travel_cap(StrokeGeometry::OriginAnchored, 1_000, 100, 512_000),
        Ok(1_757)
    );
    assert_eq!(
        travel_cap(StrokeGeometry::OriginAnchored, 1_000, 1_000, 256_000),
        Err(VelocityPlanError::InvalidRequest)
    );
    assert_eq!(
        travel_cap(StrokeGeometry::OriginAnchored, 2, 1, 2_000_000),
        Err(VelocityPlanError::ZeroMovingTravelAmplitude)
    );
}

#[test]
fn travel_binding_has_a_named_clamp_cause() {
    let mut limited = request();
    limited.max_stroke_travel_mrev = 300;
    limited.settle_travel_reserve_mrev = 100;
    let plan = plan_velocity_sweep(
        limited,
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    assert_ne!(plan.clamp_flags & CLAMP_TRAVEL, 0);
    assert_eq!(plan.planned_velocity_mrev_s, 390);
}

#[test]
fn current_span_uses_quantum_start_and_inclusive_nine_tenths_top() {
    let (p_start, p_top, usable, _ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 1_000).unwrap();
    assert_eq!(CURRENT_HEADROOM_NUM, 9);
    assert_eq!(CURRENT_HEADROOM_DEN, 10);
    assert_eq!(usable, 900);
    assert_eq!(p_start.raw(), 3);
    assert_eq!(p_top.raw(), 2_304);
    assert_eq!(u32::from(p_top.raw()) * 100 / 256, u32::from(usable));
    assert!(usable < 1_000);
}

#[test]
fn ceiling_binding_source_reports_current_limit_when_it_binds() {
    let (_p_start, _p_top, _usable, ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(100), 1_000).unwrap();
    assert_eq!(ceiling_binding_source, CEILING_BINDING_CURRENT_LIMIT);
}

#[test]
fn ceiling_binding_source_reports_representability_clamp_when_it_binds() {
    let (p_start, p_top, _usable, ceiling_binding_source) =
        current_gain_span(TmcMechanicalRpm::from_register(1), 32_767).unwrap();
    assert_eq!(
        ceiling_binding_source,
        CEILING_BINDING_REPRESENTABILITY_CLAMP
    );
    assert_eq!(p_top, PidPGain::MAX);
    assert!(p_start.raw() <= p_top.raw());
}

#[test]
fn anchored_half_octaves_do_not_compound_approximation() {
    let ladder = build_gain_ladder(350, 1_500).unwrap();
    let raw: heapless::Vec<u16, MAX_SWEEP_RUNGS> = ladder.iter().map(|gain| gain.raw()).collect();
    assert_eq!(raw.as_slice(), &[350, 495, 700, 990, 1_400, 1_500]);
}

#[test]
fn low_gain_collisions_are_deduplicated_and_top_is_explicit() {
    let powers = build_gain_ladder(1, 16).unwrap();
    let bounded = build_gain_ladder(10, 90).unwrap();
    assert_eq!(
        powers
            .iter()
            .map(|gain| gain.raw())
            .collect::<heapless::Vec<_, 32>>()
            .as_slice(),
        &[1, 2, 3, 4, 6, 8, 12, 16]
    );
    assert_eq!(
        bounded
            .iter()
            .map(|gain| gain.raw())
            .collect::<heapless::Vec<_, 32>>()
            .as_slice(),
        &[10, 15, 20, 29, 40, 57, 80, 90]
    );
}

#[test]
fn structural_floor_counts_distinct_post_dedupe_gains() {
    assert_eq!(MIN_STRUCTURAL_RUNGS, 5);
    assert!(build_gain_ladder(1, 4).is_err());
    assert_eq!(build_gain_ladder(1, 8).unwrap().len(), 6);
}

#[test]
fn maximum_domain_stays_inside_the_audited_four_k_family() {
    let ladder = build_gain_ladder(2, 32_767).unwrap();
    assert!(ladder.len() <= MAX_SWEEP_RUNGS);
    assert!(ladder.len() * usize::from(OBSERVATIONS_PER_DIRECTION_PER_RUNG) <= 128);

    let synthetic_rung_count = MAX_SWEEP_RUNGS as u16;
    assert_eq!(
        synthetic_rung_count * OBSERVATIONS_PER_DIRECTION_PER_RUNG,
        128
    );
}

#[test]
fn plan_freezes_exact_ladder_and_workflow_duration() {
    let plan = plan_velocity_sweep(
        request(),
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let base_us = u32::from(plan.rung_count)
        * u32::from(SWEEP_SLOT_COUNT)
        * (MOVING_STROKE_US + ZERO_SETTLE_US + SETTLE_ANALYSIS_BUDGET_US);
    let nominal_recovery_us = u32::from(plan.rung_count) * plan.recovery_nominal_slot_us;
    let maximum_recovery_us = u32::from(plan.rung_count) * plan.recovery_maximum_slot_us;
    assert_eq!(plan.gains[0], plan.p_start);
    assert_eq!(plan.gains[usize::from(plan.rung_count) - 1], plan.p_top);
    assert_eq!(plan.observations_per_direction, 4);
    assert_eq!(
        plan.nominal_workflow_ms,
        (base_us + nominal_recovery_us).div_ceil(1_000)
    );
    assert_eq!(
        plan.maximum_workflow_ms,
        (base_us + maximum_recovery_us).div_ceil(1_000)
    );
}

#[test]
fn observation_count_changes_the_plan_digest() {
    let mut n2 = plan_velocity_sweep(
        request(),
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    n2.observations_per_direction = 2;
    let mut n4 = n2;
    n4.observations_per_direction = 4;

    assert_ne!(n2.digest(), n4.digest());
}

#[test]
fn compact_execution_plan_regenerates_every_emitted_gain() {
    for p_start in 1_u16..=64 {
        for p_top in p_start..=p_start.saturating_mul(32).min(PidPGain::MAX.raw()) {
            let Ok(ladder) = build_gain_ladder(p_start, p_top) else {
                continue;
            };
            for (index, expected) in ladder.iter().copied().enumerate() {
                assert_eq!(
                    gain_at(p_start, p_top, ladder.len() as u8, index as u8),
                    Some(expected)
                );
            }
            assert_eq!(
                gain_at(p_start, p_top, ladder.len() as u8, ladder.len() as u8),
                None
            );
        }
    }
}

#[test]
fn execution_plan_drops_report_only_geometry() {
    let plan = plan_velocity_sweep(
        request(),
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let execution = plan.execution_plan(0x1234_5678_9abc_def0);
    let report = plan.report(7, 0);

    assert!(core::mem::size_of_val(&execution) <= 32);
    assert_eq!(execution.plan_digest, 0x1234_5678_9abc_def0);
    assert_eq!(execution.gain_at(0), Some(plan.p_start));
    assert_eq!(execution.gain_at(plan.rung_count - 1), Some(plan.p_top));
    assert_eq!(execution.gain_at(plan.rung_count), None);
    assert_eq!(execution.target_velocity_rpm, plan.target_velocity_rpm);
    assert_eq!(report.max_stroke_travel_mrev, plan.max_stroke_travel_mrev);
    assert_eq!(
        report.settle_travel_reserve_mrev,
        plan.settle_travel_reserve_mrev
    );
    assert_eq!(
        report.negative_position_headroom_mrev,
        plan.negative_position_headroom_mrev
    );
    assert_eq!(
        report.positive_position_headroom_mrev,
        plan.positive_position_headroom_mrev
    );
}

#[test]
fn analysis_budget_preserves_the_shortest_scored_rest_suffix() {
    assert_eq!(MOVING_STROKE_US, 512_000);
    assert_eq!(ZERO_SETTLE_US, 500_000);
    assert_eq!(MIN_SETTLE_CAPTURE_US, 64_000);
    assert_eq!(SETTLE_ANALYSIS_BUDGET_US, 236_000);
    const { assert!(SETTLE_ANALYSIS_BUDGET_US + MIN_SETTLE_CAPTURE_US <= ZERO_SETTLE_US) };
}

#[test]
fn host_default_primitive_budget_does_not_cap_total_workflow() {
    let mut host_request = request();
    host_request.max_duration_ms = 3_000;

    let plan = plan_velocity_sweep(
        host_request,
        4_000,
        1_000,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();

    assert!(plan.maximum_workflow_ms > u32::from(host_request.max_duration_ms));
}

#[test]
fn primitive_budget_must_cover_one_moving_and_settle_window() {
    let mut too_short = request();
    too_short.max_duration_ms = 455;

    assert_eq!(
        plan_velocity_sweep(
            too_short,
            4_000,
            1_000,
            VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
        ),
        Err(VelocityPlanError::InvalidRequest)
    );
}

#[test]
fn centred_strokes_admit_twice_the_displacement_of_origin_anchored_ones() {
    for (window, reserve, stroke_us) in [
        (1_000_u32, 100_u32, 256_000_u32),
        (1_000, 100, 512_000),
        (2_500, 250, 512_000),
    ] {
        let anchored = travel_cap(StrokeGeometry::OriginAnchored, window, reserve, stroke_us)
            .expect("anchored cap");
        let centred = travel_cap(StrokeGeometry::OriginCentred, window, reserve, stroke_us)
            .expect("centred cap");
        assert_eq!(centred / 2, anchored, "window {window} stroke {stroke_us}");
        assert!(
            centred >= anchored * 2,
            "centred must admit at least twice the displacement"
        );
    }

    assert_eq!(
        StrokeGeometry::OriginAnchored.peak_excursion_mrev(2_250),
        2_250
    );
    assert_eq!(
        StrokeGeometry::OriginCentred.peak_excursion_mrev(2_250),
        1_125
    );

    assert_eq!(
        travel_cap(StrokeGeometry::OriginAnchored, 1_000, 100, 256_000),
        Ok(3_515)
    );
    assert_eq!(
        travel_cap(StrokeGeometry::OriginAnchored, 1_000, 100, 512_000),
        Ok(1_757)
    );

    for geometry in [
        StrokeGeometry::OriginAnchored,
        StrokeGeometry::OriginCentred,
    ] {
        assert_eq!(
            travel_cap(geometry, 1_000, 1_000, 256_000),
            Err(VelocityPlanError::InvalidRequest)
        );
    }
}

#[test]
fn half_octave_candidate_uses_ceiling_rounding_not_truncation() {
    assert_eq!(half_octave_candidate(4), Some(6));
    assert_eq!(half_octave_candidate(512), Some(724));
}

#[test]
fn build_gain_ladder_generic_over_pid_i_gain_matches_p_gain_shape() {
    let p_ladder: heapless::Vec<PidPGain, MAX_SWEEP_RUNGS> =
        build_gain_ladder_generic(4, 1024, true).unwrap();
    let i_ladder: heapless::Vec<PidIGain, MAX_SWEEP_RUNGS> =
        build_gain_ladder_generic(4, 1024, true).unwrap();
    let p_raw: heapless::Vec<u16, MAX_SWEEP_RUNGS> = p_ladder.iter().map(|g| g.raw()).collect();
    let i_raw: heapless::Vec<u16, MAX_SWEEP_RUNGS> = i_ladder.iter().map(|g| g.raw()).collect();
    assert_eq!(p_raw, i_raw);
}

#[test]
fn build_gain_ladder_generic_include_exact_top_false_omits_non_octave_cap() {
    let ladder: heapless::Vec<PidPGain, MAX_SWEEP_RUNGS> =
        build_gain_ladder_generic(4, 1310, false).unwrap();
    assert_eq!(ladder.last().unwrap().raw(), 1024);
    assert!(!ladder.iter().any(|g| g.raw() == 1310));
}

#[test]
fn build_gain_ladder_p_wrapper_byte_identical_after_generify() {
    let ladder = build_gain_ladder(4, 1024).unwrap();
    let raw: heapless::Vec<u16, MAX_SWEEP_RUNGS> = ladder.iter().map(|g| g.raw()).collect();
    assert_eq!(
        raw.as_slice(),
        &[
            4, 6, 8, 12, 16, 23, 32, 46, 64, 91, 128, 181, 256, 362, 512, 724, 1024
        ]
    );
}

#[test]
fn the_nomination_ceiling_does_not_enter_the_plan_digest() {
    let base = live_recovery_request();
    let ceiled = VelocitySweepRequest {
        nomination_p_below: 896,
        ..base
    };

    let plan_a = plan_proportional_velocity_sweep(
        base,
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();
    let plan_b = plan_proportional_velocity_sweep(
        ceiled,
        4_000,
        2_816,
        VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT,
    )
    .unwrap();

    assert_eq!(plan_a.digest(), plan_b.digest());
}
