//! Diagnostic codes and counters reported to the host, plus persisted last-boot fault records.

use core::ptr::{addr_of, addr_of_mut};
use core::sync::atomic::Ordering::Relaxed;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

const BREAKAWAY_BREADCRUMB_MAGIC: u32 = 0x4642_5200;
const BREAKAWAY_BREADCRUMB_MAGIC_MASK: u32 = 0xffff_ff00;
const BREAKAWAY_BREADCRUMB_COMPLETED: u32 = 1 << 7;
const BREAKAWAY_BREADCRUMB_CHECKPOINT_MASK: u32 = 0x7f;

/// Breakaway-campaign lifecycle boundary retained across warm reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum BreakawayCheckpoint {
    CampaignInstalled = 1,
    SafetyConfigured = 2,
    GainProgramming = 3,
    TargetCommand = 4,
    Capture = 5,
    EventRouting = 6,
    CampaignTerminalHandled = 7,
    IntegralHandedOff = 8,
    ProbeSafetyEncoderRead = 9,
    ProbeGainWrite = 10,
    ProbeRampModeWrite = 11,
    ProbeGainIntegratorClear = 12,
    ProbeTargetBaselineRead = 13,
    ProbeClampInstall = 14,
    ProbeCurrentPrimitiveStart = 15,
    ProbeTargetWrite = 16,
    ProbeMovingEncoderSample = 17,
    ProbeMovingCurrentInputSample = 18,
    ProbeMovingCurrentOutputSample = 19,
    ProbeMovingBoundary = 20,
    ProbeZeroTargetWrite = 21,
    ProbeZeroIntegratorClear = 22,
    ProbeSettleBaselineRead = 23,
    ProbeSettleEncoderSample = 24,
    ProbeSettleCurrentInputSample = 25,
    ProbeSettleCurrentOutputSample = 26,
    ProbeZeroBoundary = 27,
    RecoveryGainWrite = 28,
    RecoveryMovingIntegratorClear = 29,
    RecoveryCurrentPrimitiveStart = 30,
    RecoveryTargetWrite = 31,
    RecoveryMovingSafetySample = 32,
    RecoveryMovingCurrentInputSample = 33,
    RecoveryMovingCurrentOutputSample = 34,
    RecoveryMovingBoundary = 35,
    RecoveryZeroTargetWrite = 36,
    RecoverySettleIntegratorClear = 37,
    RecoverySettleSafetySample = 38,
    RecoverySettleCurrentInputSample = 39,
    RecoverySettleCurrentOutputSample = 40,
    RecoveryZeroBoundary = 41,
    ProbeExcursionIntervalInvalid = 42,
    ProbeExcursionArithmeticInvalid = 43,
}

impl BreakawayCheckpoint {
    const fn from_raw(raw: u8) -> Option<Self> {
        match raw {
            1 => Some(Self::CampaignInstalled),
            2 => Some(Self::SafetyConfigured),
            3 => Some(Self::GainProgramming),
            4 => Some(Self::TargetCommand),
            5 => Some(Self::Capture),
            6 => Some(Self::EventRouting),
            7 => Some(Self::CampaignTerminalHandled),
            8 => Some(Self::IntegralHandedOff),
            9 => Some(Self::ProbeSafetyEncoderRead),
            10 => Some(Self::ProbeGainWrite),
            11 => Some(Self::ProbeRampModeWrite),
            12 => Some(Self::ProbeGainIntegratorClear),
            13 => Some(Self::ProbeTargetBaselineRead),
            14 => Some(Self::ProbeClampInstall),
            15 => Some(Self::ProbeCurrentPrimitiveStart),
            16 => Some(Self::ProbeTargetWrite),
            17 => Some(Self::ProbeMovingEncoderSample),
            18 => Some(Self::ProbeMovingCurrentInputSample),
            19 => Some(Self::ProbeMovingCurrentOutputSample),
            20 => Some(Self::ProbeMovingBoundary),
            21 => Some(Self::ProbeZeroTargetWrite),
            22 => Some(Self::ProbeZeroIntegratorClear),
            23 => Some(Self::ProbeSettleBaselineRead),
            24 => Some(Self::ProbeSettleEncoderSample),
            25 => Some(Self::ProbeSettleCurrentInputSample),
            26 => Some(Self::ProbeSettleCurrentOutputSample),
            27 => Some(Self::ProbeZeroBoundary),
            28 => Some(Self::RecoveryGainWrite),
            29 => Some(Self::RecoveryMovingIntegratorClear),
            30 => Some(Self::RecoveryCurrentPrimitiveStart),
            31 => Some(Self::RecoveryTargetWrite),
            32 => Some(Self::RecoveryMovingSafetySample),
            33 => Some(Self::RecoveryMovingCurrentInputSample),
            34 => Some(Self::RecoveryMovingCurrentOutputSample),
            35 => Some(Self::RecoveryMovingBoundary),
            36 => Some(Self::RecoveryZeroTargetWrite),
            37 => Some(Self::RecoverySettleIntegratorClear),
            38 => Some(Self::RecoverySettleSafetySample),
            39 => Some(Self::RecoverySettleCurrentInputSample),
            40 => Some(Self::RecoverySettleCurrentOutputSample),
            41 => Some(Self::RecoveryZeroBoundary),
            42 => Some(Self::ProbeExcursionIntervalInvalid),
            43 => Some(Self::ProbeExcursionArithmeticInvalid),
            _ => None,
        }
    }
}

/// Checkpoint recovered from persistent CCMRAM; `interrupted` means reset hit an active campaign.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct BreakawayBreadcrumb {
    pub checkpoint: BreakawayCheckpoint,
    pub interrupted: bool,
}

#[cfg(test)]
struct BreakawayBreadcrumbStore {
    word: AtomicU32,
}

#[cfg(test)]
impl BreakawayBreadcrumbStore {
    fn new_for_test(raw: u32) -> Self {
        Self {
            word: AtomicU32::new(raw),
        }
    }

    fn load(&self) -> Option<BreakawayBreadcrumb> {
        decode_breakaway_breadcrumb(self.word.load(Ordering::Acquire))
    }

    fn begin(&self, checkpoint: BreakawayCheckpoint) {
        self.store(checkpoint, false);
    }

    fn checkpoint(&self, checkpoint: BreakawayCheckpoint) {
        self.store(checkpoint, false);
    }

    fn complete(&self, checkpoint: BreakawayCheckpoint) {
        self.store(checkpoint, true);
    }

    fn store(&self, checkpoint: BreakawayCheckpoint, completed: bool) {
        self.word.store(
            encode_breakaway_breadcrumb(checkpoint, completed),
            Ordering::Release,
        );
    }

    fn raw_for_test(&self) -> u32 {
        self.word.load(Ordering::Acquire)
    }
}

const fn encode_breakaway_breadcrumb(checkpoint: BreakawayCheckpoint, completed: bool) -> u32 {
    let completion = if completed {
        BREAKAWAY_BREADCRUMB_COMPLETED
    } else {
        0
    };
    BREAKAWAY_BREADCRUMB_MAGIC | completion | checkpoint as u32
}

const fn decode_breakaway_breadcrumb(raw: u32) -> Option<BreakawayBreadcrumb> {
    if raw & BREAKAWAY_BREADCRUMB_MAGIC_MASK != BREAKAWAY_BREADCRUMB_MAGIC {
        return None;
    }
    let checkpoint =
        match BreakawayCheckpoint::from_raw((raw & BREAKAWAY_BREADCRUMB_CHECKPOINT_MASK) as u8) {
            Some(checkpoint) => checkpoint,
            None => return None,
        };
    Some(BreakawayBreadcrumb {
        checkpoint,
        interrupted: raw & BREAKAWAY_BREADCRUMB_COMPLETED == 0,
    })
}

#[cfg(not(test))]
#[cfg_attr(target_os = "none", unsafe(link_section = ".ccmram.diag.breakaway"))]
#[used]
static BREAKAWAY_BREADCRUMB: AtomicU32 = AtomicU32::new(0);

static LAST_BOOT_BREAKAWAY_BREADCRUMB: AtomicU32 = AtomicU32::new(0);

#[cfg(test)]
std::thread_local! {
    static TEST_BREAKAWAY_BREADCRUMB: core::cell::Cell<u32> = const { core::cell::Cell::new(0) };
    static PANIC_AFTER_BREAKAWAY_COMPLETION: core::cell::Cell<bool> = const {
        core::cell::Cell::new(false)
    };
}

#[cfg(not(test))]
fn persistent_breakaway_breadcrumb() -> u32 {
    BREAKAWAY_BREADCRUMB.load(Ordering::Acquire)
}

#[cfg(test)]
fn persistent_breakaway_breadcrumb() -> u32 {
    TEST_BREAKAWAY_BREADCRUMB.with(core::cell::Cell::get)
}

#[cfg(not(test))]
fn store_persistent_breakaway_breadcrumb(checkpoint: BreakawayCheckpoint, completed: bool) {
    BREAKAWAY_BREADCRUMB.store(
        encode_breakaway_breadcrumb(checkpoint, completed),
        Ordering::Release,
    );
}

#[cfg(test)]
fn store_persistent_breakaway_breadcrumb(checkpoint: BreakawayCheckpoint, completed: bool) {
    TEST_BREAKAWAY_BREADCRUMB
        .with(|word| word.set(encode_breakaway_breadcrumb(checkpoint, completed)));
}

/// Snapshots the persistent breadcrumb for this boot; call once at early startup, before any
/// campaign updates it. The persistent word is left intact.
pub fn recover_last_boot_breakaway_breadcrumb() {
    let raw = persistent_breakaway_breadcrumb();
    let recovered = if decode_breakaway_breadcrumb(raw).is_some() {
        raw
    } else {
        0
    };
    LAST_BOOT_BREAKAWAY_BREADCRUMB.store(recovered, Ordering::Release);
}

/// Takes the validated last-boot breadcrumb, at most once per boot.
pub fn take_last_boot_breakaway_breadcrumb() -> Option<BreakawayBreadcrumb> {
    decode_breakaway_breadcrumb(LAST_BOOT_BREAKAWAY_BREADCRUMB.swap(0, Ordering::AcqRel))
}

/// Mark a successfully installed breakaway campaign active.
pub fn begin_breakaway_campaign() {
    store_persistent_breakaway_breadcrumb(BreakawayCheckpoint::CampaignInstalled, false);
}

/// Record one active breakaway-campaign lifecycle boundary.
pub fn checkpoint_breakaway_campaign(checkpoint: BreakawayCheckpoint) {
    store_persistent_breakaway_breadcrumb(checkpoint, false);
}

/// Current checkpoint of an active (not completed) campaign.
pub fn active_breakaway_checkpoint() -> Option<BreakawayCheckpoint> {
    let breadcrumb = decode_breakaway_breadcrumb(persistent_breakaway_breadcrumb())?;
    if breadcrumb.interrupted {
        Some(breadcrumb.checkpoint)
    } else {
        None
    }
}

/// Records a checkpoint only if a campaign is already active, so shared primitives cannot
/// create a breadcrumb for another workflow.
pub fn checkpoint_active_breakaway_campaign(checkpoint: BreakawayCheckpoint) {
    if active_breakaway_checkpoint().is_some() {
        store_persistent_breakaway_breadcrumb(checkpoint, false);
    }
}

/// Mark a handled breakaway-campaign terminal boundary completed.
pub fn complete_breakaway_campaign(checkpoint: BreakawayCheckpoint) {
    store_persistent_breakaway_breadcrumb(checkpoint, true);
    #[cfg(test)]
    PANIC_AFTER_BREAKAWAY_COMPLETION.with(|armed| {
        if armed.replace(false) {
            panic!("simulated reset after breakaway completion");
        }
    });
}

#[cfg(test)]
pub(crate) fn set_breakaway_breadcrumb_raw_for_test(raw: u32) {
    TEST_BREAKAWAY_BREADCRUMB.with(|word| word.set(raw));
    LAST_BOOT_BREAKAWAY_BREADCRUMB.store(0, Ordering::Release);
}

#[cfg(test)]
pub(crate) fn current_breakaway_breadcrumb_for_test() -> Option<BreakawayBreadcrumb> {
    decode_breakaway_breadcrumb(TEST_BREAKAWAY_BREADCRUMB.with(core::cell::Cell::get))
}

#[cfg(test)]
pub(crate) fn panic_after_breakaway_completion_for_test() {
    PANIC_AFTER_BREAKAWAY_COMPLETION.with(|armed| armed.set(true));
}

/// Origin of a persisted last-boot fault record.
pub mod last_panic_kind {
    /// Rust panic; `line` and `file_hash` locate it.
    pub const RUST_PANIC: u8 = 1;
    /// CPU HardFault; `pc` locates it.
    pub const HARD_FAULT: u8 = 2;
}

/// Previous-boot fault record recovered from no-init RAM. `line` and `file_hash` (FNV-1a of the
/// source path) locate a [`last_panic_kind::RUST_PANIC`]; `pc` locates a
/// [`last_panic_kind::HARD_FAULT`]. Unavailable fields are 0; `kind` is one of [`last_panic_kind`].
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LastPanic {
    pub kind: u8,
    pub line: u32,
    pub pc: u32,
    pub file_hash: u32,
}

static LAST_PANIC_PRESENT: AtomicBool = AtomicBool::new(false);

static mut LAST_PANIC_RECORD: LastPanic = LastPanic {
    kind: 0,
    line: 0,
    pc: 0,
    file_hash: 0,
};

/// Stores the recovered last-boot fault record; call once at startup, before the transport is up.
pub fn set_last_panic(record: LastPanic) {
    // SAFETY: startup is single-threaded and runs before any reader; the
    // release store below publishes the write to a later acquiring reader.
    unsafe {
        *addr_of_mut!(LAST_PANIC_RECORD) = record;
    }
    LAST_PANIC_PRESENT.store(true, Ordering::Release);
}

/// Takes and clears the stored last-boot fault record, so it is reported once.
pub fn take_last_panic() -> Option<LastPanic> {
    if LAST_PANIC_PRESENT.swap(false, Ordering::AcqRel) {
        // SAFETY: the acquire above pairs with the release in `set_last_panic`,
        // and the flag guarantees no concurrent writer for this read.
        Some(unsafe { *addr_of!(LAST_PANIC_RECORD) })
    } else {
        None
    }
}

/// Fault class carried by `foci_diagnostic`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FaultClass {
    QueueFull = 1,
    UnknownOid = 2,
    ActiveReset = 3,
    InvalidChannel = 4,
}

/// Operation code carried by `foci_diagnostic`.
pub mod operation {
    /// `queue_step`.
    pub const QUEUE_STEP: u8 = 1;
    /// `set_next_step_dir`.
    pub const SET_NEXT_STEP_DIR: u8 = 2;
    /// Reserved (`reset_step_clock`); never reassign.
    pub const RESET_STEP_CLOCK: u8 = 3;
    /// `tmc_control`.
    pub const TMC_CONTROL: u8 = 4;
    /// Reserved (`stepper_stop`); never reassign.
    pub const STEPPER_STOP: u8 = 5;
    /// Stepper OID/channel lookup before a motion command.
    pub const STEPPER_LOOKUP: u8 = 6;
}

/// Bounded stepper runtime event reason carried by `foci_stepper_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum StepperEventReason {
    QueueEmpty = 1,
    MissedDeadlineLoad = 2,
    MissedDeadlineStep = 3,
    TrsyncStop = 4,
    P1Stop = 5,
    ResetStepClock = 6,
    TmcDisableSignal = 7,
    /// Diagnostic: a queue-time arm decision (`data0` = arm flags, `data1` = 1 when armed).
    ArmDecision = 8,
    /// Diagnostic: a P2 arm-check outcome (`data0` = arm flags, `data1` = 1 when armed).
    ArmCheck = 9,
    /// Diagnostic: a P2 stop-check (`data0` = arm flags before the clear).
    StopCheck = 10,
    /// Diagnostic: the step ISR skipped a channel that holds queued moves (`data0` = arm flags).
    NotReadyWithMoves = 11,
}

impl StepperEventReason {
    /// Short name for host log formatting.
    pub const fn name(self) -> &'static str {
        match self {
            Self::QueueEmpty => "queue_empty",
            Self::MissedDeadlineLoad => "missed_deadline_load",
            Self::MissedDeadlineStep => "missed_deadline_step",
            Self::TrsyncStop => "trsync_stop",
            Self::P1Stop => "p1_stop",
            Self::ResetStepClock => "reset_step_clock",
            Self::TmcDisableSignal => "tmc_disable_signal",
            Self::ArmDecision => "arm_decision",
            Self::ArmCheck => "arm_check",
            Self::StopCheck => "stop_check",
            Self::NotReadyWithMoves => "not_ready_with_moves",
        }
    }
}

/// Stepper runtime event reported to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperDiagnosticEvent {
    pub reason: StepperEventReason,
    pub channel: u8,
    pub position: i32,
    /// TIM5/Klipper clock at event time.
    pub clock: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: bool,
    /// Board-local queue length, or `u16::MAX` if unknown at the call site.
    pub queue_len: u16,
    pub direction: bool,
    /// Reason-specific value.
    pub data0: u32,
    /// Reason-specific value.
    pub data1: u32,
}

/// Board-local step queue and execution counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StepperExecutionSnapshot {
    pub channel: u8,
    pub position: i32,
    /// Accepted `queue_step` segments since boot.
    pub queued_segments: u32,
    /// Accepted motor steps since boot.
    pub queued_steps: u32,
    /// Segments loaded by the timer ISR since boot.
    pub loaded_segments: u32,
    /// Motor steps loaded by the timer ISR since boot.
    pub loaded_steps: u32,
    /// Executed positive-direction logical motor steps since boot.
    pub executed_pos_steps: u32,
    /// Executed negative-direction logical motor steps since boot.
    pub executed_neg_steps: u32,
    /// Positive-direction physical STEP pulses emitted since boot.
    pub physical_pos_pulses: u32,
    /// Negative-direction physical STEP pulses emitted since boot.
    pub physical_neg_pulses: u32,
    pub activation_count: u32,
    /// TIM5 clock when the step timer was most recently armed.
    pub last_activation_clock: u32,
    /// TIM5 clock when the first segment of the last activation loaded.
    pub first_load_now: u32,
    pub first_load_scheduled: u32,
    pub first_load_compare: u32,
    /// `first_load_scheduled - first_load_now`, interpreted as signed ticks.
    pub first_load_lead_ticks: i32,
    /// `first_load_compare - first_load_now`, interpreted as signed ticks.
    pub first_load_compare_delay_ticks: i32,
    /// TIM5 clock observed at the first rising edge of the last activation.
    pub first_step_clock: u32,
    /// `first_step_clock - last_activation_clock`, interpreted as signed ticks.
    pub first_step_delay_ticks: i32,
    pub discarded_segments: u32,
    pub discarded_steps: u32,
    pub queue_empty_count: u32,
    pub missed_deadline_count: u32,
    pub stop_count: u32,
    /// Queued segments drained by physical stop paths since boot.
    pub stop_drained_segments: u32,
    /// Queued motor steps drained by physical stop paths since boot.
    pub stop_drained_steps: u32,
    pub reset_count: u32,
    /// Queued segments drained by `reset_step_clock` since boot.
    pub reset_drained_segments: u32,
    /// Queued motor steps drained by `reset_step_clock` since boot.
    pub reset_drained_steps: u32,
    /// [`StepperEventReason`] code for the most recent physical stop.
    pub last_stop_reason: u8,
    pub last_stop_remaining_events: u32,
    pub last_stop_queue_len: u16,
    pub last_stop_drained_segments: u32,
    pub last_stop_drained_steps: u32,
    pub timer_active: bool,
    pub queue_len: u16,
}

struct StepperExecutionCounters {
    queued_segments: AtomicU32,
    queued_steps: AtomicU32,
    loaded_segments: AtomicU32,
    loaded_steps: AtomicU32,
    executed_pos_steps: AtomicU32,
    executed_neg_steps: AtomicU32,
    physical_pos_pulses: AtomicU32,
    physical_neg_pulses: AtomicU32,
    activation_count: AtomicU32,
    last_activation_clock: AtomicU32,
    activation_loaded_segments: AtomicU32,
    activation_executed_steps: AtomicU32,
    first_load_now: AtomicU32,
    first_load_scheduled: AtomicU32,
    first_load_compare: AtomicU32,
    first_load_lead_ticks: AtomicI32,
    first_load_compare_delay_ticks: AtomicI32,
    first_step_clock: AtomicU32,
    first_step_delay_ticks: AtomicI32,
    discarded_segments: AtomicU32,
    discarded_steps: AtomicU32,
    queue_empty_count: AtomicU32,
    missed_deadline_count: AtomicU32,
    stop_count: AtomicU32,
    stop_drained_segments: AtomicU32,
    stop_drained_steps: AtomicU32,
    reset_count: AtomicU32,
    reset_drained_segments: AtomicU32,
    reset_drained_steps: AtomicU32,
    last_stop_reason: AtomicU32,
    last_stop_remaining_events: AtomicU32,
    last_stop_queue_len: AtomicU32,
    last_stop_drained_segments: AtomicU32,
    last_stop_drained_steps: AtomicU32,
}

impl StepperExecutionCounters {
    const fn new() -> Self {
        Self {
            queued_segments: AtomicU32::new(0),
            queued_steps: AtomicU32::new(0),
            loaded_segments: AtomicU32::new(0),
            loaded_steps: AtomicU32::new(0),
            executed_pos_steps: AtomicU32::new(0),
            executed_neg_steps: AtomicU32::new(0),
            physical_pos_pulses: AtomicU32::new(0),
            physical_neg_pulses: AtomicU32::new(0),
            activation_count: AtomicU32::new(0),
            last_activation_clock: AtomicU32::new(0),
            activation_loaded_segments: AtomicU32::new(0),
            activation_executed_steps: AtomicU32::new(0),
            first_load_now: AtomicU32::new(0),
            first_load_scheduled: AtomicU32::new(0),
            first_load_compare: AtomicU32::new(0),
            first_load_lead_ticks: AtomicI32::new(0),
            first_load_compare_delay_ticks: AtomicI32::new(0),
            first_step_clock: AtomicU32::new(0),
            first_step_delay_ticks: AtomicI32::new(0),
            discarded_segments: AtomicU32::new(0),
            discarded_steps: AtomicU32::new(0),
            queue_empty_count: AtomicU32::new(0),
            missed_deadline_count: AtomicU32::new(0),
            stop_count: AtomicU32::new(0),
            stop_drained_segments: AtomicU32::new(0),
            stop_drained_steps: AtomicU32::new(0),
            reset_count: AtomicU32::new(0),
            reset_drained_segments: AtomicU32::new(0),
            reset_drained_steps: AtomicU32::new(0),
            last_stop_reason: AtomicU32::new(0),
            last_stop_remaining_events: AtomicU32::new(0),
            last_stop_queue_len: AtomicU32::new(0),
            last_stop_drained_segments: AtomicU32::new(0),
            last_stop_drained_steps: AtomicU32::new(0),
        }
    }
}

/// Step queue and execution counters over relaxed atomics; a snapshot may mix instants.
pub struct StepperExecutionStats<const CHANNELS: usize> {
    counters: [StepperExecutionCounters; CHANNELS],
}

impl<const CHANNELS: usize> StepperExecutionStats<CHANNELS> {
    pub const fn new() -> Self {
        Self {
            counters: [const { StepperExecutionCounters::new() }; CHANNELS],
        }
    }

    fn counters(&self, channel: u8) -> Option<&StepperExecutionCounters> {
        self.counters.get(channel as usize)
    }

    /// Record that the board armed the step timer for a new activation.
    pub fn record_timer_activation(&self, channel: u8, clock: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.activation_count.fetch_add(1, Relaxed);
            counters.last_activation_clock.store(clock, Relaxed);
            counters.activation_loaded_segments.store(0, Relaxed);
            counters.activation_executed_steps.store(0, Relaxed);
            counters.first_load_now.store(0, Relaxed);
            counters.first_load_scheduled.store(0, Relaxed);
            counters.first_load_compare.store(0, Relaxed);
            counters.first_load_lead_ticks.store(0, Relaxed);
            counters.first_load_compare_delay_ticks.store(0, Relaxed);
            counters.first_step_clock.store(0, Relaxed);
            counters.first_step_delay_ticks.store(0, Relaxed);
        }
    }

    /// Record an accepted `queue_step` segment.
    pub fn record_queued_segment(&self, channel: u8, steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.queued_segments.fetch_add(1, Relaxed);
            counters.queued_steps.fetch_add(steps, Relaxed);
        }
    }

    /// Record a segment loaded by the timer ISR.
    pub fn record_loaded_segment(&self, channel: u8, steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.loaded_segments.fetch_add(1, Relaxed);
            counters.loaded_steps.fetch_add(steps, Relaxed);
        }
    }

    /// Record a segment loaded by the timer ISR with scheduling context.
    pub fn record_loaded_segment_timing(
        &self,
        channel: u8,
        steps: u32,
        now: u32,
        scheduled: u32,
        compare: u32,
    ) {
        self.record_loaded_segment(channel, steps);
        if let Some(counters) = self.counters(channel)
            && counters.activation_loaded_segments.fetch_add(1, Relaxed) == 0
        {
            counters.first_load_now.store(now, Relaxed);
            counters.first_load_scheduled.store(scheduled, Relaxed);
            counters.first_load_compare.store(compare, Relaxed);
            counters
                .first_load_lead_ticks
                .store(scheduled.wrapping_sub(now) as i32, Relaxed);
            counters
                .first_load_compare_delay_ticks
                .store(compare.wrapping_sub(now) as i32, Relaxed);
        }
    }

    /// Totals are since-boot wrapping `u32`s, unaffected by timer activation and `reset_step_clock`.
    pub fn record_executed_step(
        &self,
        channel: u8,
        forward: bool,
        physical_pulses: u8,
        clock: u32,
    ) {
        if let Some(counters) = self.counters(channel) {
            if forward {
                counters.executed_pos_steps.fetch_add(1, Relaxed);
                counters
                    .physical_pos_pulses
                    .fetch_add(u32::from(physical_pulses), Relaxed);
            } else {
                counters.executed_neg_steps.fetch_add(1, Relaxed);
                counters
                    .physical_neg_pulses
                    .fetch_add(u32::from(physical_pulses), Relaxed);
            }
            if counters.activation_executed_steps.fetch_add(1, Relaxed) == 0 {
                let activation_clock = counters.last_activation_clock.load(Relaxed);
                counters.first_step_clock.store(clock, Relaxed);
                counters
                    .first_step_delay_ticks
                    .store(clock.wrapping_sub(activation_clock) as i32, Relaxed);
            }
        }
    }

    /// Record a `queue_step` segment discarded while awaiting reset.
    pub fn record_discarded_segment(&self, channel: u8, steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.discarded_segments.fetch_add(1, Relaxed);
            counters.discarded_steps.fetch_add(steps, Relaxed);
        }
    }

    /// Record a queue-empty step timer stop.
    pub fn record_queue_empty(&self, channel: u8) {
        if let Some(counters) = self.counters(channel) {
            counters.queue_empty_count.fetch_add(1, Relaxed);
        }
    }

    /// Record a missed step deadline.
    pub fn record_missed_deadline(&self, channel: u8) {
        if let Some(counters) = self.counters(channel) {
            counters.missed_deadline_count.fetch_add(1, Relaxed);
        }
    }

    /// Record a physical stop snapshot before the active segment is reset.
    pub fn record_physical_stop(
        &self,
        channel: u8,
        reason: StepperEventReason,
        remaining_events: u32,
        queue_len: u16,
        drained_segments: u32,
        drained_steps: u32,
    ) {
        if let Some(counters) = self.counters(channel) {
            counters.stop_count.fetch_add(1, Relaxed);
            counters
                .stop_drained_segments
                .fetch_add(drained_segments, Relaxed);
            counters
                .stop_drained_steps
                .fetch_add(drained_steps, Relaxed);
            counters.last_stop_reason.store(reason as u32, Relaxed);
            counters
                .last_stop_remaining_events
                .store(remaining_events, Relaxed);
            counters
                .last_stop_queue_len
                .store(u32::from(queue_len), Relaxed);
            counters
                .last_stop_drained_segments
                .store(drained_segments, Relaxed);
            counters
                .last_stop_drained_steps
                .store(drained_steps, Relaxed);
        }
    }

    /// Record an accepted step-clock reset.
    pub fn record_reset(&self, channel: u8, drained_segments: u32, drained_steps: u32) {
        if let Some(counters) = self.counters(channel) {
            counters.reset_count.fetch_add(1, Relaxed);
            counters
                .reset_drained_segments
                .fetch_add(drained_segments, Relaxed);
            counters
                .reset_drained_steps
                .fetch_add(drained_steps, Relaxed);
        }
    }

    /// Return a no-motion snapshot of stepper execution counters.
    pub fn snapshot(
        &self,
        channel: u8,
        position: i32,
        timer_active: bool,
        queue_len: u16,
    ) -> Option<StepperExecutionSnapshot> {
        self.counters(channel)
            .map(|counters| StepperExecutionSnapshot {
                channel,
                position,
                queued_segments: counters.queued_segments.load(Relaxed),
                queued_steps: counters.queued_steps.load(Relaxed),
                loaded_segments: counters.loaded_segments.load(Relaxed),
                loaded_steps: counters.loaded_steps.load(Relaxed),
                executed_pos_steps: counters.executed_pos_steps.load(Relaxed),
                executed_neg_steps: counters.executed_neg_steps.load(Relaxed),
                physical_pos_pulses: counters.physical_pos_pulses.load(Relaxed),
                physical_neg_pulses: counters.physical_neg_pulses.load(Relaxed),
                activation_count: counters.activation_count.load(Relaxed),
                last_activation_clock: counters.last_activation_clock.load(Relaxed),
                first_load_now: counters.first_load_now.load(Relaxed),
                first_load_scheduled: counters.first_load_scheduled.load(Relaxed),
                first_load_compare: counters.first_load_compare.load(Relaxed),
                first_load_lead_ticks: counters.first_load_lead_ticks.load(Relaxed),
                first_load_compare_delay_ticks: counters
                    .first_load_compare_delay_ticks
                    .load(Relaxed),
                first_step_clock: counters.first_step_clock.load(Relaxed),
                first_step_delay_ticks: counters.first_step_delay_ticks.load(Relaxed),
                discarded_segments: counters.discarded_segments.load(Relaxed),
                discarded_steps: counters.discarded_steps.load(Relaxed),
                queue_empty_count: counters.queue_empty_count.load(Relaxed),
                missed_deadline_count: counters.missed_deadline_count.load(Relaxed),
                stop_count: counters.stop_count.load(Relaxed),
                stop_drained_segments: counters.stop_drained_segments.load(Relaxed),
                stop_drained_steps: counters.stop_drained_steps.load(Relaxed),
                reset_count: counters.reset_count.load(Relaxed),
                reset_drained_segments: counters.reset_drained_segments.load(Relaxed),
                reset_drained_steps: counters.reset_drained_steps.load(Relaxed),
                last_stop_reason: counters.last_stop_reason.load(Relaxed) as u8,
                last_stop_remaining_events: counters.last_stop_remaining_events.load(Relaxed),
                last_stop_queue_len: counters.last_stop_queue_len.load(Relaxed) as u16,
                last_stop_drained_segments: counters.last_stop_drained_segments.load(Relaxed),
                last_stop_drained_steps: counters.last_stop_drained_steps.load(Relaxed),
                timer_active,
                queue_len,
            })
    }
}

impl<const CHANNELS: usize> Default for StepperExecutionStats<CHANNELS> {
    fn default() -> Self {
        Self::new()
    }
}

/// Bounded FIFO queue that drops the oldest entry on overflow.
pub struct StepperDiagnosticQueue<T, const N: usize> {
    queue: heapless::Deque<T, N>,
}

impl<T, const N: usize> StepperDiagnosticQueue<T, N> {
    pub const fn new() -> Self {
        Self {
            queue: heapless::Deque::new(),
        }
    }

    /// Push a new event, dropping the oldest retained event on overflow.
    pub fn push_drop_oldest(&mut self, event: T) {
        if self.queue.is_full() {
            let _ = self.queue.pop_front();
        }
        let _ = self.queue.push_back(event);
    }

    /// Pop the oldest retained event.
    pub fn pop(&mut self) -> Option<T> {
        self.queue.pop_front()
    }

    /// Return the number of retained events.
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// Return true when no events are retained.
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

impl<T, const N: usize> Default for StepperDiagnosticQueue<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Nonzero `overlap_count` means a P2 reply was sent inside P1's reply-construction window.
/// Counters only: never log at these call sites (reentrant defmt-rtt panic).
struct ReplyOverlapCounters {
    p1_in_flight: AtomicBool,
    overlap_count: AtomicU32,
}

impl ReplyOverlapCounters {
    const fn new() -> Self {
        Self {
            p1_in_flight: AtomicBool::new(false),
            overlap_count: AtomicU32::new(0),
        }
    }

    fn mark_p1_start(&self) {
        self.p1_in_flight.store(true, Ordering::Release);
    }

    fn mark_p1_end(&self) {
        self.p1_in_flight.store(false, Ordering::Release);
    }

    fn record_p2_send(&self) {
        if self.p1_in_flight.load(Ordering::Acquire) {
            self.overlap_count.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn overlap_count(&self) -> u32 {
        self.overlap_count.load(Ordering::Relaxed)
    }
}

static REPLY_OVERLAP: ReplyOverlapCounters = ReplyOverlapCounters::new();

/// Mark the start of `tmc_control`'s (P1) terminal-reply construction window.
pub fn mark_p1_reply_construction_start() {
    REPLY_OVERLAP.mark_p1_start();
}

/// Mark the end of `tmc_control`'s (P1) terminal-reply construction window.
pub fn mark_p1_reply_construction_end() {
    REPLY_OVERLAP.mark_p1_end();
}

/// Record a `usb_task` (P2) reply send, checking whether it landed inside
/// P1's reply-construction window.
pub fn record_p2_reply_send() {
    REPLY_OVERLAP.record_p2_send();
}

/// Count of P2 reply sends observed inside P1's reply-construction window.
pub fn p1_p2_reply_overlap_count() -> u32 {
    REPLY_OVERLAP.overlap_count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn breakaway_breadcrumb_decoder_rejects_cold_and_corrupt_magic_words() {
        let cold = BreakawayBreadcrumbStore::new_for_test(0);
        let corrupt_magic = BreakawayBreadcrumbStore::new_for_test(0x1234_5601);

        assert_eq!(cold.load(), None);
        assert_eq!(corrupt_magic.load(), None);
    }

    #[test]
    fn breakaway_breadcrumb_decoder_rejects_unknown_checkpoint() {
        let unknown_checkpoint = BreakawayBreadcrumbStore::new_for_test(0x4642_527f);

        assert_eq!(unknown_checkpoint.load(), None);
    }

    #[test]
    fn breakaway_breadcrumb_encoder_tracks_active_checkpoint_and_completion() {
        let store = BreakawayBreadcrumbStore::new_for_test(0);

        store.begin(BreakawayCheckpoint::CampaignInstalled);
        assert_eq!(store.raw_for_test(), 0x4642_5201);
        assert_eq!(
            store.load(),
            Some(BreakawayBreadcrumb {
                checkpoint: BreakawayCheckpoint::CampaignInstalled,
                interrupted: true,
            })
        );

        store.checkpoint(BreakawayCheckpoint::SafetyConfigured);
        assert_eq!(store.raw_for_test(), 0x4642_5202);
        assert_eq!(
            store.load(),
            Some(BreakawayBreadcrumb {
                checkpoint: BreakawayCheckpoint::SafetyConfigured,
                interrupted: true,
            })
        );

        store.complete(BreakawayCheckpoint::IntegralHandedOff);
        assert_eq!(store.raw_for_test(), 0x4642_5288);
        assert_eq!(
            store.load(),
            Some(BreakawayBreadcrumb {
                checkpoint: BreakawayCheckpoint::IntegralHandedOff,
                interrupted: false,
            })
        );
    }

    #[test]
    fn fault_class_codes_are_stable() {
        assert_eq!(FaultClass::QueueFull as u8, 1);
        assert_eq!(FaultClass::UnknownOid as u8, 2);
        assert_eq!(FaultClass::ActiveReset as u8, 3);
        assert_eq!(FaultClass::InvalidChannel as u8, 4);
    }

    #[test]
    fn operation_codes_are_stable() {
        assert_eq!(operation::QUEUE_STEP, 1);
        assert_eq!(operation::SET_NEXT_STEP_DIR, 2);
        assert_eq!(operation::RESET_STEP_CLOCK, 3);
        assert_eq!(operation::TMC_CONTROL, 4);
        assert_eq!(operation::STEPPER_STOP, 5);
        assert_eq!(operation::STEPPER_LOOKUP, 6);
    }

    #[test]
    fn stepper_diagnostic_queue_preserves_fifo_order() {
        let mut queue = StepperDiagnosticQueue::<u8, 4>::new();

        queue.push_drop_oldest(10);
        queue.push_drop_oldest(11);
        queue.push_drop_oldest(12);

        assert_eq!(queue.len(), 3);
        assert_eq!(queue.pop(), Some(10));
        assert_eq!(queue.pop(), Some(11));
        assert_eq!(queue.pop(), Some(12));
        assert_eq!(queue.pop(), None);
        assert!(queue.is_empty());
    }

    #[test]
    fn stepper_diagnostic_queue_drops_oldest_when_full() {
        let mut queue = StepperDiagnosticQueue::<u8, 3>::new();

        queue.push_drop_oldest(1);
        queue.push_drop_oldest(2);
        queue.push_drop_oldest(3);
        queue.push_drop_oldest(4);
        queue.push_drop_oldest(5);

        assert_eq!(queue.len(), 3);
        assert_eq!(queue.pop(), Some(3));
        assert_eq!(queue.pop(), Some(4));
        assert_eq!(queue.pop(), Some(5));
        assert_eq!(queue.pop(), None);
    }

    #[test]
    fn stepper_diagnostic_queue_empty_pop_returns_none() {
        let mut queue = StepperDiagnosticQueue::<u8, 2>::new();

        assert_eq!(queue.pop(), None);
        assert!(queue.is_empty());
        assert_eq!(queue.len(), 0);
    }

    #[test]
    fn stepper_event_reason_codes_are_stable() {
        assert_eq!(StepperEventReason::QueueEmpty as u8, 1);
        assert_eq!(StepperEventReason::MissedDeadlineLoad as u8, 2);
        assert_eq!(StepperEventReason::MissedDeadlineStep as u8, 3);
        assert_eq!(StepperEventReason::TrsyncStop as u8, 4);
        assert_eq!(StepperEventReason::P1Stop as u8, 5);
        assert_eq!(StepperEventReason::ResetStepClock as u8, 6);
        assert_eq!(StepperEventReason::TmcDisableSignal as u8, 7);
    }

    #[test]
    fn stepper_event_reason_names_are_short_and_stable() {
        assert_eq!(StepperEventReason::QueueEmpty.name(), "queue_empty");
        assert_eq!(
            StepperEventReason::MissedDeadlineLoad.name(),
            "missed_deadline_load"
        );
        assert_eq!(
            StepperEventReason::MissedDeadlineStep.name(),
            "missed_deadline_step"
        );
        assert_eq!(StepperEventReason::TrsyncStop.name(), "trsync_stop");
        assert_eq!(StepperEventReason::P1Stop.name(), "p1_stop");
        assert_eq!(
            StepperEventReason::ResetStepClock.name(),
            "reset_step_clock"
        );
        assert_eq!(
            StepperEventReason::TmcDisableSignal.name(),
            "tmc_disable_signal"
        );
    }

    #[test]
    fn stepper_execution_stats_tracks_per_channel_motion_and_stop_state() {
        let stats = StepperExecutionStats::<2>::new();

        stats.record_queued_segment(0, 12);
        stats.record_queued_segment(0, 8);
        stats.record_discarded_segment(1, 5);
        stats.record_loaded_segment(0, 7);
        stats.record_executed_step(0, true, 0, 1200);
        stats.record_executed_step(0, true, 1, 1225);
        stats.record_executed_step(0, false, 2, 1250);
        stats.record_queue_empty(0);
        stats.record_missed_deadline(1);
        stats.record_reset(1, 2, 44);
        stats.record_physical_stop(1, StepperEventReason::TrsyncStop, 6, 9, 3, 77);

        let channel0 = stats
            .snapshot(0, -11, true, 4)
            .expect("channel 0 snapshot exists");
        assert_eq!(channel0.channel, 0);
        assert_eq!(channel0.position, -11);
        assert_eq!(channel0.queued_segments, 2);
        assert_eq!(channel0.queued_steps, 20);
        assert_eq!(channel0.loaded_segments, 1);
        assert_eq!(channel0.loaded_steps, 7);
        assert_eq!(channel0.executed_pos_steps, 2);
        assert_eq!(channel0.executed_neg_steps, 1);
        assert_eq!(channel0.physical_pos_pulses, 1);
        assert_eq!(channel0.physical_neg_pulses, 2);
        assert_eq!(channel0.queue_empty_count, 1);
        assert!(channel0.timer_active);
        assert_eq!(channel0.queue_len, 4);

        let channel1 = stats
            .snapshot(1, 33, false, 0)
            .expect("channel 1 snapshot exists");
        assert_eq!(channel1.channel, 1);
        assert_eq!(channel1.position, 33);
        assert_eq!(channel1.queued_segments, 0);
        assert_eq!(channel1.physical_pos_pulses, 0);
        assert_eq!(channel1.physical_neg_pulses, 0);
        assert_eq!(channel1.discarded_segments, 1);
        assert_eq!(channel1.discarded_steps, 5);
        assert_eq!(channel1.missed_deadline_count, 1);
        assert_eq!(channel1.reset_count, 1);
        assert_eq!(channel1.reset_drained_segments, 2);
        assert_eq!(channel1.reset_drained_steps, 44);
        assert_eq!(channel1.stop_count, 1);
        assert_eq!(channel1.stop_drained_segments, 3);
        assert_eq!(channel1.stop_drained_steps, 77);
        assert_eq!(
            channel1.last_stop_reason,
            StepperEventReason::TrsyncStop as u8
        );
        assert_eq!(channel1.last_stop_remaining_events, 6);
        assert_eq!(channel1.last_stop_queue_len, 9);
        assert_eq!(channel1.last_stop_drained_segments, 3);
        assert_eq!(channel1.last_stop_drained_steps, 77);
        assert!(!channel1.timer_active);
        assert_eq!(channel1.queue_len, 0);

        assert!(stats.snapshot(2, 0, false, 0).is_none());
        stats.record_queued_segment(2, 99);
        assert!(stats.snapshot(2, 0, false, 0).is_none());
    }

    #[test]
    fn stepper_execution_stats_re_latches_per_activation_timing() {
        let stats = StepperExecutionStats::<1>::new();

        stats.record_timer_activation(0, 1000);
        stats.record_loaded_segment_timing(0, 3, 1010, 1030, 1040);
        stats.record_loaded_segment_timing(0, 4, 1020, 1050, 1060);
        stats.record_executed_step(0, true, 2, 1070);
        stats.record_executed_step(0, true, 1, 1080);

        let first = stats
            .snapshot(0, 0, true, 0)
            .expect("first activation snapshot exists");
        assert_eq!(first.activation_count, 1);
        assert_eq!(first.last_activation_clock, 1000);
        assert_eq!(first.loaded_segments, 2);
        assert_eq!(first.loaded_steps, 7);
        assert_eq!(first.executed_pos_steps, 2);
        assert_eq!(first.physical_pos_pulses, 3);
        assert_eq!(first.first_load_now, 1010);
        assert_eq!(first.first_load_scheduled, 1030);
        assert_eq!(first.first_load_compare, 1040);
        assert_eq!(first.first_load_lead_ticks, 20);
        assert_eq!(first.first_load_compare_delay_ticks, 30);
        assert_eq!(first.first_step_clock, 1070);
        assert_eq!(first.first_step_delay_ticks, 70);

        stats.record_timer_activation(0, 2000);
        let reset = stats
            .snapshot(0, 0, true, 0)
            .expect("reset activation snapshot exists");
        assert_eq!(reset.activation_count, 2);
        assert_eq!(reset.last_activation_clock, 2000);
        assert_eq!(reset.first_load_now, 0);
        assert_eq!(reset.first_load_scheduled, 0);
        assert_eq!(reset.first_load_compare, 0);
        assert_eq!(reset.first_load_lead_ticks, 0);
        assert_eq!(reset.first_load_compare_delay_ticks, 0);
        assert_eq!(reset.first_step_clock, 0);
        assert_eq!(reset.first_step_delay_ticks, 0);
        assert_eq!(reset.physical_pos_pulses, 3);
        assert_eq!(reset.physical_neg_pulses, 0);

        stats.record_loaded_segment_timing(0, 5, 2010, 2060, 2075);
        stats.record_reset(0, 0, 0);
        stats.record_executed_step(0, false, 2, 2090);
        let second = stats
            .snapshot(0, 0, true, 0)
            .expect("second activation snapshot exists");
        assert_eq!(second.activation_count, 2);
        assert_eq!(second.loaded_segments, 3);
        assert_eq!(second.loaded_steps, 12);
        assert_eq!(second.executed_pos_steps, 2);
        assert_eq!(second.executed_neg_steps, 1);
        assert_eq!(second.physical_pos_pulses, 3);
        assert_eq!(second.physical_neg_pulses, 2);
        assert_eq!(second.first_load_now, 2010);
        assert_eq!(second.first_load_scheduled, 2060);
        assert_eq!(second.first_load_compare, 2075);
        assert_eq!(second.first_load_lead_ticks, 50);
        assert_eq!(second.first_load_compare_delay_ticks, 65);
        assert_eq!(second.first_step_clock, 2090);
        assert_eq!(second.first_step_delay_ticks, 90);
    }

    #[test]
    fn physical_pulse_counters_wrap_u32_and_remain_directional() {
        let stats = StepperExecutionStats::<1>::new();
        stats.counters[0]
            .physical_pos_pulses
            .store(u32::MAX, Relaxed);
        stats.counters[0]
            .physical_neg_pulses
            .store(u32::MAX - 1, Relaxed);

        stats.record_executed_step(0, true, 1, 100);
        stats.record_executed_step(0, false, 2, 200);

        let snapshot = stats
            .snapshot(0, 0, false, 0)
            .expect("channel snapshot exists");
        assert_eq!(snapshot.physical_pos_pulses, 0);
        assert_eq!(snapshot.physical_neg_pulses, 0);
        assert_eq!(snapshot.executed_pos_steps, 1);
        assert_eq!(snapshot.executed_neg_steps, 1);
    }

    #[test]
    fn reply_overlap_counters_start_at_zero() {
        let counters = ReplyOverlapCounters::new();
        assert_eq!(counters.overlap_count(), 0);
    }

    #[test]
    fn reply_overlap_counters_ignore_a_send_outside_the_p1_window() {
        let counters = ReplyOverlapCounters::new();
        counters.record_p2_send();
        assert_eq!(counters.overlap_count(), 0);
    }

    #[test]
    fn reply_overlap_counters_count_a_send_inside_the_p1_window() {
        let counters = ReplyOverlapCounters::new();
        counters.mark_p1_start();
        counters.record_p2_send();
        assert_eq!(counters.overlap_count(), 1);
    }

    #[test]
    fn reply_overlap_counters_ignore_a_send_after_the_p1_window_closes() {
        let counters = ReplyOverlapCounters::new();
        counters.mark_p1_start();
        counters.mark_p1_end();
        counters.record_p2_send();
        assert_eq!(counters.overlap_count(), 0);
    }

    #[test]
    fn reply_overlap_counters_accumulate_across_multiple_overlapping_sends() {
        let counters = ReplyOverlapCounters::new();
        counters.mark_p1_start();
        counters.record_p2_send();
        counters.record_p2_send();
        counters.record_p2_send();
        assert_eq!(counters.overlap_count(), 3);
    }
}
