// midi-smooth-cc/tests/test_midi.cpp

// =============================================================================
// MIDI Smooth CC - Unit tests for MidiProcessor (spec §15)
//
// Covers:
//   Test Case 1: Pass-through of non-CC events (Note On, Pitch Bend).
//   Test Case 2: First CC on a selected (channel, CC) pair initializes
//                the FilterState (spec §14).
//   Test Case 3: Bypass mode forwards all events unchanged (spec §14).
//   Test Case 4: CC selection — selected CC consumed, unselected CC
//                passes through (spec §5).
//   Test Case 5: Output generation emits a quantized CC event at the
//                correct frame offset (spec §8).
//   Test Case 6: Output buffer overflow is handled gracefully (spec §11).
//
// These tests focus on routing, state updates, and Atom Sequence
// manipulation. They do NOT re-test the smoother math (covered in
// test_smoothing.cpp).
//
// The test fixture owns a MidiProcessor, a PluginParameters, a 2048-entry
// FilterState array, and two stack-allocated, 8-byte-aligned LV2_Atom_Sequence
// buffers (input + output). No heap allocation is performed in the fixture
// itself; std::vector is used only inside get_output_events() to collect
// results for assertion (test-side, not real-time).
//
// Note on LV2_Atom_Sequence_Body: the layout of this struct varies across
// LV2 versions (some have a `capacity` field, others have `unit`/`pad`).
// To remain compatible with all versions, the fixture zeroes the raw
// buffers with std::memset and only touches atom.type / atom.size
// explicitly. The capacity used by lv2_atom_sequence_append_event is
// passed as a separate parameter (computed from the buffer size), NOT
// read from any body field.
// =============================================================================
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"

#include "midi_processor.hpp"
#include "parameters.hpp"
#include "filter_state.hpp"
#include "smoothing.hpp"

using midi_smooth_cc::FilterState;
using midi_smooth_cc::MidiProcessor;
using midi_smooth_cc::PluginParameters;
using midi_smooth_cc::SmoothingMode;
using midi_smooth_cc::default_cc_selection;

namespace {

// =============================================================================
// Test fixture
// =============================================================================
class MidiProcessorFixture {
public:
    static constexpr LV2_URID kMidiUrid          = 1;   // dummy URID for midi:MidiEvent
    static constexpr LV2_URID kSeqType           = 0;   // dummy atom.type for sequences
    static constexpr size_t   kInputBufferBytes  = 2048;
    static constexpr size_t   kOutputBufferBytes = 2048;
    static constexpr uint32_t kDefaultCapacity   = 1024;

    explicit MidiProcessorFixture(uint32_t output_capacity = kDefaultCapacity)
        : output_capacity_(output_capacity) {
        // Zero the raw buffers so every byte of both LV2_Atom_Sequence
        // structs — including whichever body fields (capacity / unit / pad)
        // the installed LV2 version defines — starts at 0. Only atom.type
        // and atom.size are set explicitly below; the body is left zeroed.
        // The append/iterate API takes capacity as an explicit parameter
        // and never reads body fields directly, so this is safe and
        // version-independent.
        std::memset(input_storage_,  0, kInputBufferBytes);
        std::memset(output_storage_, 0, kOutputBufferBytes);

        // Initialize the processor with the dummy MIDI URID and the
        // requested output buffer capacity.
        processor_.set_midi_urid(kMidiUrid);
        processor_.set_output_capacity(output_capacity_);

        // Set up the input Atom Sequence as an empty sequence.
        // atom.type = dummy; atom.size = sizeof(LV2_Atom_Sequence_Body)
        // means "no events yet". Body fields are already zeroed by memset.
        input_ = reinterpret_cast<LV2_Atom_Sequence*>(input_storage_);
        input_->atom.type = kSeqType;
        input_->atom.size = sizeof(LV2_Atom_Sequence_Body);

        // Set up the output Atom Sequence similarly.
        output_ = reinterpret_cast<LV2_Atom_Sequence*>(output_storage_);
        output_->atom.type = kSeqType;
        output_->atom.size = sizeof(LV2_Atom_Sequence_Body);

        // Default parameters per spec §12.
        params_.enabled         = true;
        params_.smoothing_mode  = SmoothingMode::Exponential;
        params_.attack_ms       = 50;
        params_.release_ms      = 100;
        params_.rising_rate     = 500;
        params_.falling_rate    = 500;
        params_.deadband        = 0;
        params_.output_rate_hz  = 100;
        params_.all_cc          = false;
        params_.channel_mask    = 0xFFFF;             // all 16 channels
        params_.cc_selection    = default_cc_selection();  // CC7 only
    }

    // ---- Input buffer manipulation ----

    void reset_input() {
        // Only atom.type and atom.size need to be reset to mark the
        // sequence as empty. Body fields (zeroed at construction) are
        // not touched here.
        input_->atom.type = kSeqType;
        input_->atom.size = sizeof(LV2_Atom_Sequence_Body);
    }

    // Append a MIDI event to the input sequence at the given frame offset.
    // Builds a contiguous (LV2_Atom_Event header + body bytes) on the stack
    // so lv2_atom_sequence_append_event can copy both in one go.
    void add_midi_event(uint32_t frame_offset,
                        const uint8_t* bytes, size_t num_bytes) {
        struct StackEvent {
            LV2_Atom_Event header;
            uint8_t        data[256];   // ample for any test MIDI event
        } ev;

        ev.header.time.frames = frame_offset;
        ev.header.body.type   = kMidiUrid;
        ev.header.body.size   = static_cast<uint32_t>(num_bytes);
        std::memcpy(ev.data, bytes, num_bytes);

        // Input buffer capacity = total buffer bytes minus the LV2_Atom
        // header. This is passed as the explicit capacity argument to
        // lv2_atom_sequence_append_event; it is NOT read from any body
        // field of the sequence.
        const uint32_t cap =
            static_cast<uint32_t>(kInputBufferBytes - sizeof(LV2_Atom));
        LV2_Atom_Event* appended =
            lv2_atom_sequence_append_event(input_, cap, &ev.header);
        REQUIRE(appended != nullptr);
    }

    void add_midi_event(uint32_t frame_offset,
                        std::initializer_list<uint8_t> bytes) {
        add_midi_event(frame_offset, bytes.begin(), bytes.size());
    }

    // ---- Output buffer inspection ----

    struct OutputEvent {
        uint32_t             frame_offset;
        std::vector<uint8_t> bytes;
    };

    std::vector<OutputEvent> get_output_events() const {
        std::vector<OutputEvent> events;
        LV2_Atom_Event* ev = lv2_atom_sequence_begin(&output_->body);
        LV2_Atom_Event* end =
            lv2_atom_sequence_end(&output_->body, output_->atom.size);
        while (ev != end) {
            OutputEvent oe;
            oe.frame_offset = ev->time.frames;
            const uint8_t* body =
                reinterpret_cast<const uint8_t*>(ev) + sizeof(LV2_Atom_Event);
            oe.bytes.assign(body, body + ev->body.size);
            events.push_back(oe);
            ev = lv2_atom_sequence_next(ev);
        }
        return events;
    }

    size_t output_event_count() const {
        return get_output_events().size();
    }

    // Verify the output sequence is structurally valid: atom.size never
    // exceeds the configured capacity (spec §11 overflow safety invariant).
    bool output_is_valid() const {
        return output_->atom.size <= output_capacity_;
    }

    // ---- Run the processor ----

    void run(uint32_t sample_rate   = 48000,
             uint32_t current_frame = 0,
             uint32_t block_length  = 256) {
        processor_.process(input_, output_, sample_rate,
                           current_frame, block_length,
                           params_, states_);
    }

    // ---- Accessors ----

    MidiProcessor&                 processor()       { return processor_; }
    PluginParameters&              params()          { return params_; }
    std::array<FilterState, 2048>& states()          { return states_; }
    LV2_Atom_Sequence*             output()          { return output_; }
    uint32_t                       output_capacity() const { return output_capacity_; }

private:
    MidiProcessor                  processor_;
    PluginParameters               params_;
    std::array<FilterState, 2048>  states_;
    uint32_t                       output_capacity_;

    // 8-byte aligned to satisfy LV2's pad-to-8 event layout.
    alignas(8) uint8_t input_storage_[kInputBufferBytes];
    alignas(8) uint8_t output_storage_[kOutputBufferBytes];
    LV2_Atom_Sequence* input_;
    LV2_Atom_Sequence* output_;
};

// Helper: find the first 3-byte CC event on (channel, cc_num) in the output.
// Returns nullptr if not found.
const MidiProcessorFixture::OutputEvent* find_cc_event(
    const std::vector<MidiProcessorFixture::OutputEvent>& events,
    uint8_t channel, uint8_t cc_num) {
    const uint8_t status = static_cast<uint8_t>(0xB0u | (channel & 0x0Fu));
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 &&
            ev.bytes[0] == status &&
            ev.bytes[1] == cc_num) {
            return &ev;
        }
    }
    return nullptr;
}

} // anonymous namespace

// =============================================================================
// Test Case 1: Pass-through of non-CC events (spec §10)
//
// Note On and Pitch Bend must appear in the output identically and at their
// original frame offsets. Neither event type is ever delayed or modified.
// =============================================================================
TEST_CASE("Non-CC MIDI events pass through unchanged with original frame offsets",
          "[midi][pass-through]") {
    MidiProcessorFixture fx;

    // Note On: channel 0, note 60, velocity 100, at frame offset 10.
    fx.add_midi_event(10, {0x90, 60, 100});
    // Pitch Bend: channel 0, LSB=0x00, MSB=0x40 (center), at frame offset 20.
    fx.add_midi_event(20, {0xE0, 0x00, 0x40});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();
    REQUIRE(events.size() == 2);

    // Note On — verbatim, original frame offset.
    REQUIRE(events[0].frame_offset == 10);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // Pitch Bend — verbatim, original frame offset.
    REQUIRE(events[1].frame_offset == 20);
    REQUIRE(events[1].bytes.size() == 3);
    REQUIRE(events[1].bytes[0] == 0xE0);
    REQUIRE(events[1].bytes[1] == 0x00);
    REQUIRE(events[1].bytes[2] == 0x40);
}

// =============================================================================
// Test Case 2: First CC initializes the FilterState (spec §14)
//
// When a selected (channel, CC) pair receives its first CC, the filter state
// must be initialized from the incoming value — no ramp from zero. The CC is
// consumed (not passed through). last_transmitted_value is set to -1 to force
// the next output tick to emit.
// =============================================================================
TEST_CASE("First CC on a selected pair initializes FilterState from incoming value",
          "[midi][init]") {
    MidiProcessorFixture fx;

    // CC7 value 50 on channel 0 at frame offset 5.
    fx.add_midi_event(5, {0xB0, 7, 50});

    // Default params: output_rate_hz = 100, sample_rate = 48000
    // → frames_per_tick = 480. Block length = 256, so the first output tick
    // (at frame 5 + 480 = 485) falls outside the block → no emission happens
    // this block, isolating the init behavior from output generation.
    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    // CC7 is selected → consumed, not passed through. Output must be empty.
    REQUIRE(fx.output_event_count() == 0);

    // Verify FilterState for (channel 0, CC 7).
    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized        == true);
    REQUIRE(state.last_transmitted_value == -1);   // force next tick to emit

    // Both smoothers initialized to the incoming value (no ramp from zero).
    REQUIRE(state.exp_smoother.current_value() == 50.0);
    REQUIRE(state.exp_smoother.target_value()  == 50.0);
    REQUIRE(state.slew_limiter.current_value() == 50.0);
    REQUIRE(state.slew_limiter.target_value()  == 50.0);

    // last_update_frame = current_frame + event frame offset = 0 + 5 = 5.
    REQUIRE(state.last_update_frame == 5u);

    // No other state should have been touched.
    REQUIRE(fx.states()[0 * 128 + 6].is_initialized == false);
    REQUIRE(fx.states()[1 * 128 + 7].is_initialized == false);
}

// =============================================================================
// Test Case 3: Bypass mode forwards all events unchanged (spec §14)
//
// When params.enabled == false, every input event (including CCs on selected
// channel/CC pairs) must be copied verbatim to the output. FilterStates must
// not be initialized or mutated.
// =============================================================================
TEST_CASE("Bypass mode forwards CC and Note On unchanged, leaves state uninitialized",
          "[midi][bypass]") {
    MidiProcessorFixture fx;
    fx.params().enabled = false;   // bypass ON

    // CC7 (selected by default) and Note On, both on channel 0.
    fx.add_midi_event(10, {0xB0, 7, 80});
    fx.add_midi_event(20, {0x90, 60, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();
    REQUIRE(events.size() == 2);

    // CC7 must appear unchanged — even though it would normally be consumed.
    REQUIRE(events[0].frame_offset == 10);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0xB0);
    REQUIRE(events[0].bytes[1] == 7);
    REQUIRE(events[0].bytes[2] == 80);

    // Note On must appear unchanged.
    REQUIRE(events[1].frame_offset == 20);
    REQUIRE(events[1].bytes.size() == 3);
    REQUIRE(events[1].bytes[0] == 0x90);
    REQUIRE(events[1].bytes[1] == 60);
    REQUIRE(events[1].bytes[2] == 100);

    // The CC7 FilterState must remain uninitialized despite the CC input.
    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized        == false);
    REQUIRE(state.last_transmitted_value == -1);
    REQUIRE(state.last_update_frame      == 0u);
}

// =============================================================================
// Test Case 4: CC selection — selected CC consumed, unselected CC passes
// through (spec §5)
//
// Default config selects only CC7. CC1 is unselected and must pass through
// verbatim. CC7 is consumed (does not appear in output) and its FilterState
// becomes initialized.
// =============================================================================
TEST_CASE("Selected CC is consumed; unselected CC passes through unchanged",
          "[midi][selection]") {
    MidiProcessorFixture fx;

    // CC7 value 50, CC1 value 100, both on channel 0.
    fx.add_midi_event(10, {0xB0, 7, 50});
    fx.add_midi_event(20, {0xB0, 1, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // CC1 should pass through unchanged.
    const auto* cc1_ev = find_cc_event(events, /*channel=*/0, /*cc=*/1);
    REQUIRE(cc1_ev != nullptr);
    REQUIRE(cc1_ev->frame_offset == 20);
    REQUIRE(cc1_ev->bytes[2] == 100);

    // CC7 should NOT appear in the output (it was consumed into the filter).
    const auto* cc7_ev = find_cc_event(events, /*channel=*/0, /*cc=*/7);
    REQUIRE(cc7_ev == nullptr);

    // FilterState for CC7 must be initialized from the incoming value.
    FilterState& cc7_state = fx.states()[0 * 128 + 7];
    REQUIRE(cc7_state.is_initialized        == true);
    REQUIRE(cc7_state.last_transmitted_value == -1);
    REQUIRE(cc7_state.exp_smoother.current_value() == 50.0);
    REQUIRE(cc7_state.exp_smoother.target_value()  == 50.0);

    // FilterState for CC1 must remain uninitialized.
    FilterState& cc1_state = fx.states()[0 * 128 + 1];
    REQUIRE(cc1_state.is_initialized == false);
}

// =============================================================================
// Test Case 5: Output generation emits a quantized CC at the correct frame
// offset (spec §8)
//
// A pre-initialized FilterState with a stale last_update_frame and a target
// different from last_transmitted_value must trigger an emission on the first
// output tick that falls within the current block. The emitted event must be
// at the correct frame offset and carry the quantized filtered value.
// =============================================================================
TEST_CASE("Output generation emits quantized CC at the correct frame offset",
          "[midi][output]") {
    MidiProcessorFixture fx;

    // High output rate (1000 Hz) → frames_per_tick = 48000 / 1000 = 48.
    fx.params().output_rate_hz = 1000;
    // Slew mode with a very high rate so the filter reaches the target
    // exactly during the catch-up step, making the expected value predictable.
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;   // CC/s
    fx.params().falling_rate   = 10000;

    // Pre-initialize the FilterState for CC7 on channel 0.
    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    // Set target to 100 on both smoothers without advancing current_value
    // (the dt = 0 trick: alpha = 0 for exp, max_delta = 0 for slew).
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;       // different from target → will emit
    state.last_update_frame      = 0;       // in the past relative to current_frame

    // Block: current_frame = 1000, block_length = 100, sample_rate = 48000.
    // frames_per_tick = 48.
    //
    // Catch-up: from frame 0 to 1000, dt = 1000/48000 ≈ 0.0208 s.
    //   slew step = min(100, 10000 * 0.0208) = min(100, 208) = 100
    //   → current_value reaches 100, last_update_frame = 1000.
    //   (No emission for the catch-up step — it is in the past.)
    //
    // First tick in block: 1000 + 48 = 1048. frame_offset = 48.
    //   dt = 48/48000 = 0.001 s. delta = 0. Value = 100. Quantized = 100.
    //   100 != last_transmitted (0) → EMIT at frame_offset 48, value 100.
    //
    // Second tick: 1096. frame_offset = 96.
    //   Value still 100. 100 == last_transmitted (100) → no emit.
    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/100);

    const auto events = fx.get_output_events();

    // Exactly one CC7 event should have been emitted.
    size_t cc7_count = 0;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 &&
            ev.bytes[0] == 0xB0 &&
            ev.bytes[1] == 7) {
            ++cc7_count;
            REQUIRE(ev.frame_offset == 48u);
            REQUIRE(ev.bytes[2]      == 100);
        }
    }
    REQUIRE(cc7_count == 1);

    // last_transmitted_value must have been updated to 100.
    REQUIRE(state.last_transmitted_value == 100);

    // last_update_frame must have advanced past the emitted tick.
    REQUIRE(state.last_update_frame >= 1048u);
}

// =============================================================================
// Test Case 6: Output buffer overflow safety (spec §11)
//
// With a very small output_capacity (32 bytes), at most one padded MIDI
// event fits (sizeof(LV2_Atom_Event) + 3 = 15, padded to 16; 4 + 16 = 20 ≤
// 32; a second event would need 36 bytes). The processor must drop the
// overflow event gracefully, never crash, and leave the output sequence
// structurally valid (atom.size ≤ capacity).
// =============================================================================
TEST_CASE("Output buffer overflow is handled gracefully without corruption",
          "[midi][overflow]") {
    // capacity = 32 bytes. The fixture's underlying buffer is 2048 bytes,
    // so there is ample physical space; the capacity is what limits writes.
    MidiProcessorFixture fx(/*output_capacity=*/32);

    // High output rate + high slew rate so each tick changes the integer
    // value and forces an emit attempt.
    fx.params().output_rate_hz = 1000;      // frames_per_tick = 48
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;     // 10 CC per tick (dt = 0.001 s)
    fx.params().falling_rate   = 10000;

    // Pre-initialize CC7: target 100, current 0, last emitted 0.
    // Each tick moves 10 CC units: tick 48 → value 10, tick 96 → value 20.
    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 0;       // current_frame = 0 → no catch-up

    // Block: 100 frames at 48000 Hz. Ticks at frames 48 and 96.
    // Tick 48: value = 10. Emit succeeds. atom.size: 4 → 20.
    // Tick 96: value = 20. Emit fails (20 + 16 = 36 > 32). Graceful drop.
    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/100);

    // Reaching this assertion means the function returned without crashing.
    // Verify the output sequence is structurally valid.
    REQUIRE(fx.output_is_valid());
    REQUIRE(fx.output()->atom.size <= fx.output_capacity());

    // Exactly one event was successfully emitted (the first tick).
    const auto events = fx.get_output_events();
    REQUIRE(events.size() == 1);

    // The single emitted event must be well-formed.
    REQUIRE(events[0].frame_offset == 48u);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0xB0);
    REQUIRE(events[0].bytes[1] == 7);
    REQUIRE(events[0].bytes[2] == 10);   // 0 + 10000 * 0.001 = 10
}
