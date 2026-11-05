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
//   Regression tests (marked [midi][regression]):
//     R1: Unselected events strictly pass through unchanged.
//     R2: Selected CC7 triggers output generation when enabled.
//     R3: enabled=false passes ALL events through unchanged.
//     R4: Empty input yields empty output.
//     R5: Valid input yields non-empty output when events are expected.
//     R6: Full integration processing cycle — end-to-end smoke test.
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
        std::memset(input_storage_,  0, kInputBufferBytes);
        std::memset(output_storage_, 0, kOutputBufferBytes);

        processor_.set_midi_urid(kMidiUrid);
        processor_.set_output_capacity(output_capacity_);

        input_ = reinterpret_cast<LV2_Atom_Sequence*>(input_storage_);
        input_->atom.type = kSeqType;
        input_->atom.size = sizeof(LV2_Atom_Sequence_Body);

        output_ = reinterpret_cast<LV2_Atom_Sequence*>(output_storage_);
        output_->atom.type = kSeqType;
        output_->atom.size = sizeof(LV2_Atom_Sequence_Body);

        params_.enabled         = true;
        params_.smoothing_mode  = SmoothingMode::Exponential;
        params_.attack_ms       = 50;
        params_.release_ms      = 100;
        params_.rising_rate     = 500;
        params_.falling_rate    = 500;
        params_.deadband        = 0;
        params_.output_rate_hz  = 100;
        params_.all_cc          = false;
        params_.channel_mask    = 0xFFFF;
        params_.cc_selection    = default_cc_selection();
    }

    void reset_input() {
        input_->atom.type = kSeqType;
        input_->atom.size = sizeof(LV2_Atom_Sequence_Body);
    }

    void add_midi_event(uint32_t frame_offset,
                        const uint8_t* bytes, size_t num_bytes) {
        struct StackEvent {
            LV2_Atom_Event header;
            uint8_t        data[256];
        } ev;

        ev.header.time.frames = frame_offset;
        ev.header.body.type   = kMidiUrid;
        ev.header.body.size   = static_cast<uint32_t>(num_bytes);
        std::memcpy(ev.data, bytes, num_bytes);

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

    bool output_is_valid() const {
        return output_->atom.size <= output_capacity_;
    }

    void run(uint32_t sample_rate   = 48000,
             uint32_t current_frame = 0,
             uint32_t block_length  = 256) {
        processor_.process(input_, output_, sample_rate,
                           current_frame, block_length,
                           params_, states_);
    }

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

    alignas(8) uint8_t input_storage_[kInputBufferBytes];
    alignas(8) uint8_t output_storage_[kOutputBufferBytes];
    LV2_Atom_Sequence* input_;
    LV2_Atom_Sequence* output_;
};

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
// =============================================================================
TEST_CASE("Non-CC MIDI events pass through unchanged with original frame offsets",
          "[midi][pass-through]") {
    MidiProcessorFixture fx;

    fx.add_midi_event(10, {0x90, 60, 100});
    fx.add_midi_event(20, {0xE0, 0x00, 0x40});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();
    REQUIRE(events.size() == 2);

    REQUIRE(events[0].frame_offset == 10);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    REQUIRE(events[1].frame_offset == 20);
    REQUIRE(events[1].bytes.size() == 3);
    REQUIRE(events[1].bytes[0] == 0xE0);
    REQUIRE(events[1].bytes[1] == 0x00);
    REQUIRE(events[1].bytes[2] == 0x40);
}

// =============================================================================
// Test Case 2: First CC initializes the FilterState (spec §14)
// =============================================================================
TEST_CASE("First CC on a selected pair initializes FilterState from incoming value",
          "[midi][init]") {
    MidiProcessorFixture fx;

    fx.add_midi_event(5, {0xB0, 7, 50});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    REQUIRE(fx.output_event_count() == 0);

    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized        == true);
    REQUIRE(state.last_transmitted_value == -1);

    REQUIRE(state.exp_smoother.current_value() == 50.0);
    REQUIRE(state.exp_smoother.target_value()  == 50.0);
    REQUIRE(state.slew_limiter.current_value() == 50.0);
    REQUIRE(state.slew_limiter.target_value()  == 50.0);

    REQUIRE(state.last_update_frame == 5u);

    REQUIRE(fx.states()[0 * 128 + 6].is_initialized == false);
    REQUIRE(fx.states()[1 * 128 + 7].is_initialized == false);
}

// =============================================================================
// Test Case 3: Bypass mode forwards all events unchanged (spec §14)
// =============================================================================
TEST_CASE("Bypass mode forwards CC and Note On unchanged, leaves state uninitialized",
          "[midi][bypass]") {
    MidiProcessorFixture fx;
    fx.params().enabled = false;

    fx.add_midi_event(10, {0xB0, 7, 80});
    fx.add_midi_event(20, {0x90, 60, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();
    REQUIRE(events.size() == 2);

    REQUIRE(events[0].frame_offset == 10);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0xB0);
    REQUIRE(events[0].bytes[1] == 7);
    REQUIRE(events[0].bytes[2] == 80);

    REQUIRE(events[1].frame_offset == 20);
    REQUIRE(events[1].bytes.size() == 3);
    REQUIRE(events[1].bytes[0] == 0x90);
    REQUIRE(events[1].bytes[1] == 60);
    REQUIRE(events[1].bytes[2] == 100);

    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized        == false);
    REQUIRE(state.last_transmitted_value == -1);
    REQUIRE(state.last_update_frame      == 0u);
}

// =============================================================================
// Test Case 4: CC selection — selected CC consumed, unselected CC passes
// through (spec §5)
// =============================================================================
TEST_CASE("Selected CC is consumed; unselected CC passes through unchanged",
          "[midi][selection]") {
    MidiProcessorFixture fx;

    fx.add_midi_event(10, {0xB0, 7, 50});
    fx.add_midi_event(20, {0xB0, 1, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    const auto* cc1_ev = find_cc_event(events, /*channel=*/0, /*cc=*/1);
    REQUIRE(cc1_ev != nullptr);
    REQUIRE(cc1_ev->frame_offset == 20);
    REQUIRE(cc1_ev->bytes[2] == 100);

    const auto* cc7_ev = find_cc_event(events, /*channel=*/0, /*cc=*/7);
    REQUIRE(cc7_ev == nullptr);

    FilterState& cc7_state = fx.states()[0 * 128 + 7];
    REQUIRE(cc7_state.is_initialized        == true);
    REQUIRE(cc7_state.last_transmitted_value == -1);
    REQUIRE(cc7_state.exp_smoother.current_value() == 50.0);
    REQUIRE(cc7_state.exp_smoother.target_value()  == 50.0);

    FilterState& cc1_state = fx.states()[0 * 128 + 1];
    REQUIRE(cc1_state.is_initialized == false);
}

// =============================================================================
// Test Case 5: Output generation emits a quantized CC at the correct frame
// offset (spec §8)
// =============================================================================
TEST_CASE("Output generation emits quantized CC at the correct frame offset",
          "[midi][output]") {
    MidiProcessorFixture fx;

    fx.params().output_rate_hz = 1000;
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;
    fx.params().falling_rate   = 10000;

    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 0;

    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/100);

    const auto events = fx.get_output_events();

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

    REQUIRE(state.last_transmitted_value == 100);
    REQUIRE(state.last_update_frame >= 1048u);
}

// =============================================================================
// Test Case 6: Output buffer overflow safety (spec §11)
// =============================================================================
TEST_CASE("Output buffer overflow is handled gracefully without corruption",
          "[midi][overflow]") {
    MidiProcessorFixture fx(/*output_capacity=*/32);

    fx.params().output_rate_hz = 1000;
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;
    fx.params().falling_rate   = 10000;

    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 0;

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/100);

    REQUIRE(fx.output_is_valid());
    REQUIRE(fx.output()->atom.size <= fx.output_capacity());

    const auto events = fx.get_output_events();
    REQUIRE(events.size() == 1);

    REQUIRE(events[0].frame_offset == 48u);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0xB0);
    REQUIRE(events[0].bytes[1] == 7);
    REQUIRE(events[0].bytes[2] == 10);
}

// =============================================================================
// CRITICAL BUG REGRESSION TESTS
//
// The following tests guard against a bug where the plugin receives MIDI
// events but outputs nothing. Each test is preceded by a mandatory comment
// block that MUST NOT be removed during test suite optimization.
// =============================================================================

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Unselected events pass through to output unchanged",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    // Feed a Note On and a Pitch Bend — neither is a CC, so both must
    // pass through verbatim regardless of channel/CC selection.
    fx.add_midi_event(5, {0x90, 60, 100});       // Note On, ch0, note 60, vel 100
    fx.add_midi_event(15, {0xE0, 0x00, 0x40});   // Pitch Bend, ch0, center

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Output must NOT be empty — both events must appear.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 2);

    // Note On — strictly identical, same frame offset.
    REQUIRE(events[0].frame_offset == 5u);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // Pitch Bend — strictly identical, same frame offset.
    REQUIRE(events[1].frame_offset == 15u);
    REQUIRE(events[1].bytes.size() == 3);
    REQUIRE(events[1].bytes[0] == 0xE0);
    REQUIRE(events[1].bytes[1] == 0x00);
    REQUIRE(events[1].bytes[2] == 0x40);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Selected CC7 triggers output generation when enabled",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    // High output rate so at least one tick falls within a 256-frame block.
    // frames_per_tick = 48000 / 1000 = 48. Ticks at frames 48, 96, 144, 192, 240.
    fx.params().output_rate_hz = 1000;

    // Feed a CC7 event on channel 0 (selected by default). This must be
    // consumed (not passed through) AND must trigger output generation.
    fx.add_midi_event(0, {0xB0, 7, 50});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Output must NOT be empty — the CC7 event triggers output generation.
    // The CC7 input itself is consumed (not passed through), so all output
    // events must be generated CC7 emissions.
    REQUIRE_FALSE(events.empty());

    // At least one CC7 event must be present in the output.
    const auto* cc7_ev = find_cc_event(events, /*channel=*/0, /*cc=*/7);
    REQUIRE(cc7_ev != nullptr);

    // The first output tick is at frame 0 + 48 = 48.
    REQUIRE(cc7_ev->frame_offset == 48u);

    // The filter was initialized from the incoming value 50 (spec §14:
    // no ramp from zero). The first tick quantizes to 50 and is emitted
    // because last_transmitted_value was -1 (force-emit on first tick).
    REQUIRE(cc7_ev->bytes[2] == 50);

    // Verify the filter state was initialized and last_transmitted was updated.
    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized == true);
    REQUIRE(state.last_transmitted_value == 50);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Bypass (enabled=false) passes ALL events through unchanged",
          "[midi][regression]") {
    MidiProcessorFixture fx;
    fx.params().enabled = false;

    // Feed a mix of event types: Note On, CC7 (selected), Pitch Bend.
    // In bypass mode, ALL must pass through unchanged — including CC7,
    // which would normally be consumed.
    fx.add_midi_event(5,  {0x90, 60, 100});       // Note On
    fx.add_midi_event(10, {0xB0, 7, 80});         // CC7 (selected by default)
    fx.add_midi_event(15, {0xE0, 0x00, 0x40});    // Pitch Bend

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // All three events must appear in the output.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 3);

    // Note On — unchanged.
    REQUIRE(events[0].frame_offset == 5u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // CC7 — must appear in output (NOT consumed in bypass mode).
    REQUIRE(events[1].frame_offset == 10u);
    REQUIRE(events[1].bytes[0] == 0xB0);
    REQUIRE(events[1].bytes[1] == 7);
    REQUIRE(events[1].bytes[2] == 80);

    // Pitch Bend — unchanged.
    REQUIRE(events[2].frame_offset == 15u);
    REQUIRE(events[2].bytes[0] == 0xE0);
    REQUIRE(events[2].bytes[1] == 0x00);
    REQUIRE(events[2].bytes[2] == 0x40);

    // No filter states should have been initialized or mutated.
    REQUIRE(fx.states()[0 * 128 + 7].is_initialized == false);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Empty input sequence yields empty output sequence",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    // Do NOT add any input events. The input sequence is empty (atom.size
    // == sizeof(LV2_Atom_Sequence_Body), meaning zero events).
    // No filter states are pre-initialized (all is_initialized == false).
    // Therefore the output must also be empty.

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Output must be empty — no pass-through events and no generated events.
    REQUIRE(events.empty());
    REQUIRE(fx.output_event_count() == 0);

    // The output sequence must still be structurally valid (empty but
    // well-formed: atom.size == sizeof(LV2_Atom_Sequence_Body)).
    REQUIRE(fx.output()->atom.size == sizeof(LV2_Atom_Sequence_Body));
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Valid input sequence yields non-empty output when events are expected",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    // Feed a single Note On event. This is a non-CC event that must
    // pass through to the output unchanged.
    fx.add_midi_event(10, {0x90, 60, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Output must NOT be empty — the Note On must appear in the output.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 1);

    // Verify the first output event is the Note On, unchanged.
    REQUIRE(events[0].frame_offset == 10u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Full integration processing cycle — end-to-end smoke test",
          "[midi][regression][integration]") {
    MidiProcessorFixture fx;

    // Use a high output rate so CC7 output generation fires within the block.
    fx.params().output_rate_hz = 1000;  // frames_per_tick = 48 at 48000 Hz

    // Feed a realistic mix: a Note On (pass-through) and a CC7 (consumed +
    // triggers output generation). Both on channel 0.
    fx.add_midi_event(10, {0x90, 60, 100});    // Note On at frame 10
    fx.add_midi_event(20, {0xB0, 7, 50});      // CC7 at frame 20

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // --- Assertion 1: Output must NOT be empty ---
    // This is the critical regression check. If the bug is present, the
    // output would be empty despite valid input. Both the Note On
    // pass-through AND the CC7 output generation contribute events.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 2);

    // --- Assertion 2: Note On must be present (pass-through path) ---
    bool found_note_on = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0x90) {
            REQUIRE(ev.frame_offset == 10u);
            REQUIRE(ev.bytes[1] == 60);
            REQUIRE(ev.bytes[2] == 100);
            found_note_on = true;
            break;
        }
    }
    REQUIRE(found_note_on);

    // --- Assertion 3: CC7 must be present in output (generation path) ---
    // The CC7 input at frame 20 is consumed (not passed through). But the
    // output generation loop must emit at least one CC7 event. The first
    // tick after frame 20 is at 20 + 48 = 68.
    bool found_cc7_output = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            // First output tick is at frame 20 + 48 = 68.
            REQUIRE(ev.frame_offset == 68u);
            // Value quantized from the initialized filter state (50).
            REQUIRE(ev.bytes[2] == 50);
            found_cc7_output = true;
            break;
        }
    }
    REQUIRE(found_cc7_output);

    // --- Assertion 4: Filter state was initialized and updated ---
    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized == true);
    REQUIRE(state.last_transmitted_value == 50);

    // --- Assertion 5: Output sequence is structurally valid ---
    REQUIRE(fx.output_is_valid());
    REQUIRE(fx.output()->atom.size <= fx.output_capacity());
}
