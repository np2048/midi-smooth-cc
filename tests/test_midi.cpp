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
//   Chronological ordering tests (marked [midi][chronological]):
//     C1: Generated events are emitted in strict chronological order
//         relative to passthrough events.
//     C2: Multiple events with the same timestamp are handled
//         deterministically.
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
    static constexpr LV2_URID kMidiUrid          = 1;
    static constexpr LV2_URID kSeqType           = 0;
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
// Test Case 4: CC selection (spec §5)
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
// Test Case 5: Output generation (spec §8)
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
// =============================================================================

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Unselected events pass through to output unchanged",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    fx.add_midi_event(5, {0x90, 60, 100});
    fx.add_midi_event(15, {0xE0, 0x00, 0x40});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 2);

    REQUIRE(events[0].frame_offset == 5u);
    REQUIRE(events[0].bytes.size() == 3);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

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

    fx.params().output_rate_hz = 1000;

    fx.add_midi_event(0, {0xB0, 7, 50});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());

    const auto* cc7_ev = find_cc_event(events, /*channel=*/0, /*cc=*/7);
    REQUIRE(cc7_ev != nullptr);

    REQUIRE(cc7_ev->frame_offset == 48u);
    REQUIRE(cc7_ev->bytes[2] == 50);

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

    fx.add_midi_event(5,  {0x90, 60, 100});
    fx.add_midi_event(10, {0xB0, 7, 80});
    fx.add_midi_event(15, {0xE0, 0x00, 0x40});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 3);

    REQUIRE(events[0].frame_offset == 5u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    REQUIRE(events[1].frame_offset == 10u);
    REQUIRE(events[1].bytes[0] == 0xB0);
    REQUIRE(events[1].bytes[1] == 7);
    REQUIRE(events[1].bytes[2] == 80);

    REQUIRE(events[2].frame_offset == 15u);
    REQUIRE(events[2].bytes[0] == 0xE0);
    REQUIRE(events[2].bytes[1] == 0x00);
    REQUIRE(events[2].bytes[2] == 0x40);

    REQUIRE(fx.states()[0 * 128 + 7].is_initialized == false);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Empty input sequence yields empty output sequence",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE(events.empty());
    REQUIRE(fx.output_event_count() == 0);

    REQUIRE(fx.output()->atom.size == sizeof(LV2_Atom_Sequence_Body));
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue.
// This test ensures the plugin correctly routes, passes through, and emits MIDI events.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Regression: Valid input sequence yields non-empty output when events are expected",
          "[midi][regression]") {
    MidiProcessorFixture fx;

    fx.add_midi_event(10, {0x90, 60, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 1);

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

    fx.params().output_rate_hz = 1000;

    fx.add_midi_event(10, {0x90, 60, 100});
    fx.add_midi_event(20, {0xB0, 7, 50});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 2);

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

    bool found_cc7_output = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            REQUIRE(ev.frame_offset == 68u);
            REQUIRE(ev.bytes[2] == 50);
            found_cc7_output = true;
            break;
        }
    }
    REQUIRE(found_cc7_output);

    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized == true);
    REQUIRE(state.last_transmitted_value == 50);

    REQUIRE(fx.output_is_valid());
    REQUIRE(fx.output()->atom.size <= fx.output_capacity());
}

// =============================================================================
// Chronological ordering regression tests
//
// These tests verify that generated CC events and passthrough events are
// interleaved in strict chronological order by frame_offset in the output
// LV2_Atom_Sequence. The LV2 specification requires Atom Sequences to be
// ordered by timestamp; strict hosts (like Carla) may reject or silently
// drop out-of-order sequences.
//
// Suspected defect: midi_processor.cpp runs emit_output_events() AFTER the
// passthrough loop, so generated events are always appended at the END of
// the output sequence regardless of their frame_offset. If a generated CC
// event has an earlier frame_offset than a passthrough event, the output
// sequence violates the chronological ordering requirement.
// =============================================================================

TEST_CASE("Generated events are emitted in strict chronological order relative to passthrough events",
          "[midi][chronological]") {
    MidiProcessorFixture fx;

    // High output rate so generated CC events have predictable frame offsets.
    // frames_per_tick = 48000 / 1000 = 48.
    fx.params().output_rate_hz = 1000;

    // Pre-initialize the CC7 filter state so output generation produces a
    // CC event at frame_offset 0 of the current block.
    //
    // last_update_frame = 952, frames_per_tick = 48
    //   -> first tick at absolute frame 952 + 48 = 1000
    //   -> with current_frame = 1000, frame_offset = 1000 - 1000 = 0
    //
    // The generated CC at frame_offset 0 is EARLIER than the Note On
    // passthrough event at frame_offset 10. In a correctly ordered output
    // sequence, the CC must appear BEFORE the Note On.
    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    // Set initial target to 100 (current stays 0 due to dt=0 trick).
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;   // force emit on first tick (0 != filtered value)
    state.last_update_frame      = 952; // first tick lands at absolute frame 1000

    // Inject a Note On at frame_offset 10 (passthrough — will be appended
    // during the input loop, BEFORE emit_output_events runs).
    fx.add_midi_event(10, {0x90, 60, 100});

    // Inject a CC7 at frame_offset 20 (consumed — updates the target but
    // does NOT change last_update_frame since the state is already
    // initialized).
    fx.add_midi_event(20, {0xB0, 7, 50});

    // Run with current_frame = 1000 so the first output tick falls at
    // frame_offset 0 (earlier than the Note On at frame_offset 10).
    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // The output must NOT be empty — both the Note On and the generated CC
    // should be present.
    REQUIRE_FALSE(events.empty());

    // ---------------------------------------------------------------
    // Assertion 1: All output events must be sorted by frame_offset in
    // non-decreasing order (LV2 Atom Sequence requirement).
    //
    // This FAILS with the current implementation because:
    //   - The Note On (frame_offset 10) is appended during the input loop.
    //   - The generated CC (frame_offset 0) is appended during
    //     emit_output_events, AFTER the input loop.
    //   - Output order: [Note On (10), CC (0), ...]
    //   - 10 > 0 violates non-decreasing order.
    // ---------------------------------------------------------------
    for (size_t i = 1; i < events.size(); ++i) {
        INFO("Event " << i << " frame_offset=" << events[i].frame_offset
             << " follows event " << (i - 1)
             << " frame_offset=" << events[i - 1].frame_offset
             << " — sequence is NOT chronologically sorted");
        REQUIRE(events[i].frame_offset >= events[i - 1].frame_offset);
    }

    // ---------------------------------------------------------------
    // Assertion 2: The generated CC (frame_offset 0) must appear BEFORE
    // the Note On (frame_offset 10) in the output vector.
    //
    // This FAILS with the current implementation because the CC is
    // appended after the Note On (emit_output_events runs after the
    // passthrough loop), so the CC's index in the vector is HIGHER than
    // the Note On's index despite having a LOWER frame_offset.
    // ---------------------------------------------------------------
    size_t cc_index      = events.size();  // sentinel: "not found"
    size_t note_on_index = events.size();

    for (size_t i = 0; i < events.size(); ++i) {
        // Identify the generated CC: status 0xB0, CC number 7.
        // The first such event is the generated one (frame_offset 0).
        if (cc_index == events.size() &&
            events[i].bytes.size() == 3 &&
            events[i].bytes[0] == 0xB0 &&
            events[i].bytes[1] == 7) {
            cc_index = i;
        }
        // Identify the Note On: status 0x90.
        if (events[i].bytes.size() == 3 &&
            events[i].bytes[0] == 0x90) {
            note_on_index = i;
            break;
        }
    }

    REQUIRE(cc_index      < events.size());
    REQUIRE(note_on_index < events.size());

    // Verify the frame_offsets are as expected, confirming the test setup
    // is correct (CC at 0, Note On at 10).
    REQUIRE(events[cc_index].frame_offset      == 0u);
    REQUIRE(events[note_on_index].frame_offset == 10u);

    // THE KEY ASSERTION: CC (frame 0) must appear before Note On (frame 10)
    // in the output vector. This proves chronological interleaving.
    INFO("Generated CC at index " << cc_index
         << " (frame_offset=" << events[cc_index].frame_offset
         << "), Note On at index " << note_on_index
         << " (frame_offset=" << events[note_on_index].frame_offset
         << ") — CC should appear BEFORE Note On");
    REQUIRE(cc_index < note_on_index);
}

TEST_CASE("Multiple events with the same timestamp are handled deterministically",
          "[midi][chronological]") {
    MidiProcessorFixture fx;

    fx.params().output_rate_hz = 1000;

    // Pre-initialize the CC7 filter state so the first generated CC falls
    // at frame_offset 10 — the SAME frame_offset as the Note On and CC7
    // input events.
    //
    // last_update_frame = 962, frames_per_tick = 48
    //   -> first tick at absolute frame 962 + 48 = 1010
    //   -> with current_frame = 1000, frame_offset = 1010 - 1000 = 10
    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 962;  // first tick at frame_offset 10

    // Inject a Note On AND a CC7 at the EXACT SAME frame_offset (10).
    // Both input events share timestamp 10.
    fx.add_midi_event(10, {0x90, 60, 100});  // Note On (passthrough)
    fx.add_midi_event(10, {0xB0, 7, 50});    // CC7 (consumed)

    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Must not crash and must produce output.
    REQUIRE_FALSE(events.empty());

    // The output must be sorted by frame_offset in non-decreasing order.
    // Events with the same timestamp (frame_offset) may appear in any
    // relative order — the LV2 spec allows equal timestamps.
    //
    // With the current implementation:
    //   - Note On (10) is appended during the input loop.
    //   - Generated CC (10) is appended during emit_output_events.
    //   - Output: [Note On (10), CC (10), CC (58), ...]
    //   - 10 >= 10 is satisfied (non-decreasing), so this assertion PASSES
    //     even with the current buggy implementation.
    //
    // This test guards against future regressions where same-timestamp
    // handling might break (e.g., crash, corruption, or inverted order
    // that violates non-decreasing).
    for (size_t i = 1; i < events.size(); ++i) {
        INFO("Event " << i << " frame_offset=" << events[i].frame_offset
             << " follows event " << (i - 1)
             << " frame_offset=" << events[i - 1].frame_offset);
        REQUIRE(events[i].frame_offset >= events[i - 1].frame_offset);
    }

    // Verify the output sequence is structurally valid.
    REQUIRE(fx.output_is_valid());

    // Verify that at least one Note On and one CC7 are present.
    bool found_note_on = false;
    bool found_cc7     = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0x90) {
            found_note_on = true;
        }
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            found_cc7 = true;
        }
    }
    REQUIRE(found_note_on);
    REQUIRE(found_cc7);
}
