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
//
//   Output overflow regression tests (marked [midi][overflow]):
//     O1: Passthrough exhaustion drops excess events gracefully.
//     O2: Generated event exhaustion does not advance filter state past
//         the failed tick (FAILS — reveals premature last_update_frame
//         update before append).
//     O3: Mixed exhaustion preserves chronological order and structural
//         validity up to the point of exhaustion.
//     O4: Recovery after exhaustion — two-block test verifying
//         last_update_frame consistency (FAILS — same root cause as O2).
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
// =============================================================================

TEST_CASE("Generated events are emitted in strict chronological order relative to passthrough events",
          "[midi][chronological]") {
    MidiProcessorFixture fx;

    fx.params().output_rate_hz = 1000;

    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 952;

    fx.add_midi_event(10, {0x90, 60, 100});
    fx.add_midi_event(20, {0xB0, 7, 50});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());

    for (size_t i = 1; i < events.size(); ++i) {
        INFO("Event " << i << " frame_offset=" << events[i].frame_offset
             << " follows event " << (i - 1)
             << " frame_offset=" << events[i - 1].frame_offset
             << " — sequence is NOT chronologically sorted");
        REQUIRE(events[i].frame_offset >= events[i - 1].frame_offset);
    }

    size_t cc_index      = events.size();
    size_t note_on_index = events.size();

    for (size_t i = 0; i < events.size(); ++i) {
        if (cc_index == events.size() &&
            events[i].bytes.size() == 3 &&
            events[i].bytes[0] == 0xB0 &&
            events[i].bytes[1] == 7) {
            cc_index = i;
        }
        if (events[i].bytes.size() == 3 &&
            events[i].bytes[0] == 0x90) {
            note_on_index = i;
            break;
        }
    }

    REQUIRE(cc_index      < events.size());
    REQUIRE(note_on_index < events.size());

    REQUIRE(events[cc_index].frame_offset      == 0u);
    REQUIRE(events[note_on_index].frame_offset == 10u);

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

    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 962;

    fx.add_midi_event(10, {0x90, 60, 100});
    fx.add_midi_event(10, {0xB0, 7, 50});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());

    for (size_t i = 1; i < events.size(); ++i) {
        INFO("Event " << i << " frame_offset=" << events[i].frame_offset
             << " follows event " << (i - 1)
             << " frame_offset=" << events[i - 1].frame_offset);
        REQUIRE(events[i].frame_offset >= events[i - 1].frame_offset);
    }

    REQUIRE(fx.output_is_valid());

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

// =============================================================================
// Output overflow regression tests (spec §11)
//
// These tests verify that the plugin handles output buffer exhaustion safely.
// The current implementation has two suspected defects:
//
// 1. PASSTHROUGH SILENT DROP: In process(), the return value of
//    append_passthrough_event is cast to (void), meaning passthrough
//    append failures are silently ignored. Events vanish without trace.
//
// 2. PREMATURE last_update_frame UPDATE: In emit_output_events_until(),
//    state.last_update_frame is updated to next_tick BEFORE the
//    append_cc_event attempt. If the append fails, the tick is considered
//    "processed" and the value is permanently lost. The filter state
//    (current_value and last_update_frame) advances past the failed tick,
//    creating an inconsistency between the filter's internal state and the
//    actual output. On the next block, the filter resumes from the
//    advanced position, and the skipped value is never emitted.
//
// Test O1 verifies graceful passthrough exhaustion handling (guard test).
// Test O2 reveals the premature last_update_frame update (FAILS).
// Test O3 verifies mixed exhaustion preserves ordering (guard test).
// Test O4 reveals the filter state inconsistency after recovery (FAILS).
// =============================================================================

// -----------------------------------------------------------------------------
// O1: Passthrough Exhaustion
//
// A small output capacity (32) holds exactly one padded MIDI event.
// Two Note On events are injected. The first fits; the second is dropped.
// The test verifies the output remains structurally valid and iterable.
//
// This test PASSES with the current implementation because
// lv2_atom_sequence_append_event internally checks capacity and returns
// NULL when full. The (void) cast means the drop is silent, but the
// output is not corrupted.
// -----------------------------------------------------------------------------
TEST_CASE("Overflow O1: passthrough exhaustion drops excess events gracefully",
          "[midi][overflow]") {
    // Capacity = 32. One padded MIDI event = 16 bytes.
    // sizeof(LV2_Atom_Sequence_Body) varies (4/8/16).
    // After 1 event: atom.size = sizeof_body + 16.
    // Remaining: 32 - (sizeof_body + 16) < 16 for any sizeof_body.
    // Second event is rejected by lv2_atom_sequence_append_event.
    MidiProcessorFixture fx(/*output_capacity=*/32);

    fx.add_midi_event(10, {0x90, 60, 100});
    fx.add_midi_event(20, {0x90, 64, 80});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // The first Note On must be present.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 1);

    // Verify the first event is the Note On at frame 10.
    REQUIRE(events[0].frame_offset == 10u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // atom.size must not exceed the capacity.
    REQUIRE(fx.output()->atom.size <= 32u);

    // The output must be structurally valid (iterable without crash).
    // get_output_events() already iterated the sequence; reaching this
    // assertion means the structure is valid.
    REQUIRE(fx.output_is_valid());
}

// -----------------------------------------------------------------------------
// O2: Generated Event Exhaustion — premature last_update_frame update
//
// A pre-initialized slew limiter state generates CC events at 1000 Hz
// (frames_per_tick = 48). The output capacity (32) holds exactly one
// event. The block has 5 ticks (at frames 48, 96, 144, 192, 240).
//
// Tick at 48: value 10. Emitted. atom.size = sizeof_body + 16.
// Tick at 96: value 20. append fails (buffer full). BUT
//   state.last_update_frame was already set to 96 BEFORE the append
//   attempt. The filter's current_value was advanced to 20.
//
// After the block:
//   BUGGY:   last_update_frame = 96, current_value = 20
//   CORRECT: last_update_frame = 48, current_value = 10
//
// The assertions check that last_update_frame and current_value reflect
// only the last SUCCESSFULLY emitted tick, not the failed one. These
// assertions FAIL with the current implementation.
// -----------------------------------------------------------------------------
TEST_CASE("Overflow O2: generated event exhaustion does not advance filter state past failed tick",
          "[midi][overflow][regression]") {
    MidiProcessorFixture fx(/*output_capacity=*/32);

    fx.params().output_rate_hz = 1000;   // frames_per_tick = 48
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;  // 10 CC per tick (dt = 0.001s)
    fx.params().falling_rate   = 10000;

    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 0;

    // Block: 256 frames. Ticks at 48, 96, 144, 192, 240.
    // Tick 48: slew 0 + 10 = 10. Emitted (first event fits in capacity 32).
    // Tick 96: slew 10 + 10 = 20. Append FAILS (buffer full).
    //   BUG: last_update_frame set to 96 BEFORE append.
    //   current_value advanced to 20.
    //   last_transmitted stays at 10.
    //   emit_output_events_until returns immediately.
    // Ticks 144, 192, 240: never processed.
    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Only 1 event should be in the output (the first tick).
    REQUIRE(events.size() == 1);
    REQUIRE(events[0].frame_offset == 48u);
    REQUIRE(events[0].bytes[2] == 10);

    // last_transmitted_value should be 10 (the last emitted value).
    // This PASSES — last_transmitted is only updated on successful append.
    REQUIRE(state.last_transmitted_value == 10);

    // -------------------------------------------------------------------
    // KEY ASSERTION 1: last_update_frame should be at the last
    // SUCCESSFULLY emitted tick (48), not at the FAILED tick (96).
    //
    // BUG: state.last_update_frame is set to next_tick (96) BEFORE
    // append_cc_event is called. When the append fails, the function
    // returns, but last_update_frame remains at 96. The tick is
    // considered "processed" even though no event was emitted.
    //
    // This assertion FAILS: 96 != 48.
    // -------------------------------------------------------------------
    REQUIRE(state.last_update_frame == 48u);

    // -------------------------------------------------------------------
    // KEY ASSERTION 2: The filter's current_value should reflect only
    // the successfully emitted tick (value 10), not the failed tick
    // (value 20).
    //
    // BUG: slew_limiter.process() was called for tick 96 BEFORE the
    // append attempt, advancing current_value to 20. Even though the
    // event was not emitted, the filter state has moved past the value.
    //
    // This assertion FAILS: current_value is 20, not 10.
    // -------------------------------------------------------------------
    const double current = state.slew_limiter.current_value();
    INFO("slew_limiter.current_value() = " << current
         << ", expected ~10.0 (last emitted tick), but the filter was"
         << " advanced to ~20.0 (the failed tick) because process() was"
         << " called before append_cc_event failed");
    REQUIRE(current < 15.0);  // 10.0 < 15.0 (PASS), 20.0 < 15.0 (FAIL)
}

// -----------------------------------------------------------------------------
// O3: Mixed Exhaustion — chronological order preserved until exhaustion
//
// A Note On (passthrough) at frame_offset 10 fills the small buffer.
// A pre-initialized CC7 state generates a tick at frame_offset 38.
// The CC tick's append fails because the buffer is full from the Note On.
//
// The test verifies:
//   - The output contains only the Note On (CC tick dropped).
//   - The output is chronologically sorted (trivially, 1 event).
//   - atom.size does not exceed capacity.
//   - The output sequence is iterable (structurally valid).
//
// This test PASSES with the current implementation — the interleaving
// logic and append_event's capacity check work correctly even during
// overflow.
// -----------------------------------------------------------------------------
TEST_CASE("Overflow O3: mixed passthrough and generated events preserve order until exhaustion",
          "[midi][overflow]") {
    MidiProcessorFixture fx(/*output_capacity=*/32);

    fx.params().output_rate_hz = 1000;   // frames_per_tick = 48
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;
    fx.params().falling_rate   = 10000;

    // Pre-initialize CC7: first tick at frame_offset 38.
    // last_update_frame = 962, frames_per_tick = 48.
    // next_tick = 962 + 48 = 1010. With current_frame = 1000,
    // frame_offset = 1010 - 1000 = 10... wait, that's the same as
    // the Note On. Let me use last_update_frame = 990 instead.
    // next_tick = 990 + 48 = 1038. frame_offset = 1038 - 1000 = 38.
    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = 990;  // first tick at frame_offset 38

    // Note On at frame_offset 10 (passthrough, fills the buffer).
    fx.add_midi_event(10, {0x90, 60, 100});

    // Run with current_frame=1000, block_length=256.
    // Interleaving:
    //   1. Before Note On (10): emit_output_events_until(deadline=1010).
    //      next_tick = 1038. 1038 >= 1010. No ticks emitted.
    //   2. Note On (10): append. atom.size = sizeof_body + 16.
    //   3. After input: emit_output_events_until(deadline=1256).
    //      next_tick = 1038. 1038 < 1256. Process tick.
    //      Filter advanced. last_update_frame = 1038.
    //      Value computed. Try to emit.
    //      Capacity check: 32 - (sizeof_body + 16) < 16. REJECTED.
    //      Return immediately.
    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/256);

    const auto events = fx.get_output_events();

    // Only the Note On should be in the output (CC tick dropped).
    REQUIRE(events.size() == 1);
    REQUIRE(events[0].frame_offset == 10u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // The output must be chronologically sorted (trivially, 1 event).
    for (size_t i = 1; i < events.size(); ++i) {
        REQUIRE(events[i].frame_offset >= events[i - 1].frame_offset);
    }

    // atom.size must not exceed capacity.
    REQUIRE(fx.output()->atom.size <= 32u);

    // The output must be structurally valid.
    REQUIRE(fx.output_is_valid());
}

// -----------------------------------------------------------------------------
// O4: Recovery after exhaustion — two-block test
//
// Block 1: small capacity (32), pre-initialized slew state.
//   Tick at 48: value 10. Emitted.
//   Tick at 96: value 20. Append fails. last_update_frame = 96 (BUG).
//   After block 1: last_update_frame = 96, current_value = 20.
//
// Block 2: large capacity (1024), no input.
//   next_tick = 96 + 48 = 144. 144 < 256 (block_start). Catch-up.
//   The filter catches up from 96 to 256, advancing current_value.
//   Ticks within block 2 are emitted normally.
//
// The test verifies:
//   - Block 1 produces exactly 1 event (the first tick).
//   - After block 1, last_update_frame should be 48 (FAILS: 96).
//   - Block 2 produces events (recovery works, but the value 20 was lost).
//
// The last_update_frame assertion after block 1 FAILS with the current
// implementation, demonstrating the same root cause as O2: the filter
// state is advanced past the failed tick.
// -----------------------------------------------------------------------------
TEST_CASE("Overflow O4: recovery after exhaustion resumes in next block without permanent state corruption",
          "[midi][overflow][regression]") {
    // Start with small capacity for block 1.
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

    // ---- Block 1: current_frame=0, block_length=256 ----
    // Tick at 48: value 10. Emitted (capacity allows 1 event).
    // Tick at 96: value 20. Append FAILS.
    //   BUG: last_update_frame = 96 (set before append).
    //   current_value = 20 (filter advanced).
    //   last_transmitted = 10 (not updated on failure).
    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events_b1 = fx.get_output_events();

    // Block 1 should have exactly 1 event.
    REQUIRE(events_b1.size() == 1);
    REQUIRE(events_b1[0].frame_offset == 48u);
    REQUIRE(events_b1[0].bytes[2] == 10);

    // last_transmitted should be 10 (the emitted value).
    REQUIRE(state.last_transmitted_value == 10);

    // -------------------------------------------------------------------
    // KEY ASSERTION: After block 1, last_update_frame should be at the
    // last SUCCESSFULLY emitted tick (48), not at the FAILED tick (96).
    //
    // BUG: last_update_frame = 96 because it was set before the append
    // attempt. The filter state has advanced past the value 20, which
    // was never emitted. On the next block, the filter resumes from 96,
    // permanently skipping the value 20.
    //
    // This assertion FAILS: 96 != 48.
    // -------------------------------------------------------------------
    REQUIRE(state.last_update_frame == 48u);

    // ---- Block 2: switch to large capacity, advance current_frame ----
    fx.processor().set_output_capacity(1024);
    fx.reset_input();

    // Block 2: current_frame=256, block_length=256. Block [256, 512).
    // With buggy state: last_update_frame=96.
    //   next_tick = 96 + 48 = 144. 144 < 256. Catch-up.
    //   Catch-up advances filter from frame 96 to 256.
    //   Ticks at 304, 352, ... are emitted.
    //
    // With correct state: last_update_frame=48.
    //   next_tick = 48 + 48 = 96. 96 < 256. Catch-up.
    //   Catch-up advances filter from frame 48 to 256.
    //   Ticks at 304, 352, ... are emitted.
    //
    // In both cases, block 2 produces events (recovery works).
    // The difference is the filter's internal state consistency.
    fx.run(/*sample_rate=*/48000, /*current_frame=*/256, /*block_length=*/256);

    const auto events_b2 = fx.get_output_events();

    // Block 2 should produce events (recovery).
    REQUIRE_FALSE(events_b2.empty());

    // The first event in block 2 should be a CC7.
    REQUIRE(events_b2[0].bytes[0] == 0xB0);
    REQUIRE(events_b2[0].bytes[1] == 7);

    // The value should be greater than 10 (the last emitted value from
    // block 1). The filter continues converging toward 100.
    REQUIRE(events_b2[0].bytes[2] > 10);

    // The output must be structurally valid.
    REQUIRE(fx.output()->atom.size <= 1024u);

    // Events must be chronologically sorted.
    for (size_t i = 1; i < events_b2.size(); ++i) {
        REQUIRE(events_b2[i].frame_offset >= events_b2[i - 1].frame_offset);
    }
}
