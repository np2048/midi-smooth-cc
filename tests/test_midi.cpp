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
//         the failed tick.
//     O3: Mixed exhaustion preserves chronological order and structural
//         validity up to the point of exhaustion.
//     O4: Recovery after exhaustion — two-block test verifying
//         last_update_frame consistency.
//
//   64-bit frame timestamp regression tests (marked [midi][frame64]):
//     A:  Frame timestamps below 32-bit boundary are handled correctly
//         (control test, PASSES).
//     B:  Frame timestamps at and above 32-bit boundary are not truncated
//         (regression test, now PASSES after the production fix).
//     C:  Filter state correctly progresses across 32-bit boundary without
//         wraparound (regression test, now PASSES after the production fix).
//
// NOTE: The MidiProcessorFixture explicitly sets all 16 channel_selection
// entries to true (via fill(true)) so that tests are independent of the
// production default_channel_selection() values. This ensures tests
// continue to pass regardless of changes to the production defaults.
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
using midi_smooth_cc::default_channel_selection;

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
        params_.all_channels    = false;
        // Explicitly enable all 16 channels for testing, independent of
        // the production default_channel_selection() which now defaults to
        // only Channel 1 enabled. Tests send CC on MIDI channel 0
        // (0xB0 = Channel 1), so all_channels=false + channel_selection[0]=true
        // would suffice, but fill(true) is used to ensure all channels are
        // processed regardless of which MIDI channel a test event uses.
        params_.channel_selection.fill(true);
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

    // -----------------------------------------------------------------
    // run_64: Passes a 64-bit current_frame directly to process().
    //
    // MidiProcessor::process() now accepts uint64_t current_frame
    // (fixed in the production code). This method passes the full
    // 64-bit value without any narrowing cast, allowing tests to
    // verify correct behavior at and above the 32-bit boundary
    // (UINT32_MAX = 4,294,967,295, reached after ~24.9 hours at
    // 48 kHz).
    //
    // Previously, this method contained static_cast<uint32_t>(
    // current_frame) to replicate a production bug where plugin.cpp
    // narrowed current_frame_ before calling process(). That bug has
    // been fixed in the production code, so the cast has been removed.
    // -----------------------------------------------------------------
    void run_64(uint32_t sample_rate, uint64_t current_frame, uint32_t block_length) {
        processor_.process(input_, output_, sample_rate,
                           current_frame,
                           block_length,
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
// =============================================================================

TEST_CASE("Overflow O1: passthrough exhaustion drops excess events gracefully",
          "[midi][overflow]") {
    MidiProcessorFixture fx(/*output_capacity=*/32);

    fx.add_midi_event(10, {0x90, 60, 100});
    fx.add_midi_event(20, {0x90, 64, 80});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 1);

    REQUIRE(events[0].frame_offset == 10u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    REQUIRE(fx.output()->atom.size <= 32u);
    REQUIRE(fx.output_is_valid());
}

TEST_CASE("Overflow O2: generated event exhaustion does not advance filter state past failed tick",
          "[midi][overflow][regression]") {
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

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE(events.size() == 1);
    REQUIRE(events[0].frame_offset == 48u);
    REQUIRE(events[0].bytes[2] == 10);

    REQUIRE(state.last_transmitted_value == 10);

    REQUIRE(state.last_update_frame == 48u);

    const double current = state.slew_limiter.current_value();
    INFO("slew_limiter.current_value() = " << current
         << ", expected ~10.0 (last emitted tick), but the filter was"
         << " advanced to ~20.0 (the failed tick) because process() was"
         << " called before append_cc_event failed");
    REQUIRE(current < 15.0);
}

TEST_CASE("Overflow O3: mixed passthrough and generated events preserve order until exhaustion",
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
    state.last_update_frame      = 990;

    fx.add_midi_event(10, {0x90, 60, 100});

    fx.run(/*sample_rate=*/48000, /*current_frame=*/1000, /*block_length=*/256);

    const auto events = fx.get_output_events();

    REQUIRE(events.size() == 1);
    REQUIRE(events[0].frame_offset == 10u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    for (size_t i = 1; i < events.size(); ++i) {
        REQUIRE(events[i].frame_offset >= events[i - 1].frame_offset);
    }

    REQUIRE(fx.output()->atom.size <= 32u);
    REQUIRE(fx.output_is_valid());
}

TEST_CASE("Overflow O4: recovery after exhaustion resumes in next block without permanent state corruption",
          "[midi][overflow][regression]") {
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

    fx.run(/*sample_rate=*/48000, /*current_frame=*/0, /*block_length=*/256);

    const auto events_b1 = fx.get_output_events();

    REQUIRE(events_b1.size() == 1);
    REQUIRE(events_b1[0].frame_offset == 48u);
    REQUIRE(events_b1[0].bytes[2] == 10);

    REQUIRE(state.last_transmitted_value == 10);

    REQUIRE(state.last_update_frame == 48u);

    fx.processor().set_output_capacity(1024);
    fx.reset_input();

    fx.run(/*sample_rate=*/48000, /*current_frame=*/256, /*block_length=*/256);

    const auto events_b2 = fx.get_output_events();

    REQUIRE_FALSE(events_b2.empty());

    REQUIRE(events_b2[0].bytes[0] == 0xB0);
    REQUIRE(events_b2[0].bytes[1] == 7);

    REQUIRE(events_b2[0].bytes[2] > 10);

    REQUIRE(fx.output()->atom.size <= 1024u);

    for (size_t i = 1; i < events_b2.size(); ++i) {
        REQUIRE(events_b2[i].frame_offset >= events_b2[i - 1].frame_offset);
    }
}

// =============================================================================
// 64-bit frame timestamp regression tests
//
// The plugin maintains a 64-bit frame counter (current_frame_ in
// MidiSmoothCCPlugin) and passes it directly to MidiProcessor::process()
// as uint64_t. This supports sessions longer than ~24.9 hours at 48 kHz
// (the point at which a 32-bit frame counter would wrap).
//
// The run_64() fixture method passes the full uint64_t current_frame
// to process() without any narrowing cast. These tests verify correct
// behavior below, at, and above the 32-bit boundary.
//
// Reference values:
//   UINT32_MAX = 4,294,967,295
//   At 48 kHz, UINT32_MAX is reached after 4294967295 / 48000 = 89478.5
//   seconds = 24.85 hours = ~24 hours 51 minutes.
//
// Test constants:
//   4294967000 = UINT32_MAX - 295  (just below the 32-bit boundary)
//   4294968000 = UINT32_MAX + 705  (just above the 32-bit boundary)
// =============================================================================

// -----------------------------------------------------------------------------
// Test A (Control): Frame timestamps below 32-bit boundary are handled correctly
//
// Uses a normal current_frame (1000) that fits in uint32_t without
// truncation. Verifies that handle_selected_cc sets last_update_frame
// to current_frame + event_frame_offset.
// -----------------------------------------------------------------------------
TEST_CASE("Frame timestamps below 32-bit boundary are handled correctly",
          "[midi][frame64]") {
    MidiProcessorFixture fx;

    // output_rate_hz = 100 -> frames_per_tick = 480. Block length = 256.
    // No output ticks fire (480 > 256). This isolates the test to
    // handle_selected_cc's last_update_frame computation.
    const uint64_t current_frame = 1000;

    fx.add_midi_event(5, {0xB0, 7, 50});

    fx.run_64(/*sample_rate=*/48000, current_frame, /*block_length=*/256);

    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized == true);

    // handle_selected_cc sets:
    //   last_update_frame = current_frame + event_frame_offset = 1000 + 5 = 1005
    REQUIRE(state.last_update_frame == current_frame + 5ULL);
    REQUIRE(state.last_update_frame == 1005ULL);
}

// -----------------------------------------------------------------------------
// Test B (Regression): Frame timestamps at and above 32-bit boundary are
// not truncated
//
// Uses current_frame = 4294968000 (UINT32_MAX + 705). run_64 passes this
// as uint64_t directly to process(). handle_selected_cc computes
// last_update_frame = 4294968000 + 5 = 4294968005 using 64-bit arithmetic.
//
// The assertion checks that last_update_frame equals the correct 64-bit
// value. This PASSES because process() accepts uint64_t and no
// truncation occurs.
// -----------------------------------------------------------------------------
TEST_CASE("Frame timestamps at and above 32-bit boundary are not truncated",
          "[midi][frame64][regression]") {
    MidiProcessorFixture fx;

    // 4294968000 = UINT32_MAX + 705 = 0x1000002C0
    const uint64_t current_frame = 4294968000ULL;
    const uint32_t event_frame_offset = 5;

    fx.add_midi_event(event_frame_offset, {0xB0, 7, 50});

    fx.run_64(/*sample_rate=*/48000, current_frame, /*block_length=*/256);

    FilterState& state = fx.states()[0 * 128 + 7];
    REQUIRE(state.is_initialized == true);

    // process() receives current_frame as uint64_t. handle_selected_cc
    // computes last_update_frame = 4294968000 + 5 = 4294968005 using
    // 64-bit arithmetic. No truncation occurs.
    INFO("last_update_frame = " << state.last_update_frame
         << ", expected " << (current_frame + event_frame_offset));
    REQUIRE(state.last_update_frame == current_frame + static_cast<uint64_t>(event_frame_offset));
}

// -----------------------------------------------------------------------------
// Test C (Regression): Filter state correctly progresses across 32-bit
// boundary without wraparound
//
// Pre-initializes a filter state with last_update_frame = 4294967000
// (295 frames below UINT32_MAX). Runs a block with current_frame =
// 4294968000 (705 frames above UINT32_MAX).
//
// process() receives current_frame as uint64_t. block_start = 4294968000.
// next_tick = 4294967000 + 48 = 4294967048. Since 4294967048 <
// 4294968000 (block_start), catch-up triggers. The filter advances
// from 4294967000 to 4294968000. Ticks at 4294968048, 4294968096, ...
// are processed. last_update_frame advances past 4294968000.
//
// The assertion checks that last_update_frame > current_frame, proving
// the filter progressed across the boundary using 64-bit arithmetic.
// -----------------------------------------------------------------------------
TEST_CASE("Filter state correctly progresses across 32-bit boundary without wraparound",
          "[midi][frame64][regression]") {
    MidiProcessorFixture fx;

    // High output rate so multiple ticks fire within the block.
    // frames_per_tick = 48000 / 1000 = 48.
    fx.params().output_rate_hz = 1000;
    fx.params().smoothing_mode = SmoothingMode::Slew;
    fx.params().rising_rate    = 10000;
    fx.params().falling_rate   = 10000;

    // Pre-initialize the filter state with last_update_frame just below
    // UINT32_MAX. This represents a session that has been running for
    // ~24.9 hours.
    const uint64_t initial_last_update = 4294967000ULL;  // UINT32_MAX - 295
    const uint64_t current_frame       = 4294968000ULL;  // UINT32_MAX + 705

    FilterState& state = fx.states()[0 * 128 + 7];
    state.is_initialized = true;
    state.exp_smoother.reset(0.0);
    state.slew_limiter.reset(0.0);
    state.exp_smoother.process(100.0, 0.0, 0.05, 0.1);
    state.slew_limiter.process(100.0, 0.0, 10000.0, 10000.0);
    state.last_transmitted_value = 0;
    state.last_update_frame      = initial_last_update;

    // Run a block that crosses the 32-bit boundary.
    // process() receives current_frame as uint64_t.
    // block_start = 4294968000 (64-bit, no truncation).
    // next_tick = 4294967000 + 48 = 4294967048.
    // 4294967048 < 4294968000: catch-up triggers.
    // Filter advances from 4294967000 to 4294968000.
    // Ticks at 4294968048, 4294968096, ... are processed.
    // last_update_frame advances past 4294968000.
    fx.run_64(/*sample_rate=*/48000, current_frame, /*block_length=*/256);

    // last_update_frame should have advanced beyond current_frame,
    // proving the filter progressed across the 32-bit boundary using
    // 64-bit arithmetic.
    INFO("last_update_frame = " << state.last_update_frame
         << ", expected > " << current_frame);
    REQUIRE(state.last_update_frame > current_frame);
}
