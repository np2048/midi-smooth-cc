// midi-smooth-cc/tests/test_carla_replication.cpp

// =============================================================================
// MIDI Smooth CC - Carla environment replication tests
//
// These tests replicate Carla's EXACT runtime environment to catch the
// "black hole" bug where the plugin produces no output in Carla despite
// all unit tests passing.
//
// Key differences from previous tests:
//   1. URID 42 for midi:MidiEvent (more realistic than URID 1).
//   2. Exact Carla default parameter values (output_rate_hz = 100, not 1000).
//   3. Multiple output buffer capacities (1024, 4096, 8192) to test whether
//      the hardcoded output_capacity_ = 16384u in the plugin constructor
//      causes silent failures when Carla provides a smaller buffer.
//   4. Very small buffer (128 bytes) to test capacity-mismatch overflow.
//   5. Output pre-filled with garbage to test clearing behavior.
//   6. Output atom.type left unset to test if the plugin sets it.
//   7. Null control ports to test default-value fallback.
//
// Updated for Option B channel refactor:
//   The former PORT_CHANNEL_MASK (single integer port) has been replaced by
//   PORT_ALL_CHANNELS (toggle) plus PORT_CHANNEL_SEL_BASE..PORT_CHANNEL_SEL_BASE+15
//   (16 individual toggles). CarlaDefaults now sets all_channels=0.0 and all
//   16 channel selectors to 1.0, preserving the previous "all channels on"
//   default behavior.
// =============================================================================
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"

#include "plugin.hpp"
#include "parameters.hpp"
#include "filter_state.hpp"
#include "smoothing.hpp"

using midi_smooth_cc::MidiSmoothCCPlugin;
using midi_smooth_cc::PluginParameters;
using midi_smooth_cc::SmoothingMode;

namespace {

// =============================================================================
// Mock URID map — returns URID 42 for midi:MidiEvent (realistic host URID).
// =============================================================================

constexpr LV2_URID kCarlaMidiUrid = 42;

static LV2_URID carla_urid_map(LV2_URID_Map_Handle handle, const char* uri) {
    (void)handle;
    if (std::strcmp(uri, LV2_MIDI__MidiEvent) == 0) {
        return kCarlaMidiUrid;
    }
    return 0;
}

struct CarlaUridMap {
    LV2_URID_Map map;
    LV2_Feature  feature;

    CarlaUridMap() {
        map.handle   = nullptr;
        map.map      = carla_urid_map;
        feature.URI  = LV2_URID__map;
        feature.data = &map;
    }
};

// =============================================================================
// Carla exact default parameter values.
//
// Updated for Option B: the former channel_mask=65535.0f is replaced by
// all_channels=0.0f (whitelist mode) plus all 16 channel selectors = 1.0f
// (all channels enabled), which is functionally equivalent.
// =============================================================================

struct CarlaDefaults {
    float values[MidiSmoothCCPlugin::PORT_TOTAL];

    CarlaDefaults() {
        std::memset(values, 0, sizeof(values));

        // EXACT values from Carla:
        values[MidiSmoothCCPlugin::PORT_ENABLED]        = 1.0f;     // On
        values[MidiSmoothCCPlugin::PORT_SMOOTHING_MODE] = 0.0f;     // Exponential
        values[MidiSmoothCCPlugin::PORT_ATTACK_MS]      = 50.0f;
        values[MidiSmoothCCPlugin::PORT_RELEASE_MS]     = 100.0f;
        values[MidiSmoothCCPlugin::PORT_RISING_RATE]    = 500.0f;
        values[MidiSmoothCCPlugin::PORT_FALLING_RATE]   = 500.0f;
        values[MidiSmoothCCPlugin::PORT_DEADBAND]       = 0.0f;
        values[MidiSmoothCCPlugin::PORT_OUTPUT_RATE_HZ] = 100.0f;   // Carla default, NOT 1000
        values[MidiSmoothCCPlugin::PORT_ALL_CC]         = 0.0f;     // false (whitelist)
        values[MidiSmoothCCPlugin::PORT_ALL_CHANNELS]   = 0.0f;     // false (whitelist)

        // All 16 channel selectors = 1.0 (all channels enabled).
        for (uint32_t i = 0; i < MidiSmoothCCPlugin::PORT_CHANNEL_SEL_COUNT; ++i) {
            values[MidiSmoothCCPlugin::PORT_CHANNEL_SEL_BASE + i] = 1.0f;
        }

        // CC Selection 7 = 1.0 (enabled), all others 0.0.
        values[MidiSmoothCCPlugin::PORT_CC_SEL_BASE + 7] = 1.0f;
    }

    void connect_all(LV2_Handle handle, const LV2_Descriptor* desc) {
        for (uint32_t i = 2; i < MidiSmoothCCPlugin::PORT_TOTAL; ++i) {
            desc->connect_port(handle, i, &values[i]);
        }
    }

    void connect_all_direct(MidiSmoothCCPlugin* plugin) {
        for (uint32_t i = 2; i < MidiSmoothCCPlugin::PORT_TOTAL; ++i) {
            plugin->connect_port(i, &values[i]);
        }
    }
};

// =============================================================================
// Atom sequence helpers.
// =============================================================================

struct OutputEvent {
    uint32_t             frame_offset;
    std::vector<uint8_t> bytes;
};

std::vector<OutputEvent> read_output_events(const LV2_Atom_Sequence* seq) {
    std::vector<OutputEvent> events;
    LV2_Atom_Event* ev  = lv2_atom_sequence_begin(&seq->body);
    LV2_Atom_Event* end = lv2_atom_sequence_end(&seq->body, seq->atom.size);
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

void add_midi_event(LV2_Atom_Sequence* seq, size_t buffer_size,
                    uint32_t frame_offset,
                    const uint8_t* bytes, size_t num_bytes) {
    struct StackEvent {
        LV2_Atom_Event header;
        uint8_t        data[256];
    } ev;

    ev.header.time.frames = frame_offset;
    ev.header.body.type   = kCarlaMidiUrid;
    ev.header.body.size   = static_cast<uint32_t>(num_bytes);
    std::memcpy(ev.data, bytes, num_bytes);

    const uint32_t cap = static_cast<uint32_t>(buffer_size - sizeof(LV2_Atom));
    lv2_atom_sequence_append_event(seq, cap, &ev.header);
}

void add_midi_event(LV2_Atom_Sequence* seq, size_t buffer_size,
                    uint32_t frame_offset,
                    std::initializer_list<uint8_t> bytes) {
    add_midi_event(seq, buffer_size, frame_offset, bytes.begin(), bytes.size());
}

// =============================================================================
// Carla fixture — encapsulates the full plugin lifecycle with configurable
// output buffer size.
// =============================================================================

class CarlaFixture {
public:
    CarlaUridMap          urid_map;
    const LV2_Feature*    features[2];
    const LV2_Descriptor* descriptor;
    LV2_Handle            handle;
    CarlaDefaults         defaults;

    static constexpr size_t kMaxBuf = 8192;
    alignas(8) uint8_t input_storage[kMaxBuf];
    alignas(8) uint8_t output_storage[kMaxBuf];
    size_t              input_buf_size;
    size_t              output_buf_size;

    LV2_Atom_Sequence* input_seq;
    LV2_Atom_Sequence* output_seq;

    explicit CarlaFixture(size_t out_buf_size = 2048)
        : features{ &urid_map.feature, nullptr }
        , input_buf_size(kMaxBuf)
        , output_buf_size(out_buf_size)
    {
        descriptor = lv2_descriptor(0);
        REQUIRE(descriptor != nullptr);

        handle = descriptor->instantiate(descriptor, 48000.0, "", features);
        REQUIRE(handle != nullptr);

        std::memset(input_storage,  0, kMaxBuf);
        std::memset(output_storage, 0, kMaxBuf);

        input_seq  = reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
        output_seq = reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

        input_seq->atom.type  = 0;
        input_seq->atom.size  = sizeof(LV2_Atom_Sequence_Body);
        output_seq->atom.type = 0;
        output_seq->atom.size = sizeof(LV2_Atom_Sequence_Body);

        descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
        descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

        defaults.connect_all(handle, descriptor);

        descriptor->activate(handle);
    }

    ~CarlaFixture() {
        if (handle != nullptr) {
            descriptor->cleanup(handle);
        }
    }

    void add_event(uint32_t frame, std::initializer_list<uint8_t> bytes) {
        add_midi_event(input_seq, input_buf_size, frame, bytes);
    }

    void run(uint32_t n_samples = 256) {
        descriptor->run(handle, n_samples);
    }

    std::vector<OutputEvent> output_events() const {
        return read_output_events(output_seq);
    }
};

} // anonymous namespace

// =============================================================================
// Regression tests — each preceded by the mandatory comment block.
// =============================================================================

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Carla exact params with URID 42 produces non-empty output",
          "[carla][regression]") {
    CarlaFixture fx(/*out_buf_size=*/2048);

    fx.add_event(10, {0x90, 60, 100});  // Note On
    fx.add_event(20, {0xB0, 7, 50});    // CC7

    fx.run(256);

    const auto events = fx.output_events();

    // THE CRITICAL CHECK: output must NOT be empty.
    // With Carla's default output_rate_hz = 100, frames_per_tick = 480.
    // A 256-frame block is shorter than one tick, so NO CC7 output is
    // generated. But Note On MUST pass through.
    REQUIRE_FALSE(events.empty());

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
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Carla params with various output buffer capacities produce non-empty output",
          "[carla][regression][buffer-size]") {
    const size_t buf_size = GENERATE(1024, 4096, 8192);

    CarlaFixture fx(/*out_buf_size=*/buf_size);

    // Override output_rate to 1000 for reliable tick generation within
    // the block, so we test both pass-through AND output generation paths.
    fx.defaults.values[MidiSmoothCCPlugin::PORT_OUTPUT_RATE_HZ] = 1000.0f;

    fx.add_event(10, {0x90, 60, 100});
    fx.add_event(20, {0xB0, 7, 50});

    fx.run(1024);

    const auto events = fx.output_events();

    REQUIRE_FALSE(events.empty());

    bool found_note_on = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0x90) {
            found_note_on = true;
            break;
        }
    }
    REQUIRE(found_note_on);

    bool found_cc7 = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            found_cc7 = true;
            break;
        }
    }
    REQUIRE(found_cc7);

    // atom.size must not exceed the buffer capacity. If the plugin writes
    // beyond the buffer (output_capacity_ = 16384 but buffer is smaller),
    // atom.size could exceed the buffer size.
    REQUIRE(fx.output_seq->atom.size <= buf_size);
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Output buffer smaller than hardcoded 16384 capacity does not corrupt output",
          "[carla][regression][small-buffer]") {
    constexpr size_t kSmallBuf = 128;

    CarlaFixture fx(/*out_buf_size=*/kSmallBuf);

    fx.add_event(10, {0x90, 60, 100});

    fx.run(256);

    const auto events = fx.output_events();

    REQUIRE_FALSE(events.empty());

    REQUIRE(fx.output_seq->atom.size <= kSmallBuf);
    REQUIRE(fx.output_seq->atom.size >= sizeof(LV2_Atom_Sequence_Body));

    REQUIRE(events.size() == 1);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Output sequence is properly cleared and initialized by the plugin",
          "[carla][regression][clearing]") {
    CarlaFixture fx(/*out_buf_size=*/2048);

    // Pre-fill the output buffer with GARBAGE to simulate an uninitialized
    // or stale buffer. The plugin MUST clear this before writing.
    std::memset(fx.output_storage, 0xAA, 2048);

    fx.output_seq->atom.type = 0;
    fx.output_seq->atom.size = sizeof(LV2_Atom_Sequence_Body);

    fx.add_event(10, {0x90, 60, 100});

    fx.run(256);

    const auto events = fx.output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 1);
    REQUIRE(events[0].frame_offset == 10u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // atom.size must reflect exactly one event, not garbage. If the plugin
    // didn't call lv2_atom_sequence_clear(), atom.size would be 0xAAAAAAAA.
    const uint32_t expected_size = sizeof(LV2_Atom_Sequence_Body) +
        lv2_atom_pad_size(sizeof(LV2_Atom_Event) + 3);
    REQUIRE(fx.output_seq->atom.size == expected_size);
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Plugin does not crash when output atom.type is unset (host didn't initialize)",
          "[carla][regression][atom-type]") {
    CarlaFixture fx(/*out_buf_size=*/2048);

    // atom.type = 0 (unset). lv2_atom_sequence_clear() does NOT set atom.type.
    fx.output_seq->atom.type = 0;
    fx.output_seq->atom.size = sizeof(LV2_Atom_Sequence_Body);

    fx.add_event(10, {0x90, 60, 100});

    fx.run(256);

    const auto events = fx.output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 1);
    REQUIRE(events[0].bytes[0] == 0x90);
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Plugin uses safe defaults when control ports are not connected (null)",
          "[carla][regression][null-ports]") {
    CarlaUridMap urid_map;
    const LV2_Feature* features[] = { &urid_map.feature, nullptr };

    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);

    LV2_Handle handle = descriptor->instantiate(descriptor, 48000.0, "", features);
    REQUIRE(handle != nullptr);

    alignas(8) uint8_t input_storage[2048];
    alignas(8) uint8_t output_storage[2048];
    std::memset(input_storage,  0, 2048);
    std::memset(output_storage, 0, 2048);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    input_seq->atom.type  = 0;
    input_seq->atom.size  = sizeof(LV2_Atom_Sequence_Body);
    output_seq->atom.type = 0;
    output_seq->atom.size = sizeof(LV2_Atom_Sequence_Body);

    // Connect ONLY MIDI ports — all 154 control ports remain null.
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    descriptor->activate(handle);

    add_midi_event(input_seq, 2048, 10, {0x90, 60, 100});

    descriptor->run(handle, 256);

    const auto events = read_output_events(output_seq);

    descriptor->cleanup(handle);

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 1);
    REQUIRE(events[0].bytes[0] == 0x90);
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Carla default output_rate_hz=100 generates output with adequate block size",
          "[carla][regression][output-rate]") {
    // Carla's default output_rate_hz is 100 (NOT 1000).
    // At 48000 Hz: frames_per_tick = 48000 / 100 = 480.
    // A 1024-frame block → ticks at 480, 960 → 2 ticks.
    CarlaFixture fx(/*out_buf_size=*/2048);

    fx.add_event(0, {0xB0, 7, 50});  // CC7 at frame 0

    fx.run(1024);

    const auto events = fx.output_events();

    REQUIRE_FALSE(events.empty());

    bool found_cc7 = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            found_cc7 = true;
            REQUIRE(ev.frame_offset == 480u);
            REQUIRE(ev.bytes[2] == 50);
            break;
        }
    }
    REQUIRE(found_cc7);
}

// CRITICAL BUG REGRESSION: Carla Black Hole Issue.
// This test replicates Carla's exact environment to catch the bug.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Full Carla simulation: Note On + CC7 + Pitch Bend with 4096-frame block",
          "[carla][regression][full-sim]") {
    CarlaFixture fx(/*out_buf_size=*/4096);

    fx.add_event(0,    {0x90, 60, 100});    // Note On
    fx.add_event(100,  {0xB0, 7, 50});      // CC7 (breath)
    fx.add_event(2000, {0xE0, 0x00, 0x40}); // Pitch Bend

    fx.run(4096);

    const auto events = fx.output_events();

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 2);

    bool found_note_on = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0x90) {
            REQUIRE(ev.frame_offset == 0u);
            found_note_on = true;
            break;
        }
    }
    REQUIRE(found_note_on);

    bool found_pitch_bend = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xE0) {
            REQUIRE(ev.frame_offset == 2000u);
            found_pitch_bend = true;
            break;
        }
    }
    REQUIRE(found_pitch_bend);

    bool found_cc7 = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            found_cc7 = true;
            break;
        }
    }
    REQUIRE(found_cc7);

    REQUIRE(fx.output_seq->atom.size <= 4096);
    REQUIRE(fx.output_seq->atom.size > sizeof(LV2_Atom_Sequence_Body));
}
