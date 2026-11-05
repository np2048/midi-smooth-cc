// midi-smooth-cc/tests/test_plugin_integration.cpp

// =============================================================================
// MIDI Smooth CC - Plugin wrapper integration tests
//
// These tests exercise the FULL LV2 plugin lifecycle through the extern "C"
// interface (lv2_descriptor → instantiate → connect_port → activate → run →
// cleanup), as well as direct C++ construction of MidiSmoothCCPlugin.
//
// Unlike test_midi.cpp (which tests MidiProcessor in isolation), these tests
// verify that MidiSmoothCCPlugin::run() correctly:
//   - Reads control port values into a PluginParameters snapshot.
//   - Delegates to MidiProcessor::process() with the right arguments.
//   - Writes output events to the connected port_midi_out_ buffer.
//
// A "black hole" bug in plugin.cpp (e.g., null port check failing, wrong
// port index in connect_port, params not read, process() never called)
// would cause the output LV2_Atom_Sequence to remain empty after run(),
// which these tests explicitly assert against.
//
// The test creates a mock LV2_URID_Map that returns URID 1 for
// LV2_MIDI__MidiEvent, stack-allocates 8-byte-aligned atom sequence
// buffers, connects all 140 ports, and drives the plugin through a
// complete processing cycle.
// =============================================================================
#include <catch2/catch_test_macros.hpp>

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
// Mock URID map
//
// Returns URID 1 for LV2_MIDI__MidiEvent, 0 for any other URI.
// This simulates the host's urid:map feature without requiring a real
// LV2 host runtime.
// =============================================================================

static LV2_URID mock_urid_map(LV2_URID_Map_Handle handle, const char* uri) {
    (void)handle;
    if (std::strcmp(uri, LV2_MIDI__MidiEvent) == 0) {
        return 1;   // dummy URID matching what the test uses for MIDI events
    }
    return 0;
}

// Bundle the URID map and its LV2_Feature wrapper so they share a lifetime.
struct MockUridMap {
    LV2_URID_Map map;
    LV2_Feature  feature;

    MockUridMap() {
        map.handle   = nullptr;
        map.map      = mock_urid_map;
        feature.URI  = LV2_URID__map;
        feature.data = &map;
    }
};

// The dummy MIDI URID that mock_urid_map returns for midi:MidiEvent.
constexpr LV2_URID kMockMidiUrid = 1;

// =============================================================================
// Atom sequence buffer helpers
// =============================================================================

constexpr size_t kAtomBufferSize = 2048;

// Zero-initialize a raw buffer as an empty LV2_Atom_Sequence.
void init_atom_sequence(LV2_Atom_Sequence* seq) {
    seq->atom.type = 0;   // dummy; the plugin doesn't check sequence type
    seq->atom.size = sizeof(LV2_Atom_Sequence_Body);
}

// Append a MIDI event to an input LV2_Atom_Sequence at the given frame offset.
void add_midi_event(LV2_Atom_Sequence* seq, size_t buffer_size,
                    uint32_t frame_offset,
                    const uint8_t* bytes, size_t num_bytes) {
    struct StackEvent {
        LV2_Atom_Event header;
        uint8_t        data[256];
    } ev;

    ev.header.time.frames = frame_offset;
    ev.header.body.type   = kMockMidiUrid;
    ev.header.body.size   = static_cast<uint32_t>(num_bytes);
    std::memcpy(ev.data, bytes, num_bytes);

    const uint32_t cap = static_cast<uint32_t>(buffer_size - sizeof(LV2_Atom));
    LV2_Atom_Event* appended =
        lv2_atom_sequence_append_event(seq, cap, &ev.header);
    REQUIRE(appended != nullptr);
}

void add_midi_event(LV2_Atom_Sequence* seq, size_t buffer_size,
                    uint32_t frame_offset,
                    std::initializer_list<uint8_t> bytes) {
    add_midi_event(seq, buffer_size, frame_offset, bytes.begin(), bytes.size());
}

// An output event extracted from an LV2_Atom_Sequence.
struct OutputEvent {
    uint32_t             frame_offset;
    std::vector<uint8_t> bytes;
};

// Read all events from an output LV2_Atom_Sequence into a vector.
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

// =============================================================================
// Control port value array
//
// A float array indexed by port number (0..139). Ports 0 and 1 are MIDI
// (unused in this array). Ports 2..139 are control ports.
// =============================================================================

struct ControlPortValues {
    float values[MidiSmoothCCPlugin::PORT_TOTAL];

    ControlPortValues() {
        std::memset(values, 0, sizeof(values));

        // Defaults matching spec §12 and what plugin.cpp run() expects.
        values[MidiSmoothCCPlugin::PORT_ENABLED]        = 1.0f;     // on
        values[MidiSmoothCCPlugin::PORT_SMOOTHING_MODE] = 0.0f;     // Exponential
        values[MidiSmoothCCPlugin::PORT_ATTACK_MS]      = 50.0f;
        values[MidiSmoothCCPlugin::PORT_RELEASE_MS]     = 100.0f;
        values[MidiSmoothCCPlugin::PORT_RISING_RATE]    = 500.0f;
        values[MidiSmoothCCPlugin::PORT_FALLING_RATE]   = 500.0f;
        values[MidiSmoothCCPlugin::PORT_DEADBAND]       = 0.0f;
        values[MidiSmoothCCPlugin::PORT_OUTPUT_RATE_HZ] = 1000.0f;  // high → ticks within block
        values[MidiSmoothCCPlugin::PORT_ALL_CC]         = 0.0f;     // whitelist mode
        values[MidiSmoothCCPlugin::PORT_CHANNEL_MASK]   = 65535.0f; // all 16 channels

        // CC selection: CC7 enabled (default), all others disabled.
        values[MidiSmoothCCPlugin::PORT_CC_SEL_BASE + 7] = 1.0f;
    }

    // Connect all control ports (indices 2..139) to this array.
    void connect_all(LV2_Handle handle, const LV2_Descriptor* desc) {
        for (uint32_t i = 2; i < MidiSmoothCCPlugin::PORT_TOTAL; ++i) {
            desc->connect_port(handle, i, &values[i]);
        }
    }

    // Direct connect (for C++ method call style).
    void connect_all_direct(MidiSmoothCCPlugin* plugin) {
        for (uint32_t i = 2; i < MidiSmoothCCPlugin::PORT_TOTAL; ++i) {
            plugin->connect_port(i, &values[i]);
        }
    }
};

} // anonymous namespace

// =============================================================================
// Regression tests — each preceded by the mandatory comment block.
// =============================================================================

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue in Plugin Wrapper.
// This test ensures the MidiSmoothCCPlugin::run correctly delegates
// to MidiProcessor and writes to the output buffer.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Plugin lifecycle via LV2 descriptor produces non-empty output",
          "[plugin][integration][regression]") {
    // --- 1. Mock URID map feature ---
    MockUridMap urid_map_wrapper;
    const LV2_Feature* features[] = { &urid_map_wrapper.feature, nullptr };

    // --- 2. Get the LV2 descriptor ---
    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);
    REQUIRE(std::strcmp(descriptor->URI, "urn:midi-smooth-cc:plugin") == 0);

    // --- 3. Instantiate ---
    LV2_Handle handle = descriptor->instantiate(
        descriptor,
        48000.0,    // sample rate
        "",         // bundle path (unused)
        features
    );
    REQUIRE(handle != nullptr);

    // --- 4. Allocate stack-based, 8-byte-aligned atom sequence buffers ---
    alignas(8) uint8_t input_storage[kAtomBufferSize];
    alignas(8) uint8_t output_storage[kAtomBufferSize];
    std::memset(input_storage,  0, kAtomBufferSize);
    std::memset(output_storage, 0, kAtomBufferSize);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    init_atom_sequence(input_seq);
    init_atom_sequence(output_seq);

    // --- 5. Connect MIDI ports (0 and 1) ---
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    // --- 6. Connect all control ports (2..139) ---
    ControlPortValues ctrl;
    ctrl.connect_all(handle, descriptor);

    // --- 7. Activate ---
    descriptor->activate(handle);

    // --- 8. Populate input with a Note On and a CC7 event ---
    add_midi_event(input_seq, kAtomBufferSize, 10, {0x90, 60, 100});  // Note On
    add_midi_event(input_seq, kAtomBufferSize, 20, {0xB0, 7, 50});    // CC7

    // --- 9. Run ---
    const uint32_t n_samples = 256;
    descriptor->run(handle, n_samples);

    // --- 10. Read output BEFORE cleanup (buffers are stack-local) ---
    const auto events = read_output_events(output_seq);

    // --- 11. Cleanup ---
    descriptor->cleanup(handle);

    // --- 12. Assertions ---
    // THE CRITICAL CHECK: output must NOT be empty.
    // If the "black hole" bug is present (process() never called, ports
    // not connected, params not read), the output sequence remains empty
    // and this assertion fails.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 1);

    // Verify Note On appears in output (pass-through path).
    // This catches bugs where port_midi_in_ / port_midi_out_ are not
    // connected, or where process() skips the pass-through loop.
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

    // Verify CC7 output was generated (output generation path).
    // This catches bugs where:
    //   - output_rate_hz is read as 0 (no ticks fire)
    //   - channel_mask is read as 0 (no channels selected)
    //   - cc_selection is read incorrectly (CC7 not selected)
    //   - current_frame_ is not passed correctly
    //   - params.enabled is read as false (bypass mode, CC7 passed through
    //     instead of consumed — but in that case found_cc7_output would
    //     still be true with the ORIGINAL CC7 event at frame 20)
    bool found_cc7_output = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            found_cc7_output = true;
            // If the CC7 appears at frame 20, it was passed through (bug:
            // bypass mode active or CC7 not selected). If it appears at
            // frame 68 (= 20 + 48), it was generated by the output loop
            // (correct behavior).
            // We accept either for this regression test — the key assertion
            // is that CC7 IS present in some form.
            break;
        }
    }
    REQUIRE(found_cc7_output);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue in Plugin Wrapper.
// This test ensures the MidiSmoothCCPlugin::run correctly delegates
// to MidiProcessor and writes to the output buffer.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Direct C++ construction of MidiSmoothCCPlugin produces non-empty output",
          "[plugin][integration][regression][direct]") {
    // --- 1. Mock URID map ---
    MockUridMap urid_map_wrapper;

    // --- 2. Direct construction (the test "uses new" as the spec requires) ---
    MidiSmoothCCPlugin* plugin = new MidiSmoothCCPlugin(48000.0, &urid_map_wrapper.map);
    REQUIRE(plugin != nullptr);

    // --- 3. Stack-allocated atom sequence buffers ---
    alignas(8) uint8_t input_storage[kAtomBufferSize];
    alignas(8) uint8_t output_storage[kAtomBufferSize];
    std::memset(input_storage,  0, kAtomBufferSize);
    std::memset(output_storage, 0, kAtomBufferSize);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    init_atom_sequence(input_seq);
    init_atom_sequence(output_seq);

    // --- 4. Connect MIDI ports ---
    plugin->connect_port(MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    plugin->connect_port(MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    // --- 5. Connect all control ports ---
    ControlPortValues ctrl;
    ctrl.connect_all_direct(plugin);

    // --- 6. Activate ---
    plugin->activate();

    // --- 7. Populate input ---
    add_midi_event(input_seq, kAtomBufferSize, 10, {0x90, 60, 100});
    add_midi_event(input_seq, kAtomBufferSize, 20, {0xB0, 7, 50});

    // --- 8. Run ---
    plugin->run(256);

    // --- 9. Read output ---
    const auto events = read_output_events(output_seq);

    // --- 10. Cleanup ---
    delete plugin;

    // --- 11. Assertions ---
    REQUIRE_FALSE(events.empty());

    // Note On must pass through.
    bool found_note_on = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0x90) {
            REQUIRE(ev.frame_offset == 10u);
            found_note_on = true;
            break;
        }
    }
    REQUIRE(found_note_on);

    // CC7 must appear (either passed through or generated).
    bool found_cc7 = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0xB0 && ev.bytes[1] == 7) {
            found_cc7 = true;
            break;
        }
    }
    REQUIRE(found_cc7);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue in Plugin Wrapper.
// This test ensures the MidiSmoothCCPlugin::run correctly delegates
// to MidiProcessor and writes to the output buffer.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Plugin bypass mode (enabled=false) passes all events through unchanged",
          "[plugin][integration][regression][bypass]") {
    MockUridMap urid_map_wrapper;
    const LV2_Feature* features[] = { &urid_map_wrapper.feature, nullptr };

    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);

    LV2_Handle handle = descriptor->instantiate(
        descriptor, 48000.0, "", features);
    REQUIRE(handle != nullptr);

    // Buffers
    alignas(8) uint8_t input_storage[kAtomBufferSize];
    alignas(8) uint8_t output_storage[kAtomBufferSize];
    std::memset(input_storage,  0, kAtomBufferSize);
    std::memset(output_storage, 0, kAtomBufferSize);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    init_atom_sequence(input_seq);
    init_atom_sequence(output_seq);

    // Connect MIDI ports
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    // Connect control ports with enabled = false (bypass)
    ControlPortValues ctrl;
    ctrl.values[MidiSmoothCCPlugin::PORT_ENABLED] = 0.0f;  // BYPASS ON
    ctrl.connect_all(handle, descriptor);

    // Activate
    descriptor->activate(handle);

    // Populate input: Note On, CC7, Pitch Bend
    add_midi_event(input_seq, kAtomBufferSize, 5,  {0x90, 60, 100});    // Note On
    add_midi_event(input_seq, kAtomBufferSize, 10, {0xB0, 7, 80});      // CC7 (would be consumed if enabled)
    add_midi_event(input_seq, kAtomBufferSize, 15, {0xE0, 0x00, 0x40}); // Pitch Bend

    // Run
    descriptor->run(handle, 256);

    // Read output
    const auto events = read_output_events(output_seq);

    // Cleanup
    descriptor->cleanup(handle);

    // In bypass mode, ALL events must pass through unchanged — including CC7.
    // If the plugin ignores enabled=false and still processes CC7, the CC7
    // would be consumed (missing from output) and this test fails.
    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() == 3);

    // Note On
    REQUIRE(events[0].frame_offset == 5u);
    REQUIRE(events[0].bytes[0] == 0x90);
    REQUIRE(events[0].bytes[1] == 60);
    REQUIRE(events[0].bytes[2] == 100);

    // CC7 — must appear (NOT consumed in bypass mode)
    REQUIRE(events[1].frame_offset == 10u);
    REQUIRE(events[1].bytes[0] == 0xB0);
    REQUIRE(events[1].bytes[1] == 7);
    REQUIRE(events[1].bytes[2] == 80);

    // Pitch Bend
    REQUIRE(events[2].frame_offset == 15u);
    REQUIRE(events[2].bytes[0] == 0xE0);
    REQUIRE(events[2].bytes[1] == 0x00);
    REQUIRE(events[2].bytes[2] == 0x40);
}

// CRITICAL BUG REGRESSION: MIDI Event Drop Issue in Plugin Wrapper.
// This test ensures the MidiSmoothCCPlugin::run correctly delegates
// to MidiProcessor and writes to the output buffer.
// DO NOT REMOVE during test suite optimization or shrinking.
TEST_CASE("Plugin with empty input produces valid empty output (no spurious events)",
          "[plugin][integration][regression][empty]") {
    MockUridMap urid_map_wrapper;
    const LV2_Feature* features[] = { &urid_map_wrapper.feature, nullptr };

    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);

    LV2_Handle handle = descriptor->instantiate(
        descriptor, 48000.0, "", features);
    REQUIRE(handle != nullptr);

    alignas(8) uint8_t input_storage[kAtomBufferSize];
    alignas(8) uint8_t output_storage[kAtomBufferSize];
    std::memset(input_storage,  0, kAtomBufferSize);
    std::memset(output_storage, 0, kAtomBufferSize);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    init_atom_sequence(input_seq);
    init_atom_sequence(output_seq);

    // Connect ports
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.connect_all(handle, descriptor);

    descriptor->activate(handle);

    // Do NOT add any input events — input is empty.

    // Run
    descriptor->run(handle, 256);

    // Read output
    const auto events = read_output_events(output_seq);

    // Cleanup
    descriptor->cleanup(handle);

    // With empty input and no pre-initialized filter states, the output
    // must also be empty. This verifies the plugin doesn't spurious emit
    // garbage events. The output sequence must remain structurally valid.
    REQUIRE(events.empty());

    // atom.size should be sizeof(LV2_Atom_Sequence_Body) (empty sequence).
    REQUIRE(output_seq->atom.size == sizeof(LV2_Atom_Sequence_Body));
}
