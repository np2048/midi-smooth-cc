// midi-smooth-cc/tests/test_plugin_integration.cpp

// =============================================================================
// MIDI Smooth CC - Plugin wrapper integration tests
//
// These tests exercise the FULL LV2 plugin lifecycle through the extern "C"
// interface (lv2_descriptor -> instantiate -> connect_port -> activate ->
// run -> cleanup), as well as direct C++ construction of MidiSmoothCCPlugin.
//
// This file also contains regression tests for output buffer capacity
// handling. The plugin constructor hardcodes
// midi_processor_.set_output_capacity(16384u), ignoring the actual buffer
// size allocated by the host. If the host provides a smaller buffer, the
// plugin will instruct lv2_atom_sequence_append_event that it has 16384
// bytes of capacity, leading to buffer overflows when the plugin writes
// beyond the host's actual allocation.
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
// =============================================================================

static LV2_URID mock_urid_map(LV2_URID_Map_Handle handle, const char* uri) {
    (void)handle;
    if (std::strcmp(uri, LV2_MIDI__MidiEvent) == 0) {
        return 1;
    }
    return 0;
}

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

constexpr LV2_URID kMockMidiUrid = 1;

// =============================================================================
// Atom sequence buffer helpers
// =============================================================================

constexpr size_t kAtomBufferSize = 2048;

void init_atom_sequence(LV2_Atom_Sequence* seq) {
    seq->atom.type = 0;
    seq->atom.size = sizeof(LV2_Atom_Sequence_Body);
}

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

// =============================================================================
// Control port value array
// =============================================================================

struct ControlPortValues {
    float values[MidiSmoothCCPlugin::PORT_TOTAL];

    ControlPortValues() {
        std::memset(values, 0, sizeof(values));

        values[MidiSmoothCCPlugin::PORT_ENABLED]        = 1.0f;
        values[MidiSmoothCCPlugin::PORT_SMOOTHING_MODE] = 0.0f;
        values[MidiSmoothCCPlugin::PORT_ATTACK_MS]      = 50.0f;
        values[MidiSmoothCCPlugin::PORT_RELEASE_MS]     = 100.0f;
        values[MidiSmoothCCPlugin::PORT_RISING_RATE]    = 500.0f;
        values[MidiSmoothCCPlugin::PORT_FALLING_RATE]   = 500.0f;
        values[MidiSmoothCCPlugin::PORT_DEADBAND]       = 0.0f;
        values[MidiSmoothCCPlugin::PORT_OUTPUT_RATE_HZ] = 1000.0f;
        values[MidiSmoothCCPlugin::PORT_ALL_CC]         = 0.0f;
        values[MidiSmoothCCPlugin::PORT_CHANNEL_MASK]   = 65535.0f;
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

} // anonymous namespace

// =============================================================================
// Existing integration tests
// =============================================================================

TEST_CASE("Plugin lifecycle via LV2 descriptor produces non-empty output",
          "[plugin][integration][regression]") {
    MockUridMap urid_map_wrapper;
    const LV2_Feature* features[] = { &urid_map_wrapper.feature, nullptr };

    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);
    REQUIRE(std::strcmp(descriptor->URI, "urn:midi-smooth-cc:plugin") == 0);

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

    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.connect_all(handle, descriptor);

    descriptor->activate(handle);

    add_midi_event(input_seq, kAtomBufferSize, 10, {0x90, 60, 100});
    add_midi_event(input_seq, kAtomBufferSize, 20, {0xB0, 7, 50});

    descriptor->run(handle, 256);

    const auto events = read_output_events(output_seq);

    descriptor->cleanup(handle);

    REQUIRE_FALSE(events.empty());
    REQUIRE(events.size() >= 1);

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
            found_cc7_output = true;
            break;
        }
    }
    REQUIRE(found_cc7_output);
}

TEST_CASE("Direct C++ construction of MidiSmoothCCPlugin produces non-empty output",
          "[plugin][integration][regression][direct]") {
    MockUridMap urid_map_wrapper;

    MidiSmoothCCPlugin* plugin = new MidiSmoothCCPlugin(48000.0, &urid_map_wrapper.map);
    REQUIRE(plugin != nullptr);

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

    plugin->connect_port(MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    plugin->connect_port(MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.connect_all_direct(plugin);

    plugin->activate();

    add_midi_event(input_seq, kAtomBufferSize, 10, {0x90, 60, 100});
    add_midi_event(input_seq, kAtomBufferSize, 20, {0xB0, 7, 50});

    plugin->run(256);

    const auto events = read_output_events(output_seq);

    delete plugin;

    REQUIRE_FALSE(events.empty());

    bool found_note_on = false;
    for (const auto& ev : events) {
        if (ev.bytes.size() == 3 && ev.bytes[0] == 0x90) {
            REQUIRE(ev.frame_offset == 10u);
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
}

TEST_CASE("Plugin bypass mode (enabled=false) passes all events through unchanged",
          "[plugin][integration][regression][bypass]") {
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

    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.values[MidiSmoothCCPlugin::PORT_ENABLED] = 0.0f;
    ctrl.connect_all(handle, descriptor);

    descriptor->activate(handle);

    add_midi_event(input_seq, kAtomBufferSize, 5,  {0x90, 60, 100});
    add_midi_event(input_seq, kAtomBufferSize, 10, {0xB0, 7, 80});
    add_midi_event(input_seq, kAtomBufferSize, 15, {0xE0, 0x00, 0x40});

    descriptor->run(handle, 256);

    const auto events = read_output_events(output_seq);

    descriptor->cleanup(handle);

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
}

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

    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.connect_all(handle, descriptor);

    descriptor->activate(handle);

    descriptor->run(handle, 256);

    const auto events = read_output_events(output_seq);

    descriptor->cleanup(handle);

    REQUIRE(events.empty());
    REQUIRE(output_seq->atom.size == sizeof(LV2_Atom_Sequence_Body));
}

// =============================================================================
// Output buffer capacity regression tests
//
// These tests verify that the plugin never writes beyond the actual output
// buffer allocated by the host. The plugin constructor hardcodes:
//     midi_processor_.set_output_capacity(16384u);
// This value is passed to lv2_atom_sequence_append_event as the capacity
// parameter, regardless of the actual buffer size connected by the host.
// If the host provides a smaller buffer, the plugin will write beyond the
// buffer boundary, causing memory corruption or crashes.
//
// The tests below allocate deliberately small stack buffers and verify that
// atom.size (the total body size written by the plugin) does not exceed
// the buffer's physical capacity.
// =============================================================================

TEST_CASE("Plugin output does not exceed actual host-allocated buffer capacity",
          "[plugin][regression][capacity]") {
    MockUridMap urid_map_wrapper;
    const LV2_Feature* features[] = { &urid_map_wrapper.feature, nullptr };

    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);

    LV2_Handle handle = descriptor->instantiate(
        descriptor, 48000.0, "", features);
    REQUIRE(handle != nullptr);

    // Allocate a 48-byte output buffer. With sizeof(LV2_Atom) = 8 and
    // sizeof(LV2_Atom_Sequence_Body) typically 8, the body area is 40 bytes.
    // Two padded MIDI events (16 bytes each) use 32 bytes, leaving 8 bytes
    // of headroom. This buffer can safely hold exactly two events.
    //
    // The plugin hardcodes output_capacity_ = 16384, so it believes it has
    // 16384 bytes of body capacity. If the plugin writes more data than the
    // 48-byte buffer can hold, atom.size will exceed 48 (or more precisely,
    // sizeof(LV2_Atom) + atom.size will exceed 48), proving the overflow.
    constexpr size_t kOutputBufSize = 48;
    alignas(8) uint8_t input_storage[kAtomBufferSize];
    alignas(8) uint8_t output_storage[kOutputBufSize];
    std::memset(input_storage,  0, sizeof(input_storage));
    std::memset(output_storage, 0, kOutputBufSize);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    init_atom_sequence(input_seq);
    init_atom_sequence(output_seq);

    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.connect_all(handle, descriptor);

    descriptor->activate(handle);

    // Inject TWO passthrough Note On events. Both are non-CC events that
    // must be copied to the output unchanged. Each event occupies 16 bytes
    // (sizeof(LV2_Atom_Event) + 3 = 15, padded to 16) in the output body.
    add_midi_event(input_seq, kAtomBufferSize, 10, {0x90, 60, 100});
    add_midi_event(input_seq, kAtomBufferSize, 20, {0x90, 64, 80});

    descriptor->run(handle, 256);

    // Capture atom.size BEFORE cleanup (the buffer is stack-local and
    // will be destroyed when the test case exits).
    const uint32_t output_atom_size = output_seq->atom.size;

    descriptor->cleanup(handle);

    // ---------------------------------------------------------------
    // Assertion 1: atom.size must not exceed the buffer's total size.
    // The plugin must respect the ACTUAL buffer size, not its hardcoded
    // 16384 capacity.
    // ---------------------------------------------------------------
    REQUIRE(output_atom_size <= kOutputBufSize);

    // ---------------------------------------------------------------
    // Assertion 2: the total buffer usage (LV2_Atom header + body) must
    // not exceed the buffer. This is the precise overflow check:
    // sizeof(LV2_Atom) + atom.size <= buffer_size.
    // If the plugin writes beyond the buffer, this sum exceeds the
    // buffer size, proving a buffer overflow.
    // ---------------------------------------------------------------
    REQUIRE(static_cast<size_t>(sizeof(LV2_Atom)) + output_atom_size <= kOutputBufSize);
}

TEST_CASE("Plugin gracefully handles output buffer too small for even one event",
          "[plugin][regression][capacity][tiny]") {
    MockUridMap urid_map_wrapper;
    const LV2_Feature* features[] = { &urid_map_wrapper.feature, nullptr };

    const LV2_Descriptor* descriptor = lv2_descriptor(0);
    REQUIRE(descriptor != nullptr);

    LV2_Handle handle = descriptor->instantiate(
        descriptor, 48000.0, "", features);
    REQUIRE(handle != nullptr);

    // Allocate a 16-byte output buffer. With sizeof(LV2_Atom) = 8 and
    // sizeof(LV2_Atom_Sequence_Body) typically 8, the body area is 8 bytes.
    // A single padded MIDI event requires 16 bytes, which exceeds the
    // available body area. There is not enough room for even one event.
    //
    // The plugin hardcodes output_capacity_ = 16384, so it will attempt to
    // write the event despite the buffer being too small. The capacity
    // check in lv2_atom_sequence_append_event (16384 - sizeof_body >= 16)
    // passes, and the write proceeds -- but the write goes beyond the
    // 16-byte buffer boundary, corrupting adjacent stack memory.
    //
    // Under AddressSanitizer, this triggers a stack-buffer-overflow error.
    // Without ASAN, the behavior is undefined: the test may crash, or it
    // may reach the assertions below with a corrupted atom.size.
    constexpr size_t kOutputBufSize = 16;
    alignas(8) uint8_t input_storage[kAtomBufferSize];
    alignas(8) uint8_t output_storage[kOutputBufSize];
    std::memset(input_storage,  0, sizeof(input_storage));
    std::memset(output_storage, 0, kOutputBufSize);

    LV2_Atom_Sequence* input_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(input_storage);
    LV2_Atom_Sequence* output_seq =
        reinterpret_cast<LV2_Atom_Sequence*>(output_storage);

    init_atom_sequence(input_seq);
    init_atom_sequence(output_seq);

    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_IN,  input_seq);
    descriptor->connect_port(handle, MidiSmoothCCPlugin::PORT_MIDI_OUT, output_seq);

    ControlPortValues ctrl;
    ctrl.connect_all(handle, descriptor);

    descriptor->activate(handle);

    // Inject ONE Note On event. This single event (16 bytes padded) does
    // not fit in the 16-byte buffer's body area (only 8 bytes available
    // after the LV2_Atom header and body header).
    add_midi_event(input_seq, kAtomBufferSize, 10, {0x90, 60, 100});

    // Run the plugin. If the plugin writes beyond the buffer, this may
    // crash (stack corruption) or silently corrupt memory. Reaching the
    // assertions below without crashing means the plugin did not trigger
    // a fatal signal -- but it does NOT mean the write was safe.
    descriptor->run(handle, 256);

    // Capture atom.size. If the plugin wrote an event, atom.size will be
    // sizeof(LV2_Atom_Sequence_Body) + 16 (one padded event), which
    // exceeds the buffer's body area.
    const uint32_t output_atom_size = output_seq->atom.size;

    descriptor->cleanup(handle);

    // ---------------------------------------------------------------
    // Assertion 1: The plugin must not crash. Reaching this point means
    // no fatal signal was raised. (This is an implicit assertion -- if
    // the plugin crashes, the test fails before reaching this line.)
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // Assertion 2: atom.size must not exceed the buffer's total size.
    // If the plugin wrote an event despite the buffer being too small,
    // atom.size will be sizeof(LV2_Atom_Sequence_Body) + 16, which
    // exceeds 16 bytes. This proves the plugin wrote beyond the buffer.
    //
    // With the current buggy implementation (output_capacity_ = 16384):
    //   - lv2_atom_sequence_clear sets atom.size = sizeof(LV2_Atom_Sequence_Body)
    //   - append_event checks: 16384 - sizeof_body >= 16. This passes.
    //   - The event is written at body offset 0, extending 16 bytes into
    //     the body area. But the body area is only 8 bytes (16 - 8 = 8).
    //     The write extends 8 bytes beyond the buffer.
    //   - atom.size becomes sizeof_body + 16 = 24 (or similar).
    //   - 24 > 16: ASSERTION FAILS.
    // ---------------------------------------------------------------
    REQUIRE(output_atom_size <= kOutputBufSize);

    // ---------------------------------------------------------------
    // Assertion 3: The total buffer usage must not exceed the buffer.
    // sizeof(LV2_Atom) + atom.size <= 16.
    // With the buggy implementation: 8 + 24 = 32 > 16. FAILS.
    // ---------------------------------------------------------------
    REQUIRE(static_cast<size_t>(sizeof(LV2_Atom)) + output_atom_size <= kOutputBufSize);
}
