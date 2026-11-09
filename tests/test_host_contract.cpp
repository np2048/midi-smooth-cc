// midi-smooth-cc/tests/test_host_contract.cpp

//
// Host-contract regression tests: what a spec-compliant LV2 host (Carla)
// does that the earlier tests did not replicate.
//   1. Output buffer arrives typed atom:Chunk; plugin must emit atom:Sequence.
//   2. Host sets output atom.size to the buffer CAPACITY before run(); the
//      plugin must never write beyond it (no sequenceSize feature given).
//   3. Hosts disconnect ports with connect_port(nullptr).
//
// Updated for Option B: the former PORT_CHANNEL_MASK (single integer) is
// replaced by PORT_ALL_CHANNELS + PORT_CHANNEL_SEL_BASE..+15. The Host
// fixture now sets all_channels=0.0 and all 16 channel selectors=1.0,
// preserving "all channels enabled" behavior.
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <vector>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"

#include "plugin.hpp"

using midi_smooth_cc::MidiSmoothCCPlugin;

namespace {

constexpr LV2_URID kMidi  = 11;
constexpr LV2_URID kSeq   = 12;
constexpr LV2_URID kChunk = 13;

LV2_URID map_fn(LV2_URID_Map_Handle, const char* uri) {
    if (!std::strcmp(uri, LV2_MIDI__MidiEvent)) return kMidi;
    if (!std::strcmp(uri, LV2_ATOM__Sequence))  return kSeq;
    if (!std::strcmp(uri, LV2_ATOM__Chunk))     return kChunk;
    return 0;
}

struct Host {
    LV2_URID_Map          map;
    LV2_Feature           feature;
    const LV2_Feature*    features[2];
    const LV2_Descriptor* d = nullptr;
    LV2_Handle            h = nullptr;
    float                 ctl[MidiSmoothCCPlugin::PORT_TOTAL];
    alignas(8) uint8_t    in_buf[2048];
    LV2_Atom_Sequence*    in;

    Host() {
        map.handle = nullptr;
        map.map    = map_fn;
        feature.URI  = LV2_URID__map;
        feature.data = &map;
        features[0] = &feature;
        features[1] = nullptr;

        std::memset(ctl, 0, sizeof(ctl));
        ctl[MidiSmoothCCPlugin::PORT_ENABLED]         = 1.0f;
        ctl[MidiSmoothCCPlugin::PORT_ATTACK_MS]       = 50.0f;
        ctl[MidiSmoothCCPlugin::PORT_RELEASE_MS]      = 100.0f;
        ctl[MidiSmoothCCPlugin::PORT_RISING_RATE]     = 500.0f;
        ctl[MidiSmoothCCPlugin::PORT_FALLING_RATE]    = 500.0f;
        ctl[MidiSmoothCCPlugin::PORT_OUTPUT_RATE_HZ]  = 100.0f;
        ctl[MidiSmoothCCPlugin::PORT_ALL_CC]          = 0.0f;
        ctl[MidiSmoothCCPlugin::PORT_ALL_CHANNELS]    = 0.0f;
        // All 16 channel selectors = 1.0 (all channels enabled).
        for (uint32_t i = 0; i < MidiSmoothCCPlugin::PORT_CHANNEL_SEL_COUNT; ++i) {
            ctl[MidiSmoothCCPlugin::PORT_CHANNEL_SEL_BASE + i] = 1.0f;
        }
        ctl[MidiSmoothCCPlugin::PORT_CC_SEL_BASE + 7] = 1.0f;

        d = lv2_descriptor(0);
        REQUIRE(d != nullptr);
        h = d->instantiate(d, 48000.0, "", features);
        REQUIRE(h != nullptr);

        std::memset(in_buf, 0, sizeof(in_buf));
        in = reinterpret_cast<LV2_Atom_Sequence*>(in_buf);
        in->atom.type = kSeq;
        in->atom.size = sizeof(LV2_Atom_Sequence_Body);

        d->connect_port(h, MidiSmoothCCPlugin::PORT_MIDI_IN, in);
        for (uint32_t i = 2; i < MidiSmoothCCPlugin::PORT_TOTAL; ++i) {
            d->connect_port(h, i, &ctl[i]);
        }
    }
    ~Host() { if (h) d->cleanup(h); }

    void add_event(uint32_t frame, std::initializer_list<uint8_t> bytes) {
        struct { LV2_Atom_Event ev; uint8_t data[8]; } e;
        e.ev.time.frames = frame;
        e.ev.body.type   = kMidi;
        e.ev.body.size   = static_cast<uint32_t>(bytes.size());
        std::memcpy(e.data, bytes.begin(), bytes.size());
        REQUIRE(lv2_atom_sequence_append_event(
                    in, sizeof(in_buf) - sizeof(LV2_Atom), &e.ev) != nullptr);
    }
};

size_t count_events(const LV2_Atom_Sequence* seq) {
    size_t n = 0;
    LV2_Atom_Event* ev  = lv2_atom_sequence_begin(&seq->body);
    LV2_Atom_Event* end = lv2_atom_sequence_end(&seq->body, seq->atom.size);
    while (ev != end) { ++n; ev = lv2_atom_sequence_next(ev); }
    return n;
}

} // namespace

// Test 1: the plugin must stamp the output as atom:Sequence.
TEST_CASE("Output buffer handed over as atom:Chunk is returned as atom:Sequence",
          "[host-contract][regression]") {
    const float enabled = GENERATE(1.0f, 0.0f);   // normal and bypass paths
    Host host;
    host.ctl[MidiSmoothCCPlugin::PORT_ENABLED] = enabled;

    alignas(8) uint8_t out_buf[512];
    std::memset(out_buf, 0xAA, sizeof(out_buf));
    auto* out = reinterpret_cast<LV2_Atom_Sequence*>(out_buf);
    out->atom.type = kChunk;
    out->atom.size = sizeof(out_buf) - sizeof(LV2_Atom);   // host: capacity
    host.d->connect_port(host.h, MidiSmoothCCPlugin::PORT_MIDI_OUT, out);
    host.d->activate(host.h);

    host.add_event(10, {0x90, 60, 100});
    host.d->run(host.h, 256);

    REQUIRE(out->atom.type == kSeq);
    REQUIRE(out->body.unit == 0u);
    REQUIRE(out->body.pad  == 0u);
    REQUIRE(count_events(out) == 1);
}

// Test 2: never write past the capacity the host announced via atom.size.
TEST_CASE("Plugin honours host-announced output capacity (atom.size on entry)",
          "[host-contract][regression][capacity]") {
    Host host;

    constexpr uint32_t kCapacity = 64;   // body capacity announced by the host

    struct Guarded {
        alignas(8) uint8_t buf[sizeof(LV2_Atom) + kCapacity];
        uint8_t            canary[64];
    } g;
    std::memset(g.buf, 0, sizeof(g.buf));
    std::memset(g.canary, 0xCD, sizeof(g.canary));

    auto* out = reinterpret_cast<LV2_Atom_Sequence*>(g.buf);
    out->atom.type = kChunk;
    out->atom.size = kCapacity;              // what a spec-compliant host does
    host.d->connect_port(host.h, MidiSmoothCCPlugin::PORT_MIDI_OUT, out);
    host.d->activate(host.h);

    for (uint32_t i = 0; i < 10; ++i) {      // 10 events >> capacity
        host.add_event(i, {0x90, static_cast<uint8_t>(40 + i), 100});
    }
    host.d->run(host.h, 256);

    for (uint8_t b : g.canary) REQUIRE(b == 0xCD);   // no overrun
    REQUIRE(out->atom.size <= kCapacity);

    // Derive how many 3-byte MIDI events fit from the real struct sizes
    // (LV2_Atom_Event is 16 bytes on 64-bit, so one padded event is 24 bytes).
    const uint32_t event_size = lv2_atom_pad_size(
        static_cast<uint32_t>(sizeof(LV2_Atom_Event)) + 3u);
    const uint32_t expected =
        (kCapacity - static_cast<uint32_t>(sizeof(LV2_Atom_Sequence_Body))) / event_size;
    REQUIRE(count_events(out) == expected);   // (64 - 8) / 24 = 2 on 64-bit
}

// Test 3: connect_port(nullptr) must disconnect, not leave a dangling pointer.
TEST_CASE("Disconnecting the MIDI output port with nullptr stops all writes",
          "[host-contract][regression][disconnect]") {
    Host host;

    alignas(8) uint8_t out_buf[256];
    std::memset(out_buf, 0, sizeof(out_buf));
    auto* out = reinterpret_cast<LV2_Atom_Sequence*>(out_buf);
    out->atom.type = kSeq;
    out->atom.size = sizeof(LV2_Atom_Sequence_Body);
    host.d->connect_port(host.h, MidiSmoothCCPlugin::PORT_MIDI_OUT, out);
    host.d->activate(host.h);

    host.add_event(5, {0x90, 60, 100});
    host.d->run(host.h, 256);
    REQUIRE(count_events(out) == 1);

    std::memset(out_buf, 0xEE, sizeof(out_buf));
    std::vector<uint8_t> snapshot(out_buf, out_buf + sizeof(out_buf));

    host.d->connect_port(host.h, MidiSmoothCCPlugin::PORT_MIDI_OUT, nullptr);
    host.d->run(host.h, 256);   // must not crash, must not touch old buffer

    REQUIRE(std::memcmp(out_buf, snapshot.data(), sizeof(out_buf)) == 0);
}
