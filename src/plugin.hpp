// midi-smooth-cc/src/plugin.hpp

// =============================================================================
// MIDI Smooth CC - LV2 plugin entry-point class
//
// Declares the C++ class that backs the LV2 plugin descriptor. The host
// never instantiates this class directly; instead, the extern "C"
// `lv2_descriptor()` returns a static LV2_Descriptor whose `instantiate`
// callback creates a MidiSmoothCCPlugin via `new` and returns it as an
// LV2_Handle (opaque void*).
//
// Lifecycle (all orchestrated by the host):
//   instantiate()       : new MidiSmoothCCPlugin(...). Non-real-time.
//   connect_port()      : store port connection pointers.
//   activate()          : reset timing state.
//   run()               : THE real-time callback. Top of run() reads all
//                         control port values into a local PluginParameters
//                         snapshot (no mutexes), then calls
//                         MidiProcessor::process().
//   deactivate()        : no-op.
//   cleanup()           : delete the instance.
//   extension_data()    : return nullptr for v1.0.
//
// Port layout (indices MUST match lv2/plugin.ttl.in):
//   Index 0   : MIDI input  (atom:Sequence, input)
//   Index 1   : MIDI output (atom:Sequence, output)
//   Index 2   : enabled
//   Index 3   : smoothing_mode (0=Exp 1=Slew)
//   Index 4-9 : attack_ms, release_ms, rising_rate, falling_rate,
//               deadband, output_rate_hz
//   Index 10  : all_cc
//   Index 11  : channel_mask (16-bit bitmask)
//   Index 12..139 : cc_selection[0..127]
//   Total: 140 ports.
//
// 64-BIT FRAME COUNTER:
//   current_frame_ is uint64_t to support sessions longer than ~24.9 hours
//   at 48 kHz. It is passed directly to MidiProcessor::process() without
//   any narrowing cast. The process() method accepts uint64_t.
//
// The plugin URI string MUST match manifest.ttl.in and plugin.ttl.in:
//     "urn:midi-smooth-cc:plugin"
// =============================================================================
#pragma once

#include <array>
#include <cstdint>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"

#include "midi_processor.hpp"
#include "parameters.hpp"
#include "filter_state.hpp"

namespace midi_smooth_cc {

class MidiSmoothCCPlugin {
public:
    MidiSmoothCCPlugin(double sample_rate,
                       LV2_URID_Map* urid_map,
                       uint32_t output_capacity = 16384u);

    MidiSmoothCCPlugin(const MidiSmoothCCPlugin&)            = delete;
    MidiSmoothCCPlugin& operator=(const MidiSmoothCCPlugin&) = delete;
    MidiSmoothCCPlugin(MidiSmoothCCPlugin&&)                 = delete;
    MidiSmoothCCPlugin& operator=(MidiSmoothCCPlugin&&)      = delete;

    ~MidiSmoothCCPlugin() = default;

    void connect_port(uint32_t port, void* data);
    void activate();
    void run(uint32_t n_samples);
    void deactivate() {}

    static constexpr uint32_t PORT_MIDI_IN         = 0u;
    static constexpr uint32_t PORT_MIDI_OUT        = 1u;
    static constexpr uint32_t PORT_ENABLED         = 2u;
    static constexpr uint32_t PORT_SMOOTHING_MODE  = 3u;
    static constexpr uint32_t PORT_ATTACK_MS       = 4u;
    static constexpr uint32_t PORT_RELEASE_MS      = 5u;
    static constexpr uint32_t PORT_RISING_RATE     = 6u;
    static constexpr uint32_t PORT_FALLING_RATE    = 7u;
    static constexpr uint32_t PORT_DEADBAND        = 8u;
    static constexpr uint32_t PORT_OUTPUT_RATE_HZ  = 9u;
    static constexpr uint32_t PORT_ALL_CC          = 10u;
    static constexpr uint32_t PORT_CHANNEL_MASK    = 11u;
    static constexpr uint32_t PORT_CC_SEL_BASE     = 12u;
    static constexpr uint32_t PORT_CC_SEL_COUNT    = 128u;
    static constexpr uint32_t PORT_TOTAL           = PORT_CC_SEL_BASE + PORT_CC_SEL_COUNT; // 140

private:
    MidiProcessor                 midi_processor_;
    std::array<FilterState, 2048> states_;

    const LV2_Atom_Sequence* port_midi_in_  = nullptr;
    LV2_Atom_Sequence*       port_midi_out_ = nullptr;

    float* port_enabled_         = nullptr;
    float* port_smoothing_mode_  = nullptr;
    float* port_attack_ms_       = nullptr;
    float* port_release_ms_      = nullptr;
    float* port_rising_rate_     = nullptr;
    float* port_falling_rate_    = nullptr;
    float* port_deadband_        = nullptr;
    float* port_output_rate_hz_  = nullptr;
    float* port_all_cc_          = nullptr;
    float* port_channel_mask_    = nullptr;

    float* port_cc_selection_[PORT_CC_SEL_COUNT] = {};

    uint32_t sample_rate_   = 44100u;
    // current_frame_ is uint64_t to prevent wraparound at UINT32_MAX
    // (~24.9 hours at 48 kHz). It is passed directly to process() without
    // any narrowing cast.
    uint64_t current_frame_ = 0u;
};

} // namespace midi_smooth_cc

extern "C" const LV2_Descriptor* lv2_descriptor(uint32_t index);
