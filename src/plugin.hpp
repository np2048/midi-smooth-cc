// midi-smooth-cc/src/plugin.hpp

//
// MIDI Smooth CC - LV2 plugin entry-point class.
//
// Port layout (must match lv2/plugin.ttl.in):
//   0  MIDI in
//   1  MIDI out
//   2  enabled / bypass
//   3  smoothing_mode
//   4  attack_ms
//   5  release_ms
//   6  rising_rate
//   7  falling_rate
//   8  deadband
//   9  output_rate_hz
//   10 all_cc
//   11 all_channels
//   12..27  channel_1 .. channel_16   (16 individual toggle ports)
//   28..155 cc_selection_0 .. cc_selection_127  (128 individual toggle ports)
//
// The previous single channel_mask integer port (old port 11) has been
// replaced by all_channels (port 11) plus 16 per-channel toggle ports
// (ports 12..27). This mirrors the all_cc / cc_selection design and lets
// hosts render each channel as a checkbox rather than requiring the user
// to compute a 16-bit bitmask.
//
// Output capacity handling (host-contract fix):
//   A spec-compliant host sets the output atom.size to the buffer CAPACITY
//   before run(). run() adopts that value (host_capacity_) unless it equals
//   the size we ourselves left behind (stale), falling back to
//   default_capacity_ (buf-size feature or 16384) for hosts that do not
//   pre-set it.
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

    // ---- Port index constants (must match lv2/plugin.ttl.in) ----
    static constexpr uint32_t PORT_MIDI_IN          = 0u;
    static constexpr uint32_t PORT_MIDI_OUT         = 1u;
    static constexpr uint32_t PORT_ENABLED          = 2u;
    static constexpr uint32_t PORT_SMOOTHING_MODE   = 3u;
    static constexpr uint32_t PORT_ATTACK_MS        = 4u;
    static constexpr uint32_t PORT_RELEASE_MS       = 5u;
    static constexpr uint32_t PORT_RISING_RATE      = 6u;
    static constexpr uint32_t PORT_FALLING_RATE     = 7u;
    static constexpr uint32_t PORT_DEADBAND         = 8u;
    static constexpr uint32_t PORT_OUTPUT_RATE_HZ   = 9u;
    static constexpr uint32_t PORT_ALL_CC           = 10u;
    static constexpr uint32_t PORT_ALL_CHANNELS     = 11u;
    static constexpr uint32_t PORT_CHANNEL_SEL_BASE  = 12u;
    static constexpr uint32_t PORT_CHANNEL_SEL_COUNT = 16u;
    static constexpr uint32_t PORT_CC_SEL_BASE      = 28u;
    static constexpr uint32_t PORT_CC_SEL_COUNT     = 128u;
    static constexpr uint32_t PORT_TOTAL            = PORT_CC_SEL_BASE + PORT_CC_SEL_COUNT; // 156

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
    float* port_all_channels_    = nullptr;

    float* port_channel_selection_[PORT_CHANNEL_SEL_COUNT] = {};
    float* port_cc_selection_[PORT_CC_SEL_COUNT] = {};

    uint32_t sample_rate_   = 44100u;
    uint64_t current_frame_ = 0u;

    // Output capacity bookkeeping (see header comment).
    uint32_t default_capacity_   = 16384u;  // fallback when host does not pre-set
    uint32_t host_capacity_      = 0u;      // 0 = not yet learned from host
    uint32_t last_output_size_   = 0u;      // atom.size we left after last run
};

} // namespace midi_smooth_cc

extern "C" const LV2_Descriptor* lv2_descriptor(uint32_t index);
