// midi-smooth-cc/src/parameters.hpp

// =============================================================================
// MIDI Smooth CC - User-configurable plugin parameters
//
// Declares the PluginParameters struct mirroring the control ports listed in
// Section 12 of the specification. All members are POD or std::array of POD;
// the struct is trivially copyable. The plugin reads LV2 control port floats
// at the top of run() and stores the resulting values into a PluginParameters
// instance, which is then consulted throughout the real-time processing path.
//
// Default values follow Section 12:
//   enabled           = true
//   smoothing_mode    = Exponential
//   attack_ms         = 50
//   release_ms        = 100
//   rising_rate       = 500   (CC/s)
//   falling_rate      = 500   (CC/s)
//   deadband          = 0
//   output_rate_hz    = 100
//   all_cc            = false
//   channel_mask      = 0xFFFF  (all 16 channels enabled)
//   cc_selection      = CC7 (index 7) only
//
// No dynamic memory allocation: the only container is the fixed-size
// std::array<bool, 128> for CC selection.
// =============================================================================
#pragma once

#include <array>
#include <cstdint>

#include "smoothing.hpp"

namespace midi_smooth_cc {

// Returns the default CC selection array: CC7 (index 7) enabled, all others
// disabled, matching Section 12 of the specification. Used as the default
// value for PluginParameters::cc_selection.
//
// Marked inline (not constexpr) because in C++17 std::array's copy
// constructor is not constexpr, so a constexpr function returning std::array
// by value would not compile. The function is called once per
// PluginParameters default construction, which is a non-real-time path.
inline std::array<bool, 128> default_cc_selection() {
    std::array<bool, 128> sel{};   // value-initialize all entries to false
    sel[7] = true;                 // CC7 enabled by default
    return sel;
}

struct PluginParameters {
    // Master enable / bypass. When false, every MIDI event passes through
    // unchanged and no smoothing is applied (Section 14 bypass semantics).
    bool enabled = true;

    // Active smoothing algorithm. The non-active smoother's state is still
    // preserved per FilterState so toggling modes does not lose context.
    SmoothingMode smoothing_mode = SmoothingMode::Exponential;

    // Exponential filter time constants (Section 6.1). Range 1..1000 ms.
    uint32_t attack_ms  = 50;
    uint32_t release_ms = 100;

    // Slew rate limiter caps (Section 6.2). Range 1..10000 CC units/sec.
    uint32_t rising_rate  = 500;
    uint32_t falling_rate = 500;

    // Minimum |new_target - current_filtered| required to accept a target
    // update (Section 7). Range 0..10 CC units; 0 disables deadband.
    uint32_t deadband = 0;

    // Rate at which intermediate CC events are emitted on the output port
    // (Section 8). Range 20..500 Hz.
    uint32_t output_rate_hz = 100;

    // When true, all CC numbers are processed unless explicitly disabled in
    // cc_selection (cc_selection acts as a blacklist). When false, only the
    // CC numbers selected in cc_selection are processed (whitelist).
    bool all_cc = false;

    // Bit i (i in 0..15) selects MIDI channel i+1. Default: all 16 channels.
    uint16_t channel_mask = 0xFFFF;

    // Per-CC-number enable flag. Index 0..127 corresponds to CC number.
    // Default: only CC7 (index 7) is selected, per Section 12.
    std::array<bool, 128> cc_selection = default_cc_selection();
};

} // namespace midi_smooth_cc
