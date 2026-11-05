// midi-smooth-cc/src/filter_state.hpp

// =============================================================================
// MIDI Smooth CC - Per (channel, CC) filter state
//
// Declares the FilterState struct that holds the smoothing state for a single
// (channel, CC number) pair. The plugin allocates a single fixed-size array
// of 16 x 128 = 2048 FilterState entries once in instantiate() and reuses it
// across every run() call; no per-run allocation occurs.
//
// Per Section 9 each FilterState holds:
//   - Current filtered value and current target value: held inside the two
//     smoother instances (ExponentialSmoother and SlewRateLimiter each store
//     two doubles).
//   - Last transmitted integer value (0..127), or -1 to force the next
//     output tick to emit (Section 8: "send a MIDI CC event only if the
//     integer value has changed").
//   - Last update timestamp in audio frames since plugin instantiation,
//     used to compute dt for the time-aware smoothing math (Section 6.1).
//   - Filter initialization flag: false until the first CC event arrives
//     for this (channel, CC) pair, per Section 14 first-CC semantics.
//
// Both smoothers are held simultaneously; only the one matching the active
// SmoothingMode is advanced on each step. The other's state is preserved so
// that toggling modes does not produce value jumps.
//
// Real-time safety:
//   - No std:: containers, no heap, no virtuals.
//   - Plain data members plus two ExponentialSmoother/SlewRateLimiter
//     objects (which themselves hold only two doubles each).
//   - Default-constructible and copyable so it can be placed in
//     std::array<FilterState, 2048>.
// =============================================================================
#pragma once

#include <cstdint>

#include "smoothing.hpp"

namespace midi_smooth_cc {

struct FilterState {
    // Smoothing instances for both algorithms. The one matching the active
    // SmoothingMode in PluginParameters is advanced each step; the other is
    // kept idle so its state survives a mode switch (Section 14).
    ExponentialSmoother exp_smoother;
    SlewRateLimiter     slew_limiter;

    // Last integer CC value (0..127) transmitted on the output port, or -1
    // to force the next output tick to emit. Section 8: emit only if the
    // integer value has changed since the last transmission.
    int last_transmitted_value = -1;

    // False until the first CC event arrives for this (channel, CC) pair.
    // Section 14: when the first CC arrives, the filter state is initialized
    // from the incoming value rather than ramping from zero.
    bool is_initialized = false;

    // Audio frame count at the last filter update for this state. Used to
    // compute dt = (current_frame - last_update_frame) / sample_rate for
    // the time-aware exponential and slew math. Section 9 lists "last update
    // timestamp" as a required per-state field.
    uint64_t last_update_frame = 0;
};

} // namespace midi_smooth_cc
