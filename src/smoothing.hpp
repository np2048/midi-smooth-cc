// midi-smooth-cc/src/smoothing.hpp

// =============================================================================
// MIDI Smooth CC - Smoothing algorithms
//
// Implements the two smoothing modes required by Section 6 of the spec:
//
//   6.1 Exponential Smoothing (first-order time-aware low-pass)
//       alpha = 1 - exp(-dt / tau)
//       y    = y_prev + alpha * (x - y_prev)
//       Attack (rising target) and Release (falling target) use independent
//       tau values, derived from the Attack/Release control ports.
//
//   6.2 Slew Rate Limiter
//       Limits the maximum rate of change in CC units per second, with
//       independent rising/falling rates.
//
// Both algorithms operate in CC-units-as-double internally and clamp the
// output to [0, 127] before quantization.
//
// Real-time safety: all methods must be leaf functions: no allocation, no
// locks, no I/O. The dt parameter is the elapsed time in seconds (or frames,
// converted by the caller) since the last update for THIS filter state.
// =============================================================================
#pragma once

#include <cstdint>

namespace midi_smooth_cc {

// TODO: enum class SmoothingMode { Exponential, Slew };
// TODO: class ExponentialSmoother { ... };
// TODO: class SlewRateLimiter    { ... };

} // namespace midi_smooth_cc
