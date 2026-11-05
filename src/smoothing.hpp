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
// output to [0, 127] before storing and before returning. Clamping before
// storage prevents out-of-bounds state from accumulating across calls when
// a target outside the MIDI range is sustained.
//
// Real-time safety: all methods are leaf functions: no allocation, no locks,
// no I/O. State is two plain doubles per instance. The dt parameter is the
// elapsed time in seconds since the last update for THIS filter state.
// =============================================================================
#pragma once

#include <cstdint>

namespace midi_smooth_cc {

// Smoothing algorithm selector exposed to the user via the Smoothing Mode
// control port (Section 12 of the specification).
enum class SmoothingMode {
    Exponential,  // First-order time-aware low-pass (Section 6.1)
    Slew          // Slew rate limiter (Section 6.2)
};

// First-order time-aware exponential low-pass filter with independent attack
// and release time constants.
class ExponentialSmoother {
public:
    // Construct with an initial filtered value. Default is 0.0.
    explicit ExponentialSmoother(double initial_value = 0.0);

    // Reinitialize both current and target state to initial_value with no
    // interpolation. Used by Section 14 first-CC semantics.
    void reset(double initial_value);

    // Advance the filter by one step.
    //   target              : new target CC value (may be outside [0,127]).
    //   dt_seconds          : elapsed time since the last call, in seconds.
    //   tau_attack_seconds  : time constant used when target > current.
    //   tau_release_seconds : time constant used when target <= current.
    // Returns the new filtered value, clamped to [0.0, 127.0].
    double process(double target,
                   double dt_seconds,
                   double tau_attack_seconds,
                   double tau_release_seconds);

    double current_value() const noexcept;
    double target_value()  const noexcept;

private:
    double current_value_;
    double target_value_;
};

// Slew rate limiter. Caps the maximum rate of change in CC units per second
// with independent rising and falling rates.
class SlewRateLimiter {
public:
    // Construct with an initial filtered value. Default is 0.0.
    explicit SlewRateLimiter(double initial_value = 0.0);

    // Reinitialize both current and target state to initial_value with no
    // interpolation. Used by Section 14 first-CC semantics.
    void reset(double initial_value);

    // Advance the filter by one step.
    //   target                  : new target CC value (may be outside [0,127]).
    //   dt_seconds              : elapsed time since the last call, in seconds.
    //   rising_rate_cc_per_sec  : max upward rate, CC units per second.
    //   falling_rate_cc_per_sec : max downward rate, CC units per second.
    // Returns the new filtered value, clamped to [0.0, 127.0].
    double process(double target,
                   double dt_seconds,
                   double rising_rate_cc_per_sec,
                   double falling_rate_cc_per_sec);

    double current_value() const noexcept;
    double target_value()  const noexcept;

private:
    double current_value_;
    double target_value_;
};

} // namespace midi_smooth_cc
