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
//
// COMPUTE / COMMIT PATTERN (added for overflow-safe output generation):
//
//   The compute() method calculates what the new filtered value WOULD be
//   for a given (target, dt, params) triple, WITHOUT mutating current_value_
//   or target_value_. This allows the caller to "preview" the result and
//   only commit it if the downstream event append succeeds.
//
//   The commit() method applies a previously computed value by setting
//   current_value_ (and target_value_ if a new target is provided). This
//   is called ONLY after the event has been successfully written to the
//   output buffer, guaranteeing that the filter state never advances past
//   a failed emission.
//
//   The existing process() method is retained for backward compatibility
//   and convenience. It calls compute() then commit() internally.
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
    explicit ExponentialSmoother(double initial_value = 0.0);
    void reset(double initial_value);

    // Advance the filter by one step (compute + commit).
    double process(double target,
                   double dt_seconds,
                   double tau_attack_seconds,
                   double tau_release_seconds);

    // Compute the new filtered value WITHOUT mutating internal state.
    // This is used for overflow-safe output generation: the caller computes
    // the value, attempts to append it to the output buffer, and only calls
    // commit() if the append succeeds.
    double compute(double target,
                   double dt_seconds,
                   double tau_attack_seconds,
                   double tau_release_seconds) const;

    // Commit a previously computed value. Updates target_value_ and
    // current_value_. Called ONLY after successful event append.
    void commit(double target, double new_value);

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
    explicit SlewRateLimiter(double initial_value = 0.0);
    void reset(double initial_value);

    // Advance the filter by one step (compute + commit).
    double process(double target,
                   double dt_seconds,
                   double rising_rate_cc_per_sec,
                   double falling_rate_cc_per_sec);

    // Compute the new filtered value WITHOUT mutating internal state.
    double compute(double target,
                   double dt_seconds,
                   double rising_rate_cc_per_sec,
                   double falling_rate_cc_per_sec) const;

    // Commit a previously computed value. Updates target_value_ and
    // current_value_. Called ONLY after successful event append.
    void commit(double target, double new_value);

    double current_value() const noexcept;
    double target_value()  const noexcept;

private:
    double current_value_;
    double target_value_;
};

} // namespace midi_smooth_cc
