// midi-smooth-cc/src/smoothing.cpp

// =============================================================================
// MIDI Smooth CC - Smoothing algorithm implementations
//
// Implements the methods declared in smoothing.hpp. The math and the
// real-time constraints are documented in that header.
//
// All methods are leaf functions: no allocation, no locks, no I/O. State is
// two doubles per instance. The output is clamped to [0.0, 127.0] before
// being stored so that a sustained out-of-range target cannot push the
// internal state out of the MIDI range.
// =============================================================================
#include "smoothing.hpp"

#include <algorithm>
#include <cmath>

namespace midi_smooth_cc {

// -----------------------------------------------------------------------------
// ExponentialSmoother
// -----------------------------------------------------------------------------

ExponentialSmoother::ExponentialSmoother(double initial_value)
    : current_value_(initial_value)
    , target_value_(initial_value)
{
}

void ExponentialSmoother::reset(double initial_value) {
    current_value_ = initial_value;
    target_value_  = initial_value;
}

double ExponentialSmoother::process(double target,
                                    double dt_seconds,
                                    double tau_attack_seconds,
                                    double tau_release_seconds) {
    target_value_ = target;

    const double delta = target - current_value_;

    // Select the time constant based on the sign of the delta. When delta is
    // exactly zero the choice is harmless because alpha multiplies zero.
    const double tau = (delta > 0.0) ? tau_attack_seconds : tau_release_seconds;

    double new_value;
    if (tau <= 0.0) {
        // Zero or negative time constant: jump straight to target. This also
        // avoids 0/0 NaN if dt_seconds == 0 and tau == 0.
        new_value = target;
    } else {
        // alpha = 1 - exp(-dt / tau)
        // dt == 0 naturally yields alpha == 0, leaving current_value unchanged.
        const double alpha = 1.0 - std::exp(-dt_seconds / tau);
        new_value = current_value_ + alpha * delta;
    }

    // Clamp before storing to prevent out-of-range state accumulation.
    new_value = std::clamp(new_value, 0.0, 127.0);
    current_value_ = new_value;
    return current_value_;
}

double ExponentialSmoother::current_value() const noexcept {
    return current_value_;
}

double ExponentialSmoother::target_value() const noexcept {
    return target_value_;
}

// -----------------------------------------------------------------------------
// SlewRateLimiter
// -----------------------------------------------------------------------------

SlewRateLimiter::SlewRateLimiter(double initial_value)
    : current_value_(initial_value)
    , target_value_(initial_value)
{
}

void SlewRateLimiter::reset(double initial_value) {
    current_value_ = initial_value;
    target_value_  = initial_value;
}

double SlewRateLimiter::process(double target,
                                double dt_seconds,
                                double rising_rate_cc_per_sec,
                                double falling_rate_cc_per_sec) {
    target_value_ = target;

    const double delta = target - current_value_;

    if (delta > 0.0) {
        // Rising: cap upward movement at rising_rate * dt.
        const double max_delta = rising_rate_cc_per_sec * dt_seconds;
        const double step = std::min(delta, max_delta);
        current_value_ += step;
    } else if (delta < 0.0) {
        // Falling: cap downward movement at falling_rate * dt.
        const double max_delta = falling_rate_cc_per_sec * dt_seconds;
        const double step = std::min(-delta, max_delta);
        current_value_ -= step;
    }
    // delta == 0.0: no movement.

    // Clamp before returning to guarantee the MIDI range invariant.
    current_value_ = std::clamp(current_value_, 0.0, 127.0);
    return current_value_;
}

double SlewRateLimiter::current_value() const noexcept {
    return current_value_;
}

double SlewRateLimiter::target_value() const noexcept {
    return target_value_;
}

} // namespace midi_smooth_cc
