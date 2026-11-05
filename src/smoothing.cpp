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

double ExponentialSmoother::compute(double target,
                                    double dt_seconds,
                                    double tau_attack_seconds,
                                    double tau_release_seconds) const {
    const double delta = target - current_value_;
    const double tau   = (delta > 0.0) ? tau_attack_seconds : tau_release_seconds;

    double new_value;
    if (tau <= 0.0) {
        new_value = target;
    } else {
        const double alpha = 1.0 - std::exp(-dt_seconds / tau);
        new_value = current_value_ + alpha * delta;
    }

    return std::clamp(new_value, 0.0, 127.0);
}

void ExponentialSmoother::commit(double target, double new_value) {
    target_value_  = target;
    current_value_ = new_value;
}

double ExponentialSmoother::process(double target,
                                    double dt_seconds,
                                    double tau_attack_seconds,
                                    double tau_release_seconds) {
    const double new_value = compute(target, dt_seconds, tau_attack_seconds, tau_release_seconds);
    commit(target, new_value);
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

double SlewRateLimiter::compute(double target,
                                double dt_seconds,
                                double rising_rate_cc_per_sec,
                                double falling_rate_cc_per_sec) const {
    const double delta = target - current_value_;
    double new_value   = current_value_;

    if (delta > 0.0) {
        const double max_delta = rising_rate_cc_per_sec * dt_seconds;
        const double step = std::min(delta, max_delta);
        new_value += step;
    } else if (delta < 0.0) {
        const double max_delta = falling_rate_cc_per_sec * dt_seconds;
        const double step = std::min(-delta, max_delta);
        new_value -= step;
    }

    return std::clamp(new_value, 0.0, 127.0);
}

void SlewRateLimiter::commit(double target, double new_value) {
    target_value_  = target;
    current_value_ = new_value;
}

double SlewRateLimiter::process(double target,
                                double dt_seconds,
                                double rising_rate_cc_per_sec,
                                double falling_rate_cc_per_sec) {
    const double new_value = compute(target, dt_seconds, rising_rate_cc_per_sec, falling_rate_cc_per_sec);
    commit(target, new_value);
    return current_value_;
}

double SlewRateLimiter::current_value() const noexcept {
    return current_value_;
}

double SlewRateLimiter::target_value() const noexcept {
    return target_value_;
}

} // namespace midi_smooth_cc
