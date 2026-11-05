// midi-smooth-cc/tests/test_smoothing.cpp

// =============================================================================
// Unit tests for the smoothing algorithms (Section 6 of the specification).
//
// These tests exercise the public API declared in smoothing.hpp:
//   - ExponentialSmoother: first-order time-aware low-pass filter
//   - SlewRateLimiter:     rate-of-change limiter
//
// The Catch2 main() entry point lives in tests/main.cpp so that additional
// test_*.cpp files can be added to the same test executable without causing
// duplicate-main linker errors.
//
// The tests verify:
//   - Exponential smoothing math: alpha = 1 - exp(-dt / tau)
//   - Independent attack and release time constants
//   - Slew rate limiter caps the rate of change at rising_rate * dt and
//     falling_rate * dt
//   - Independent rising and falling rates for the slew limiter
//   - Edge cases: dt = 0, extreme target jumps, convergence over time,
//     very large dt, target equal to current value
//   - Output never exceeds the MIDI CC range [0, 127]
//   - Section 14 reset semantics: initial state is taken from the incoming
//     value, with no ramp from zero
//
// API contract exercised by these tests (declared in smoothing.hpp):
//
//   class ExponentialSmoother {
//     public:
//       explicit ExponentialSmoother(double initial_value = 0.0);
//       void   reset(double initial_value);
//       double process(double target,
//                      double dt_seconds,
//                      double tau_attack_seconds,
//                      double tau_release_seconds);
//       double current_value() const noexcept;
//       double target_value()  const noexcept;
//   };
//
//   class SlewRateLimiter {
//     public:
//       explicit SlewRateLimiter(double initial_value = 0.0);
//       void   reset(double initial_value);
//       double process(double target,
//                      double dt_seconds,
//                      double rising_rate_cc_per_sec,
//                      double falling_rate_cc_per_sec);
//       double current_value() const noexcept;
//       double target_value()  const noexcept;
//   };
//
// Both process() methods advance the internal filter state by one step and
// return the new filtered value, clamped to [0, 127]. They are leaf
// functions: no allocation, no locks, no I/O.
// =============================================================================
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "smoothing.hpp"

#include <cmath>

using midi_smooth_cc::ExponentialSmoother;
using midi_smooth_cc::SlewRateLimiter;

// =============================================================================
// Helpers
// =============================================================================

// Absolute tolerance for floating-point comparisons. The math involves exp()
// and small time intervals; a tight absolute tolerance is sufficient because
// the values being compared are bounded by the MIDI range [0, 127].
constexpr double kAbsTol = 1e-6;

// =============================================================================
// Exponential Smoothing - math verification (Section 6.1)
// =============================================================================

TEST_CASE("ExponentialSmoother computes alpha = 1 - exp(-dt/tau) on a rising target",
          "[smoothing][exponential]") {
    // y_prev = 0, target = 100, dt = 0.1 s, tau = 0.1 s.
    // alpha  = 1 - exp(-1)              = 0.6321205588...
    // y      = 0 + alpha * (100 - 0)    = 63.21205588...
    ExponentialSmoother s{0.0};
    const double target  = 100.0;
    const double dt      = 0.1;
    const double tau_atk = 0.1;
    const double tau_rel = 0.1;
    const double alpha   = 1.0 - std::exp(-dt / tau_atk);
    const double expected = 0.0 + alpha * (target - 0.0);

    const double result = s.process(target, dt, tau_atk, tau_rel);

    REQUIRE_THAT(result,            Catch::Matchers::WithinAbs(expected, kAbsTol));
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(expected, kAbsTol));
    REQUIRE_THAT(s.target_value(),  Catch::Matchers::WithinAbs(target,   kAbsTol));
}

TEST_CASE("ExponentialSmoother uses tau_attack when target rises above current value",
          "[smoothing][exponential]") {
    // y_prev = 50, target = 100 (rising). tau_attack is tiny so the filter
    // should move almost the full delta. tau_release is huge; if the
    // implementation incorrectly used tau_release for a rising target the
    // result would barely move, so this case discriminates the two paths.
    ExponentialSmoother s{50.0};

    const double target  = 100.0;
    const double dt      = 0.05;
    const double tau_atk = 0.01;
    const double tau_rel = 10.0;

    const double alpha_atk = 1.0 - std::exp(-dt / tau_atk);
    const double expected  = 50.0 + alpha_atk * (target - 50.0);

    const double result = s.process(target, dt, tau_atk, tau_rel);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(expected, kAbsTol));
}

TEST_CASE("ExponentialSmoother uses tau_release when target falls below current value",
          "[smoothing][exponential]") {
    // y_prev = 100, target = 0 (falling). tau_release is tiny so the filter
    // should move almost the full delta. tau_attack is huge; if the
    // implementation incorrectly used tau_attack for a falling target the
    // result would barely move.
    ExponentialSmoother s{100.0};

    const double target  = 0.0;
    const double dt      = 0.05;
    const double tau_atk = 10.0;
    const double tau_rel = 0.01;

    const double alpha_rel = 1.0 - std::exp(-dt / tau_rel);
    const double expected  = 100.0 + alpha_rel * (target - 100.0);

    const double result = s.process(target, dt, tau_atk, tau_rel);

    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(expected, kAbsTol));
}

TEST_CASE("ExponentialSmoother attack is faster than release for the same dt and jump",
          "[smoothing][exponential]") {
    // Identical setup except tau_attack < tau_release. The rising case must
    // move further toward the target than the falling case for one step.
    const double y_prev  = 50.0;
    const double dt      = 0.02;
    const double tau_atk = 0.01;
    const double tau_rel = 1.0;

    ExponentialSmoother rising {y_prev};
    ExponentialSmoother falling{y_prev};

    const double r_up   = rising.process(100.0, dt, tau_atk, tau_rel);
    const double r_down = falling.process(0.0,   dt, tau_atk, tau_rel);

    REQUIRE(r_up - y_prev > y_prev - r_down);
}

// =============================================================================
// Exponential Smoothing - edge cases
// =============================================================================

TEST_CASE("ExponentialSmoother with dt = 0 leaves the filtered value unchanged",
          "[smoothing][exponential][edge]") {
    // alpha = 1 - exp(-0/tau) = 1 - 1 = 0  =>  y = y_prev
    ExponentialSmoother s{42.0};
    const double result = s.process(127.0, 0.0, 0.05, 0.05);
    REQUIRE_THAT(result,            Catch::Matchers::WithinAbs(42.0, kAbsTol));
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(42.0, kAbsTol));
    // Target is still recorded even though the filtered value did not move.
    REQUIRE_THAT(s.target_value(),  Catch::Matchers::WithinAbs(127.0, kAbsTol));
}

TEST_CASE("ExponentialSmoother with very large dt reaches the target",
          "[smoothing][exponential][edge]") {
    // dt >> tau  =>  alpha approaches 1  =>  y approaches target
    ExponentialSmoother s{0.0};
    const double result = s.process(100.0, 100.0, 0.05, 0.05);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(100.0, 1e-3));
}

TEST_CASE("ExponentialSmoother with target equal to current value stays put",
          "[smoothing][exponential][edge]") {
    ExponentialSmoother s{50.0};
    const double result = s.process(50.0, 0.1, 0.05, 0.05);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(50.0, kAbsTol));
}

TEST_CASE("ExponentialSmoother converges to a constant target over repeated steps",
          "[smoothing][exponential][convergence]") {
    // Repeatedly applying the same target must converge asymptotically to it.
    ExponentialSmoother s{0.0};
    const double target = 100.0;
    const double dt     = 0.01;
    const double tau    = 0.05;

    for (int i = 0; i < 10000; ++i) {
        s.process(target, dt, tau, tau);
    }
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(target, 1e-3));
}

TEST_CASE("ExponentialSmoother handles an extreme target jump without leaving [0,127]",
          "[smoothing][exponential][edge]") {
    // From 0 to 127 in one step with a small dt and tau. The single-step
    // response is bounded by alpha < 1, and the returned value must also be
    // clamped to the MIDI range so a caller cannot accidentally drive it out.
    ExponentialSmoother s{0.0};
    const double result = s.process(127.0, 0.001, 0.001, 0.001);
    REQUIRE(result >= 0.0);
    REQUIRE(result <= 127.0);
}

TEST_CASE("ExponentialSmoother never reports a filtered value outside [0,127]",
          "[smoothing][exponential][range]") {
    // Drive the filter with a target above the MIDI range and verify the
    // returned value never exceeds 127 across many iterations.
    ExponentialSmoother s{127.0};
    for (int i = 0; i < 100; ++i) {
        const double r = s.process(200.0, 0.01, 0.01, 0.01);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 127.0);
    }
}

// =============================================================================
// Slew Rate Limiter - math verification (Section 6.2)
// =============================================================================

TEST_CASE("SlewRateLimiter caps rising rate to rising_rate * dt",
          "[smoothing][slew]") {
    // y_prev = 50, target = 100 (delta = +50). rising_rate = 100 CC/s,
    // dt = 0.1 s  =>  max_delta = 10. Output must be 50 + 10 = 60.
    SlewRateLimiter s{50.0};
    const double result = s.process(100.0, 0.1, /*rising*/100.0, /*falling*/100.0);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(60.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter caps falling rate to falling_rate * dt",
          "[smoothing][slew]") {
    // y_prev = 100, target = 0 (delta = -100). falling_rate = 50 CC/s,
    // dt = 0.1 s  =>  max_delta = 5. Output must be 100 - 5 = 95.
    SlewRateLimiter s{100.0};
    const double result = s.process(0.0, 0.1, /*rising*/100.0, /*falling*/50.0);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(95.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter reaches the target exactly when within the rate cap",
          "[smoothing][slew]") {
    // y_prev = 50, target = 55 (delta = +5). rising_rate = 100 CC/s,
    // dt = 0.1 s  =>  max_delta = 10. |delta| < max_delta, so output = 55.
    SlewRateLimiter s{50.0};
    const double result = s.process(55.0, 0.1, /*rising*/100.0, /*falling*/100.0);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(55.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter rising and falling rates are independent",
          "[smoothing][slew]") {
    // Same magnitude of jump up vs. down, but rising_rate >> falling_rate.
    // The upward move must be larger than the downward move for one step.
    SlewRateLimiter up  {50.0};
    SlewRateLimiter down{50.0};
    const double dt = 0.1;
    const double r_up   = up.process(100.0, dt, /*rising*/1000.0, /*falling*/1.0);
    const double r_down = down.process(0.0,   dt, /*rising*/1000.0, /*falling*/1.0);
    REQUIRE(r_up - 50.0 > 50.0 - r_down);
}

// =============================================================================
// Slew Rate Limiter - edge cases
// =============================================================================

TEST_CASE("SlewRateLimiter with dt = 0 leaves the filtered value unchanged",
          "[smoothing][slew][edge]") {
    // max_delta = rate * 0 = 0  =>  y = y_prev
    SlewRateLimiter s{42.0};
    const double result = s.process(127.0, 0.0, /*rising*/1000.0, /*falling*/1000.0);
    REQUIRE_THAT(result,            Catch::Matchers::WithinAbs(42.0, kAbsTol));
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(42.0, kAbsTol));
    REQUIRE_THAT(s.target_value(),  Catch::Matchers::WithinAbs(127.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter with very large dt reaches the target in one step",
          "[smoothing][slew][edge]") {
    // dt huge  =>  max_delta huge  =>  no effective rate limit  =>  y = target
    SlewRateLimiter s{0.0};
    const double result = s.process(100.0, 100.0, 100.0, 100.0);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(100.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter with target equal to current value stays put",
          "[smoothing][slew][edge]") {
    SlewRateLimiter s{50.0};
    const double result = s.process(50.0, 0.1, 100.0, 100.0);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(50.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter handles an extreme target jump without exceeding the rate",
          "[smoothing][slew][edge]") {
    // y_prev = 0, target = 127 (delta = +127). rising_rate = 100 CC/s,
    // dt = 0.001 s  =>  max_delta = 0.1. Output must be 0 + 0.1 = 0.1.
    SlewRateLimiter s{0.0};
    const double result = s.process(127.0, 0.001, /*rising*/100.0, /*falling*/100.0);
    REQUIRE_THAT(result, Catch::Matchers::WithinAbs(0.1, kAbsTol));
    REQUIRE(result >= 0.0);
    REQUIRE(result <= 127.0);
}

TEST_CASE("SlewRateLimiter converges to the target over repeated steps",
          "[smoothing][slew][convergence]") {
    // With a constant target and repeated small steps, the limiter reaches
    // the target in ceil(|delta| / (rate * dt)) steps and then holds there.
    SlewRateLimiter s{0.0};
    const double target = 100.0;
    const double dt     = 0.01;
    const double rate   = 1000.0;   // 10 CC per step

    for (int i = 0; i < 1000; ++i) {
        s.process(target, dt, rate, rate);
    }
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(target, kAbsTol));
}

TEST_CASE("SlewRateLimiter never reports a filtered value outside [0,127]",
          "[smoothing][slew][range]") {
    // Even with a target far above 127 and very high rates, the returned
    // value stays clamped to the MIDI range.
    SlewRateLimiter s{127.0};
    for (int i = 0; i < 100; ++i) {
        const double r = s.process(200.0, 0.01, /*rising*/10000.0, /*falling*/10000.0);
        REQUIRE(r >= 0.0);
        REQUIRE(r <= 127.0);
    }
}

// =============================================================================
// Reset behavior (Section 14)
// =============================================================================

TEST_CASE("ExponentialSmoother reset reinitializes state without ramping from zero",
          "[smoothing][exponential][reset]") {
    // When a selected CC is first received the filter state must be
    // initialized from the incoming value, NOT ramped from zero.
    ExponentialSmoother s{0.0};
    s.reset(64.0);
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(64.0, kAbsTol));
    // A subsequent dt = 0 step must hold the reset value.
    const double r = s.process(64.0, 0.0, 0.05, 0.05);
    REQUIRE_THAT(r, Catch::Matchers::WithinAbs(64.0, kAbsTol));
}

TEST_CASE("SlewRateLimiter reset reinitializes state without ramping from zero",
          "[smoothing][slew][reset]") {
    SlewRateLimiter s{0.0};
    s.reset(64.0);
    REQUIRE_THAT(s.current_value(), Catch::Matchers::WithinAbs(64.0, kAbsTol));
    const double r = s.process(64.0, 0.0, 100.0, 100.0);
    REQUIRE_THAT(r, Catch::Matchers::WithinAbs(64.0, kAbsTol));
}
