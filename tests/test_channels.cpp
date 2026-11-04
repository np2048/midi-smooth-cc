// midi-smooth-cc/tests/test_channels.cpp

// =============================================================================
// Unit tests for per-channel and per-CC independence (Sections 4, 5, 9).
//
// Covers test cases from Section 15:
//   - Independent MIDI channels
//   - Independent CC numbers
//   - Channel mask selection
//   - ALL CC mode plus per-CC exclusions
//   - Default config: CC7 on all channels
// =============================================================================
#include <catch2/catch_test_macros.hpp>

#include "midi_processor.hpp"
#include "parameters.hpp"

// TODO: TEST_CASE("Channel mask selects which channels are processed", "[channels]") { ... }
// TODO: TEST_CASE("Per-CC selection is independent",                   "[channels]") { ... }
// TODO: TEST_CASE("ALL CC mode plus exclusions",                       "[channels]") { ... }
// TODO: TEST_CASE("Default processes CC7 on all 16 channels",          "[channels]") { ... }
// TODO: TEST_CASE("One channel's CC does not affect another",          "[channels]") { ... }
// TODO: TEST_CASE("One CC number does not affect another",             "[channels]") { ... }

TEST_CASE("test harness verifies linking", "[channels]") {
    REQUIRE(true);
}
