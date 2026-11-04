// midi-smooth-cc/tests/test_midi.cpp

// =============================================================================
// Unit tests for MIDI event parsing, pass-through, and output generation.
//
// Covers test cases from Section 15:
//   - Note On / Note Off preservation (never delayed)
//   - Pitch Bend preservation
//   - Unselected CC pass-through
//   - Output buffer overflow safety
//   - Bypass behavior
//   - Filter initialization on first CC event
//   - Variable processing block sizes
// =============================================================================
#include <catch2/catch_test_macros.hpp>

#include "midi_processor.hpp"

// TODO: TEST_CASE("Note On/Off pass through unchanged",     "[midi]") { ... }
// TODO: TEST_CASE("Pitch Bend passes through unchanged",    "[midi]") { ... }
// TODO: TEST_CASE("Unselected CC passes through unchanged", "[midi]") { ... }
// TODO: TEST_CASE("Output buffer overflow is safe",         "[midi]") { ... }
// TODO: TEST_CASE("Bypass forwards all events unchanged",   "[midi]") { ... }
// TODO: TEST_CASE("First CC initializes filter state",      "[midi]") { ... }
// TODO: TEST_CASE("Handles variable block sizes",           "[midi]") { ... }
// TODO: TEST_CASE("Output rate is respected",               "[midi]") { ... }

TEST_CASE("test harness verifies linking", "[midi]") {
    REQUIRE(true);
}
