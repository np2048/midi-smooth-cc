// midi-smooth-cc/src/filter_state.hpp

// =============================================================================
// MIDI Smooth CC - Per (channel, CC) filter state
//
// The plugin maintains independent filter state for every (channel, CC number)
// combination: 16 channels x 128 CC numbers = 2048 states.
//
// Each FilterState holds at minimum (Section 9 of the specification):
//   - current filtered value (double, for arithmetic precision)
//   - current target value (double)
//   - last transmitted integer value (0..127)
//   - last update timestamp (in frames since plugin instantiation)
//   - filter initialization flag (whether the first event has arrived)
//
// Real-time safety: this struct MUST be plain POD-like data, allocated once at
// instantiate() time and reused across every run() call. No std:: containers.
// =============================================================================
#pragma once

#include <cstdint>

namespace midi_smooth_cc {

// TODO: struct FilterState { ... };
// Total fixed-size array: FilterState states_[16][128];  // 2048 entries

} // namespace midi_smooth_cc
