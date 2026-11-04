// midi-smooth-cc/src/midi_processor.hpp

// =============================================================================
// MIDI Smooth CC - MIDI event parsing, dispatch, and output generation
//
// Responsibilities:
//   - Parse incoming MIDI bytes from LV2 Atom Sequence MIDI events.
//   - Identify CC messages and route them to the appropriate FilterState.
//   - Pass through all non-selected events (Note On/Off, Pitch Bend, etc.)
//     unchanged and never delayed.
//   - Apply the deadband test (Section 7) before updating a filter target.
//   - Drive the output event generation loop (Section 8): at each configured
//     output interval, sample the filtered value, quantize to 0..127, and
//     emit a CC event ONLY if the integer value has changed since the last
//     transmitted one.
//   - Safely handle output buffer overflow: never corrupt the input stream.
//
// Real-time safety contract:
//   - No dynamic allocation in any method called from run().
//   - No std::mutex / std::lock_guard.
//   - Preallocated output Atom Sequence frame is written via
//     lv2_atom_sequence_append_event() with explicit capacity checks.
//   - Timestamps are relative to the start of the current processing block.
// =============================================================================
#pragma once

#include <cstdint>

namespace midi_smooth_cc {

// TODO: class MidiProcessor { ... };

} // namespace midi_smooth_cc
