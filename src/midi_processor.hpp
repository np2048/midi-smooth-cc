// midi-smooth-cc/src/midi_processor.hpp

// =============================================================================
// MIDI Smooth CC - MIDI event parsing, dispatch, and output generation
//
// Responsibilities (spec §8, §9, §10, §11):
//   - Parse incoming MIDI bytes from an LV2 Atom Sequence of MIDI events.
//   - Identify CC messages (status 0xB0 | channel) and route them to the
//     appropriate FilterState entry (index = channel * 128 + cc_number).
//   - Pass through all non-selected events unchanged and never delayed:
//     Note On/Off, Poly Aftertouch, Channel Pressure, Pitch Bend, Program
//     Change, MIDI Clock, System Real-Time, System Exclusive (spec §10).
//   - Apply the deadband test (spec §7) before accepting a new target.
//   - Apply first-CC initialization (spec §14) when a previously-unseen
//     (channel, CC) pair receives its first event.
//   - Drive the output generation loop (spec §8): at each tick of
//     params.output_rate_hz, sample the active smoother, quantize to 0..127,
//     and emit a MIDI CC event ONLY if the integer value changed since the
//     last transmission.
//   - INTERLEAVE generated CC events with passthrough events so the output
//     LV2_Atom_Sequence is strictly ordered by frame_offset (non-decreasing),
//     as required by the LV2 specification.
//   - Safely handle output buffer overflow: lv2_atom_sequence_append_event's
//     return value is checked; on overflow the event is gracefully dropped,
//     the input stream is never corrupted, and the sequence remains
//     structurally valid.
//
// Real-time safety contract (spec §11):
//   - No dynamic allocation (no new/delete, no std::vector, no std::string)
//     in any method called from run().
//   - No mutexes, no locks, no blocking operations.
//   - No filesystem access, no logging to stdout/stderr.
//   - The preallocated output Atom Sequence frame is written via
//     lv2_atom_sequence_append_event() with explicit capacity checks.
//   - All MIDI output values are clamped to [0, 127] and cast to uint8_t
//     before emission.
//   - All smoothing math uses double precision.
//   - Timestamps on emitted events are relative to the start of the current
//     processing block (frame offsets in [0, block_length)).
//
// 64-BIT FRAME TIMESTAMP HANDLING:
//   The process() method accepts current_frame as uint64_t to support
//   long-running sessions exceeding ~24.9 hours at 48 kHz (the point at
//   which a 32-bit frame counter wraps). All internal absolute frame
//   position calculations (block_start, block_end, deadline_frame,
//   next_tick, last_update_frame) use uint64_t. Block-relative offsets
//   (ev->time.frames, frame_offset in emitted events) remain uint32_t
//   per the LV2 specification (they are always in [0, block_length)).
//
// Thread safety:
//   midi_urid_ and output_capacity_ are set once in instantiate() (host
//   thread) and read-only in run() (audio thread). No synchronization is
//   required for these members.
// =============================================================================
#pragma once

#include <array>
#include <cstdint>

#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"

#include "filter_state.hpp"
#include "parameters.hpp"
#include "smoothing.hpp"

namespace midi_smooth_cc {

class MidiProcessor {
public:
    MidiProcessor() = default;

    void set_midi_urid(LV2_URID urid) noexcept { midi_urid_ = urid; }

    void set_output_capacity(uint32_t capacity) noexcept {
        output_capacity_ = capacity;
    }

    // Main real-time entry point. Called exactly once per run() block by
    // the LV2 plugin.
    //
    // current_frame is uint64_t to support sessions longer than ~24.9 hours
    // at 48 kHz. It is the absolute frame position since plugin
    // instantiation. block_length and ev->time.frames remain uint32_t
    // (block-relative, always < block_length per LV2 spec).
    void process(const LV2_Atom_Sequence* input,
                 LV2_Atom_Sequence* output,
                 uint32_t sample_rate,
                 uint64_t current_frame,
                 uint32_t block_length,
                 const PluginParameters& params,
                 std::array<FilterState, 2048>& states);

private:
    static bool is_channel_selected(uint8_t channel, uint16_t mask) noexcept {
        return (mask >> channel) & 1u;
    }

    static bool is_cc_selected(uint8_t cc,
                               bool all_cc,
                               const std::array<bool, 128>& sel) noexcept {
        return all_cc ? !sel[cc] : sel[cc];
    }

    bool is_midi_event(const LV2_Atom* atom) const noexcept {
        return atom->type == midi_urid_;
    }

    // handle_selected_cc uses uint64_t current_frame for the absolute frame
    // position. event_frame_offset is uint32_t (block-relative, from
    // ev->time.frames).
    static void handle_selected_cc(uint8_t value,
                                   uint32_t event_frame_offset,
                                   uint64_t current_frame,
                                   const PluginParameters& params,
                                   FilterState& state);

    // emit_output_events_until uses uint64_t for all absolute frame
    // positions (block_start, deadline_frame, next_tick). This ensures
    // correct catch-up and tick scheduling across the 32-bit boundary.
    void emit_output_events_until(LV2_Atom_Sequence* output,
                                  uint32_t sample_rate,
                                  uint64_t block_start,
                                  uint64_t deadline_frame,
                                  const PluginParameters& params,
                                  std::array<FilterState, 2048>& states);

    bool append_cc_event(LV2_Atom_Sequence* output,
                         uint8_t channel, uint8_t cc_num, uint8_t value,
                         uint32_t frame_offset);

    bool append_passthrough_event(LV2_Atom_Sequence* output,
                                  const LV2_Atom_Event* event);

    LV2_URID midi_urid_       = 0;
    uint32_t output_capacity_ = 16384;
};

} // namespace midi_smooth_cc
