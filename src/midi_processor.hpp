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
//     as required by the LV2 specification. Strict hosts (e.g., Carla) reject
//     or silently drop out-of-order sequences.
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
//     lv2_atom_sequence_append_event() with explicit capacity checks; the
//     capacity is set once at instantiation from the host's buf-size options
//     (or a sensible default of 16384 bytes).
//   - All MIDI output values are clamped to [0, 127] and cast to uint8_t
//     before emission.
//   - All smoothing math uses double precision.
//   - Timestamps on emitted events are relative to the start of the current
//     processing block (frame offsets in [0, block_length)).
//
// Thread safety:
//   midi_urid_ and output_capacity_ are set once in instantiate() (host
//   thread) and read-only in run() (audio thread). No synchronization is
//   required for these members. The filter states array and PluginParameters
//   are owned by the plugin and only accessed within run().
//
// Modern LV2 Atom Sequence API (Golden Rule 2):
//   Input iteration uses lv2_atom_sequence_begin() / lv2_atom_sequence_end()
//   / lv2_atom_sequence_next(). The deprecated LV2_Event_Buffer API is never
//   used.
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
    // the LV2 plugin. Performs:
    //   1. Clears the output Atom Sequence.
    //   2. If bypassed: copies every input event to the output unchanged
    //      and returns.
    //   3. Otherwise: iterates the input sequence IN ORDER, and before
    //      appending each passthrough event, calls emit_output_events_until()
    //      to flush any generated CC events whose frame_offset is strictly
    //      less than the passthrough event's frame_offset. This interleaving
    //      guarantees the output sequence is chronologically sorted.
    //   4. After the input loop, calls emit_output_events_until() with
    //      deadline = block_end to flush remaining generated CC events.
    void process(const LV2_Atom_Sequence* input,
                 LV2_Atom_Sequence* output,
                 uint32_t sample_rate,
                 uint32_t current_frame,
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

    static void handle_selected_cc(uint8_t value,
                                   uint32_t event_frame_offset,
                                   uint32_t current_frame,
                                   const PluginParameters& params,
                                   FilterState& state);

    // Emit generated CC events for all active filter states whose next
    // output tick falls within [block_start, deadline_frame).
    //
    // This method is called multiple times from process(): once before each
    // passthrough event (with deadline = the passthrough event's absolute
    // frame) and once after the input loop (with deadline = block_end).
    //
    // Each call resumes from where the previous call left off, using the
    // per-state last_update_frame field to determine the next tick. The
    // catch-up step (advancing the filter to block_start) is performed
    // at most once across all calls within a single process() invocation.
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
