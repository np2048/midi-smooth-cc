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
//     (or a sensible default of 16384 bytes if the host does not provide one).
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
    // Construct with safe defaults. midi_urid_ and output_capacity_ should
    // be set by the plugin via set_midi_urid() and set_output_capacity()
    // before the first call to process().
    MidiProcessor() = default;

    // Cache the URID that the host's URID map feature assigns to the
    // midi:MidiEvent URI. Must be called once during plugin instantiate()
    // (non-real-time). Stored read-only and used in process() to identify
    // MIDI events inside the input Atom Sequence.
    void set_midi_urid(LV2_URID urid) noexcept { midi_urid_ = urid; }

    // Cache the total capacity in bytes of the output Atom Sequence buffer
    // that the host allocates for the MIDI output port. The plugin reads
    // this from the host's buf-size#sequenceSize option (if provided) or
    // falls back to a sensible default (16384 bytes). Must be called once
    // during plugin instantiate() or connect_port() (non-real-time).
    // Used by append_cc_event() and append_passthrough_event() as the
    // capacity argument to lv2_atom_sequence_append_event().
    void set_output_capacity(uint32_t capacity) noexcept {
        output_capacity_ = capacity;
    }

    // Main real-time entry point. Called exactly once per run() block by
    // the LV2 plugin. Performs:
    //   1. Clears the output Atom Sequence.
    //   2. If bypassed (params.enabled == false): copies every input event
    //      to the output unchanged and returns (spec §14 bypass semantics).
    //   3. Otherwise: iterates the input sequence, routes CC events on
    //      selected (channel, CC) pairs into the filter states, and copies
    //      every other event unchanged to the output (spec §10).
    //   4. Runs the output generation loop (spec §8): for every active
    //      filter state whose next output tick falls within the current
    //      block, advances the smoother, quantizes, and emits a CC event
    //      only if the integer value changed.
    void process(const LV2_Atom_Sequence* input,
                 LV2_Atom_Sequence* output,
                 uint32_t sample_rate,
                 uint32_t current_frame,
                 uint32_t block_length,
                 const PluginParameters& params,
                 std::array<FilterState, 2048>& states);

private:
    // ---- Helpers: parameter interpretation ----

    // True if MIDI channel `channel` (0..15) is enabled in `mask`.
    // Bit i of mask corresponds to channel i. (spec §4)
    static bool is_channel_selected(uint8_t channel, uint16_t mask) noexcept {
        return (mask >> channel) & 1u;
    }

    // True if CC number `cc` (0..127) is selected for processing.
    // In all_cc=false mode (whitelist): selected iff sel[cc] is true.
    // In all_cc=true  mode (blacklist): selected iff sel[cc] is false
    //   (sel[cc]==true means "explicitly disabled"). (spec §5)
    static bool is_cc_selected(uint8_t cc,
                               bool all_cc,
                               const std::array<bool, 128>& sel) noexcept {
        return all_cc ? !sel[cc] : sel[cc];
    }

    // ---- Helpers: atom inspection ----

    // True if `atom` is a MIDI event (its type URID matches midi_urid_).
    bool is_midi_event(const LV2_Atom* atom) const noexcept {
        return atom->type == midi_urid_;
    }

    // ---- Helpers: state mutation ----

    // Apply first-CC initialization (spec §14) and deadband check (spec §7)
    // for an incoming CC value on an already-selected (channel, CC) pair.
    // Updates the target on BOTH smoothers (via process() with dt=0, which
    // sets target_value_ without advancing current_value_) so that a later
    // mode switch begins ramping from the correct target. The current_value
    // of the inactive smoother is left untouched (its state is "preserved"
    // per the FilterState design); a mode switch may therefore produce a
    // brief ramp from a stale current_value, which is acceptable per spec
    // §14 ("no unexpected value JUMPS" — a smooth ramp is not a jump).
    static void handle_selected_cc(uint8_t value,
                                   uint32_t event_frame_offset,
                                   uint32_t current_frame,
                                   const PluginParameters& params,
                                   FilterState& state);

    // ---- Helpers: output generation ----

    // Run the output generation loop (spec §8). For every active filter
    // state whose next output tick falls within the current block, advance
    // the active smoother, quantize to 0..127, and emit a CC event only if
    // the integer value changed since the last transmission. Ticks that
    // fell behind the current block (e.g., after bypass) are caught up
    // with a single capped-dt advancement to prevent huge jumps (spec §14).
    void emit_output_events(LV2_Atom_Sequence* output,
                            uint32_t sample_rate,
                            uint32_t current_frame,
                            uint32_t block_length,
                            const PluginParameters& params,
                            std::array<FilterState, 2048>& states);

    // Append a 3-byte MIDI CC event (status 0xB0|channel, cc_num, value)
    // to the output Atom Sequence at the given frame offset within the
    // current block. Returns true on success, false if the output buffer
    // is full (the event is dropped gracefully; spec §11).
    bool append_cc_event(LV2_Atom_Sequence* output,
                         uint8_t channel, uint8_t cc_num, uint8_t value,
                         uint32_t frame_offset);

    // Append a pre-existing MIDI event (any size, including SysEx) to the
    // output unchanged. Used for pass-through of non-selected events.
    // Returns true on success, false if the output buffer is full.
    bool append_passthrough_event(LV2_Atom_Sequence* output,
                                  const LV2_Atom_Event* event);

    // ---- State ----

    LV2_URID midi_urid_       = 0;
    uint32_t output_capacity_ = 16384;  // sensible default; overridden by host options
};

} // namespace midi_smooth_cc
