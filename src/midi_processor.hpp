// midi-smooth-cc/src/midi_processor.hpp

//
// MIDI Smooth CC - MIDI event parsing, dispatch, and output generation
// (spec §8, §9, §10, §11). See midi_processor.cpp for the algorithm notes.
//
// Real-time contract: no allocation, locks, I/O. All output writes go
// through lv2_atom_sequence_append_event() with an explicit capacity.
//
// HOST CONTRACT (fix): process() now stamps the output as atom:Sequence
// (set_sequence_urid) and initialises body.unit/pad. The capacity passed to
// set_output_capacity() must be the real body capacity of the host's buffer.
//
// 64-bit frame timestamps: all absolute positions are uint64_t; block-relative
// offsets (ev->time.frames) stay uint32_t per the LV2 specification.
//
// Channel selection (Option B refactor): the previous single channel_mask
// uint16_t has been replaced by an all_channels boolean plus a 16-element
// channel_selection array, mirroring the all_cc / cc_selection semantics.
//   - all_channels=false: channel_selection is a whitelist.
//   - all_channels=true:  channel_selection is a blacklist.
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

    // URID of atom:Sequence. When non-zero, process() writes it into the
    // output atom.type each cycle (hosts hand over an atom:Chunk buffer).
    void set_sequence_urid(LV2_URID urid) noexcept { sequence_urid_ = urid; }

    void set_output_capacity(uint32_t capacity) noexcept {
        output_capacity_ = capacity;
    }

    void process(const LV2_Atom_Sequence* input,
                 LV2_Atom_Sequence* output,
                 uint32_t sample_rate,
                 uint64_t current_frame,
                 uint32_t block_length,
                 const PluginParameters& params,
                 std::array<FilterState, 2048>& states);

private:
    // Channel selection logic mirroring is_cc_selected:
    //   all_channels=false → whitelist: channel_selection[channel]==true means
    //     "process this channel".
    //   all_channels=true  → blacklist: channel_selection[channel]==true means
    //     "exclude this channel".
    static bool is_channel_selected(uint8_t channel,
                                    bool all_channels,
                                    const std::array<bool, 16>& sel) noexcept {
        return all_channels ? !sel[channel] : sel[channel];
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
                                   uint64_t current_frame,
                                   const PluginParameters& params,
                                   FilterState& state);

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
    LV2_URID sequence_urid_   = 0;
    uint32_t output_capacity_ = 16384;
};

} // namespace midi_smooth_cc
