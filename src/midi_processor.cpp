// midi-smooth-cc/src/midi_processor.cpp

// =============================================================================
// MIDI Smooth CC - MIDI event processing implementation
//
// Implements the methods declared in midi_processor.hpp. This file is the
// heart of the real-time path and must obey every constraint in spec §11.
//
// Layout of this file:
//   1. process()                  - top-level entry point.
//   2. handle_selected_cc()       - first-CC init + deadband + target update.
//   3. emit_output_events()       - the §8 output generation loop.
//   4. append_cc_event()          - construct + emit a 3-byte CC event.
//   5. append_passthrough_event() - copy an existing event unchanged.
// =============================================================================
#include "midi_processor.hpp"

#include <algorithm>
#include <cmath>

namespace midi_smooth_cc {

// =============================================================================
// 1. process()
// =============================================================================
void MidiProcessor::process(const LV2_Atom_Sequence* input,
                            LV2_Atom_Sequence* output,
                            uint32_t sample_rate,
                            uint32_t current_frame,
                            uint32_t block_length,
                            const PluginParameters& params,
                            std::array<FilterState, 2048>& states) {
    // ----- Step 1: clear the output sequence for this block -----
    // lv2_atom_sequence_clear() resets atom.size to sizeof(LV2_Atom_Sequence_Body)
    // (i.e., empty). The host initializes atom.type once; we do not touch it.
    lv2_atom_sequence_clear(output);

    // ----- Step 2: iterate the input sequence -----
    // Use the modern LV2 Atom Sequence API (lv2_atom_sequence_begin / _end /
    // _next), never the deprecated LV2_Event_Buffer. (spec §11, Golden Rule 2)
    //
    // Note on const-correctness: the LV2 API returns LV2_Atom_Event* (non-
    // const) even from a const LV2_Atom_Sequence_Body*. This is a known LV2
    // quirk; we treat the events as read-only by never writing through them.
    LV2_Atom_Event* ev     = lv2_atom_sequence_begin(&input->body);
    LV2_Atom_Event* ev_end = lv2_atom_sequence_end(&input->body, input->atom.size);

    // ----- Bypass mode (spec §14): copy every event unchanged, no smoothing -----
    if (!params.enabled) {
        while (ev != ev_end) {
            (void)append_passthrough_event(output, ev);
            ev = lv2_atom_sequence_next(ev);
        }
        // Filter states are intentionally NOT updated during bypass; their
        // current/target values are preserved so that on bypass-deactivate
        // the output resumes smoothly from the last known state.
        return;
    }

    // ----- Normal mode: route CCs, pass through everything else -----
    while (ev != ev_end) {
        if (is_midi_event(&ev->body) && ev->body.size >= 1) {
            // The MIDI bytes follow immediately after the LV2_Atom header
            // inside the LV2_Atom_Event. LV2_ATOM_BODY_CONST returns a
            // const uint8_t* to the body data.
            const uint8_t* midi_bytes =
                reinterpret_cast<const uint8_t*>(LV2_ATOM_BODY_CONST(&ev->body));
            const uint8_t status       = midi_bytes[0];
            const uint8_t status_high  = static_cast<uint8_t>(status & 0xF0u);

            // CC messages are exactly 3 bytes: status, CC number, value.
            // We additionally require ev->body.size >= 3 to avoid reading
            // past the end of a malformed event.
            if (status_high == 0xB0u && ev->body.size >= 3) {
                const uint8_t channel = static_cast<uint8_t>(status & 0x0Fu);
                const uint8_t cc_num  = midi_bytes[1];
                const uint8_t value   = midi_bytes[2];

                const bool ch_sel =
                    is_channel_selected(channel, params.channel_mask);
                const bool cc_sel =
                    is_cc_selected(cc_num, params.all_cc, params.cc_selection);

                if (ch_sel && cc_sel) {
                    // Selected: route into the filter state, do NOT pass
                    // through. The output loop will emit smoothed values.
                    FilterState& state =
                        states[static_cast<size_t>(channel) * 128u + cc_num];
                    handle_selected_cc(value, ev->time.frames,
                                       current_frame, params, state);
                } else {
                    // CC on an unselected channel/CC: pass through unchanged.
                    (void)append_passthrough_event(output, ev);
                }
            } else {
                // Any non-CC MIDI message (Note On/Off, Aftertouch, Pitch
                // Bend, Program Change, Clock, Real-Time, SysEx...): pass
                // through unchanged. (spec §10) Note On/Off are never
                // delayed because they are emitted at their original
                // ev->time.frames offset within the block.
                (void)append_passthrough_event(output, ev);
            }
        } else {
            // Non-MIDI atom (e.g., Property, Blank, Object): pass through
            // unchanged so we never corrupt the host's control stream.
            (void)append_passthrough_event(output, ev);
        }
        ev = lv2_atom_sequence_next(ev);
    }

    // ----- Step 3: output generation (spec §8) -----
    emit_output_events(output, sample_rate, current_frame, block_length,
                       params, states);
}

// =============================================================================
// 2. handle_selected_cc()
// =============================================================================
void MidiProcessor::handle_selected_cc(uint8_t value,
                                       uint32_t event_frame_offset,
                                       uint32_t current_frame,
                                       const PluginParameters& params,
                                       FilterState& state) {
    const double value_d = static_cast<double>(value);

    // ----- First-CC initialization (spec §14) -----
    if (!state.is_initialized) {
        // Initialize both smoothers to the incoming value. This avoids an
        // artificial ramp from zero. (spec §14: "Initialize its filter
        // state from the incoming value.")
        state.exp_smoother.reset(value_d);
        state.slew_limiter.reset(value_d);

        state.is_initialized         = true;
        state.last_transmitted_value = -1;  // force next output tick to emit
        state.last_update_frame      =
            static_cast<uint64_t>(current_frame) + event_frame_offset;
        // Both smoothers' target_value AND current_value are now `value_d`
        // (set by reset()). No deadband check on the first event.
        return;
    }

    // ----- Deadband check (spec §7) -----
    // The spec wording is "minimum difference between the current filtered
    // value and a new target value". We compare against the active
    // smoother's TARGET (not its current_value) because:
    //   - Comparing against current_value would cause gradual intentional
    //     changes (e.g., a slow ramp from 50 to 60 in 1-unit steps) to be
    //     permanently lost, since each small step would be within the
    //     deadband of the (slowly moving) current_value. Spec §7 explicitly
    //     forbids this: "small but intentional changes in breath pressure
    //     are not permanently lost."
    //   - Comparing against target means gradual changes accumulate until
    //     they exceed the deadband relative to the LAST ACCEPTED target,
    //     at which point they are accepted. This preserves intent.
    // The target interpretation also keeps behavior stable regardless of
    // how far current_value has converged toward the target.
    const double current_target =
        (params.smoothing_mode == SmoothingMode::Exponential)
            ? state.exp_smoother.target_value()
            : state.slew_limiter.target_value();

    if (params.deadband > 0u) {
        const double diff = std::abs(value_d - current_target);
        if (diff < static_cast<double>(params.deadband)) {
            return;  // within deadband: reject update, preserve current target
        }
    }

    // ----- Update target on BOTH smoothers -----
    // We call process() with dt=0.0, which sets target_value_ to the new
    // value WITHOUT advancing current_value_:
    //   - For ExponentialSmoother: alpha = 1 - exp(0) = 0, so
    //     new_value = current + 0 * delta = current. Target is updated.
    //   - For SlewRateLimiter: max_delta = rate * 0 = 0, so step = 0.
    //     Target is updated.
    // This keeps the two smoothers' targets in sync so a later mode
    // switch begins ramping toward the correct target. The inactive
    // smoother's current_value is left untouched (preserved per the
    // FilterState design).
    const double tau_attack  = static_cast<double>(params.attack_ms)  / 1000.0;
    const double tau_release = static_cast<double>(params.release_ms) / 1000.0;
    const double rising      = static_cast<double>(params.rising_rate);
    const double falling     = static_cast<double>(params.falling_rate);

    (void)state.exp_smoother.process(value_d, 0.0, tau_attack, tau_release);
    (void)state.slew_limiter.process(value_d, 0.0, rising, falling);

    // Note: we deliberately do NOT update last_update_frame here. That
    // field tracks the last time the filter was ADVANCED (in the output
    // loop), not the last time the target was set. Updating it here would
    // cause the output loop to skip owed tick advancements.
}

// =============================================================================
// 3. emit_output_events()
// =============================================================================
void MidiProcessor::emit_output_events(LV2_Atom_Sequence* output,
                                       uint32_t sample_rate,
                                       uint32_t current_frame,
                                       uint32_t block_length,
                                       const PluginParameters& params,
                                       std::array<FilterState, 2048>& states) {
    if (params.output_rate_hz == 0u || sample_rate == 0u) {
        return;
    }

    // frames_per_tick = how many audio frames elapse between two consecutive
    // output emissions at the configured output_rate_hz. (spec §8)
    const uint32_t frames_per_tick = sample_rate / params.output_rate_hz;
    if (frames_per_tick == 0u) {
        // output_rate_hz > sample_rate: cannot sub-sample. Bail out safely
        // rather than dividing by zero or entering an infinite loop.
        return;
    }

    const uint64_t block_start = static_cast<uint64_t>(current_frame);
    const uint64_t block_end   =
        static_cast<uint64_t>(current_frame) + static_cast<uint64_t>(block_length);

    // Pre-compute the smoother parameters outside the per-state loop so the
    // compiler hoists them out of the inner loop.
    const double tau_attack  = static_cast<double>(params.attack_ms)  / 1000.0;
    const double tau_release = static_cast<double>(params.release_ms) / 1000.0;
    const double rising      = static_cast<double>(params.rising_rate);
    const double falling     = static_cast<double>(params.falling_rate);

    // Cap on dt to prevent huge jumps after long inactivity (e.g., after
    // bypass is deactivated, or after the plugin is first instantiated and
    // a CC arrives many seconds in). Both exponential and slew math are
    // correct for any dt, but a very large dt would make the filter snap
    // instantly to the target (alpha -> 1 for exp; step unbounded for slew
    // up to the target), violating spec §14's "no unexpected value jumps"
    // rule. A 200ms cap is generous: at the default 100ms release time the
    // exponential filter is ~86% converged after 200ms, and at the default
    // 500 CC/s slew rate the limiter moves 100 CC units in 200ms - enough
    // to absorb typical target jumps smoothly.
    constexpr double kMaxDtSeconds = 0.2;

    for (uint16_t ch = 0u; ch < 16u; ++ch) {
        if (!is_channel_selected(static_cast<uint8_t>(ch), params.channel_mask)) {
            continue;
        }
        for (int cc = 0; cc < 128; ++cc) {
            if (!is_cc_selected(static_cast<uint8_t>(cc),
                                params.all_cc, params.cc_selection)) {
                continue;
            }

            FilterState& state =
                states[static_cast<size_t>(ch) * 128u + static_cast<size_t>(cc)];
            if (!state.is_initialized) {
                continue;
            }

            // The next output tick is one frames_per_tick after the last
            // time we advanced this filter. If we've fallen behind (e.g.,
            // after bypass, or because the state was just initialized at
            // an early frame), this tick may be in the past relative to
            // the current block. We catch up in ONE capped-dt step (below)
            // to the start of the current block, then process ticks within
            // the block normally.
            uint64_t next_tick = state.last_update_frame + frames_per_tick;

            if (next_tick < block_start) {
                // ----- Catch-up: advance filter from last_update_frame to
                // block_start in one capped step. We do NOT emit for this
                // catch-up step because the tick is in the past (outside
                // the current block); emitting at a past frame offset
                // would corrupt the output sequence's monotonic time
                // ordering. The first emit will be at block_start + fpt.
                const double dt_catchup =
                    static_cast<double>(block_start - state.last_update_frame) /
                    static_cast<double>(sample_rate);
                const double dt_eff =
                    (dt_catchup > kMaxDtSeconds) ? kMaxDtSeconds : dt_catchup;

                const double target_cu =
                    (params.smoothing_mode == SmoothingMode::Exponential)
                        ? state.exp_smoother.target_value()
                        : state.slew_limiter.target_value();

                if (params.smoothing_mode == SmoothingMode::Exponential) {
                    (void)state.exp_smoother.process(target_cu, dt_eff,
                                                     tau_attack, tau_release);
                } else {
                    (void)state.slew_limiter.process(target_cu, dt_eff,
                                                     rising, falling);
                }

                state.last_update_frame = block_start;
                next_tick = block_start + frames_per_tick;
            }

            // ----- Process every output tick that falls within this block -----
            // The loop count is bounded by block_length / frames_per_tick,
            // which for typical settings (block_length <= 4096, fpt = 441 at
            // 44.1kHz / 100Hz) is at most ~9 iterations per active state.
            while (next_tick < block_end) {
                const double dt =
                    static_cast<double>(next_tick - state.last_update_frame) /
                    static_cast<double>(sample_rate);
                // dt here is at most frames_per_tick / sample_rate, which is
                // tiny (~10ms at default settings) - no need to cap.

                // Read the active smoother's target and advance it by dt.
                // The inactive smoother is NOT advanced (its state is
                // preserved per the FilterState design). On a mode switch,
                // the newly-active smoother may briefly ramp from a stale
                // current_value to the (in-sync) target - this is a smooth
                // ramp, not an instantaneous jump, and is acceptable.
                double smoothed;
                if (params.smoothing_mode == SmoothingMode::Exponential) {
                    const double target = state.exp_smoother.target_value();
                    smoothed = state.exp_smoother.process(target, dt,
                                                          tau_attack, tau_release);
                } else {
                    const double target = state.slew_limiter.target_value();
                    smoothed = state.slew_limiter.process(target, dt,
                                                          rising, falling);
                }

                // Mark this tick as processed BEFORE the emit attempt, so
                // that even if the buffer is full and we bail out, the
                // state is consistent for the next block.
                state.last_update_frame = next_tick;

                // Quantize to MIDI integer with strict clamping to [0, 127].
                // std::lround returns long; on x86_64 Linux long is 64-bit,
                // so the cast to int is safe for the clamped range.
                const long rounded = std::lround(smoothed);
                int int_val;
                if (rounded < 0L) {
                    int_val = 0;
                } else if (rounded > 127L) {
                    int_val = 127;
                } else {
                    int_val = static_cast<int>(rounded);
                }

                // Emit only if the integer value changed since the last
                // transmission (spec §8: "avoid redundant MIDI messages").
                if (int_val != state.last_transmitted_value) {
                    const uint32_t frame_offset =
                        static_cast<uint32_t>(next_tick - block_start);
                    if (!append_cc_event(output,
                                         static_cast<uint8_t>(ch),
                                         static_cast<uint8_t>(cc),
                                         static_cast<uint8_t>(int_val),
                                         frame_offset)) {
                        // Output buffer full (spec §11): stop emitting
                        // immediately. Already-advanced last_update_frame
                        // values are preserved; the next block resumes
                        // from there with a small (sub-block) dt that is
                        // well below the kMaxDtSeconds cap. We return from
                        // the entire function (not just break) because if
                        // the buffer is full for this (ch, cc) it will be
                        // full for all subsequent ones too.
                        return;
                    }
                    state.last_transmitted_value = int_val;
                }

                next_tick += frames_per_tick;
            }
        }
    }
}

// =============================================================================
// 4. append_cc_event()
// =============================================================================
bool MidiProcessor::append_cc_event(LV2_Atom_Sequence* output,
                                    uint8_t channel, uint8_t cc_num, uint8_t value,
                                    uint32_t frame_offset) {
    if (output_capacity_ == 0u) {
        return false;
    }

    // A MIDI CC event is exactly 3 bytes: status | channel, CC number, value.
    // We construct the LV2_Atom_Event header on the stack immediately
    // followed by the 3 MIDI bytes, then hand the whole thing to
    // lv2_atom_sequence_append_event() which copies both the header
    // (sizeof(LV2_Atom_Event) bytes) and the body data (event->body.size
    // bytes) into the output sequence.
    //
    // Struct layout: LV2_Atom_Event is 12 bytes (uint32 time.frames + LV2_Atom
    // body of uint32 type + uint32 size). The 3 uint8_t MIDI bytes follow
    // immediately (alignment 1, no internal padding). Trailing struct
    // padding may exist but append_event only reads sizeof(LV2_Atom_Event)
    // + body.size = 15 bytes, so the padding is never touched.
    struct StackMidiEvent {
        LV2_Atom_Event header;
        uint8_t        data[3];
    } ev;

    ev.header.time.frames = frame_offset;
    ev.header.body.type   = midi_urid_;
    ev.header.body.size   = 3u;
    // MIDI status byte for CC on the given channel, with channel masked to
    // the low 4 bits and CC number / value masked to 7 bits (defensive).
    ev.data[0] = static_cast<uint8_t>(0xB0u | (channel & 0x0Fu));
    ev.data[1] = static_cast<uint8_t>(cc_num & 0x7Fu);
    ev.data[2] = static_cast<uint8_t>(value   & 0x7Fu);

    // lv2_atom_sequence_append_event returns NULL if the buffer does not
    // have room for the padded event. (spec §11: handle overflow safely,
    // never corrupt the output sequence.)
    LV2_Atom_Event* appended = lv2_atom_sequence_append_event(
        output, output_capacity_, &ev.header);

    return (appended != nullptr);
}

// =============================================================================
// 5. append_passthrough_event()
// =============================================================================
bool MidiProcessor::append_passthrough_event(LV2_Atom_Sequence* output,
                                             const LV2_Atom_Event* event) {
    if (output_capacity_ == 0u) {
        return false;
    }

    // Pass-through copies an existing event verbatim, including its
    // variable-length body (SysEx, etc.). lv2_atom_sequence_append_event
    // copies sizeof(LV2_Atom_Event) bytes of header plus event->body.size
    // bytes of body data. (spec §10: preserve original channel and data;
    // spec §10: Note On/Off are never delayed - they are appended at their
    // original ev->time.frames offset within the block.)
    LV2_Atom_Event* appended = lv2_atom_sequence_append_event(
        output, output_capacity_, event);

    return (appended != nullptr);
}

} // namespace midi_smooth_cc
