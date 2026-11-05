// midi-smooth-cc/src/midi_processor.cpp

// =============================================================================
// MIDI Smooth CC - MIDI event processing implementation
//
// Layout of this file:
//   1. process()                  - top-level entry point with interleaving.
//   2. handle_selected_cc()       - first-CC init + deadband + target update.
//   3. emit_output_events_until() - incremental output generation loop.
//   4. append_cc_event()          - construct + emit a 3-byte CC event.
//   5. append_passthrough_event() - copy an existing event unchanged.
// =============================================================================
#include "midi_processor.hpp"

#include <algorithm>
#include <cmath>

namespace midi_smooth_cc {

// =============================================================================
// 1. process()
//
// CHRONOLOGICAL INTERLEAVING FIX:
//
// The previous implementation appended ALL passthrough events during the
// input loop, and only afterward called emit_output_events() to append
// generated CC events. This violated the LV2 specification, which requires
// LV2_Atom_Sequence events to be strictly ordered by timestamp (non-
// decreasing frame_offset). If a generated CC event had an earlier
// frame_offset than a passthrough event, the output sequence was out of
// order, and strict hosts like Carla would reject or silently drop it.
//
// The fix interleaves generated CC events with passthrough events:
//   - The host guarantees the input sequence is sorted by timestamp.
//   - Before appending each passthrough event at frame_offset T, we call
//     emit_output_events_until(deadline = block_start + T) to flush any
//     generated CC events whose frame_offset is strictly less than T.
//   - After the input loop, we call emit_output_events_until(deadline =
//     block_end) to flush remaining generated CC events.
//
// This produces a chronologically sorted output sequence without any
// sorting step or dynamic allocation. The cost is that emit_output_events_
// until() is called (N+1) times for N input events, but each call is O(2048)
// with early-exit on non-initialized states, which is bounded and acceptable
// for real-time.
// =============================================================================
void MidiProcessor::process(const LV2_Atom_Sequence* input,
                            LV2_Atom_Sequence* output,
                            uint32_t sample_rate,
                            uint32_t current_frame,
                            uint32_t block_length,
                            const PluginParameters& params,
                            std::array<FilterState, 2048>& states) {
    // Step 1: clear the output sequence for this block.
    lv2_atom_sequence_clear(output);

    const uint64_t block_start = static_cast<uint64_t>(current_frame);
    const uint64_t block_end   =
        block_start + static_cast<uint64_t>(block_length);

    // Step 2: iterate the input sequence.
    LV2_Atom_Event* ev     = lv2_atom_sequence_begin(&input->body);
    LV2_Atom_Event* ev_end = lv2_atom_sequence_end(&input->body, input->atom.size);

    // Bypass mode (spec §14): copy every event unchanged, no smoothing.
    if (!params.enabled) {
        while (ev != ev_end) {
            (void)append_passthrough_event(output, ev);
            ev = lv2_atom_sequence_next(ev);
        }
        return;
    }

    // Normal mode: iterate input events in chronological order (the host
    // guarantees the input sequence is sorted by timestamp). Before
    // appending each passthrough event, flush any generated CC events
    // that are scheduled at a strictly earlier frame_offset. This ensures
    // the output sequence is strictly ordered by frame_offset (non-
    // decreasing), as required by the LV2 specification.
    while (ev != ev_end) {
        // Compute the absolute frame of the current input event.
        // Generated CC events with frame_offset < ev->time.frames must be
        // emitted BEFORE this passthrough event to maintain chronological
        // order.
        const uint64_t ev_absolute =
            block_start + static_cast<uint64_t>(ev->time.frames);
        // Clamp to block_end in case of malformed input with timestamps
        // beyond the block boundary.
        const uint64_t deadline =
            (ev_absolute > block_end) ? block_end : ev_absolute;

        // Flush generated CC events up to (but not including) the deadline.
        // Using strict < ensures passthrough events at frame_offset T are
        // preceded only by generated CC events at frame_offset < T.
        // Generated CC events at the SAME frame_offset as a passthrough
        // event are emitted after the passthrough event (in a later call
        // or the final flush), which is valid per LV2's non-decreasing
        // order requirement.
        emit_output_events_until(output, sample_rate, block_start, deadline,
                                 params, states);

        // Process the input event.
        if (is_midi_event(&ev->body) && ev->body.size >= 1) {
            const uint8_t* midi_bytes =
                reinterpret_cast<const uint8_t*>(LV2_ATOM_BODY_CONST(&ev->body));
            const uint8_t status       = midi_bytes[0];
            const uint8_t status_high  = static_cast<uint8_t>(status & 0xF0u);

            // CC messages are exactly 3 bytes: status, CC number, value.
            if (status_high == 0xB0u && ev->body.size >= 3) {
                const uint8_t channel = static_cast<uint8_t>(status & 0x0Fu);
                const uint8_t cc_num  = midi_bytes[1];
                const uint8_t value   = midi_bytes[2];

                const bool ch_sel =
                    is_channel_selected(channel, params.channel_mask);
                const bool cc_sel =
                    is_cc_selected(cc_num, params.all_cc, params.cc_selection);

                if (ch_sel && cc_sel) {
                    // Selected CC: route into the filter state, do NOT pass
                    // through. The output loop will emit smoothed values.
                    FilterState& state =
                        states[static_cast<size_t>(channel) * 128u + cc_num];
                    handle_selected_cc(value, ev->time.frames,
                                       current_frame, params, state);
                } else {
                    // Unselected CC: pass through unchanged.
                    (void)append_passthrough_event(output, ev);
                }
            } else {
                // Any non-CC MIDI message (Note On/Off, Aftertouch, Pitch
                // Bend, Program Change, Clock, Real-Time, SysEx...): pass
                // through unchanged. (spec §10)
                (void)append_passthrough_event(output, ev);
            }
        } else {
            // Non-MIDI atom: pass through unchanged.
            (void)append_passthrough_event(output, ev);
        }

        ev = lv2_atom_sequence_next(ev);
    }

    // After all input events, flush remaining generated CC events up to the
    // end of the block.
    emit_output_events_until(output, sample_rate, block_start, block_end,
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

    // First-CC initialization (spec §14).
    if (!state.is_initialized) {
        state.exp_smoother.reset(value_d);
        state.slew_limiter.reset(value_d);

        state.is_initialized         = true;
        state.last_transmitted_value = -1;
        state.last_update_frame      =
            static_cast<uint64_t>(current_frame) + event_frame_offset;
        return;
    }

    // Deadband check (spec §7). Compare against the active smoother's
    // target so gradual changes accumulate and are not permanently lost.
    const double current_target =
        (params.smoothing_mode == SmoothingMode::Exponential)
            ? state.exp_smoother.target_value()
            : state.slew_limiter.target_value();

    if (params.deadband > 0u) {
        const double diff = std::abs(value_d - current_target);
        if (diff < static_cast<double>(params.deadband)) {
            return;
        }
    }

    // Update target on BOTH smoothers via process() with dt=0.
    const double tau_attack  = static_cast<double>(params.attack_ms)  / 1000.0;
    const double tau_release = static_cast<double>(params.release_ms) / 1000.0;
    const double rising      = static_cast<double>(params.rising_rate);
    const double falling     = static_cast<double>(params.falling_rate);

    (void)state.exp_smoother.process(value_d, 0.0, tau_attack, tau_release);
    (void)state.slew_limiter.process(value_d, 0.0, rising, falling);
}

// =============================================================================
// 3. emit_output_events_until()
//
// Emit generated CC events for all active filter states whose next output
// tick falls within [block_start, deadline_frame).
//
// This method is designed for incremental invocation: it can be called
// multiple times within a single process() block, each time with a
// progressively later deadline_frame. Each call resumes from where the
// previous call left off, using the per-state last_update_frame field.
//
// The catch-up step (advancing the filter from a stale last_update_frame
// to block_start) is performed at most once across all calls within a
// single process() invocation, because after catch-up last_update_frame
// is set to block_start and subsequent calls see next_tick >= block_start.
// =============================================================================
void MidiProcessor::emit_output_events_until(LV2_Atom_Sequence* output,
                                             uint32_t sample_rate,
                                             uint64_t block_start,
                                             uint64_t deadline_frame,
                                             const PluginParameters& params,
                                             std::array<FilterState, 2048>& states) {
    if (params.output_rate_hz == 0u || sample_rate == 0u) {
        return;
    }

    const uint32_t frames_per_tick = sample_rate / params.output_rate_hz;
    if (frames_per_tick == 0u) {
        return;
    }

    // If the deadline is at or before the block start, there are no ticks
    // to process (ticks fall at block_start + k*frames_per_tick, all > 0).
    // This early exit avoids iterating 2048 states when the first input
    // event is at frame_offset 0.
    if (deadline_frame <= block_start) {
        return;
    }

    // Pre-compute smoother parameters outside the per-state loop.
    const double tau_attack  = static_cast<double>(params.attack_ms)  / 1000.0;
    const double tau_release = static_cast<double>(params.release_ms) / 1000.0;
    const double rising      = static_cast<double>(params.rising_rate);
    const double falling     = static_cast<double>(params.falling_rate);

    // Cap on dt to prevent huge jumps after long inactivity (spec §14).
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
            // time we advanced this filter.
            uint64_t next_tick = state.last_update_frame + frames_per_tick;

            // Catch-up: if we've fallen behind the block start, advance
            // the filter in one capped step to block_start. This happens
            // at most once per process() call (on the first
            // emit_output_events_until invocation for this state).
            if (next_tick < block_start) {
                const double dt_catchup =
                    static_cast<double>(block_start - state.last_update_frame) /
                    static_cast<double>(sample_rate);
                const double dt_eff =
                    (dt_catchup > kMaxDtSeconds) ? kMaxDtSeconds : dt_catchup;

                const double target =
                    (params.smoothing_mode == SmoothingMode::Exponential)
                        ? state.exp_smoother.target_value()
                        : state.slew_limiter.target_value();

                if (params.smoothing_mode == SmoothingMode::Exponential) {
                    (void)state.exp_smoother.process(target, dt_eff,
                                                     tau_attack, tau_release);
                } else {
                    (void)state.slew_limiter.process(target, dt_eff,
                                                     rising, falling);
                }

                state.last_update_frame = block_start;
                next_tick = block_start + frames_per_tick;
            }

            // Process every output tick that falls within
            // [block_start, deadline_frame).
            while (next_tick < deadline_frame) {
                const double dt =
                    static_cast<double>(next_tick - state.last_update_frame) /
                    static_cast<double>(sample_rate);

                // Read the active smoother's target and advance it by dt.
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

                // Mark this tick as processed BEFORE the emit attempt.
                state.last_update_frame = next_tick;

                // Quantize to MIDI integer with strict clamping.
                const long rounded = std::lround(smoothed);
                int int_val;
                if (rounded < 0L) {
                    int_val = 0;
                } else if (rounded > 127L) {
                    int_val = 127;
                } else {
                    int_val = static_cast<int>(rounded);
                }

                // Emit only if the integer value changed (spec §8).
                if (int_val != state.last_transmitted_value) {
                    const uint32_t frame_offset =
                        static_cast<uint32_t>(next_tick - block_start);
                    if (!append_cc_event(output,
                                         static_cast<uint8_t>(ch),
                                         static_cast<uint8_t>(cc),
                                         static_cast<uint8_t>(int_val),
                                         frame_offset)) {
                        // Output buffer full (spec §11): stop emitting.
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

    struct StackMidiEvent {
        LV2_Atom_Event header;
        uint8_t        data[3];
    } ev;

    ev.header.time.frames = frame_offset;
    ev.header.body.type   = midi_urid_;
    ev.header.body.size   = 3u;
    ev.data[0] = static_cast<uint8_t>(0xB0u | (channel & 0x0Fu));
    ev.data[1] = static_cast<uint8_t>(cc_num & 0x7Fu);
    ev.data[2] = static_cast<uint8_t>(value   & 0x7Fu);

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

    LV2_Atom_Event* appended = lv2_atom_sequence_append_event(
        output, output_capacity_, event);

    return (appended != nullptr);
}

} // namespace midi_smooth_cc
