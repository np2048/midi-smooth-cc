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
//
// 64-BIT FRAME TIMESTAMP HANDLING:
//   process() accepts current_frame as uint64_t. All internal absolute
//   frame calculations (block_start, block_end, deadline_frame, next_tick)
//   use uint64_t. This prevents wraparound at UINT32_MAX (~24.9 hours
//   at 48 kHz). Block-relative offsets (ev->time.frames, frame_offset)
//   remain uint32_t per the LV2 specification.
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
                            uint64_t current_frame,
                            uint32_t block_length,
                            const PluginParameters& params,
                            std::array<FilterState, 2048>& states) {
    // Step 1: clear the output sequence for this block.
    lv2_atom_sequence_clear(output);

    // block_start and block_end are uint64_t to support frame positions
    // beyond UINT32_MAX. current_frame is already uint64_t (no cast needed).
    const uint64_t block_start = current_frame;
    const uint64_t block_end   = block_start + static_cast<uint64_t>(block_length);

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

    // Normal mode: iterate input events in chronological order. Before
    // appending each passthrough event, flush any generated CC events
    // that are scheduled at a strictly earlier frame_offset.
    while (ev != ev_end) {
        // ev->time.frames is uint32_t (block-relative offset, per LV2 spec).
        // Compute the absolute deadline as uint64_t to preserve the full
        // frame position across the 32-bit boundary.
        const uint64_t ev_absolute =
            block_start + static_cast<uint64_t>(ev->time.frames);
        const uint64_t deadline =
            (ev_absolute > block_end) ? block_end : ev_absolute;

        emit_output_events_until(output, sample_rate, block_start, deadline,
                                 params, states);

        // Process the input event.
        if (is_midi_event(&ev->body) && ev->body.size >= 1) {
            const uint8_t* midi_bytes =
                reinterpret_cast<const uint8_t*>(LV2_ATOM_BODY_CONST(&ev->body));
            const uint8_t status       = midi_bytes[0];
            const uint8_t status_high  = static_cast<uint8_t>(status & 0xF0u);

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
                    // through. Pass current_frame as uint64_t to preserve
                    // the full absolute frame position.
                    FilterState& state =
                        states[static_cast<size_t>(channel) * 128u + cc_num];
                    handle_selected_cc(value, ev->time.frames,
                                       current_frame, params, state);
                } else {
                    // Unselected CC: pass through unchanged.
                    (void)append_passthrough_event(output, ev);
                }
            } else {
                // Any non-CC MIDI message: pass through unchanged.
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
//
// current_frame is uint64_t to preserve the full absolute frame position.
// event_frame_offset is uint32_t (block-relative, from ev->time.frames).
// The sum current_frame + event_frame_offset is computed in uint64_t,
// preventing truncation when current_frame > UINT32_MAX.
// =============================================================================
void MidiProcessor::handle_selected_cc(uint8_t value,
                                       uint32_t event_frame_offset,
                                       uint64_t current_frame,
                                       const PluginParameters& params,
                                       FilterState& state) {
    const double value_d = static_cast<double>(value);

    // First-CC initialization (spec §14).
    if (!state.is_initialized) {
        state.exp_smoother.reset(value_d);
        state.slew_limiter.reset(value_d);

        state.is_initialized         = true;
        state.last_transmitted_value = -1;
        // uint64_t + uint32_t promotes to uint64_t. No truncation.
        state.last_update_frame      = current_frame + event_frame_offset;
        return;
    }

    // Deadband check (spec §7).
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
// All absolute frame positions (block_start, deadline_frame, next_tick,
// state.last_update_frame) are uint64_t. This ensures correct catch-up
// and tick scheduling across the 32-bit boundary.
//
// OVERFLOW-SAFE COMPUTE/COMMIT PATTERN:
//   For each tick, the filter's next value is COMPUTED without mutating
//   internal state. The event is appended. ONLY if the append succeeds
//   is the filter state COMMITTED. If the append fails, the function
//   returns without advancing any state, allowing the tick to be retried
//   on the next block.
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

    if (deadline_frame <= block_start) {
        return;
    }

    const double tau_attack  = static_cast<double>(params.attack_ms)  / 1000.0;
    const double tau_release = static_cast<double>(params.release_ms) / 1000.0;
    const double rising      = static_cast<double>(params.rising_rate);
    const double falling     = static_cast<double>(params.falling_rate);

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

            // next_tick is uint64_t. state.last_update_frame is uint64_t.
            // frames_per_tick is uint32_t. The addition promotes to
            // uint64_t, preventing truncation.
            uint64_t next_tick = state.last_update_frame + frames_per_tick;

            // Catch-up: if we've fallen behind the block start, advance
            // the filter in one capped step to block_start. Both
            // next_tick and block_start are uint64_t, so the comparison
            // is correct even when current_frame > UINT32_MAX.
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
            // [block_start, deadline_frame). Both are uint64_t.
            while (next_tick < deadline_frame) {
                const double dt =
                    static_cast<double>(next_tick - state.last_update_frame) /
                    static_cast<double>(sample_rate);

                // COMPUTE phase: calculate the new value without mutating state.
                double smoothed;
                if (params.smoothing_mode == SmoothingMode::Exponential) {
                    const double target = state.exp_smoother.target_value();
                    smoothed = state.exp_smoother.compute(target, dt,
                                                          tau_attack, tau_release);
                } else {
                    const double target = state.slew_limiter.target_value();
                    smoothed = state.slew_limiter.compute(target, dt,
                                                          rising, falling);
                }

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

                // Only emit if the integer value changed (spec §8).
                if (int_val != state.last_transmitted_value) {
                    // frame_offset is uint32_t (block-relative, per LV2 spec).
                    // next_tick - block_start is uint64_t, but the result
                    // is always < block_length (which is uint32_t), so the
                    // cast to uint32_t is safe.
                    const uint32_t frame_offset =
                        static_cast<uint32_t>(next_tick - block_start);

                    // APPEND ATTEMPT: if this fails, return without
                    // committing state.
                    if (!append_cc_event(output,
                                         static_cast<uint8_t>(ch),
                                         static_cast<uint8_t>(cc),
                                         static_cast<uint8_t>(int_val),
                                         frame_offset)) {
                        return;
                    }

                    // COMMIT phase: only reached on successful append.
                    if (params.smoothing_mode == SmoothingMode::Exponential) {
                        const double target = state.exp_smoother.target_value();
                        state.exp_smoother.commit(target, smoothed);
                    } else {
                        const double target = state.slew_limiter.target_value();
                        state.slew_limiter.commit(target, smoothed);
                    }

                    state.last_transmitted_value = int_val;
                } else {
                    // Value unchanged: no event emitted, but advance the filter.
                    if (params.smoothing_mode == SmoothingMode::Exponential) {
                        const double target = state.exp_smoother.target_value();
                        state.exp_smoother.commit(target, smoothed);
                    } else {
                        const double target = state.slew_limiter.target_value();
                        state.slew_limiter.commit(target, smoothed);
                    }
                }

                // last_update_frame is uint64_t. next_tick is uint64_t.
                state.last_update_frame = next_tick;

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
