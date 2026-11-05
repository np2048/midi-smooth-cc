// midi-smooth-cc/src/plugin.cpp

// =============================================================================
// MIDI Smooth CC - LV2 plugin entry-point implementation
//
// Exports lv2_descriptor() and implements the standard LV2 lifecycle
// callbacks: instantiate / connect_port / activate / run / deactivate /
// cleanup / extension_data.
//
// 64-BIT FRAME COUNTER:
//   current_frame_ is uint64_t. In run(), it is passed directly to
//   midi_processor_.process() without any static_cast<uint32_t>. The
//   process() method accepts uint64_t. This prevents the frame counter
//   from wrapping at UINT32_MAX (~24.9 hours at 48 kHz), which would
//   break all timing and catch-up logic in long-running sessions.
//
// Real-time safety (spec §11, Golden Rules 1-3):
//   1. No dynamic allocation in run(), activate(), or deactivate().
//   2. Uses the modern LV2 Atom Sequence API.
//   3. No mutexes. Parameters are read once at the top of run().
// =============================================================================
#include "plugin.hpp"

#include <cstring>

#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/buf-size/buf-size.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"

namespace midi_smooth_cc {

// =============================================================================
// Constructor
// =============================================================================
MidiSmoothCCPlugin::MidiSmoothCCPlugin(double sample_rate,
                                       LV2_URID_Map* urid_map,
                                       uint32_t output_capacity)
    : sample_rate_(sample_rate > 0.0 ? static_cast<uint32_t>(sample_rate) : 44100u)
    , current_frame_(0u) {
    if (urid_map != nullptr && urid_map->map != nullptr) {
        const LV2_URID midi_urid =
            urid_map->map(urid_map->handle, LV2_MIDI__MidiEvent);
        midi_processor_.set_midi_urid(midi_urid);
    }

    midi_processor_.set_output_capacity(output_capacity);
}

// =============================================================================
// connect_port
// =============================================================================
void MidiSmoothCCPlugin::connect_port(uint32_t port, void* data) {
    if (data == nullptr) {
        return;
    }

    switch (port) {
        case PORT_MIDI_IN:
            port_midi_in_ = static_cast<const LV2_Atom_Sequence*>(data);
            break;
        case PORT_MIDI_OUT:
            port_midi_out_ = static_cast<LV2_Atom_Sequence*>(data);
            break;
        case PORT_ENABLED:
            port_enabled_ = static_cast<float*>(data);
            break;
        case PORT_SMOOTHING_MODE:
            port_smoothing_mode_ = static_cast<float*>(data);
            break;
        case PORT_ATTACK_MS:
            port_attack_ms_ = static_cast<float*>(data);
            break;
        case PORT_RELEASE_MS:
            port_release_ms_ = static_cast<float*>(data);
            break;
        case PORT_RISING_RATE:
            port_rising_rate_ = static_cast<float*>(data);
            break;
        case PORT_FALLING_RATE:
            port_falling_rate_ = static_cast<float*>(data);
            break;
        case PORT_DEADBAND:
            port_deadband_ = static_cast<float*>(data);
            break;
        case PORT_OUTPUT_RATE_HZ:
            port_output_rate_hz_ = static_cast<float*>(data);
            break;
        case PORT_ALL_CC:
            port_all_cc_ = static_cast<float*>(data);
            break;
        case PORT_CHANNEL_MASK:
            port_channel_mask_ = static_cast<float*>(data);
            break;
        default:
            if (port >= PORT_CC_SEL_BASE && port < PORT_TOTAL) {
                port_cc_selection_[port - PORT_CC_SEL_BASE] =
                    static_cast<float*>(data);
            }
            break;
    }
}

// =============================================================================
// activate
// =============================================================================
void MidiSmoothCCPlugin::activate() {
    current_frame_ = 0u;

    for (auto& state : states_) {
        state.last_update_frame = 0u;
    }
}

// =============================================================================
// run -- THE CRITICAL REAL-TIME CALLBACK
//
// current_frame_ (uint64_t) is passed DIRECTLY to process() without any
// narrowing cast. This is the key fix for the 64-bit timestamp defect:
// the previous code executed static_cast<uint32_t>(current_frame_),
// which discarded the upper 32 bits and caused wraparound after ~24.9
// hours at 48 kHz.
// =============================================================================
void MidiSmoothCCPlugin::run(uint32_t n_samples) {
    const auto read_float = [](float* port, float default_val) -> float {
        return (port != nullptr) ? *port : default_val;
    };

    const auto clamp_u32 = [](float v, uint32_t lo, uint32_t hi) -> uint32_t {
        const float flo = static_cast<float>(lo);
        const float fhi = static_cast<float>(hi);
        if (v < flo) return lo;
        if (v > fhi) return hi;
        return static_cast<uint32_t>(v);
    };

    PluginParameters params;

    params.enabled =
        (read_float(port_enabled_, 1.0f) > 0.5f);
    params.all_cc =
        (read_float(port_all_cc_, 0.0f) > 0.5f);

    {
        const float mode_f = read_float(port_smoothing_mode_, 0.0f);
        params.smoothing_mode =
            (mode_f > 0.5f) ? SmoothingMode::Slew : SmoothingMode::Exponential;
    }

    params.attack_ms      = clamp_u32(read_float(port_attack_ms_,       50.0f),   1u,  1000u);
    params.release_ms     = clamp_u32(read_float(port_release_ms_,     100.0f),   1u,  1000u);
    params.rising_rate    = clamp_u32(read_float(port_rising_rate_,    500.0f),   1u, 10000u);
    params.falling_rate   = clamp_u32(read_float(port_falling_rate_,   500.0f),   1u, 10000u);
    params.deadband       = clamp_u32(read_float(port_deadband_,         0.0f),   0u,    10u);
    params.output_rate_hz = clamp_u32(read_float(port_output_rate_hz_, 100.0f),  20u,   500u);

    {
        const float mask_f = read_float(port_channel_mask_, 65535.0f);
        params.channel_mask = static_cast<uint16_t>(mask_f);
    }

    for (uint32_t i = 0u; i < PORT_CC_SEL_COUNT; ++i) {
        const float default_val = (i == 7u) ? 1.0f : 0.0f;
        params.cc_selection[i] =
            (read_float(port_cc_selection_[i], default_val) > 0.5f);
    }

    if (port_midi_in_ != nullptr && port_midi_out_ != nullptr) {
        // ----------------------------------------------------------------
        // KEY FIX: Pass current_frame_ directly as uint64_t.
        // The previous code had:
        //   static_cast<uint32_t>(current_frame_)
        // which truncated the upper 32 bits, causing wraparound after
        // ~24.9 hours at 48 kHz. Now process() accepts uint64_t, and
        // current_frame_ is passed without any cast.
        // ----------------------------------------------------------------
        midi_processor_.process(
            port_midi_in_,
            port_midi_out_,
            sample_rate_,
            current_frame_,
            n_samples,
            params,
            states_);
    }

    // Advance the global frame counter. uint64_t + uint32_t promotes
    // to uint64_t. No truncation.
    current_frame_ += static_cast<uint64_t>(n_samples);
}

} // namespace midi_smooth_cc

// =============================================================================
// LV2 C interface
// =============================================================================
extern "C" {

static const char* const kMidiSmoothCC_URI = "urn:midi-smooth-cc:plugin";

static LV2_Handle instantiate(const LV2_Descriptor*     descriptor,
                              double                    sample_rate,
                              const char*               bundle_path,
                              const LV2_Feature* const* features) {
    (void)descriptor;
    (void)bundle_path;

    LV2_URID_Map* urid_map       = nullptr;
    uint32_t      seq_size       = 16384u;

    if (features != nullptr) {
        for (const LV2_Feature* const* fp = features; *fp != nullptr; ++fp) {
            const LV2_Feature* f = *fp;

            if (std::strcmp(f->URI, LV2_URID__map) == 0) {
                urid_map = static_cast<LV2_URID_Map*>(f->data);
            }
            else if (std::strcmp(f->URI, LV2_BUF_SIZE__sequenceSize) == 0) {
                if (f->data != nullptr) {
                    seq_size = *static_cast<const uint32_t*>(f->data);
                }
            }
        }
    }

    if (urid_map == nullptr) {
        return nullptr;
    }

    try {
        auto* plugin = new midi_smooth_cc::MidiSmoothCCPlugin(
            sample_rate, urid_map, seq_size);
        return static_cast<LV2_Handle>(plugin);
    } catch (...) {
        return nullptr;
    }
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->connect_port(port, data);
    }
}

static void activate(LV2_Handle instance) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->activate();
    }
}

static void run(LV2_Handle instance, uint32_t n_samples) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->run(n_samples);
    }
}

static void deactivate(LV2_Handle instance) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->deactivate();
    }
}

static void cleanup(LV2_Handle instance) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    delete plugin;
}

static const void* extension_data(const char* uri) {
    (void)uri;
    return nullptr;
}

static const LV2_Descriptor kMidiSmoothCC_Descriptor = {
    kMidiSmoothCC_URI,
    &instantiate,
    &connect_port,
    &activate,
    &run,
    &deactivate,
    &cleanup,
    &extension_data
};

const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    if (index == 0u) {
        return &kMidiSmoothCC_Descriptor;
    }
    return nullptr;
}

} // extern "C"
