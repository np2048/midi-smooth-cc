// midi-smooth-cc/src/plugin.cpp

// =============================================================================
// MIDI Smooth CC - LV2 plugin entry-point implementation
//
// Exports lv2_descriptor() and implements the standard LV2 lifecycle
// callbacks: instantiate / connect_port / activate / run / deactivate /
// cleanup / extension_data.
//
// The plugin URI MUST be "urn:midi-smooth-cc:plugin" and must match the
// subject of manifest.ttl.in and plugin.ttl.in exactly.
//
// Real-time safety (spec §11, Golden Rules 1-3):
//   1. No dynamic allocation (new/delete, std::vector, std::string) in
//      run(), activate(), or deactivate(). The ONLY place `new` and
//      `delete` appear is in instantiate() and cleanup() respectively.
//   2. Uses the modern LV2 Atom Sequence API (lv2_atom_sequence_*),
//      delegated entirely to MidiProcessor.
//   3. No mutexes. Control port values are read once at the top of run()
//      into a local PluginParameters snapshot and used throughout the
//      block. The host guarantees connect_port() is called outside run().
//
// Buffer capacity handling:
//   The plugin queries the host's buf-size#sequenceSize feature during
//   instantiate() to learn the exact byte capacity of the output MIDI
//   atom sequence port buffer. This value is passed as the third
//   constructor argument to MidiSmoothCCPlugin, which forwards it to
//   MidiProcessor::set_output_capacity(). This ensures
//   lv2_atom_sequence_append_event() uses the CORRECT capacity for its
//   bounds check, preventing out-of-bounds writes when the host allocates
//   a smaller buffer than the plugin's default of 16384.
//   If the host does not provide the feature, the plugin falls back to
//   16384 bytes (a safe default for typical host configurations).
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
//
// Called once by the extern "C" instantiate() wrapper (host thread, non-
// real-time). Heap allocation is permitted here.
//
// The output_capacity parameter is the byte capacity of the output MIDI
// atom sequence port buffer, as reported by the host's
// buf-size#sequenceSize feature. It is forwarded directly to
// MidiProcessor::set_output_capacity() so that the processor's
// lv2_atom_sequence_append_event() calls use the correct capacity.
// =============================================================================
MidiSmoothCCPlugin::MidiSmoothCCPlugin(double sample_rate,
                                       LV2_URID_Map* urid_map,
                                       uint32_t output_capacity)
    : sample_rate_(sample_rate > 0.0 ? static_cast<uint32_t>(sample_rate) : 44100u)
    , current_frame_(0u) {
    // Map the midi:MidiEvent URI to an integer URID via the host's URID
    // map feature. This URID is used by MidiProcessor to identify MIDI
    // events inside LV2 Atom Sequences.
    if (urid_map != nullptr && urid_map->map != nullptr) {
        const LV2_URID midi_urid =
            urid_map->map(urid_map->handle, LV2_MIDI__MidiEvent);
        midi_processor_.set_midi_urid(midi_urid);
    }

    // Set the output buffer capacity from the host-provided value (or
    // the default 16384 if the host did not provide buf-size#sequenceSize).
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
        midi_processor_.process(
            port_midi_in_,
            port_midi_out_,
            sample_rate_,
            static_cast<uint32_t>(current_frame_),
            n_samples,
            params,
            states_);
    }

    current_frame_ += static_cast<uint64_t>(n_samples);
}

} // namespace midi_smooth_cc

// =============================================================================
// LV2 C interface
// =============================================================================
extern "C" {

static const char* const kMidiSmoothCC_URI = "urn:midi-smooth-cc:plugin";

// -------------------------------------------------------------------------
// instantiate -- create a new plugin instance (non-real-time).
//
// This function iterates the host-provided features array to find:
//   1. urid:map (REQUIRED) -- for mapping midi:MidiEvent to a URID.
//   2. buf-size#sequenceSize (OPTIONAL) -- for learning the exact byte
//      capacity of the output MIDI atom sequence port buffer.
//
// The buf-size#sequenceSize feature's data pointer points to a uint32_t
// value specifying the maximum size in bytes of an atom:Sequence port
// buffer allocated by the host. This value is passed as the third
// constructor argument to MidiSmoothCCPlugin, which forwards it to
// MidiProcessor::set_output_capacity(). This ensures
// lv2_atom_sequence_append_event() performs its capacity check against
// the ACTUAL buffer size rather than a hardcoded guess, preventing
// out-of-bounds writes when the host allocates a smaller buffer.
//
// If the host does not provide buf-size#sequenceSize, the plugin uses
// a default of 16384 bytes, which is safe for typical host configurations.
// -------------------------------------------------------------------------
static LV2_Handle instantiate(const LV2_Descriptor*     descriptor,
                              double                    sample_rate,
                              const char*               bundle_path,
                              const LV2_Feature* const* features) {
    (void)descriptor;
    (void)bundle_path;

    // Find required and optional features in the host-provided array.
    LV2_URID_Map* urid_map       = nullptr;
    uint32_t      seq_size       = 16384u;  // default fallback

    if (features != nullptr) {
        for (const LV2_Feature* const* fp = features; *fp != nullptr; ++fp) {
            const LV2_Feature* f = *fp;

            // Required: urid:map -- for mapping MIDI event URIs.
            if (std::strcmp(f->URI, LV2_URID__map) == 0) {
                urid_map = static_cast<LV2_URID_Map*>(f->data);
            }
            // Optional: buf-size#sequenceSize -- the actual output buffer
            // capacity in bytes allocated by the host for atom:Sequence
            // ports. The data pointer points to a uint32_t value.
            else if (std::strcmp(f->URI, LV2_BUF_SIZE__sequenceSize) == 0) {
                if (f->data != nullptr) {
                    seq_size = *static_cast<const uint32_t*>(f->data);
                }
            }
        }
    }

    // Required feature not provided -- refuse to instantiate.
    if (urid_map == nullptr) {
        return nullptr;
    }

    try {
        // Pass the resolved output capacity (host-provided or default) as
        // the third constructor argument. The constructor forwards it to
        // midi_processor_.set_output_capacity().
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
