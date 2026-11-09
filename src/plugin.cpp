// midi-smooth-cc/src/plugin.cpp

//
// MIDI Smooth CC - LV2 plugin entry-point implementation.
//
// Real-time safety (spec §11): no allocation, locks or I/O in run().
// Parameters are read once at the top of run().
//
// Host-contract fixes in this revision:
//   1. atom:Sequence URID is mapped and stamped on the output each cycle
//      (done inside MidiProcessor::process).
//   2. The output capacity is learned from the host-provided atom.size.
//   3. connect_port(nullptr) now disconnects instead of being ignored.
//
// Channel selection (Option B): the former single port_channel_mask_ float*
// has been replaced by port_all_channels_ plus port_channel_selection_[16].
// run() builds params.channel_selection from the 16 toggle ports, and
// params.all_channels from the master toggle, mirroring the all_cc path.
//
// Defaults: all_channels defaults to 1.0 (blacklist mode), channel_1
// defaults to 1.0, channels 2-16 default to 0.0. This means Channel 1 is
// excluded and channels 2-16 are processed by default.
#include "plugin.hpp"

#include <cstring>

#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/buf-size/buf-size.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"

namespace midi_smooth_cc {

MidiSmoothCCPlugin::MidiSmoothCCPlugin(double sample_rate,
                                       LV2_URID_Map* urid_map,
                                       uint32_t output_capacity)
    : sample_rate_(sample_rate > 0.0 ? static_cast<uint32_t>(sample_rate) : 44100u)
    , current_frame_(0u)
    , default_capacity_(output_capacity) {
    if (urid_map != nullptr && urid_map->map != nullptr) {
        midi_processor_.set_midi_urid(
            urid_map->map(urid_map->handle, LV2_MIDI__MidiEvent));
        midi_processor_.set_sequence_urid(
            urid_map->map(urid_map->handle, LV2_ATOM__Sequence));
    }

    midi_processor_.set_output_capacity(default_capacity_);
}

// =============================================================================
// connect_port
//
// nullptr is a legitimate value: hosts use it to disconnect a port. It MUST
// be stored (run() null-checks every pointer); ignoring it leaves a dangling
// pointer that run() would keep writing through.
//
// Port ranges:
//   0..11  : scalar / master-toggle ports (handled by named cases).
//   12..27 : channel_1 .. channel_16 individual toggles.
//   28..155: cc_selection_0 .. cc_selection_127 individual toggles.
// =============================================================================
void MidiSmoothCCPlugin::connect_port(uint32_t port, void* data) {
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
        case PORT_ALL_CHANNELS:
            port_all_channels_ = static_cast<float*>(data);
            break;
        default:
            if (port >= PORT_CHANNEL_SEL_BASE && port < PORT_CC_SEL_BASE) {
                port_channel_selection_[port - PORT_CHANNEL_SEL_BASE] =
                    static_cast<float*>(data);
            } else if (port >= PORT_CC_SEL_BASE && port < PORT_TOTAL) {
                port_cc_selection_[port - PORT_CC_SEL_BASE] =
                    static_cast<float*>(data);
            }
            break;
    }
}

void MidiSmoothCCPlugin::activate() {
    current_frame_ = 0u;
    last_output_size_ = 0u;

    for (auto& state : states_) {
        state.last_update_frame = 0u;
    }
}

// =============================================================================
// run -- real-time callback
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

    params.enabled = (read_float(port_enabled_, 1.0f) > 0.5f);
    params.all_cc  = (read_float(port_all_cc_, 0.0f) > 0.5f);
    // all_channels defaults to 1.0 (true) — blacklist mode.
    params.all_channels = (read_float(port_all_channels_, 1.0f) > 0.5f);

    {
        const float mode_f = read_float(port_smoothing_mode_, 0.0f);
        params.smoothing_mode =
            (mode_f > 0.5f) ? SmoothingMode::Slew : SmoothingMode::Exponential;
    }

    params.attack_ms      = clamp_u32(read_float(port_attack_ms_,       30.0f),   1u,  1000u);
    params.release_ms     = clamp_u32(read_float(port_release_ms_,      25.0f),   1u,  1000u);
    params.rising_rate    = clamp_u32(read_float(port_rising_rate_,    200.0f),   1u, 10000u);
    params.falling_rate   = clamp_u32(read_float(port_falling_rate_,   500.0f),   1u, 10000u);
    params.deadband       = clamp_u32(read_float(port_deadband_,         5.0f),   0u,    20u);
    params.output_rate_hz = clamp_u32(read_float(port_output_rate_hz_, 250.0f),  10u,   500u);

    // Read the 16 per-channel toggles. Default: channel_1 (index 0) = 1.0,
    // channels 2-16 (indices 1..15) = 0.0. Combined with all_channels=true
    // (blacklist), this excludes Channel 1 and processes channels 2-16.
    for (uint32_t i = 0u; i < PORT_CHANNEL_SEL_COUNT; ++i) {
        const float default_val = (i == 0u) ? 1.0f : 0.0f;
        params.channel_selection[i] =
            (read_float(port_channel_selection_[i], default_val) > 0.5f);
    }

    for (uint32_t i = 0u; i < PORT_CC_SEL_COUNT; ++i) {
        const float default_val = (i == 7u) ? 1.0f : 0.0f;
        params.cc_selection[i] =
            (read_float(port_cc_selection_[i], default_val) > 0.5f);
    }

    if (port_midi_in_ != nullptr && port_midi_out_ != nullptr) {
        // Learn the real output capacity. Per the LV2 Atom spec the host sets
        // the output atom.size to the buffer capacity before run(). A value
        // equal to what we left after the previous cycle is stale (host did
        // not reset it) and is ignored.
        const uint32_t incoming = port_midi_out_->atom.size;
        if (incoming > sizeof(LV2_Atom_Sequence_Body) &&
            incoming != last_output_size_) {
            host_capacity_ = incoming;
        }
        midi_processor_.set_output_capacity(
            host_capacity_ != 0u ? host_capacity_ : default_capacity_);

        midi_processor_.process(
            port_midi_in_,
            port_midi_out_,
            sample_rate_,
            current_frame_,
            n_samples,
            params,
            states_);

        last_output_size_ = port_midi_out_->atom.size;
    }

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

    LV2_URID_Map* urid_map = nullptr;
    uint32_t      seq_size = 16384u;

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
    auto* plugin = static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->connect_port(port, data);
    }
}

static void activate(LV2_Handle instance) {
    auto* plugin = static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->activate();
    }
}

static void run(LV2_Handle instance, uint32_t n_samples) {
    auto* plugin = static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->run(n_samples);
    }
}

static void deactivate(LV2_Handle instance) {
    auto* plugin = static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->deactivate();
    }
}

static void cleanup(LV2_Handle instance) {
    delete static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
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
