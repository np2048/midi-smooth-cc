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
// =============================================================================
#include "plugin.hpp"

#include <cstring>

#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"

namespace midi_smooth_cc {

// =============================================================================
// Constructor
//
// Called once by the extern "C" instantiate() wrapper (host thread, non-
// real-time). Heap allocation is permitted here.
// =============================================================================
MidiSmoothCCPlugin::MidiSmoothCCPlugin(double sample_rate, LV2_URID_Map* urid_map)
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

    // Set a default output buffer capacity. The host typically allocates
    // at least 16 KB for MIDI atom sequence output ports; if the host
    // provides a different size via the buf-size#sequenceSize option, a
    // future version of this plugin could query it here. For v1.0, the
    // default is safe and sufficient.
    midi_processor_.set_output_capacity(16384u);
}

// =============================================================================
// connect_port
//
// Called by the host (non-real-time) to connect a port buffer to the
// plugin. The host calls this before run() for every port, and may call
// it again between run() cycles if the buffer changes.
// =============================================================================
void MidiSmoothCCPlugin::connect_port(uint32_t port, void* data) {
    // A null data pointer means the host disconnected the port. Leave the
    // stored pointer unchanged (it remains nullptr from construction or
    // retains its last value); run() will use safe defaults for null ports.
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
            // CC selection ports [12 .. 139].
            if (port >= PORT_CC_SEL_BASE && port < PORT_TOTAL) {
                port_cc_selection_[port - PORT_CC_SEL_BASE] =
                    static_cast<float*>(data);
            }
            // Unknown port index: ignore silently (forward-compatible).
            break;
    }
}

// =============================================================================
// activate
//
// Called by the host (non-real-time) before the first run() of a session,
// and again after any deactivate(). No heap allocation here.
// =============================================================================
void MidiSmoothCCPlugin::activate() {
    // Reset the global frame counter so block-relative timing starts clean.
    current_frame_ = 0u;

    // Re-align per-state timing. After a deactivate/activate cycle, the
    // FilterState entries may still hold stale last_update_frame values
    // from the previous session. If we did not reset them, the output
    // loop's next_tick computation would point far into the future
    // (relative to the reset current_frame_ = 0), and no ticks would fire
    // until current_frame_ caught up — potentially seconds of silence.
    //
    // We reset ONLY last_update_frame, preserving is_initialized,
    // current_value, target_value, and last_transmitted_value so the
    // plugin resumes smoothly from where it left off (spec §14: "Resume
    // smoothing without producing unexpected value jumps"). The filter
    // continues converging to its preserved target on the next block.
    //
    // At initial activation (first call after instantiate), states_ is
    // already default-constructed (all last_update_frame == 0), so this
    // loop is a no-op write of 0 over 0.
    for (auto& state : states_) {
        state.last_update_frame = 0u;
    }
}

// =============================================================================
// run — THE CRITICAL REAL-TIME CALLBACK
//
// Called by the host once per processing block (audio/MIDI thread).
// MUST NOT allocate, lock, log, or perform any blocking operation.
// =============================================================================
void MidiSmoothCCPlugin::run(uint32_t n_samples) {
    // -------------------------------------------------------------------
    // Step 1: Snapshot all control port values into a local
    // PluginParameters. This is the LV2-idiomatic pattern for lock-free
    // parameter handling (Golden Rule 3): read once, use throughout.
    // -------------------------------------------------------------------

    // Helper: safely read a float from a possibly-null port pointer.
    const auto read_float = [](float* port, float default_val) -> float {
        return (port != nullptr) ? *port : default_val;
    };

    // Helper: clamp a float to a uint32_t range (spec §12).
    const auto clamp_u32 = [](float v, uint32_t lo, uint32_t hi) -> uint32_t {
        const float flo = static_cast<float>(lo);
        const float fhi = static_cast<float>(hi);
        if (v < flo) return lo;
        if (v > fhi) return hi;
        return static_cast<uint32_t>(v);
    };

    PluginParameters params;

    // Boolean toggles: > 0.5 is ON.
    params.enabled =
        (read_float(port_enabled_, 1.0f) > 0.5f);
    params.all_cc =
        (read_float(port_all_cc_, 0.0f) > 0.5f);

    // Smoothing mode: 0.0 = Exponential, 1.0 = Slew.
    {
        const float mode_f = read_float(port_smoothing_mode_, 0.0f);
        params.smoothing_mode =
            (mode_f > 0.5f) ? SmoothingMode::Slew : SmoothingMode::Exponential;
    }

    // Time/rate parameters with spec §12 range clamping.
    params.attack_ms      = clamp_u32(read_float(port_attack_ms_,       50.0f),   1u,  1000u);
    params.release_ms     = clamp_u32(read_float(port_release_ms_,     100.0f),   1u,  1000u);
    params.rising_rate    = clamp_u32(read_float(port_rising_rate_,    500.0f),   1u, 10000u);
    params.falling_rate   = clamp_u32(read_float(port_falling_rate_,   500.0f),   1u, 10000u);
    params.deadband       = clamp_u32(read_float(port_deadband_,         0.0f),   0u,    10u);
    params.output_rate_hz = clamp_u32(read_float(port_output_rate_hz_, 100.0f),  20u,   500u);

    // Channel mask: 16-bit bitmask encoded as a float (0x0000 .. 0xFFFF).
    // float can represent all integers up to 2^24 exactly, so 16-bit
    // values round-trip without precision loss.
    {
        const float mask_f = read_float(port_channel_mask_, 65535.0f); // 0xFFFF
        params.channel_mask = static_cast<uint16_t>(mask_f);
    }

    // CC selection: 128 individual float ports → bool array.
    // Default (unconnected port) is CC7 enabled, all others disabled,
    // matching spec §12.
    for (uint32_t i = 0u; i < PORT_CC_SEL_COUNT; ++i) {
        const float default_val = (i == 7u) ? 1.0f : 0.0f;
        params.cc_selection[i] =
            (read_float(port_cc_selection_[i], default_val) > 0.5f);
    }

    // -------------------------------------------------------------------
    // Step 2: Delegate to MidiProcessor.
    //
    // current_frame_ is uint64_t to avoid overflow across long sessions
    // (at 48 kHz, uint32_t overflows after ~24 hours). MidiProcessor::
    // process() takes uint32_t, so we truncate. This is safe because all
    // frame arithmetic inside MidiProcessor is relative (block offsets,
    // dt calculations), and FilterState::last_update_frame is uint64_t.
    // The only consequence of uint32_t wraparound is that, once every
    // ~24 hours, a single block may produce no output while the catch-up
    // logic re-aligns — acceptable for v1.0.
    // -------------------------------------------------------------------
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
    // If either MIDI port is null, skip processing but still advance
    // current_frame_ so the global frame counter stays in sync with the
    // host (prevents a stall if the host connects ports mid-session).

    // -------------------------------------------------------------------
    // Step 3: Advance the global frame counter.
    // -------------------------------------------------------------------
    current_frame_ += static_cast<uint64_t>(n_samples);
}

} // namespace midi_smooth_cc

// =============================================================================
// LV2 C interface
//
// The host locates `lv2_descriptor` via dlsym() after loading the shared
// library identified by manifest.ttl. All functions below have C linkage
// to ensure stable ABI across compilers and host implementations.
// =============================================================================
extern "C" {

// The plugin URI. MUST match manifest.ttl.in and plugin.ttl.in exactly.
static const char* const kMidiSmoothCC_URI = "urn:midi-smooth-cc:plugin";

// -------------------------------------------------------------------------
// instantiate — create a new plugin instance (non-real-time).
// -------------------------------------------------------------------------
static LV2_Handle instantiate(const LV2_Descriptor*     descriptor,
                              double                    sample_rate,
                              const char*               bundle_path,
                              const LV2_Feature* const* features) {
    (void)descriptor;    // unused — we only export one plugin
    (void)bundle_path;   // unused — no bundle resources needed

    // Find the urid:map feature (required, declared in plugin.ttl.in).
    LV2_URID_Map* urid_map = nullptr;
    if (features != nullptr) {
        for (const LV2_Feature* const* fp = features; *fp != nullptr; ++fp) {
            if (std::strcmp((*fp)->URI, LV2_URID__map) == 0) {
                urid_map = static_cast<LV2_URID_Map*>((*fp)->data);
                break;
            }
        }
    }
    if (urid_map == nullptr) {
        // Required feature not provided by the host. Refuse to instantiate.
        return nullptr;
    }

    // Allocate the plugin instance. This is the ONLY place `new` is used.
    // Wrap in try/catch so a failed construction returns nullptr instead
    // of propagating a C++ exception across the C ABI boundary (which
    // would be undefined behavior).
    try {
        auto* plugin =
            new midi_smooth_cc::MidiSmoothCCPlugin(sample_rate, urid_map);
        return static_cast<LV2_Handle>(plugin);
    } catch (...) {
        return nullptr;
    }
}

// -------------------------------------------------------------------------
// connect_port — store a port buffer pointer (non-real-time).
// -------------------------------------------------------------------------
static void connect_port(LV2_Handle instance, uint32_t port, void* data) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->connect_port(port, data);
    }
}

// -------------------------------------------------------------------------
// activate — prepare for run() (non-real-time).
// -------------------------------------------------------------------------
static void activate(LV2_Handle instance) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->activate();
    }
}

// -------------------------------------------------------------------------
// run — process one block of audio/MIDI (real-time).
// -------------------------------------------------------------------------
static void run(LV2_Handle instance, uint32_t n_samples) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->run(n_samples);
    }
}

// -------------------------------------------------------------------------
// deactivate — end of a run session (non-real-time).
// -------------------------------------------------------------------------
static void deactivate(LV2_Handle instance) {
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    if (plugin != nullptr) {
        plugin->deactivate();
    }
}

// -------------------------------------------------------------------------
// cleanup — destroy the plugin instance (non-real-time).
// -------------------------------------------------------------------------
static void cleanup(LV2_Handle instance) {
    // The ONLY place `delete` is used. Safe on nullptr (delete is a no-op).
    auto* plugin =
        static_cast<midi_smooth_cc::MidiSmoothCCPlugin*>(instance);
    delete plugin;
}

// -------------------------------------------------------------------------
// extension_data — return extension interfaces by URI.
//
// v1.0 returns nullptr for all queries (no state, worker, or options
// interfaces yet). A future version could implement LV2_State__interface
// for parameter persistence and LV2_Options__interface for buffer-size
// discovery.
// -------------------------------------------------------------------------
static const void* extension_data(const char* uri) {
    (void)uri;
    return nullptr;
}

// -------------------------------------------------------------------------
// The static descriptor. The host reads this via lv2_descriptor(0).
// -------------------------------------------------------------------------
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

// -------------------------------------------------------------------------
// lv2_descriptor — the single exported entry point.
//
// Returns the plugin descriptor for index 0, nullptr for any other index
// (this library exports exactly one plugin).
// -------------------------------------------------------------------------
const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    if (index == 0u) {
        return &kMidiSmoothCC_Descriptor;
    }
    return nullptr;
}

} // extern "C"
