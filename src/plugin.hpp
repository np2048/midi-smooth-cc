// midi-smooth-cc/src/plugin.hpp

// =============================================================================
// MIDI Smooth CC - LV2 plugin entry-point class
//
// Declares the C++ class that backs the LV2 plugin descriptor. The host
// never instantiates this class directly; instead, the extern "C"
// `lv2_descriptor()` returns a static LV2_Descriptor whose `instantiate`
// callback creates a MidiSmoothCCPlugin via `new` and returns it as an
// LV2_Handle (opaque void*).
//
// Lifecycle (all orchestrated by the host):
//   instantiate()       : new MidiSmoothCCPlugin(...). Non-real-time.
//                         Heap allocation is permitted here.
//   connect_port()      : store port connection pointers. Called by the
//                         host immediately before each run() cycle.
//   activate()          : reset timing state. Non-real-time.
//   run()               : THE real-time callback. Top of run() reads all
//                         control port values into a local PluginParameters
//                         snapshot (no mutexes — Golden Rule 3), then
//                         calls MidiProcessor::process().
//   deactivate()        : no-op.
//   cleanup()           : delete the instance. The ONLY place deletion
//                         is permitted.
//   extension_data()    : return nullptr for v1.0 (no state/worker/opts
//                         interfaces yet).
//
// Port layout (indices MUST match lv2/plugin.ttl.in):
//   Index 0   : MIDI input  (atom:Sequence, input)
//   Index 1   : MIDI output (atom:Sequence, output)
//   Index 2   : enabled             (ControlPort, input, bool)
//   Index 3   : smoothing_mode      (ControlPort, input, 0=Exp 1=Slew)
//   Index 4   : attack_ms           (ControlPort, input, 1..1000)
//   Index 5   : release_ms          (ControlPort, input, 1..1000)
//   Index 6   : rising_rate         (ControlPort, input, 1..10000 CC/s)
//   Index 7   : falling_rate        (ControlPort, input, 1..10000 CC/s)
//   Index 8   : deadband            (ControlPort, input, 0..10)
//   Index 9   : output_rate_hz      (ControlPort, input, 20..500)
//   Index 10  : all_cc              (ControlPort, input, bool)
//   Index 11  : channel_mask        (ControlPort, input, 16-bit bitmask)
//   Index 12..139 : cc_selection[0..127] (ControlPort × 128, bool each)
//   Total: 140 ports.
//
// Real-time safety: no mutexes. Parameters are read at the top of run()
// and stored in a local stack copy, then used throughout the block. This
// is the LV2-idiomatic pattern for thread-safe parameter handling without
// locks (Golden Rule 3). No dynamic allocation occurs in run(), activate(),
// or deactivate().
//
// The plugin URI string MUST match manifest.ttl.in and plugin.ttl.in:
//     "urn:midi-smooth-cc:plugin"
// =============================================================================
#pragma once

#include <array>
#include <cstdint>

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"

#include "midi_processor.hpp"
#include "parameters.hpp"
#include "filter_state.hpp"

namespace midi_smooth_cc {

class MidiSmoothCCPlugin {
public:
    // ---------------------------------------------------------------------
    // Construction / destruction.
    //
    // Called by the extern "C" instantiate() wrapper (non-real-time, host
    // thread). Heap allocation is permitted in the constructor.
    //
    //   sample_rate : host sample rate in Hz (e.g., 44100, 48000).
    //   urid_map    : host-provided LV2_URID_Map feature, used to resolve
    //                 the midi:MidiEvent URI to an integer URID.
    // ---------------------------------------------------------------------
    MidiSmoothCCPlugin(double sample_rate, LV2_URID_Map* urid_map);

    // Not copyable or movable — the host holds a raw pointer.
    MidiSmoothCCPlugin(const MidiSmoothCCPlugin&)            = delete;
    MidiSmoothCCPlugin& operator=(const MidiSmoothCCPlugin&) = delete;
    MidiSmoothCCPlugin(MidiSmoothCCPlugin&&)                 = delete;
    MidiSmoothCCPlugin& operator=(MidiSmoothCCPlugin&&)      = delete;

    // Default destructor (non-real-time; called from cleanup() via `delete`).
    ~MidiSmoothCCPlugin() = default;

    // ---------------------------------------------------------------------
    // LV2 lifecycle methods (called by the extern "C" wrappers).
    // ---------------------------------------------------------------------

    // Store a port connection pointer. Called by the host before run().
    // Non-real-time. Safe to call with data == nullptr (port left
    // unconnected, run() will use a safe default).
    void connect_port(uint32_t port, void* data);

    // Prepare the plugin for run(). Resets current_frame_ to 0 and
    // re-aligns per-state timing so the output loop does not stall after
    // a deactivate/activate cycle. Non-real-time; no heap allocation.
    void activate();

    // THE real-time callback. Reads all control ports into a local
    // PluginParameters snapshot at the top (no mutexes), delegates to
    // MidiProcessor::process(), then advances current_frame_.
    void run(uint32_t n_samples);

    // No-op. The host calls this after the last run() in a session.
    void deactivate() {}

    // ---------------------------------------------------------------------
    // Port index constants. These MUST match the port declarations in
    // lv2/plugin.ttl.in exactly, and MUST match the order in which
    // connect_port() expects them.
    // ---------------------------------------------------------------------
    static constexpr uint32_t PORT_MIDI_IN         = 0u;
    static constexpr uint32_t PORT_MIDI_OUT        = 1u;
    static constexpr uint32_t PORT_ENABLED         = 2u;
    static constexpr uint32_t PORT_SMOOTHING_MODE  = 3u;
    static constexpr uint32_t PORT_ATTACK_MS       = 4u;
    static constexpr uint32_t PORT_RELEASE_MS      = 5u;
    static constexpr uint32_t PORT_RISING_RATE     = 6u;
    static constexpr uint32_t PORT_FALLING_RATE    = 7u;
    static constexpr uint32_t PORT_DEADBAND        = 8u;
    static constexpr uint32_t PORT_OUTPUT_RATE_HZ  = 9u;
    static constexpr uint32_t PORT_ALL_CC          = 10u;
    static constexpr uint32_t PORT_CHANNEL_MASK    = 11u;
    static constexpr uint32_t PORT_CC_SEL_BASE     = 12u;
    static constexpr uint32_t PORT_CC_SEL_COUNT    = 128u;
    static constexpr uint32_t PORT_TOTAL           = PORT_CC_SEL_BASE + PORT_CC_SEL_COUNT; // 140

private:
    // ---------------------------------------------------------------------
    // Core processing engine and per-(channel, CC) filter state array.
    //
    // states_ is 16 × 128 = 2048 entries, allocated once at construction
    // time (on the heap, as part of the plugin instance) and reused across
    // every run() call. No per-run allocation occurs.
    // ---------------------------------------------------------------------
    MidiProcessor                 midi_processor_;
    std::array<FilterState, 2048> states_;

    // ---------------------------------------------------------------------
    // MIDI atom sequence ports.
    // ---------------------------------------------------------------------
    const LV2_Atom_Sequence* port_midi_in_  = nullptr;
    LV2_Atom_Sequence*       port_midi_out_ = nullptr;

    // ---------------------------------------------------------------------
    // Control ports (float*). The host writes float values to these
    // buffers before calling run(); run() reads them once at the top.
    //
    // All pointers default to nullptr. run() uses safe default values
    // for any port that remains unconnected.
    // ---------------------------------------------------------------------
    float* port_enabled_         = nullptr;
    float* port_smoothing_mode_  = nullptr;  // 0.0 = Exponential, 1.0 = Slew
    float* port_attack_ms_       = nullptr;
    float* port_release_ms_      = nullptr;
    float* port_rising_rate_     = nullptr;
    float* port_falling_rate_    = nullptr;
    float* port_deadband_        = nullptr;
    float* port_output_rate_hz_  = nullptr;
    float* port_all_cc_          = nullptr;
    float* port_channel_mask_    = nullptr;  // 16-bit bitmask as float

    // 128 individual CC-selection toggle ports (one float per CC number).
    // Zero-initialized to nullptr; run() defaults CC7 to enabled, all
    // others to disabled, if a port is unconnected.
    float* port_cc_selection_[PORT_CC_SEL_COUNT] = {};

    // ---------------------------------------------------------------------
    // Plugin global state.
    // ---------------------------------------------------------------------
    uint32_t sample_rate_   = 44100u;   // Hz, from instantiate()
    uint64_t current_frame_ = 0u;       // running frame count across run() calls
};

} // namespace midi_smooth_cc

// =============================================================================
// LV2 C entry point (defined in plugin.cpp). The host locates this via
// dlsym() on the shared library after reading manifest.ttl.
// =============================================================================
extern "C" const LV2_Descriptor* lv2_descriptor(uint32_t index);
