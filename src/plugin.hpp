// midi-smooth-cc/src/plugin.hpp

// =============================================================================
// MIDI Smooth CC - LV2 plugin entry-point class
//
// Declares the C++ class that backs the LV2 plugin descriptor:
//
//   - instantiate()       : allocate the plugin instance + all 2048 filter
//                           states + cache URID map feature.
//   - connect_port()      : store port connection pointers (called by host
//                           immediately before each run()).
//   - activate()          : (re)initialize filter states and timing.
//   - run()               : THE real-time callback. Top of run() reads all
//                           control port values into local copies; then calls
//                           MidiProcessor::process().
//   - deactivate()        : no-op or light cleanup.
//   - cleanup()           : free the instance (the ONLY place deletion is OK).
//   - extension_data()    : return any LV2 extension data (e.g. state interface).
//
// The plugin URI string MUST match manifest.ttl.in and plugin.ttl.in:
//     "urn:midi-smooth-cc:plugin"
//
// Real-time safety: no mutexes. Parameters are read at the top of run() and
// stored in local stack copies, then used throughout the block. This is the
// LV2-idiomatic pattern for thread-safe parameter handling without locks.
// =============================================================================
#pragma once

#include <cstdint>

// Forward-declare LV2 handle types so this header stays usable without
// pulling in the full LV2 C headers in every consumer.
struct LV2_Handle;
struct LV2_Descriptor;
struct LV2_Feature;
struct LV2_URID_Map;

namespace midi_smooth_cc {

// TODO: class MidiSmoothCCPlugin { ... };

} // namespace midi_smooth_cc

// TODO: extern "C" const LV2_Descriptor* lv2_descriptor(uint32_t index);
