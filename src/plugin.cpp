// midi-smooth-cc/src/plugin.cpp

// =============================================================================
// MIDI Smooth CC - LV2 plugin entry-point implementation
//
// Exports lv2_descriptor() and implements the standard LV2 lifecycle methods:
// instantiate / connect_port / activate / run / deactivate / cleanup /
// extension_data.
//
// The plugin URI MUST be "urn:midi-smooth-cc:plugin" and must match the
// subject of manifest.ttl.in and plugin.ttl.in exactly.
//
// Real-time safety:
//   1. No dynamic allocation in run() or any method it calls.
//   2. Use the modern LV2 Atom Sequence API (lv2_atom_sequence_*); never the
//      deprecated LV2_Event_Buffer.
//   3. No mutexes. Read control port values directly at the top of run().
// =============================================================================
#include "plugin.hpp"

// LV2 headers - included here, NOT in the public plugin.hpp, so the rest of
// the project does not transitively depend on LV2.
#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/urid/urid.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"
#include "lv2/lv2plug.in/ns/ext/atom/util.h"
#include "lv2/lv2plug.in/ns/ext/midi/midi.h"

namespace midi_smooth_cc {

// TODO: implement MidiSmoothCCPlugin methods here.

} // namespace midi_smooth_cc

// =============================================================================
// LV2 plugin descriptor export
// =============================================================================
extern "C" {

// The URI MUST match manifest.ttl.in and plugin.ttl.in exactly.
static const char* const kMidiSmoothCC_URI = "urn:midi-smooth-cc:plugin";

const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    // TODO: return &descriptor_singleton for index == 0, else nullptr.
    (void)index;
    return nullptr;
}

} // extern "C"
