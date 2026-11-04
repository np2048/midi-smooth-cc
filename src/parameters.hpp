// midi-smooth-cc/src/parameters.hpp

// =============================================================================
// MIDI Smooth CC - LV2 MIDI CC Smoothing Plugin
//
// Central declaration of all user-facing control port parameters:
//   - Parameter indices (must match the order of lv2:port entries in plugin.ttl)
//   - Default values (Section 12 of the specification)
//   - Valid ranges
//
// Real-time safety: all values here are plain POD / enums. They are read from
// LV2 control ports at the top of run() and must NOT require mutex protection.
// =============================================================================
#pragma once

#include <cstdint>

namespace midi_smooth_cc {

// TODO: Define enum class PortIndex { ... } matching plugin.ttl port order.
// TODO: Define struct ParameterDefaults with the values from Section 12.
// TODO: Define struct ParameterRanges with min/max/default per parameter.

} // namespace midi_smooth_cc
