# midi-smooth-cc/docs/USER_GUIDE.md

# User Guide

## What MIDI Smooth CC Does

MIDI Smooth CC is a real-time LV2 MIDI effect that smooths noisy or abruptly
changing MIDI Control Change (CC) values. It is primarily designed for
Electronic Wind Instruments (EWI), breath controllers, and other continuous
MIDI controllers whose sensor-derived values fluctuate rapidly.

## Why Breath Controllers Benefit

EWIs and breath controllers transmit CC values (commonly CC7 Volume or CC11
Expression) derived from analog pressure sensors. These sensors exhibit:

- Quantization artifacts at low signal levels.
- Noise from physical vibration and air turbulence.
- Inconsistent output rates between devices.

Smoothing produces a musically natural output without losing intentional
expression.

## Loading in Carla

1. Install the plugin (see [INSTALL.md](INSTALL.md)).
2. Open Carla, rescan plugins.
3. Add `MIDI Smooth CC` as a MIDI plugin.
4. Connect: source MIDI device -> MIDI Smooth CC input -> destination synth.

## PipeWire Connection Example

```bash
# List MIDI ports
pw-link -o
pw-link -i

# Connect EWI to plugin, plugin to synthesizer
pw-link "EWI:midi_out" "MIDI Smooth CC:midi_in"
pw-link "MIDI Smooth CC:midi_out" "ZynAddSubFX:midi_in"
```

## Example EWI Configuration

(to be expanded)

## Troubleshooting

(to be expanded)
