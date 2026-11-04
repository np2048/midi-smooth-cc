# MIDI Smooth CC

## Master Software Development Specification

**Project Type:** Open Source MIDI Processing Plugin  
**Version:** 1.0  
**Target Platform:** Linux x86_64  
**Primary Host:** Carla  
**MIDI Environment:** PipeWire / JACK  
**Plugin Format:** LV2 MIDI Effect  
**Programming Language:** C++17  
**Build System:** CMake  
**License:** GNU GPL-3.0-or-later

---

# 1. PROJECT OVERVIEW

Develop a lightweight, reliable, real-time MIDI processing plugin named **MIDI Smooth CC**.

The primary purpose of this plugin is to smooth abrupt changes, noise, and quantization artifacts in incoming MIDI Control Change (CC) messages.

The plugin is particularly intended for Electronic Wind Instruments (EWI), breath controllers, expression controllers, and other MIDI devices that continuously transmit control values derived from analog sensors.

A typical use case involves an EWI transmitting MIDI CC7 (Channel Volume) continuously in response to breath pressure.

Due to sensor noise, limited sensor resolution, and quantization, the incoming MIDI values may fluctuate rapidly, particularly at low signal levels.

Example of noisy input:

```
0, 1, 4, 2, 7, 3, 9, 5, 12, 6, 15
```

The plugin must transform this stream into a smoother, temporally interpolated output while preserving intentional musical expression and the overall dynamic envelope.

This must be a genuine time-based MIDI smoothing processor, NOT merely a MIDI event filter, CC remapper, or message-rate limiter.

The plugin must be universal and configurable so that users can selectively smooth any MIDI CC number on any MIDI channel.

# 2. CORE REQUIREMENTS

The plugin must:

1. Receive incoming MIDI events.
    
2. Identify MIDI Control Change messages.
    
3. Allow selective processing of MIDI channels 1–16.
    
4. Allow selective processing of CC numbers 0–127.
    
5. Smooth selected CC values over time.
    
6. Generate intermediate MIDI CC values between incoming events.
    
7. Preserve the original MIDI channel and CC number.
    
8. Pass all non-selected MIDI events through unchanged.
    
9. Support independent smoothing states for every MIDI channel and CC number.
    
10. Operate in real time with low latency.
    
11. Avoid dynamic memory allocation and blocking operations in the real-time processing path.
    
12. Work reliably inside Carla on Linux.
    

# 3. PLUGIN ARCHITECTURE

Implement the plugin as a native LV2 MIDI Effect.

Use the standard LV2 MIDI event interface and Atom Sequence processing.

The plugin must expose:

- One MIDI input port.
    
- One MIDI output port.
    
- Control ports for all user-configurable parameters.
    

The plugin must not require an audio input or output port.

Use standard LV2 extensions wherever appropriate.

The plugin must be discoverable by Carla through the normal LV2 plugin scanning mechanism.

# 4. MIDI CHANNEL SELECTION

Provide a selectable channel mask containing 16 independent channel switches.

Each channel must be independently configurable.

The user must be able to:

- Enable channel 1 only.
    
- Enable channels 1 and 2.
    
- Enable any arbitrary combination of channels.
    
- Enable all 16 channels.
    
- Disable all channels.
    

Unselected channels must pass through unchanged.

MIDI channel numbering in the user interface must be 1–16.

Internally, MIDI channel numbering must follow the MIDI specification (0–15).

# 5. MIDI CC SELECTION

Provide selective CC processing.

The user must be able to:

- Select individual CC numbers.
    
- Select multiple CC numbers.
    
- Enable ALL CC messages.
    
- Disable individual CC numbers while ALL mode is active.
    

CC numbers range from 0 to 127.

The plugin must preserve the original CC number.

The plugin must preserve the original MIDI channel.

# 6. SMOOTHING ALGORITHMS

Implement at least two independent smoothing algorithms.

## 6.1 Exponential Smoothing

Implement a time-aware first-order exponential low-pass filter.

Mathematical definition:

```
alpha = 1 - exp(-dt / tau)

y = y_previous + alpha * (x - y_previous)
```

Where:

- x = latest incoming MIDI CC value.
    
- y = filtered MIDI CC value.
    
- dt = elapsed time.
    
- tau = smoothing time constant.
    
- y_previous = previous filtered value.
    

The implementation must use elapsed time rather than assuming a fixed MIDI event rate.

This is essential because different MIDI controllers transmit CC messages at different rates.

Attack and Release must be independently configurable.

Attack controls the response when the target value increases.

Release controls the response when the target value decreases.

Recommended parameter ranges:

- Attack: 1–1000 ms.
    
- Release: 1–1000 ms.
    

Provide sensible default values suitable for breath-controller operation.

## 6.2 Slew Rate Limiter

Implement a second algorithm that limits the maximum rate of change.

Parameters:

- Rising rate: configurable CC units per second.
    
- Falling rate: configurable CC units per second.
    

The algorithm must independently control increasing and decreasing CC values.

The filtered value must never exceed the permitted rate of change.

This mode is particularly useful for suppressing abrupt sensor spikes.

# 7. DEADBAND

Implement an optional deadband function.

The deadband parameter defines the minimum difference between the current filtered value and a new target value that is required before updating the filter target.

Parameter range:

0–10 MIDI CC units.

Default: 0 (disabled).

Deadband must be applied carefully so that small but intentional changes in breath pressure are not permanently lost.

The final output must still converge to the latest valid target value.

# 8. OUTPUT EVENT GENERATION

This is a critical component of the plugin.

The plugin must generate intermediate MIDI CC events over time.

It must NOT simply forward incoming events at a reduced frequency.

Provide a configurable output update rate:

- Minimum: 20 Hz.
    
- Default: 100 Hz.
    
- Maximum: 500 Hz.
    

At each output interval:

1. Calculate the current filtered value.
    
2. Convert it to an integer MIDI CC value in the range 0–127.
    
3. Compare it with the last transmitted value.
    
4. Send a MIDI CC event only if the integer value has changed.
    

Avoid redundant MIDI messages.

When the input stops changing, the output must continue updating until the filtered value converges to the target.

The plugin must not introduce unnecessary latency beyond the selected smoothing parameters and output update interval.

# 9. INDEPENDENT FILTER STATES

Maintain independent filter states for all 2048 possible combinations:

16 MIDI channels × 128 CC numbers.

Each state must contain at least:

- Current filtered value.
    
- Current target value.
    
- Last transmitted integer value.
    
- Last update timestamp.
    
- Filter initialization status.
    

One channel's CC value must never affect another channel or CC number.

# 10. MIDI EVENT PRESERVATION

All non-selected MIDI messages must pass through unchanged.

This includes:

- Note On.
    
- Note Off.
    
- Polyphonic Aftertouch.
    
- Channel Pressure.
    
- Pitch Bend.
    
- Program Change.
    
- MIDI Clock.
    
- System Real-Time messages.
    
- System Exclusive messages.
    

The plugin must preserve the original MIDI channel and event data.

Note On and Note Off events must never be delayed by the smoothing algorithm.

Unselected CC messages must also pass through unchanged.

# 11. REAL-TIME SAFETY

The plugin must be suitable for real-time MIDI processing.

The processing callback must:

- Avoid dynamic memory allocation.
    
- Avoid mutexes and blocking locks.
    
- Avoid filesystem operations.
    
- Avoid logging to standard output.
    
- Avoid unpredictable execution time.
    
- Use preallocated event buffers where necessary.
    

The implementation must handle variable processing block sizes.

It must correctly handle MIDI event timestamps within LV2 Atom Sequences.

Output events must be timestamped correctly relative to the current processing block.

Handle output buffer overflow safely.

Never corrupt MIDI event streams if an output buffer becomes full.

# 12. PLUGIN PARAMETERS

Expose the following parameters:

|Parameter|Range|Default|
|---|---|---|
|Enabled|On / Off|On|
|Smoothing Mode|Exponential / Slew|Exponential|
|Attack|1–1000 ms|50 ms|
|Release|1–1000 ms|100 ms|
|Rising Rate|1–10000 CC/s|500 CC/s|
|Falling Rate|1–10000 CC/s|500 CC/s|
|Deadband|0–10|0|
|Output Rate|20–500 Hz|100 Hz|
|All CC|On / Off|Off|
|MIDI Channel Mask|16 switches|All enabled|
|CC Selection|128 switches|CC7 enabled|

The initial default configuration must process CC7 on all MIDI channels.

All other CC numbers must be disabled by default.

# 13. USER INTERFACE

Version 1.0 should prioritize compatibility and reliability over visual complexity.

Use standard LV2 control ports and provide appropriate metadata for host-generated controls.

The interface should organize parameters into the following groups:

1. MIDI Channel Selection.
    
2. MIDI CC Selection.
    
3. Smoothing Algorithm.
    
4. Timing Parameters.
    
5. Deadband.
    
6. Output Rate.
    
7. Bypass / Enable.
    

A custom graphical interface is optional and should NOT delay delivery of the first functional version.

# 14. INITIALIZATION AND RESET

When a selected CC is first received:

- Initialize its filter state from the incoming value.
    
- Avoid generating an artificial ramp from zero unless explicitly configured.
    

When bypass is activated:

- Pass MIDI events through unchanged.
    

When bypass is deactivated:

- Resume smoothing without producing unexpected value jumps.
    

When parameters change:

- Update the relevant filter state safely.
    

# 15. TESTING REQUIREMENTS

Provide automated tests for:

1. Constant CC input.
    
2. Rapidly fluctuating CC input.
    
3. Slowly increasing CC input.
    
4. Slowly decreasing CC input.
    
5. Sudden CC spikes.
    
6. Values fluctuating around zero.
    
7. Independent MIDI channels.
    
8. Independent CC numbers.
    
9. Unselected CC pass-through.
    
10. Note On / Note Off preservation.
    
11. Pitch Bend preservation.
    
12. Different MIDI event rates.
    
13. Variable processing block sizes.
    
14. Output event buffer overflow.
    
15. Bypass behavior.
    
16. Filter initialization.
    
17. Final convergence to target values.
    

Include tests verifying that the output never exceeds the MIDI CC range of 0–127.

# 16. BUILD AND INSTALLATION

Provide a CMake-based build system.

The project must compile natively on Linux x86_64.

Document dependencies and installation procedures for:

- Arch Linux.
    
- Manjaro Linux.
    
- Other compatible Linux distributions.
    

Provide commands for:

- Installing dependencies.
    
- Building the plugin.
    
- Running tests.
    
- Installing the LV2 bundle.
    
- Refreshing or rescanning plugins in Carla.
    

The resulting LV2 bundle must contain:

- plugin binary.
    
- manifest.ttl.
    
- plugin.ttl.
    
- Any additional required metadata.
    

The plugin must be installable into:

```
~/.lv2/
```

# 17. SOURCE CODE ORGANIZATION

Use a clean and maintainable project structure.

Suggested layout:

```
midi-smooth-cc/
├── CMakeLists.txt
├── README.md
├── LICENSE
├── CHANGELOG.md
├── .gitignore
├── src/
│   ├── plugin.cpp
│   ├── plugin.hpp
│   ├── midi_processor.cpp
│   ├── midi_processor.hpp
│   ├── smoothing.cpp
│   ├── smoothing.hpp
│   ├── filter_state.hpp
│   └── parameters.hpp
├── lv2/
│   ├── manifest.ttl.in
│   └── plugin.ttl.in
├── tests/
│   ├── test_smoothing.cpp
│   ├── test_midi.cpp
│   └── test_channels.cpp
└── docs/
    ├── INSTALL.md
    ├── USER_GUIDE.md
    └── ALGORITHM.md
```

# 18. DOCUMENTATION

The README must explain:

- What MIDI Smooth CC does.
    
- Why breath controllers can benefit from MIDI smoothing.
    
- How exponential smoothing works.
    
- How slew-rate limiting works.
    
- How to select channels and CC numbers.
    
- How to configure Attack and Release.
    
- How to install the plugin.
    
- How to load it in Carla.
    
- How to connect it using PipeWire.
    
- How to troubleshoot common issues.
    

Include example configurations for EWI users.

# 19. OPEN SOURCE REQUIREMENTS

Publish the project under GNU GPL-3.0-or-later.

The GitHub repository must include:

- Complete source code.
    
- Build instructions.
    
- Installation instructions.
    
- License.
    
- Changelog.
    
- Issue reporting guidelines.
    
- Contribution guidelines.
    

Do not include proprietary dependencies or undocumented third-party source code.

# 20. DEVELOPMENT PRIORITIES

Development must follow these priorities:

**Priority 1 — Functional correctness**

A working LV2 MIDI plugin with exponential smoothing, independent CC/channel states, MIDI event preservation, and configurable output rate.

**Priority 2 — Advanced processing**

Add slew-rate limiting, deadband, and advanced channel/CC selection.

**Priority 3 — User experience**

Improve host parameter presentation and add a custom GUI if necessary.

**Priority 4 — Distribution**

Provide tested installation packages or reproducible build instructions for Arch Linux and Manjaro.

# 21. DEVELOPMENT INSTRUCTIONS FOR THE AI CODING AGENT

Before generating code:

1. Review the complete specification.
    
2. Identify all technical dependencies.
    
3. Verify LV2 API compatibility.
    
4. Design the internal event-processing architecture.
    
5. Identify potential real-time safety problems.
    
6. Explain any necessary design decisions.
    

Then implement the project incrementally.

Do not generate placeholder implementations.

Do not silently omit required functionality.

Do not claim that the plugin is compatible with Carla until the resulting LV2 bundle has been tested in Carla.

Do not claim that the plugin is real-time safe without reviewing its processing path.

Compile the project and run automated tests.

Fix all compiler errors and test failures before declaring a milestone complete.

When generating source files, provide the complete contents of every file and maintain consistency between files.

The final result must be a functional, buildable, documented Linux LV2 MIDI plugin.

---

### ⚠️ Golden Rules for Prompting LLMs for LV2/Real-Time Code

When working with LLMs on this specific project, you must enforce these rules, or the LLM will write code that crashes Carla:

1. **Ban Dynamic Allocation in the Audio Thread:** LLMs love using `std::vector::push_back()` or `new`. You must explicitly remind it: _"Do not use dynamic memory allocation in the `run()` method or any method called by it."_
2. **Force the Atom Sequence API:** Older LV2 tutorials use the deprecated `LV2_Event_Buffer`. You must specify: _"Use the modern LV2 Atom Sequence API (`lv2_atom_sequence_*`), not the deprecated MIDI event API."_
3. **Thread Safety for Parameters:** LLMs will often put a `std::mutex` around parameter updates. In LV2, the host calls `connect_port` right before `run`. You don't need mutexes if you use atomic variables or if you only read the parameters at the start of the `run()` block. Tell the LLM: _"Do not use mutexes. Read control port values directly at the start of the `run()` cycle."_
4. **Context Window Limits:** If the LLM starts forgetting the spec, remind it of the specific section. (e.g., _"Refer back to Section 6.1 for the exact exponential smoothing formula."_)
5. **Carla Specifics:** Carla is strict about LV2 compliance. If Carla refuses to load the plugin, it's usually a TTL syntax error or a missing required port. Ask the LLM: _"Carla is rejecting the plugin. Review the `plugin.ttl` for missing `lv2:requiredFeature` or incorrect port indices."_

