# midi-smooth-cc/docs/ALGORITHM.md

# Algorithm Reference

## 1. Exponential Smoothing

First-order time-aware low-pass filter (Section 6.1 of the spec):

```
alpha = 1 - exp(-dt / tau)
y     = y_prev + alpha * (x - y_prev)
```

- `x`       : latest incoming MIDI CC value.
- `y_prev`  : previous filtered value.
- `dt`      : elapsed time since the last update for this filter state.
- `tau`     : time constant. `tau_attack` for rising targets,
              `tau_release` for falling targets.

The implementation MUST use elapsed time rather than assuming a fixed event
rate, because different MIDI controllers transmit at different rates.

## 2. Slew Rate Limiter

Caps the maximum rate of change in CC units per second (Section 6.2):

```
max_delta = rate * dt
y         = clamp(x, y_prev - falling_max_delta, y_prev + rising_max_delta)
```

Useful for suppressing sudden sensor spikes while still tracking slow musical
envelopes.

## 3. Deadband

Optional minimum threshold below which a target change is ignored
(Section 7). Range 0..10 CC units, default 0 (disabled).

The deadband is applied to the target value, not the filtered value, so that
the filter still converges to the latest valid target without losing small
intentional changes permanently.

## 4. Output Generation

At each output interval (default 100 Hz, range 20..500 Hz):

1. Compute current filtered value for every active (channel, CC) state.
2. Quantize to integer 0..127.
3. Compare with last transmitted value.
4. Emit a CC event only if the integer value changed.

This guarantees no redundant MIDI traffic while ensuring convergence after
the input stops changing.
