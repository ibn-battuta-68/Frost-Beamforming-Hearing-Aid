# Frost Beamforming on RP2040

A four-microphone adaptive beamforming project using fixed-point C, RP2040
PIO assembly, and DMA. The goal is to preserve a desired sound arriving
broadside to the array while suppressing interference.

```text
Four microphones -> I2S / PIO -> Dual DMA -> Fixed-point Frost -> Mono audio
```

## Project highlights

- **Frost constrained LMS:** four microphones with 16 FIR taps each; adaptive
  weights minimize output power while preserving the desired response.
- **Fixed-point arithmetic:** Q20 coefficients, integer PCM, 64-bit
  accumulators, and bounded coefficient updates.
- **Iterative matrix inversion:** Newton-Schulz refinement computes the
  constraint Gram inverse during initialization, outside the audio loop.
- **PIO and dual DMA:** assembly generates I2S clocks and receives two stereo
  data wires at a nominal 8 kHz sample rate. One DMA channel serves each wire.
- **Real-time processing design:** 32-frame blocks, explicit buffer ownership,
  USB writes on the second core, and counters for processing time and overruns.

## Files

| File | What it demonstrates |
| --- | --- |
| [beamformer.c](beamformer.c) | Fixed-point Frost algorithm, constraint projection, and iterative inversion |
| [beamformer.h](beamformer.h) | Algorithm configuration and filter state |
| [main.c](main.c) | RP2040 PIO setup, dual DMA capture, buffering, and audio processing |
| [i2s_input.pio](i2s_input.pio) | I2S clock generation and microphone sampling in PIO assembly |
| [simulation.py](simulation.py) | Standalone floating-point illustration with synthetic microphones and SNR comparison |

## How the beamformer works

For the microphone history vector `x` and filter weights `w`:

```text
y = w^T x
v = w - mu * y * x / (epsilon + x^T x)
w = v - C (C^T C)^-1 (C^T v - f)
```

`C` groups the four microphone weights at each tap. With
`f = [1, 0, ..., 0]`, the first tap's weights sum to one and the remaining
sums are zero. This preserves an aligned broadside signal while allowing
adaptation to reject interference. Integer remainders are redistributed
after projection to prevent constraint drift.

The inverse is refined using `Xnext = X (2I - AX)`. For these particular
constraints, `C^T C = 4I`; the general iterative routine is retained in the
C implementation, although this diagonal case also has a closed-form inverse.

## Hardware overview

| GPIO | Connection |
| --- | --- |
| 16 | Shared I2S bit clock |
| 17 | Shared I2S word select |
| 18 | Microphones 1 and 2, opposite left/right slots |
| 19 | Microphones 3 and 4, opposite left/right slots |
| 20 | Optional timing probe, high during block processing |

Microphones must support 8 kHz frames, a 512 kHz bit clock, 32-bit slots,
and sharing an SD wire through complementary left/right selection. The
current filter assumes matched microphones and a broadside target.

## Simulation

Run with Python 3:

```sh
python simulation.py
```

The example generates a common desired signal, a delayed interferer, and
independent microphone noise, then compares input and output SNR after
adaptation. It illustrates the algorithm in floating point; the embedded
implementation uses fixed point.

This repository presents the project's source and design. The RP2040 code
uses Pico SDK APIs and a header generated from the PIO source; SDK files,
build configuration, and generated artifacts are intentionally omitted.
USB output is raw mono PCM16.

Reference: O. L. Frost III, *An algorithm for linearly constrained adaptive
array processing*, [1972](https://doi.org/10.1109/PROC.1972.8817).
