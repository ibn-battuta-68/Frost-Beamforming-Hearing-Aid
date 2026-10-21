"""Floating-point illustration of broadside Frost constrained LMS.

The embedded fixed-point implementation is in beamformer.c. This example
uses only Python's standard library and synthetic audio, with a fixed seed.
"""
import argparse
import math
import random

SAMPLE_RATE = 8000
MICROPHONES = 4
TAPS = 16
MU = 1 / 8


def simulate(seconds=3, seed=7):
    rng = random.Random(seed)
    count = seconds * SAMPLE_RATE
    interferer = [rng.uniform(-0.7, 0.7) for _ in range(count + MICROPHONES)]
    history = [[0.0] * MICROPHONES for _ in range(TAPS)]
    weights = [[0.0] * MICROPHONES for _ in range(TAPS)]
    weights[0] = [1 / MICROPHONES] * MICROPHONES
    signal_power = input_error = output_error = 0.0
    max_constraint_error = 0.0

    for n in range(count):
        desired = (0.15 * math.sin(2 * math.pi * 310 * n / SAMPLE_RATE)
                   + 0.10 * math.sin(2 * math.pi * 710 * n / SAMPLE_RATE))
        # Desired sound arrives together; interference is delayed per sensor.
        mics = [desired + interferer[n + MICROPHONES - 1 - k]
                + rng.uniform(-0.03, 0.03) for k in range(MICROPHONES)]
        history = [mics] + history[:-1]
        output = sum(w * x for row_w, row_x in zip(weights, history)
                     for w, x in zip(row_w, row_x))
        energy = 0.01 + sum(x * x for row in history for x in row)
        gain = MU * output / energy

        for tap in range(TAPS):
            row = [w - gain * x for w, x in zip(weights[tap], history[tap])]
            target = 1.0 if tap == 0 else 0.0
            # C^T C = 4I: projection is a per-tap mean correction.
            correction = (sum(row) - target) / MICROPHONES
            weights[tap] = [w - correction for w in row]
            max_constraint_error = max(max_constraint_error,
                                       abs(sum(weights[tap]) - target))

        if n >= count // 2:  # discard the adaptation transient
            signal_power += desired * desired
            input_error += (mics[0] - desired) ** 2
            output_error += (output - desired) ** 2

    input_snr = 10 * math.log10(signal_power / max(input_error, 1e-20))
    output_snr = 10 * math.log10(signal_power / max(output_error, 1e-20))
    print(f"Synthetic example: {MICROPHONES} microphones, {TAPS} taps, {SAMPLE_RATE} Hz")
    print(f"Input SNR:           {input_snr:7.2f} dB")
    print(f"Output SNR:          {output_snr:7.2f} dB")
    print(f"SNR improvement:     {output_snr - input_snr:7.2f} dB")
    print(f"Max constraint error: {max_constraint_error:.2e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, default=3)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.seconds < 1:
        parser.error("--seconds must be at least 1")
    simulate(args.seconds, args.seed)
