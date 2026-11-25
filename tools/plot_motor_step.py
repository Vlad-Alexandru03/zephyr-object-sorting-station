import argparse
import csv
from pathlib import Path
from time import monotonic

import matplotlib.pyplot as plt
import serial


def read_step(port, *, offline=False):
    samples = []
    deadline = monotonic() + 60
    while monotonic() < deadline:
        raw = port.readline()
        if offline and not raw:
            break
        line = raw.decode(errors="replace").strip()
        if line.startswith("STEP_ABORT,"):
            raise ValueError(line.partition(",")[2])
        if line == "STEP_DONE":
            return samples
        if line.startswith("STEP,"):
            _, elapsed, duty, pulses = line.split(",")
            samples.append((int(elapsed), int(duty), int(pulses)))
            if len(samples) == 1:
                print("Recording motor response...", flush=True)
                deadline = monotonic() + 30
    raise ValueError("The log does not contain a complete step test." if offline
                     else "Timed out waiting for a complete step test.")


def save_response(samples, pulses_per_rev, output):
    if len(samples) < 3 or samples[0][0] != 0 or samples[0][2] != 0:
        raise ValueError("Incomplete step test. Reset and repeat the test.")

    low_duty, high_duty = samples[0][1], samples[-1][1]
    if not 0 <= low_duty < high_duty <= 100:
        raise ValueError("The log must contain an increase in PWM duty.")
    step_index = next(i for i, sample in enumerate(samples) if sample[1] != low_duty)
    if any(duty != high_duty for _, duty, _ in samples[step_index:]):
        raise ValueError("The log must contain exactly one PWM step.")
    step_ms = samples[step_index][0]
    times = []
    speeds = []
    for previous, current in zip(samples, samples[1:]):
        before_ms, _, before_count = previous
        after_ms, _, after_count = current
        interval_ms = after_ms - before_ms
        if interval_ms <= 0 or after_count < before_count:
            raise ValueError("Invalid sample times or encoder counts.")
        times.append(((before_ms + after_ms) / 2 - step_ms) / 1000)
        speeds.append((after_count - before_count) * 60000 / (pulses_per_rev * interval_ms))

    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(("elapsed_ms", "pwm_percent", "encoder_count"))
        writer.writerows(samples)

    figure, (pwm_axis, speed_axis) = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
    pwm_axis.step([(elapsed - step_ms) / 1000 for elapsed, _, _ in samples],
                  [duty for _, duty, _ in samples], where="post")
    pwm_axis.set_ylabel("PWM (%)")
    pwm_axis.set_ylim(0, min(100, max(70, high_duty + 10)))
    pwm_axis.set_title("Conveyor step response")
    speed_axis.plot(times, speeds, ".-", markersize=3)
    speed_axis.set_ylabel("Gearbox output (RPM)")
    speed_axis.set_xlabel("Time relative to PWM step (s)")
    for axis in (pwm_axis, speed_axis):
        axis.axvline(0, color="gray", linestyle="--")
        axis.grid(alpha=0.3)
    figure.tight_layout()
    png_path = output.with_suffix(".png")
    figure.savefig(png_path, dpi=150)
    plt.close(figure)
    print(f"Saved {csv_path.resolve()} and {png_path.resolve()}")


def main():
    parser = argparse.ArgumentParser(description="Record and plot the conveyor motor step test.")
    parser.add_argument("port", nargs="?", help="Serial port, such as COM10")
    parser.add_argument("--log", type=Path, help="Read a saved serial log instead")
    parser.add_argument("--pulses-per-rev", type=int, default=20)
    parser.add_argument("--output", type=Path, default=Path("motor_step"))
    args = parser.parse_args()
    if bool(args.port) == bool(args.log):
        parser.error("Choose either a serial port or --log.")
    if args.pulses_per_rev <= 0:
        parser.error("--pulses-per-rev must be positive")

    try:
        if args.log:
            with args.log.open("rb") as file:
                samples = read_step(file, offline=True)
        else:
            with serial.Serial(args.port, 115200, timeout=0.25) as port:
                print("Waiting for the board's step test.", flush=True)
                samples = read_step(port)
        save_response(samples, args.pulses_per_rev, args.output)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Motor step test failed: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "\nStopped listening. Any active test continues on the board.\n")


if __name__ == "__main__":
    main()
