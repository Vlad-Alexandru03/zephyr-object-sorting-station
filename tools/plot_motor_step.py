import argparse
import csv
from pathlib import Path
from time import monotonic

import matplotlib.pyplot as plt
import serial


def read_step(port):
    samples = []
    deadline = monotonic() + 60
    while monotonic() < deadline:
        line = port.readline().decode(errors="replace").strip()
        if line.startswith("STEP_ABORT,"):
            raise ValueError(line.partition(",")[2])
        if line == "STEP_DONE":
            return samples
        if line.startswith("STEP,"):
            _, elapsed, duty, pulses = line.split(",")
            samples.append((int(elapsed), int(duty), int(pulses)))
            if len(samples) == 1:
                print("Recording motor response for 10 seconds...", flush=True)
                deadline = monotonic() + 15
    raise ValueError("Timed out waiting for a complete step test.")


def save_response(samples, pulses_per_rev, output):
    if len(samples) < 2 or samples[0] != (0, 35, 0) or samples[-1][1] != 60:
        raise ValueError("Incomplete 35% to 60% step. Reset and repeat the test.")

    step_ms = next(elapsed for elapsed, duty, _ in samples if duty == 60)
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
    pwm_axis.set_ylim(0, 70)
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
    parser.add_argument("port", help="ESP32 UART port, such as COM10")
    parser.add_argument("--pulses-per-rev", type=int, default=20)
    parser.add_argument("--output", type=Path, default=Path("motor_step"))
    args = parser.parse_args()
    if args.pulses_per_rev <= 0:
        parser.error("--pulses-per-rev must be positive")

    try:
        with serial.Serial(args.port, 115200, timeout=0.25) as port:
            print("Press BOOT once. If the test already ran, tap RESET first.", flush=True)
            samples = read_step(port)
        save_response(samples, args.pulses_per_rev, args.output)
    except (OSError, ValueError) as error:
        parser.exit(1, f"Motor step test failed: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "\nStopped listening. Any active test ends on the ESP32.\n")


if __name__ == "__main__":
    main()
