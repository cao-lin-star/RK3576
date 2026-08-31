#!/usr/bin/env python3
"""Characterize motor breakaway and running duty with PID fully bypassed.

Requires firmware exposing the UART4-only ``pwm <left> <right>`` command.
Every motion command is refreshed at 20 Hz and followed by STOP.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import time
from pathlib import Path
from typing import TextIO


@dataclasses.dataclass
class Trial:
    kind: str
    side: str
    direction: int
    duty_percent: float
    trial_index: int
    delta_ticks: int
    peak_speed_mm_s: int
    tail_moving_ratio: float
    fault: int
    passed: bool


def parse_record(line: str, prefix: str) -> dict[str, int] | None:
    if not line.startswith(prefix):
        return None
    values: dict[str, int] = {}
    for item in line[len(prefix) :].split(","):
        if "=" not in item:
            continue
        key, raw = item.split("=", 1)
        try:
            values[key] = int(raw, 0)
        except ValueError:
            continue
    return values


class Console:
    def __init__(self, serial_port: object, transcript: TextIO) -> None:
        self.serial = serial_port
        self.transcript = transcript
        self.latest_tel: dict[str, int] | None = None

    def send(self, command: str) -> None:
        self.transcript.write(f"TX,{command}\n")
        self.transcript.flush()
        self.serial.write((command + "\r\n").encode("ascii"))

    def read_for(self, duration_s: float, show: bool = False) -> list[dict[str, int]]:
        deadline = time.monotonic() + duration_s
        telemetry: list[dict[str, int]] = []
        while time.monotonic() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            line = raw.rstrip(b"\r\n").decode("ascii", errors="replace")
            self.transcript.write(line + "\n")
            self.transcript.flush()
            if show:
                print(line, flush=True)
            parsed = parse_record(line, "TEL,")
            if parsed is not None:
                self.latest_tel = parsed
                telemetry.append(parsed)
        return telemetry

    def status(self) -> dict[str, int]:
        self.send("status")
        samples = self.read_for(0.25)
        if samples:
            return samples[-1]
        if self.latest_tel is None:
            raise RuntimeError("UART4 returned no TEL record")
        return self.latest_tel

    def stop_and_settle(self, settle_s: float) -> dict[str, int]:
        self.send("stop")
        self.read_for(settle_s)
        return self.status()

    def drive_pwm(
        self,
        left_percent: float,
        right_percent: float,
        duration_s: float,
        rate_hz: float,
    ) -> list[dict[str, int]]:
        command = f"pwm {left_percent:.3f} {right_percent:.3f}"
        period_s = 1.0 / rate_hz
        deadline = time.monotonic() + duration_s
        next_send = time.monotonic()
        telemetry: list[dict[str, int]] = []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                self.send(command)
                next_send += period_s
            telemetry.extend(self.read_for(min(0.015, max(0.0, deadline - now))))
        self.send("status")
        telemetry.extend(self.read_for(0.12))
        return telemetry


def wheel_values(side: str, direction: int, duty: float) -> tuple[float, float]:
    signed = direction * duty
    return (signed, 0.0) if side == "left" else (0.0, signed)


def extract_metrics(
    samples: list[dict[str, int]], side: str, direction: int, start_ticks: int
) -> tuple[int, int, float, int]:
    if not samples:
        raise RuntimeError("no telemetry collected during PWM trial")
    suffix = "l" if side == "left" else "r"
    end_ticks = samples[-1][f"enc_{suffix}"]
    delta_ticks = end_ticks - start_ticks
    speeds = [sample.get(f"spd_{suffix}_mm_s", 0) for sample in samples]
    peak_speed = max((abs(value) for value in speeds), default=0)
    tail = speeds[max(0, len(speeds) // 2) :]
    moving = sum(1 for value in tail if direction * value > 0)
    tail_ratio = moving / len(tail) if tail else 0.0
    fault = samples[-1].get("fault", 0)
    return delta_ticks, peak_speed, tail_ratio, fault


def dangerous_fault(fault: int) -> bool:
    # Encoder, overrun, invalid command, E-stop, and obstacle bits.
    return (fault & 0x07B0) != 0


def run_breakaway_trial(
    console: Console,
    side: str,
    direction: int,
    duty: float,
    index: int,
    duration_s: float,
    settle_s: float,
    rate_hz: float,
    tick_threshold: int,
) -> Trial:
    start = console.stop_and_settle(settle_s)
    suffix = "l" if side == "left" else "r"
    left, right = wheel_values(side, direction, duty)
    samples = console.drive_pwm(left, right, duration_s, rate_hz)
    delta, peak, tail_ratio, fault = extract_metrics(
        samples, side, direction, start[f"enc_{suffix}"]
    )
    console.stop_and_settle(settle_s)
    if direction * delta < -tick_threshold:
        raise RuntimeError(f"{side} encoder polarity reversed: delta={delta}")
    if dangerous_fault(fault):
        raise RuntimeError(f"dangerous fault 0x{fault:04X} during breakaway trial")
    passed = direction * delta >= tick_threshold
    return Trial("breakaway", side, direction, duty, index, delta, peak, tail_ratio, fault, passed)


def run_running_trial(
    console: Console,
    side: str,
    direction: int,
    duty: float,
    index: int,
    spinup_duty: float,
    spinup_s: float,
    duration_s: float,
    settle_s: float,
    rate_hz: float,
    tick_threshold: int,
    tail_ratio_required: float,
) -> Trial:
    console.stop_and_settle(settle_s)
    suffix = "l" if side == "left" else "r"
    spin_left, spin_right = wheel_values(side, direction, spinup_duty)
    spin_samples = console.drive_pwm(spin_left, spin_right, spinup_s, rate_hz)
    if not spin_samples:
        raise RuntimeError("no telemetry during spin-up")
    start_ticks = spin_samples[-1][f"enc_{suffix}"]
    left, right = wheel_values(side, direction, duty)
    samples = console.drive_pwm(left, right, duration_s, rate_hz)
    delta, peak, tail_ratio, fault = extract_metrics(samples, side, direction, start_ticks)
    console.stop_and_settle(settle_s)
    if dangerous_fault(fault):
        raise RuntimeError(f"dangerous fault 0x{fault:04X} during running trial")
    passed = direction * delta >= tick_threshold and tail_ratio >= tail_ratio_required
    return Trial("running", side, direction, duty, index, delta, peak, tail_ratio, fault, passed)


def values(start: float, stop: float, step: float) -> list[float]:
    if step <= 0.0:
        raise ValueError("step must be positive")
    result: list[float] = []
    current = start
    if start <= stop:
        while current <= stop + 1e-6:
            result.append(round(current, 3))
            current += step
    else:
        while current >= stop - 1e-6:
            result.append(round(current, 3))
            current -= step
    return result


def emit(trial: Trial) -> None:
    print(
        f"{trial.kind},{trial.side},{trial.direction:+d},{trial.duty_percent:.2f},"
        f"{trial.trial_index},{trial.delta_ticks},{trial.peak_speed_mm_s},"
        f"{trial.tail_moving_ratio:.2f},0x{trial.fault:04X},{int(trial.passed)}",
        flush=True,
    )


def all_pass(trials: list[Trial]) -> bool:
    return bool(trials) and all(trial.passed for trial in trials)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--side", choices=("both", "left", "right"), default="both")
    parser.add_argument(
        "--direction", choices=("both", "forward", "reverse"), default="both"
    )
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--settle", type=float, default=0.55)
    parser.add_argument("--breakaway-duration", type=float, default=0.65)
    parser.add_argument("--running-duration", type=float, default=1.20)
    parser.add_argument("--spinup-duration", type=float, default=0.55)
    parser.add_argument("--coarse-min", type=float, default=4.0)
    parser.add_argument("--coarse-max", type=float, default=26.0)
    parser.add_argument("--coarse-step", type=float, default=2.0)
    parser.add_argument("--fine-step", type=float, default=0.5)
    parser.add_argument("--fine-trials", type=int, default=3)
    parser.add_argument("--running-step", type=float, default=1.0)
    parser.add_argument("--running-trials", type=int, default=2)
    parser.add_argument("--breakaway-ticks", type=int, default=4)
    parser.add_argument("--running-ticks", type=int, default=20)
    parser.add_argument("--tail-ratio", type=float, default=0.75)
    parser.add_argument("--transcript", required=True)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--i-understand-motors-will-move", action="store_true")
    args = parser.parse_args()
    if not args.i_understand_motors_will_move:
        parser.error("secure/lift the chassis and pass --i-understand-motors-will-move")
    if args.rate < 10.0 or args.rate > 50.0:
        parser.error("rate must be 10..50 Hz")
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required") from exc

    transcript_path = Path(args.transcript)
    summary_path = Path(args.summary)
    transcript_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    results: list[Trial] = []
    summary: dict[str, dict[str, float]] = {}
    print("kind,side,direction,duty_percent,trial,delta_ticks,peak_speed_mm_s,tail_ratio,fault,passed")

    with transcript_path.open("w", encoding="utf-8", newline="\n") as transcript:
        with serial.Serial(args.port, args.baud, timeout=0.02) as uart:
            uart.reset_input_buffer()
            console = Console(uart, transcript)
            console.send("stream on")
            console.send("pid both 0 0 0")
            console.read_for(0.4)
            console.send("pwm 0 0")
            probe = console.read_for(0.35)
            if not any(sample.get("open_loop", 0) == 1 for sample in probe):
                raise RuntimeError("firmware did not enter UART4 open-loop PWM mode")
            console.stop_and_settle(args.settle)
            try:
                sides = ("left", "right") if args.side == "both" else (args.side,)
                directions = (
                    (1, -1)
                    if args.direction == "both"
                    else ((1,) if args.direction == "forward" else (-1,))
                )
                for side in sides:
                    for direction in directions:
                        key = f"{side}_{'forward' if direction > 0 else 'reverse'}"
                        coarse_hit: float | None = None
                        for duty in values(args.coarse_min, args.coarse_max, args.coarse_step):
                            trial = run_breakaway_trial(
                                console, side, direction, duty, 1,
                                args.breakaway_duration, args.settle, args.rate,
                                args.breakaway_ticks,
                            )
                            results.append(trial)
                            emit(trial)
                            if trial.passed:
                                coarse_hit = duty
                                break
                        if coarse_hit is None:
                            raise RuntimeError(f"no breakaway found for {key}")
                        fine_min = max(args.coarse_min, coarse_hit - args.coarse_step)
                        breakaway: float | None = None
                        fine_max = min(
                            args.coarse_max, coarse_hit + 2.0 * args.coarse_step
                        )
                        for duty in values(fine_min, fine_max, args.fine_step):
                            group: list[Trial] = []
                            for index in range(1, args.fine_trials + 1):
                                trial = run_breakaway_trial(
                                    console, side, direction, duty, index,
                                    args.breakaway_duration, args.settle, args.rate,
                                    args.breakaway_ticks,
                                )
                                group.append(trial)
                                results.append(trial)
                                emit(trial)
                            if all_pass(group):
                                breakaway = duty
                                break
                        if breakaway is None:
                            raise RuntimeError(f"no repeatable breakaway found for {key}")

                        spinup = min(35.0, max(breakaway + 6.0, 22.0))
                        running: float | None = None
                        for duty in values(spinup, args.coarse_min, args.running_step):
                            group = []
                            for index in range(1, args.running_trials + 1):
                                trial = run_running_trial(
                                    console, side, direction, duty, index, spinup,
                                    args.spinup_duration, args.running_duration,
                                    args.settle, args.rate, args.running_ticks,
                                    args.tail_ratio,
                                )
                                group.append(trial)
                                results.append(trial)
                                emit(trial)
                            if all_pass(group):
                                running = duty
                            else:
                                break
                        if running is None:
                            raise RuntimeError(f"no running threshold found for {key}")
                        summary[key] = {
                            "breakaway_percent": breakaway,
                            "running_percent": running,
                            "spinup_percent": spinup,
                        }
            finally:
                console.stop_and_settle(args.settle)
                console.send("pid both 0.05 0.001 0")
                console.read_for(0.4)
                console.send("stop")
                console.read_for(0.25)

    payload = {
        "generated_unix_s": time.time(),
        "port": args.port,
        "baud": args.baud,
        "pid_bypassed": True,
        "summary": summary,
        "trials": [dataclasses.asdict(trial) for trial in results],
    }
    summary_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print("SUMMARY," + json.dumps(summary, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
