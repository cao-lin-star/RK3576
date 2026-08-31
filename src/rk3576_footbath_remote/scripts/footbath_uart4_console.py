#!/usr/bin/env python3
"""ASCII UART4 monitor and safe command sender for the STM32F407 chassis."""

from __future__ import annotations

import argparse
import threading
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="F407 UART4 ASCII debug console")
    parser.add_argument("--port", required=True, help="Windows COM port, for example COM19")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--command", action="append", default=[], help="send one ASCII command")
    parser.add_argument("--linear", type=float, help="continuous vel command, m/s")
    parser.add_argument("--angular", type=float, default=0.0, help="continuous vel command, rad/s")
    parser.add_argument("--left", type=float, help="continuous left wheel command, m/s")
    parser.add_argument("--right", type=float, help="continuous right wheel command, m/s")
    parser.add_argument("--duration", type=float, default=2.0, help="motion/monitor duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="motion refresh rate in Hz")
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="monitor indefinitely; Ctrl+C to exit (no command is sent by default)",
    )
    args = parser.parse_args()

    wheel_requested = args.left is not None or args.right is not None
    vel_requested = args.linear is not None
    if wheel_requested and vel_requested:
        parser.error("choose either --linear/--angular or --left/--right")
    if wheel_requested and (args.left is None or args.right is None):
        parser.error("--left and --right must be supplied together")
    if args.duration <= 0.0 or args.rate <= 0.0:
        parser.error("duration and rate must be positive")

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Missing pyserial. Run: python -m pip install pyserial") from exc

    stop_reader = threading.Event()
    motion_requested = vel_requested or wheel_requested

    with serial.Serial(args.port, args.baud, timeout=0.1) as uart:
        uart.reset_input_buffer()

        def reader() -> None:
            pending = bytearray()
            while not stop_reader.is_set():
                pending.extend(uart.read(256))
                while b"\n" in pending:
                    raw, _, remainder = pending.partition(b"\n")
                    pending[:] = remainder
                    print(raw.rstrip(b"\r").decode("ascii", errors="replace"), flush=True)

        reader_thread = threading.Thread(target=reader, daemon=True)
        reader_thread.start()
        try:
            for command in args.command:
                uart.write((command.rstrip("\r\n") + "\r\n").encode("ascii"))

            if motion_requested:
                deadline = time.monotonic() + args.duration
                period = 1.0 / args.rate
                if vel_requested:
                    line = f"vel {args.linear:.6f} {args.angular:.6f}\r\n".encode("ascii")
                else:
                    line = f"wheel {args.left:.6f} {args.right:.6f}\r\n".encode("ascii")
                while time.monotonic() < deadline:
                    uart.write(line)
                    time.sleep(period)
                uart.write(b"stop\r\n")
                uart.flush()
                time.sleep(0.3)
            elif args.monitor:
                while True:
                    time.sleep(0.5)
            else:
                time.sleep(args.duration)
        except KeyboardInterrupt:
            if motion_requested:
                uart.write(b"stop\r\n")
                uart.flush()
        finally:
            stop_reader.set()
            reader_thread.join(timeout=0.3)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
