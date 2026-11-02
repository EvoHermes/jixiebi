#!/usr/bin/env python3
"""Send one right-up target to the STM32 A130 controller."""

import argparse
import time

import serial


def make_target_frame(x: int, y: int) -> bytes:
    if not (0 <= x <= 0xFFFF and 0 <= y <= 0xFFFF):
        raise ValueError("x and y must be unsigned 16-bit pixel coordinates")

    frame = [
        0xEF,
        0xFE,
        x & 0xFF,
        (x >> 8) & 0xFF,
        y & 0xFF,
        (y >> 8) & 0xFF,
    ]
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--x", type=int, default=440)
    parser.add_argument("--y", type=int, default=140)
    args = parser.parse_args()

    frame = make_target_frame(args.x, args.y)
    print("target:", args.x, args.y)
    print("frame:", frame.hex(" "))

    with serial.Serial(args.port, args.baud, timeout=0.5) as link:
        link.reset_input_buffer()
        time.sleep(3.2)  # allow the firmware startup homing sequence to finish
        link.write(frame)
        link.flush()
        deadline = time.monotonic() + 15.0
        received = bytearray()
        while time.monotonic() < deadline:
            received.extend(link.read(1))
            if received[-4:] == b"\xEF\xFE\x7E\x6B":
                print("grasp complete:", received[-4:].hex(" "))
                return
        print("completion frame not received within 15 s")


if __name__ == "__main__":
    main()
