"""Minimal camera-to-A130 visual servo controller.

The controller follows a colored target and sends the A130 MoveIt-style
16-byte joint frame used by the supplied STM32F103 firmware.

This is deliberately a first prototype:
* target detection is HSV color segmentation;
* the camera is assumed to point approximately down along the gripper axis;
* x/y are corrected from image-center error and z from target size;
* wrist orientation is held fixed (rx=ry=0).

Install runtime dependencies with:
    python -m pip install opencv-python pyserial

Run a hardware-free check with:
    python a130_visual_servo.py --self-test

Run with a USB camera in dry-run mode first:
    python a130_visual_servo.py --port COM10 --dry-run
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


PI = math.pi


@dataclass(frozen=True)
class JointLimits:
    minimum: float
    maximum: float


JOINT_LIMITS = (
    JointLimits(-2.35, 2.35),
    JointLimits(-1.57, 1.57),
    JointLimits(-2.30, 1.57),
    JointLimits(-1.57, 1.00),
    JointLimits(-1.57, 1.57),
)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def angle_in_limits(angles: Sequence[float]) -> bool:
    return len(angles) == 5 and all(
        limit.minimum <= angle <= limit.maximum
        for angle, limit in zip(angles, JOINT_LIMITS)
    )


class A130Kinematics:
    """A direct Python port of the kinematic equations in control.c."""

    A2 = 0.105
    A3 = 0.0975
    A4 = 0.02822
    D1 = 0.1085
    D5 = 0.17283  # 0.0501 m wrist offset + 0.12273 m gripper offset

    @classmethod
    def fk(cls, angles: Sequence[float]) -> tuple[float, float, float]:
        t1, t2, t3, t4, _t5 = angles
        t23 = t2 + t3
        t234 = t23 + t4
        reach = (
            cls.A3 * math.sin(t23)
            + cls.A2 * math.sin(t2)
            + cls.A4 * math.cos(t234)
            + cls.D5 * math.sin(t234)
        )
        x = -math.cos(t1) * reach
        y = -math.sin(t1) * reach
        z = (
            cls.D1
            + cls.A3 * math.cos(t23)
            + cls.A2 * math.cos(t2)
            + cls.D5 * math.cos(t234)
            - cls.A4 * math.sin(t234)
        )
        return x, y, z

    @classmethod
    def ik(
        cls,
        x: float,
        y: float,
        z: float,
        rx: float = 0.0,
        ry: float = 0.0,
        current: Optional[Sequence[float]] = None,
    ) -> Optional[tuple[float, float, float, float, float]]:
        """Solve the same 5-DOF IK used by the firmware.

        The returned solution is the valid candidate closest to ``current``;
        this reduces branch jumping while the camera loop is running.
        """
        # The firmware builds T05 from x, y, z, rx and ry.
        r00 = math.sin(ry)
        r02 = math.cos(ry)
        r10 = -math.cos(ry) * math.sin(rx)
        r11 = -math.cos(rx)
        r12 = math.sin(rx) * math.sin(ry)
        r20 = math.cos(rx) * math.cos(ry)
        r21 = -math.sin(rx)
        r22 = -math.cos(rx) * math.sin(ry)

        # The firmware creates four (t1, t5) branches, then duplicates each
        # branch with the positive/negative t3 solution for eight candidates.
        t1_candidates = [
            math.atan2(y, x),
            math.atan2(y, x),
            math.atan2(-y, -x),
            math.atan2(-y, -x),
        ]
        if abs(r22 - 1.0) < 1e-3:
            t5_candidates = [math.atan2(-r10, -r00) - t1 for t1 in t1_candidates]
        elif abs(r22 + 1.0) < 1e-3:
            t5_candidates = [t1 - math.atan2(r10, r00) for t1 in t1_candidates]
        else:
            t5_candidates = [
                math.atan2(r21, -r20),
                math.atan2(-r21, r20),
                math.atan2(r21, -r20),
                math.atan2(-r21, r20),
            ]

        candidates: list[tuple[float, float, float, float, float]] = []
        for t1, t5 in zip(t1_candidates, t5_candidates):
            if abs(math.sin(t5)) < 1e-3:
                t234 = math.atan2(-r20 / math.cos(t5), r22)
            else:
                t234 = math.atan2(r21 / math.sin(t5), r22)
            s1 = math.sin(t1)
            c1 = math.cos(t1)
            s234 = math.sin(t234)
            c234 = math.cos(t234)
            if abs(s1) < 1e-3:
                m = (-x / c1) - cls.A4 * c234 - cls.D5 * s234
            else:
                m = (-y / s1) - cls.A4 * c234 - cls.D5 * s234
            n = z - cls.D1 + cls.A4 * s234 - cls.D5 * c234
            k = (m * m + n * n - cls.A2**2 - cls.A3**2) / (2 * cls.A2 * cls.A3)
            if k < -1.0 - 1e-3 or k > 1.0 + 1e-3:
                continue
            k = clamp(k, -1.0, 1.0)
            t3_options = (math.acos(k), -math.acos(k))
            for t3 in t3_options:
                p = cls.A2 + cls.A3 * math.cos(t3)
                q = cls.A3 * math.sin(t3)
                denominator = p * p + q * q
                s2 = (p * m - q * n) / denominator
                c2 = (q * m + p * n) / denominator
                t2 = math.atan2(s2, c2)
                t4 = t234 - t2 - t3
                candidate = tuple(_wrap_angle(a) for a in (t1, t2, t3, t4, t5))
                if angle_in_limits(candidate):
                    candidates.append(candidate)  # type: ignore[arg-type]

        if not candidates:
            return None
        reference = tuple(current) if current is not None and len(current) == 5 else (0.0,) * 5
        return min(candidates, key=lambda candidate: sum((a - b) ** 2 for a, b in zip(candidate, reference)))


def _wrap_angle(angle: float) -> float:
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle


def pack_moveit_frame(angles: Sequence[float], follower_mode: int = 2) -> bytes:
    """Pack the 16-byte 0xAA...0xBB frame parsed by the supplied firmware."""
    if len(angles) != 5:
        raise ValueError("expected five arm joint angles")
    # Firmware negates joint 3 and 4 after decoding; compensate here.
    raw = (
        angles[0],
        angles[1],
        -angles[2],
        -angles[3],
        angles[4],
        0.0,  # gripper is held open/neutral by default
    )
    scaled = [int(round(value * 1000.0)) for value in raw]
    if any(value < -32768 or value > 32767 for value in scaled):
        raise ValueError("joint angle outside int16 packet range")
    frame = bytearray([0xAA])
    for value in scaled:
        frame.extend(struct.pack(">h", value))
    frame.append(follower_mode & 0xFF)
    frame.append(0)
    frame[14] = _xor(frame[:14])
    frame.append(0xBB)
    return bytes(frame)


def _xor(values: Iterable[int]) -> int:
    result = 0
    for value in values:
        result ^= int(value)
    return result & 0xFF


@dataclass
class Target:
    x: float
    y: float
    width: float
    height: float
    area: float


class ColorTargetDetector:
    def __init__(self, color: str, min_area: float = 500.0):
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore
        except ImportError as exc:
            raise RuntimeError("camera mode needs opencv-python and numpy") from exc
        self.cv2 = cv2
        self.np = np
        self.color = color
        self.min_area = min_area

    def detect(self, frame) -> Optional[Target]:
        cv2 = self.cv2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ranges = {
            "red": ((0, 80, 60), (10, 255, 255), (170, 80, 60), (179, 255, 255)),
            "green": ((35, 60, 50), (85, 255, 255)),
            "blue": ((90, 60, 50), (135, 255, 255)),
            "yellow": ((18, 70, 70), (38, 255, 255)),
        }
        if self.color not in ranges:
            raise ValueError(f"unsupported color: {self.color}")
        values = ranges[self.color]
        mask = cv2.inRange(hsv, values[0], values[1])
        if len(values) == 4:
            mask |= cv2.inRange(hsv, values[2], values[3])
        kernel = self.np.ones((5, 5), self.np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return None
        x, y, width, height = cv2.boundingRect(contour)
        return Target(x + width / 2.0, y + height / 2.0, float(width), float(height), area)


@dataclass
class ControllerConfig:
    x_step: float = 0.0025
    y_step: float = 0.0025
    z_step: float = 0.0020
    deadband: float = 0.08
    target_width_px: float = 120.0
    x_sign: float = -1.0
    y_sign: float = 1.0
    z_sign: float = -1.0
    min_z: float = 0.16
    max_z: float = 0.38


class VisualServo:
    def __init__(self, config: ControllerConfig, pose=(0.23, 0.0, 0.30, 0.0, 0.0)):
        self.config = config
        self.pose = list(pose)
        self.last_angles: Optional[tuple[float, ...]] = None
        self.stable_frames = 0

    def update(self, target: Target, width: int, height: int) -> tuple[bool, Optional[tuple[float, ...]]]:
        cx, cy = width / 2.0, height / 2.0
        ex = (target.x - cx) / cx
        ey = (target.y - cy) / cy
        e_size = (self.config.target_width_px - target.width) / self.config.target_width_px
        locked = abs(ex) < self.config.deadband and abs(ey) < self.config.deadband and abs(e_size) < 0.15
        if locked:
            self.stable_frames += 1
        else:
            self.stable_frames = 0
            self.pose[0] += self.config.x_sign * clamp(ex, -1.0, 1.0) * self.config.x_step
            self.pose[1] += self.config.y_sign * clamp(ey, -1.0, 1.0) * self.config.y_step
            self.pose[2] = clamp(
                self.pose[2] + self.config.z_sign * clamp(e_size, -1.0, 1.0) * self.config.z_step,
                self.config.min_z,
                self.config.max_z,
            )
        angles = A130Kinematics.ik(*self.pose, current=self.last_angles)
        if angles is not None:
            self.last_angles = angles
        return locked, angles


def run_self_test() -> None:
    # VisualServo holds rx=ry=0, so test reachable Cartesian poses for that
    # fixed wrist orientation rather than arbitrary joint-angle orientations.
    samples = (
        (0.23, 0.00, 0.30),
        (0.20, 0.05, 0.25),
        (0.25, -0.10, 0.28),
    )
    for position in samples:
        solved = A130Kinematics.ik(*position, rx=0.0, ry=0.0)
        if solved is None:
            raise AssertionError(f"IK failed for target={position}")
        solved_position = A130Kinematics.fk(solved)
        if max(abs(a - b) for a, b in zip(position, solved_position)) > 2e-4:
            raise AssertionError(f"FK/IK mismatch: {position} vs {solved_position}")
        frame = pack_moveit_frame(solved)
        if len(frame) != 16 or frame[0] != 0xAA or frame[-1] != 0xBB or frame[14] != _xor(frame[:14]):
            raise AssertionError("invalid serial frame")
    print("self-test passed: FK/IK and 16-byte serial frame")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--port", default="COM10", help="STM32 USART3 adapter, e.g. COM10 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--color", choices=("red", "green", "blue", "yellow"), default="red")
    parser.add_argument("--dry-run", action="store_true", help="do not open serial; print outgoing frames")
    parser.add_argument("--x-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument("--y-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--z-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    parser.add_argument("--target-width", type=float, default=120.0)
    parser.add_argument("--min-area", type=float, default=500.0)
    parser.add_argument("--x0", type=float, default=0.23, help="initial Cartesian x in metres")
    parser.add_argument("--y0", type=float, default=0.0, help="initial Cartesian y in metres")
    parser.add_argument("--z0", type=float, default=0.30, help="initial Cartesian z in metres")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0

    try:
        import cv2  # type: ignore
    except ImportError:
        print("camera mode needs opencv-python: python -m pip install opencv-python pyserial", file=sys.stderr)
        return 2
    detector = ColorTargetDetector(args.color, args.min_area)
    transport = None
    if not args.dry_run:
        try:
            import serial  # type: ignore
            transport = serial.Serial(args.port, args.baud, timeout=0.1)
        except Exception as exc:  # pragma: no cover - hardware dependent
            print(f"cannot open serial port {args.port}: {exc}", file=sys.stderr)
            return 2
    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        print(f"cannot open camera {args.camera}", file=sys.stderr)
        return 2
    config = ControllerConfig(
        target_width_px=args.target_width,
        x_sign=args.x_sign,
        y_sign=args.y_sign,
        z_sign=args.z_sign,
    )
    controller = VisualServo(config, pose=(args.x0, args.y0, args.z0, 0.0, 0.0))
    print("tracking started; press q or Esc to stop")
    last_send = 0.0
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                continue
            target = detector.detect(frame)
            locked = False
            if target is not None:
                locked, angles = controller.update(target, frame.shape[1], frame.shape[0])
                if angles is not None and time.monotonic() - last_send >= 0.12:
                    packet = pack_moveit_frame(angles)
                    if transport is not None:
                        transport.write(packet)
                    else:
                        print("TX", packet.hex(" "), "pose", [round(v, 4) for v in controller.pose])
                    last_send = time.monotonic()
                cv2.circle(frame, (int(target.x), int(target.y)), 8, (0, 255, 0), 2)
            cv2.drawMarker(frame, (frame.shape[1] // 2, frame.shape[0] // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            status = "LOCK" if locked else ("TRACK" if target is not None else "SEARCH")
            cv2.putText(frame, f"A130 {status}  z={controller.pose[2]:.3f}m", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("A130 visual servo", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
        if transport is not None:
            transport.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
