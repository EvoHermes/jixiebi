"""Check real Pi v002 packets against the host-built STM32 receiver."""

import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("release", type=Path, help="Pi release folder or mango_vision directory")
    args = parser.parse_args()
    root = args.release
    if (root / "mango_vision").is_dir():
        root = root / "mango_vision"

    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("pi_v002_protocol", root / "pi/arm_snapshot_protocol.py")
    protocol = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(protocol)
    config = json.loads((root / "arm_continuous_config.json").read_text(encoding="utf-8"))
    width, height = config["camera"]["width"], config["camera"]["height"]
    exe = Path(__file__).resolve().parent / "vision_regression.exe"

    def receive(packet):
        result = subprocess.run([str(exe), packet.hex(" ")], capture_output=True, text=True, check=True)
        valid, x, y, j1, j4 = result.stdout.strip().split()
        return int(valid), int(x), int(y), float(j1), float(j4)

    cx, cy = width / 2, height / 2
    cases = [
        ([cx-10, cy-10, cx+10, cy+10], (int(cx), int(cy)), 0, 0),
        ([cx+5, cy-25, cx+25, cy-5], (int(cx+15), int(cy+15)), -1, 1),
        ([cx-25, cy+5, cx-5, cy+25], (int(cx-15), int(cy-15)), 1, -1),
        ([0, height-0.2, 0.2, height], (0, 0), 1, -1),
        ([width-0.2, 0, width, 0.2], (width, height), -1, 1),
    ]
    for box, expected, j1_sign, j4_sign in cases:
        xy = protocol.pixel_offset({"bbox_xyxy_px": box}, width, height, origin="bottom_left")
        assert xy == expected, (box, xy, expected)
        valid, x, y, j1, j4 = receive(protocol.coordinate_packet(*xy))
        assert (valid, x, y) == (1, *expected), (expected, valid, x, y)
        assert (j1 == 0 if j1_sign == 0 else j1 * j1_sign > 0), (xy, j1)
        assert (j4 == 0 if j4_sign == 0 else j4 * j4_sign > 0), (xy, j4)

    for xy in [(width+1, int(cy)), (int(cx), height+1), (-1, int(cy)), (int(cx), -1)]:
        assert receive(protocol.coordinate_packet(*xy))[0] == 0, xy
    bad = bytearray(protocol.coordinate_packet(int(cx), int(cy)))
    bad[-1] ^= 1
    assert receive(bad)[0] == 0

    response = subprocess.run([str(exe), "--home"], capture_output=True, text=True, check=True)
    sent_home = bytes.fromhex(response.stdout.strip())
    pi_parser = protocol.HomeParser()
    assert pi_parser.feed(sent_home[:2]) == 0
    assert pi_parser.feed(sent_home[2:]) == 1
    print(f"PASS: Pi v002 {width}x{height} packets, edges, directions, invalid frames and HOME reply")


if __name__ == "__main__":
    main()
