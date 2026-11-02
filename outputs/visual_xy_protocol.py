"""Build test frames for the STM32 visual XY protocol."""

import struct


def pack_visual_xy(x_mm: int, y_mm: int, enabled: bool = True) -> bytes:
    if not (-32768 <= x_mm <= 32767 and -32768 <= y_mm <= 32767):
        raise ValueError("x_mm and y_mm must fit signed int16")
    frame = bytearray((0xA5, 0x5A))
    frame.extend(struct.pack(">hh", x_mm, y_mm))
    frame.append(1 if enabled else 0)
    checksum = 0
    for value in frame:
        checksum ^= value
    frame.append(checksum)
    frame.append(0x0A)
    return bytes(frame)


if __name__ == "__main__":
    print(pack_visual_xy(230, 0).hex(" "))
