#!/usr/bin/env python3
"""
解析 OpenOCD mdw 命令读出的 g_dm4310 内存转储。

用法:
  python tools/parse_memdump.py 00000001 0000384d 20210101 bfc2da00 ...

  或从 stdin 读取 OpenOCD mdw 输出:
  sudo openocd ... -c "mdw 0x200005CC 24" -c "exit" | python tools/parse_memdump.py
"""

import re
import struct
import sys


def unpack_float(hex_val):
    return struct.unpack("f", struct.pack("I", hex_val))[0]


def parse_motor(raw_words):
    """解析 6 个 32-bit word 为电机状态"""
    rx_count = raw_words[0]
    last_ms = raw_words[1]
    b = raw_words[2]
    online = b & 0xFF
    motor_state = (b >> 8) & 0xFF
    mos_temp = (b >> 16) & 0xFF
    coil_temp = (b >> 24) & 0xFF
    pos = unpack_float(raw_words[3])
    vel = unpack_float(raw_words[4])
    tor = unpack_float(raw_words[5])
    return (rx_count, last_ms, online, motor_state, mos_temp, coil_temp, pos, vel, tor)


def main():
    text = sys.stdin.read() if not sys.stdin.isatty() else " ".join(sys.argv[1:])

    # 从 OpenOCD mdw 输出或命令行提取 hex 值
    hex_vals = re.findall(r"[0-9a-fA-F]{8}", text)
    if not hex_vals:
        print("未找到 32-bit hex 值。用法见文件头部注释。", file=sys.stderr)
        sys.exit(1)

    words = [int(h, 16) for h in hex_vals]

    print(f"{'Motor':>6} | online | state | mos°C | coil°C | pos_rad    | vel_radps   | torque_nm   | rx_count | last_ms")
    print("-" * 100)

    for m in range(min(4, len(words) // 6)):
        base = m * 6
        if base + 5 >= len(words):
            break
        rx, lm, on, st, mos, coil, pos, vel, tor = parse_motor(words[base:base + 6])
        print(f"  M{m+1}   |   {on}    |   {st}   |  {mos:3d}  |  {coil:3d}   | {pos:+.6f} | {vel:+.6f} | {tor:+.6f} |    {rx:5d}  | {lm}")


if __name__ == "__main__":
    main()
