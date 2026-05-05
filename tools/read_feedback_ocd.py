#!/usr/bin/env python3
"""
通过 OpenOCD 直接读取 g_dm4310.motor[] 内存，获取电机反馈数据。

当 CMSIS-DAP CDC UART 桥不可用时，此脚本绕过串口，
直接通过 SWD 调试接口读取 MCU 内存中的电机状态。

用法:
  sudo python tools/read_feedback_ocd.py

需要: sudo 权限（访问 CMSIS-DAP USB），OpenOCD

原理:
  每 100ms 用 OpenOCD halt → mdw 读内存 → resume 循环采样。
  内存布局 (g_dm4310 @ 0x20000540):
    motor[0] @ 0x200005CC (pos_rad @ +12)
    motor[1] @ 0x200005E4
    motor[2] @ 0x200005FC
    motor[3] @ 0x20000614
  每个 dm4310_motor_status = 24 bytes (6 words)
"""

import re
import struct
import subprocess
import sys
import time

MEM_BASE = 0x20000540
MOTOR_OFFSET = 0x8C  # motor[0] offset from g_dm4310 base
MOTOR_SIZE = 24       # bytes per motor status

OCD_SCRIPT = """
source [find interface/cmsis-dap.cfg]
source [find target/stm32f4x.cfg]
adapter speed 500
init
halt
mdw 0x{MOTOR0_ADDR:08X} 24
resume
exit
"""


def unpack_float(hex_val):
    return struct.unpack("f", struct.pack("I", hex_val))[0]


def read_motors():
    """运行 OpenOCD 读取四台电机状态，返回 (words, raw_output)"""
    motor0_addr = MEM_BASE + MOTOR_OFFSET
    script = OCD_SCRIPT.format(MOTOR0_ADDR=motor0_addr)

    proc = subprocess.run(
        ["sudo", "openocd", "-c", script],
        capture_output=True, text=True, timeout=15
    )
    output = proc.stderr + proc.stdout

    # 从 OpenOCD 输出提取 hex 值
    hex_vals = re.findall(r"[0-9a-fA-F]{8}", output)
    words = [int(h, 16) for h in hex_vals]

    return words, output


def parse_motor(words, motor_idx):
    """从 6 个 word 解析单台电机状态"""
    base = motor_idx * 6
    if base + 5 >= len(words):
        return None
    w = words[base:base + 6]
    online = w[2] & 0xFF
    motor_state = (w[2] >> 8) & 0xFF
    mos_temp = (w[2] >> 16) & 0xFF
    coil_temp = (w[2] >> 24) & 0xFF
    return {
        "rx_count": w[0],
        "last_ms": w[1],
        "online": online,
        "motor_state": motor_state,
        "mos_temp": mos_temp,
        "coil_temp": coil_temp,
        "pos_rad": unpack_float(w[3]),
        "vel_radps": unpack_float(w[4]),
        "torque_nm": unpack_float(w[5]),
    }


def main():
    print("# t_ms, M1_rad, M2_rad, M3_rad, M4_rad, M1_vel, M2_vel, M3_vel, M4_vel")
    print("# 通过 OpenOCD SWD 内存直读 (CMSIS-DAP CDC 桥不可用时使用)")
    sys.stdout.flush()

    while True:
        try:
            words, _ = read_motors()

            if len(words) < 24:
                print(f"# WARN: only got {len(words)} words, expected 24",
                      file=sys.stderr, flush=True)
                time.sleep(0.5)
                continue

            motors = []
            for i in range(4):
                m = parse_motor(words, i)
                if m is None:
                    continue
                motors.append(m)

            if len(motors) < 4:
                time.sleep(0.5)
                continue

            t = time.time() * 1000
            print(
                f"{t:.0f},"
                f"{motors[0]['pos_rad']:.4f},{motors[1]['pos_rad']:.4f},"
                f"{motors[2]['pos_rad']:.4f},{motors[3]['pos_rad']:.4f},"
                f"{motors[0]['vel_radps']:.4f},{motors[1]['vel_radps']:.4f},"
                f"{motors[2]['vel_radps']:.4f},{motors[3]['vel_radps']:.4f}"
            )
            sys.stdout.flush()

        except subprocess.TimeoutExpired:
            print("# WARN: OpenOCD timeout", file=sys.stderr, flush=True)
        except KeyboardInterrupt:
            print("\n# stopped", file=sys.stderr)
            break
        except Exception as e:
            print(f"# ERROR: {e}", file=sys.stderr, flush=True)

        time.sleep(0.1)


if __name__ == "__main__":
    main()
