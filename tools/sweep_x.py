#!/usr/bin/env python3
"""X轴水平移动 — 保持Y不变, X往复 (丝滑版: 每目标多发逼近delta limit)."""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
Y = int(sys.argv[2]) if len(sys.argv) > 2 else 90
X_MIN = int(sys.argv[3]) if len(sys.argv) > 3 else 60
X_MAX = int(sys.argv[4]) if len(sys.argv) > 4 else 110
CYCLES = int(sys.argv[5]) if len(sys.argv) > 5 else 3

RESEND = 25       # 每个目标重复发送次数 (delta limit 0.06rad → 逐步逼近)
PERIOD_S = 0.015  # 发送间隔 (~66Hz)

ser = serial.Serial(PORT, 115200, timeout=0.1)
time.sleep(0.1)

# 细粒度插值路径
xs = list(range(X_MIN, X_MAX + 1, 2)) + list(range(X_MAX, X_MIN - 1, -2))

print(f'Sweep X: Y={Y}mm, X {X_MIN}↔{X_MAX}mm, {CYCLES} cycles', flush=True)

for c in range(CYCLES):
    for x in xs:
        cmd = f'leg left_xy {x} {Y}\r\nleg right_xy {x} {Y}\r\n'.encode()
        for _ in range(RESEND):
            ser.write(cmd)
            time.sleep(PERIOD_S)
    print(f'Cycle {c+1}/{CYCLES}', flush=True)

ser.close()
print('Done')
