#!/usr/bin/env python3
"""Y轴水平移动."""
import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
X = int(sys.argv[2]) if len(sys.argv) > 2 else 82
CYCLES = int(sys.argv[3]) if len(sys.argv) > 3 else 5

ser = serial.Serial(PORT, 115200, timeout=0.3)
time.sleep(0.1)

for c in range(CYCLES):
    for y in [50, 70, 90, 110, 90, 70]:
        ser.write(f'leg left_xy {X} {y}\r\nleg right_xy {X} {y}\r\n'.encode())
        time.sleep(0.4)
    print(f'Cycle {c+1}/{CYCLES}', flush=True)

ser.close()
print('Done')
