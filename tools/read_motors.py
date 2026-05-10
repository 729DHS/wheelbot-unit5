#!/usr/bin/env python3
"""快速读取 DM4310 电机位置."""
import serial, time, sys, math

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
COUNT = int(sys.argv[2]) if len(sys.argv) > 2 else 1

ser = serial.Serial(PORT, 115200, timeout=0.3)
time.sleep(0.05)

for _ in range(COUNT):
    ser.write(b'motor status\r\n')
    time.sleep(0.4)
    out = b''
    while True:
        d = ser.read(2048)
        if not d: break
        out += d
    text = out.decode(errors='replace')
    ms = [0.0, 0.0, 0.0, 0.0]
    for line in text.split('\n'):
        if 'M1:' in line: ms[0] = float(line.split('pos=')[1].split()[0])
        if 'M2:' in line: ms[1] = float(line.split('pos=')[1].split()[0])
        if 'M3:' in line: ms[2] = float(line.split('pos=')[1].split()[0])
        if 'M4:' in line: ms[3] = float(line.split('pos=')[1].split()[0])
    print(f'M1={math.degrees(ms[0]):+.1f} M2={math.degrees(ms[1]):+.1f} M3={math.degrees(ms[2]):+.1f} M4={math.degrees(ms[3]):+.1f}', flush=True)

ser.close()
