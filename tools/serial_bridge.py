#!/usr/bin/env python3
"""
串口 CSV → UDP 桥接，供仿真 Agent 消费。

通过 UDP 每收到一行 CSV 就直接转发，不做聚合。

用法:
  python3 tools/serial_bridge.py --port /dev/ttyACM0 --target 127.0.0.1:9999

需要: pip install pyserial
"""

import argparse
import re
import sys
import time
import socket


def parse_args():
    p = argparse.ArgumentParser(description="DM4310 串口 CSV → UDP 桥接")
    p.add_argument("--port", required=True, help="串口设备, 例如 /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    p.add_argument("--target", default="127.0.0.1:9999",
                   help="UDP 目标地址:端口 (默认 127.0.0.1:9999)")
    p.add_argument("--quiet", action="store_true", help="不打印转发日志")
    return p.parse_args()


# 匹配 CSV 行 (兼容 5 列旧格式和 12 列新格式)
CSV_RE = re.compile(
    r"(\d+),"                    # t_ms
    r"(-?\d+\.\d+),"             # m1
    r"(-?\d+\.\d+),"             # m2
    r"(-?\d+\.\d+),"             # m3
    r"(-?\d+\.\d+)"              # m4 (最少 5 列)
)


def main():
    args = parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("请先安装 pyserial: pip install pyserial")

    host, port_str = args.target.rsplit(":", 1)
    try:
        port_num = int(port_str)
    except ValueError:
        sys.exit(f"无效端口: {port_str}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"无法打开串口 {args.port}: {e}")

    # 启动 CSV 流
    time.sleep(1)
    ser.write(b"\r\n")
    time.sleep(0.1)
    ser.write(b"motor csv on\r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    count = 0
    start_time = time.time()

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                text = raw.decode("utf-8", errors="replace")
            except UnicodeDecodeError:
                continue

            m = CSV_RE.search(text)
            if not m:
                continue

            # 原样转发
            payload = m.group(0).encode("utf-8")
            sock.sendto(payload, (host, port_num))

            count += 1
            if not args.quiet:
                elapsed = time.time() - start_time
                rate = count / elapsed if elapsed > 0 else 0
                print(f"\r→ {host}:{port_num}  {count} 帧  {rate:.0f} Hz", end="", flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"motor csv off\r\n")
        time.sleep(0.05)
        ser.close()
        sock.close()
        if not args.quiet:
            print(f"\n已退出, 共转发 {count} 帧")


if __name__ == "__main__":
    main()
