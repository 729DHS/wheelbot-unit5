#!/usr/bin/env python3
"""
DM4310 四电机角度实时终端仪表盘

通过串口自动启用 Shell CSV 流，实时显示 4 路电机角度。

用法:
  python3 tools/monitor_angles.py --port /dev/ttyACM0

需要: pip install pyserial
"""

import argparse
import math
import re
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="DM4310 电机角度终端仪表盘")
    p.add_argument("--port", required=True, help="串口设备, 例如 /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    return p.parse_args()


def rad2deg(r):
    return r * 180.0 / math.pi


def bar(value, vmin=-math.pi, vmax=math.pi, width=20):
    """文本进度条"""
    clamped = max(vmin, min(vmax, value))
    frac = (clamped - vmin) / (vmax - vmin)
    filled = int(frac * width)
    bar_str = "█" * filled + "·" * (width - filled)
    if value > vmax:
        bar_str = bar_str[:width - 1] + ">"
    elif value < vmin:
        bar_str = "<" + bar_str[1:]
    return bar_str


def clear_screen():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


# 匹配 CSV 行: t_ms,m1,m2,m3,m4[,t1,t2,t3,t4,pitch,pitch_rate,dt_us]
# 兼容 5 列 (旧格式) 和 12 列 (新格式)
CSV_RE = re.compile(
    r"(\d+),"                           # t_ms
    r"(-?\d+\.\d+),"                     # m1
    r"(-?\d+\.\d+),"                     # m2
    r"(-?\d+\.\d+),"                     # m3
    r"(-?\d+\.\d+)"                      # m4
    r"(?:,(-?\d+\.\d+)"                  # t1 (可选)
    r",(-?\d+\.\d+)"                     # t2 (可选)
    r",(-?\d+\.\d+)"                     # t3 (可选)
    r",(-?\d+\.\d+)"                     # t4 (可选)
    r",(-?\d+\.\d+)"                     # pitch (可选)
    r",(-?\d+\.\d+)"                     # pitch_rate (可选)
    r",(\d+))?"                           # dt_us (可选)
)


def main():
    args = parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("请先安装 pyserial: pip install pyserial")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"无法打开串口 {args.port}: {e}")

    print(f"已连接 {args.port} @ {args.baud}")
    time.sleep(1)

    # 发送 motor csv on 启动 CSV 流
    ser.write(b"\r\n")
    time.sleep(0.2)
    ser.write(b"motor csv on\r\n")
    time.sleep(0.5)
    # 清空缓冲区
    ser.reset_input_buffer()

    line_count = 0
    start_time = time.time()
    angles = [0.0, 0.0, 0.0, 0.0]
    show_raw = False  # 调试用, 设为 True 可看原始数据

    try:
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print(f"\n串口错误: {e}")
                break

            if not raw:
                continue

            try:
                text = raw.decode("utf-8", errors="replace")
            except UnicodeDecodeError:
                continue

            if show_raw:
                sys.stderr.write(f"RAW: {text!r}\n")

            # 匹配 CSV 数据 (可能被 shell 转义序列包裹)
            m = CSV_RE.search(text)
            if not m:
                continue

            t_ms = int(m.group(1))
            angles[0] = float(m.group(2))
            angles[1] = float(m.group(3))
            angles[2] = float(m.group(4))
            angles[3] = float(m.group(5))
            torques = None
            dt_us = 0
            if m.lastindex and m.lastindex >= 12:
                torques = [
                    float(m.group(6)),
                    float(m.group(7)),
                    float(m.group(8)),
                    float(m.group(9)),
                ]
                dt_us = int(m.group(12))

            line_count += 1
            elapsed = time.time() - start_time
            rate = line_count / elapsed if elapsed > 0 else 0

            clear_screen()

            print("\033[1;36m=== DM4310 电机角度 ===\033[0m")
            print(f"  时间: {t_ms} ms | 速率: {rate:.1f} Hz | 运行: {elapsed:.0f} s | 端口: {args.port}")
            if dt_us:
                print(f"  loop_dt: {dt_us} us (DWT 实测)")
            print()

            motor_names = [
                "M1 (CAN1 左θa)",
                "M2 (CAN1 左θb)",
                "M3 (CAN2 右θa)",
                "M4 (CAN2 右θb)",
            ]

            for i, (name, rad) in enumerate(zip(motor_names, angles)):
                deg = rad2deg(rad)
                color = "\033[1;3%dm" % ((i % 4) + 2)
                reset = "\033[0m"
                t_str = ""
                if torques:
                    t_str = f"  τ={torques[i]:+.3f}Nm"

                print(f"  {color}{name}{reset}{t_str}")
                print(f"    {bar(rad):22s} {rad:+8.4f} rad  {deg:+8.2f}°")
                print()

            print("\033[90m按 Ctrl+C 退出\033[0m")

    except KeyboardInterrupt:
        pass
    finally:
        # 关闭 CSV 流
        ser.write(b"motor csv off\r\n")
        time.sleep(0.1)
        ser.close()
        clear_screen()
        print("已退出")


if __name__ == "__main__":
    main()
