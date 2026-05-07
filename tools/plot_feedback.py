#!/usr/bin/env python3
"""
DM4310 反馈数据实时可视化

用法:
  # 从串口实时读取
  python plot_feedback.py --port /dev/ttyACM0 --baud 115200

  # 从保存的 CSV 文件读取
  python plot_feedback.py --file data.csv

CSV 格式: t_ms,M1_rad,M2_rad,M3_rad,M4_rad

需要: pip install pyserial matplotlib
"""

import argparse
import sys
import time
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="DM4310 反馈数据可视化")
    p.add_argument("--port", help="串口设备，例如 /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="波特率")
    p.add_argument("--file", help="CSV 数据文件")
    p.add_argument("--window", type=float, default=5.0, help="显示窗口(秒)")
    p.add_argument("--save", help="保存 CSV 到文件")
    return p.parse_args()


class DataStore:
    def __init__(self):
        self.data = {1: ([], []), 2: ([], []), 3: ([], []), 4: ([], [])}
        self.t0 = None

    def add(self, t_ms, m1, m2, m3, m4):
        if self.t0 is None:
            self.t0 = t_ms
        t_rel = (t_ms - self.t0) / 1000.0
        for mid, val in [(1, m1), (2, m2), (3, m3), (4, m4)]:
            self.data[mid][0].append(t_rel)
            self.data[mid][1].append(val)


def read_serial(port, baud, store, save_file=None):
    import serial
    ser = serial.Serial(port, baud, timeout=1)
    f_out = open(save_file, "w") if save_file else None

    print(f"Reading from {port} @ {baud}...")
    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if f_out:
            f_out.write(line + "\n")
        if not line or line.startswith("#") or line.startswith("="):
            if line:
                print(line)
            continue
        try:
            parts = line.split(",")
            if len(parts) < 5:
                continue
            t_ms = int(parts[0])
            m1 = float(parts[1])
            m2 = float(parts[2])
            m3 = float(parts[3])
            m4 = float(parts[4])
            store.add(t_ms, m1, m2, m3, m4)
        except (ValueError, IndexError):
            continue


def read_file(filepath, store):
    count = 0
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            t_ms = int(parts[0])
            m1 = float(parts[1])
            m2 = float(parts[2])
            m3 = float(parts[3])
            m4 = float(parts[4])
            store.add(t_ms, m1, m2, m3, m4)
            count += 1
    print(f"Loaded {count} frames from {filepath}")


def plot(store, window):
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 5))
    fig.suptitle("DM4310 电机角度 (M1/M2=CAN1 左腿, M3/M4=CAN2 右腿)")

    colors = {1: "tab:blue", 2: "tab:orange", 3: "tab:green", 4: "tab:red"}
    labels = {1: "M1 左θa", 2: "M2 左θb", 3: "M3 右θa", 4: "M4 右θb"}

    lines = {}
    for mid in [1, 2, 3, 4]:
        lines[mid], = ax.plot([], [], color=colors[mid],
                               label=labels[mid], linewidth=0.8)

    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_ylabel("角度 (rad)")
    ax.set_xlabel("时间 (s)")

    while True:
        all_t = []
        for mid in [1, 2, 3, 4]:
            ts, vals = store.data[mid]
            if ts:
                lines[mid].set_data(ts, vals)
                all_t.append(max(ts))

        if all_t:
            t_max = max(all_t)
            t_min = max(0, t_max - window)
            ax.set_xlim(t_min, t_max)

        ax.relim()
        ax.autoscale_view(scalex=False)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.1)


def main():
    args = parse_args()
    store = DataStore()

    if args.file:
        read_file(args.file, store)
        plot(store, args.window)

    elif args.port:
        import threading
        t = threading.Thread(target=read_serial,
                             args=(args.port, args.baud, store, args.save),
                             daemon=True)
        t.start()
        time.sleep(1)
        plot(store, args.window)

    else:
        print("请指定 --port 或 --file")
        sys.exit(1)


if __name__ == "__main__":
    main()
