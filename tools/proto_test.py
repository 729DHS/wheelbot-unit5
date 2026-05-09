#!/usr/bin/env python3
"""
Unit5 协议测试工具 — 通过 USART1 与小车通信

协议格式: CMD:arg1:arg2\\n  (文本行)
USART1: PA9 TX / PB7 RX, 115200

用法:
  python3 tools/proto_test.py --port /dev/ttyUSB0 move 80 10
  python3 tools/proto_test.py --port /dev/ttyUSB0 status
  python3 tools/proto_test.py --port /dev/ttyUSB0 stream on
  python3 tools/proto_test.py --port /dev/ttyUSB0 stop
"""

import argparse
import serial
import sys
import time


def send_cmd(ser, cmd_str, timeout=2.0):
    """发送命令, 读取响应行"""
    ser.write((cmd_str + "\n").encode("ascii"))
    ser.flush()
    lines = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                lines.append(line)
        else:
            time.sleep(0.01)
    return lines


def cmd_move(ser, args):
    if len(args) < 2:
        print("Usage: move <h_mm> <phi_deg>")
        return
    h, phi = args[0], args[1]
    resp = send_cmd(ser, f"MOVE:{h}:{phi}")
    for line in resp:
        print(f"  {line}")


def cmd_jog_h(ser, args):
    if len(args) < 1:
        print("Usage: jog_h <delta_mm>")
        return
    resp = send_cmd(ser, f"JOG_H:{args[0]}")
    for line in resp:
        print(f"  {line}")


def cmd_jog_phi(ser, args):
    if len(args) < 1:
        print("Usage: jog_phi <delta_deg>")
        return
    resp = send_cmd(ser, f"JOG_PHI:{args[0]}")
    for line in resp:
        print(f"  {line}")


def cmd_stop(ser, args):
    resp = send_cmd(ser, "STOP")
    for line in resp:
        print(f"  {line}")


def cmd_status(ser, args):
    resp = send_cmd(ser, "STATUS")
    for line in resp:
        print(f"  {line}")


def cmd_stream(ser, args):
    if len(args) < 1:
        print("Usage: stream <on|off>")
        return
    resp = send_cmd(ser, f"STREAM:{args[0].upper()}")
    for line in resp:
        print(f"  {line}")

    if args[0].lower() == "on":
        print("Streaming telemetry (Ctrl+C to stop)...")
        try:
            while True:
                if ser.in_waiting:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                    if line:
                        print(line)
        except KeyboardInterrupt:
            send_cmd(ser, "STREAM:OFF")
            print("\nStream stopped.")


def cmd_mode(ser, args):
    if len(args) < 1:
        print("Usage: mode <drag|hold>")
        return
    resp = send_cmd(ser, f"MODE:{args[0].upper()}")
    for line in resp:
        print(f"  {line}")


def main():
    parser = argparse.ArgumentParser(description="Unit5 Protocol Test Tool")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="USART1 serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("cmd", help="Command: move, jog_h, jog_phi, stop, status, stream, mode")
    parser.add_argument("args", nargs="*", help="Command arguments")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        print(f"Connected to {args.port} @ {args.baud}")
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        sys.exit(1)

    cmds = {
        "move": cmd_move,
        "jog_h": cmd_jog_h,
        "jog_phi": cmd_jog_phi,
        "stop": cmd_stop,
        "status": cmd_status,
        "stream": cmd_stream,
        "mode": cmd_mode,
    }

    fn = cmds.get(args.cmd)
    if fn:
        fn(ser, args.args)
    else:
        print(f"Unknown command: {args.cmd}")
        print(f"Available: {', '.join(cmds.keys())}")

    ser.close()


if __name__ == "__main__":
    main()
