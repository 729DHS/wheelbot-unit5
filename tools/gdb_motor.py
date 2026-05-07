#!/usr/bin/env python3
"""
GDB 批处理电机控制 — 免进入交互 GDB，一行命令操作电机。

用法:
  python3 tools/gdb_motor.py enable 1         使能 M1
  python3 tools/gdb_motor.py enable "1 2 3 4" 使能全部
  python3 tools/gdb_motor.py disable all       失能全部
  python3 tools/gdb_motor.py zero "1 2 3 4"   全部置零
  python3 tools/gdb_motor.py zero 2            M2 置零
  python3 tools/gdb_motor.py status            查看全部状态
  python3 tools/gdb_motor.py csv               单次角度输出
  python3 tools/gdb_motor.py kp 3 0.5          M3 KP=0.5
  python3 tools/gdb_motor.py kd 1 0.05         M1 KD=0.05
  python3 tools/gdb_motor.py stop              紧急停止

需要: arm-zephyr-eabi-gdb, OpenOCD 已在后台运行 (./scripts/debug_gdb.sh)
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[1]
ELF = PROJECT_DIR / "build" / "zephyr" / "zephyr.elf"

GDB = "/opt/zephyr-sdk-0.17.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb"
GDB_BASE_ARGS = [
    GDB, "--batch",
    "-iex", "set confirm off",
    "-iex", "set pagination off",
]


def gdb_run(*ex_args: str) -> str:
    """运行 GDB 批处理，返回 stdout+stderr"""
    cmd = list(GDB_BASE_ARGS) + [str(ELF)]
    for a in ex_args:
        cmd.extend(["-ex", a])
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    return proc.stdout + "\n" + proc.stderr


def cmd_enable(args):
    ids = _parse_ids(args.motor)
    for mid in ids:
        gdb_run(
            f"target extended-remote :3333",
            f"set g_gdb_cmd[{mid - 1}] = 1",
            "continue",
            "interrupt",
            "quit",
        )
        print(f"M{mid} ENABLE 已排队 → 主循环已执行")


def cmd_disable(args):
    ids = _parse_ids(args.motor)
    for mid in ids:
        gdb_run(
            f"target extended-remote :3333",
            f"set g_gdb_cmd[{mid - 1}] = 2",
            "continue",
            "interrupt",
            "quit",
        )
        print(f"M{mid} DISABLE 已排队 → 主循环已执行")


def cmd_zero(args):
    ids = _parse_ids(args.motor)
    for mid in ids:
        gdb_run(
            f"target extended-remote :3333",
            f"set g_gdb_cmd[{mid - 1}] = 3",
            "continue",
            "interrupt",
            "quit",
        )
        print(f"M{mid} ZERO 已排队 → 主循环已执行")


def cmd_status(args):
    out = gdb_run(
        "target extended-remote :3333",
        'print g_dm4310.online_mask',
        'print g_dm4310.bringup_done',
        'print g_dm4310.loops',
        'print g_dm4310.motor[0]',
        'print g_dm4310.motor[1]',
        'print g_dm4310.motor[2]',
        'print g_dm4310.motor[3]',
        'print g_dm4310.hold_kp[0]',
        'print g_dm4310.hold_kd[0]',
        'print g_gdb_cmd[0]',
        'print g_gdb_cmd[1]',
        'print g_gdb_cmd[2]',
        'print g_gdb_cmd[3]',
        "quit",
    )
    for line in out.splitlines():
        line = line.strip()
        if line and not line.startswith("$") and not line.startswith("0x"):
            print(line)


def cmd_csv(args):
    out = gdb_run(
        "target extended-remote :3333",
        'printf "%u,%.4f,%.4f,%.4f,%.4f\\n", g_dm4310.loops, (double)g_dm4310.motor[0].pos_rad, (double)g_dm4310.motor[1].pos_rad, (double)g_dm4310.motor[2].pos_rad, (double)g_dm4310.motor[3].pos_rad',
        "quit",
    )
    for line in out.splitlines():
        line = line.strip()
        if re.match(r"\d+,-?\d", line):
            print(line)


def cmd_kp(args):
    mid = _parse_single(args.motor)
    out = gdb_run(
        "target extended-remote :3333",
        f"set g_dm4310.hold_kp[{mid - 1}] = (float){args.value}",
        "set g_dm4310.hold_updates = 1",
        f'printf "M{mid} KP = %.3f\\n", (double)g_dm4310.hold_kp[{mid - 1}]',
        "quit",
    )
    for line in out.splitlines():
        if line.strip().startswith(f"M{mid} KP"):
            print(line.strip())


def cmd_kd(args):
    mid = _parse_single(args.motor)
    out = gdb_run(
        "target extended-remote :3333",
        f"set g_dm4310.hold_kd[{mid - 1}] = (float){args.value}",
        "set g_dm4310.hold_updates = 1",
        f'printf "M{mid} KD = %.3f\\n", (double)g_dm4310.hold_kd[{mid - 1}]',
        "quit",
    )
    for line in out.splitlines():
        if line.strip().startswith(f"M{mid} KD"):
            print(line.strip())


def cmd_stop(args):
    out = gdb_run(
        "target extended-remote :3333",
        "call dm4310_stop_all()",
        "quit",
    )
    ret = _extract_return(out)
    print(f"STOP ALL  ret={ret}")


def _parse_ids(arg: str):
    arg = arg.strip()
    if arg.lower() == "all":
        return [1, 2, 3, 4]
    # 空格分隔的多个 ID: "1 2 3 4"
    ids = []
    for part in arg.split():
        mid = int(part)
        if mid < 1 or mid > 4:
            sys.exit(f"无效电机 ID: {mid}")
        ids.append(mid)
    return ids


def _parse_single(arg: str):
    mid = int(arg.strip())
    if mid < 1 or mid > 4:
        sys.exit(f"无效电机 ID: {mid}")
    return mid


def _extract_return(output: str):
    """从 GDB 输出提取函数返回值 ($N = value)"""
    m = re.search(r"\$\d+\s*=\s*(-?\d+)", output)
    if m:
        return m.group(1)
    return "?"


def main():
    if not ELF.exists():
        sys.exit(f"找不到 {ELF}，请先构建: ./scripts/build.sh")

    parser = argparse.ArgumentParser(description="DM4310 GDB 批处理电机控制")
    sub = parser.add_subparsers(dest="cmd")

    p = sub.add_parser("enable", help="使能电机")
    p.add_argument("motor", help="1-4 或 all")
    p = sub.add_parser("disable", help="失能电机")
    p.add_argument("motor", help="1-4 或 all")
    p = sub.add_parser("zero", help="置零")
    p.add_argument("motor", help="1-4 或 all")
    p = sub.add_parser("status", help="查看全部状态")
    p = sub.add_parser("csv", help="单次角度输出")
    p = sub.add_parser("stop", help="紧急停止")

    p = sub.add_parser("kp", help="设置 KP")
    p.add_argument("motor", help="1-4")
    p.add_argument("value", type=float, help="KP 值")
    p = sub.add_parser("kd", help="设置 KD")
    p.add_argument("motor", help="1-4")
    p.add_argument("value", type=float, help="KD 值")

    args = parser.parse_args()
    if args.cmd is None:
        parser.print_help()
        sys.exit(1)

    dispatch = {
        "enable": cmd_enable, "disable": cmd_disable, "zero": cmd_zero,
        "status": cmd_status, "csv": cmd_csv,
        "kp": cmd_kp, "kd": cmd_kd, "stop": cmd_stop,
    }
    dispatch[args.cmd](args)


if __name__ == "__main__":
    main()
