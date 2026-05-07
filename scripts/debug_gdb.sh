#!/bin/bash
# GDB 调试会话 — 通过 OpenOCD 连接 STM32F407
#
# 用法:
#   ./scripts/debug_gdb.sh [cmsis|stlink]
#
# GDB 内置 motor 快捷命令:
#   motor_enable 1      使能 M1
#   motor_disable 2     失能 M2
#   motor_zero 3        M3 置零
#   motor_status        查看全部状态
#   motor_csv           单次角度输出
#   motor_kp 1 0.5      设置 M1 KP
#   motor_kd 2 0.05     设置 M2 KD

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
GDB_INIT="$PROJECT_DIR/scripts/gdb_init.gdb"
ELF="$PROJECT_DIR/build/zephyr/zephyr.elf"

if [[ ! -f "$ELF" ]]; then
    echo "错误: 找不到 $ELF，请先构建: ./scripts/build.sh"
    exit 1
fi

INTERFACE="${1:-cmsis}"

case "$INTERFACE" in
    cmsis)
        OCD_CFG="interface/cmsis-dap.cfg"
        ;;
    stlink)
        OCD_CFG="interface/stlink.cfg"
        ;;
    *)
        echo "用法: $0 [cmsis|stlink]"
        exit 1
        ;;
esac

OCD_LOG=$(mktemp /tmp/openocd_gdb.XXXXXX.log)
trap "rm -f $OCD_LOG; kill %1 2>/dev/null || true" EXIT

echo "=== 启动 OpenOCD ($INTERFACE) ==="
sudo openocd \
    -f "$OCD_CFG" \
    -f target/stm32f4x.cfg \
    -c "adapter speed 500" \
    > "$OCD_LOG" 2>&1 &

# 等待 OpenOCD 就绪
for i in $(seq 1 10); do
    if grep -q "Listening on port 3333" "$OCD_LOG" 2>/dev/null; then
        echo "OpenOCD 已就绪"
        break
    fi
    sleep 0.5
done

echo "=== 启动 GDB ==="
/opt/zephyr-sdk-0.17.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb \
    -iex "set confirm off" \
    -iex "set pagination off" \
    -iex "add-auto-load-safe-path /" \
    -x "$GDB_INIT" \
    "$ELF"

echo "=== 调试会话结束 ==="
