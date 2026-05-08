#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

# 解除 CMSIS-DAP HID 驱动占用 (动态查找路径)
HID_PRODUCT=$(find /sys/bus/usb/drivers/usbhid/ -maxdepth 2 -name product \
	-exec sh -c 'grep -l "CMSIS-DAP" "$1" 2>/dev/null' _ {} \; \
	2>/dev/null | head -1 || true)
HID_PATH=""
if [ -n "${HID_PRODUCT}" ]; then
	HID_PATH="$(basename "$(dirname "${HID_PRODUCT}")")"
fi
if [ -n "${HID_PATH}" ] && [ -e "/sys/bus/usb/drivers/usbhid/${HID_PATH}" ]; then
	echo "Unbinding HID driver from ${HID_PATH}..."
	sudo sh -c "echo '${HID_PATH}' > /sys/bus/usb/drivers/usbhid/unbind" 2>/dev/null || true
fi

sudo openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
	-c "adapter speed 100" \
	-c "program build/zephyr/zephyr.elf verify reset exit"
