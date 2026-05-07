#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

# 解除 CMSIS-DAP HID 驱动占用 (动态查找路径)
HID_PATH=$(find /sys/bus/usb/drivers/usbhid/ -maxdepth 1 -name '*-*:*.*' \
	-exec sh -c 'grep -l "CMSIS-DAP" "$1/product" 2>/dev/null' _ {} \; \
	2>/dev/null | head -1 | xargs dirname | xargs basename 2>/dev/null || true)
if [ -n "${HID_PATH}" ] && [ -e "/sys/bus/usb/drivers/usbhid/${HID_PATH}" ]; then
	echo "Unbinding HID driver from ${HID_PATH}..."
	sudo sh -c "echo '${HID_PATH}' > /sys/bus/usb/drivers/usbhid/unbind" 2>/dev/null || true
fi

sudo openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
	-c "adapter speed 500" \
	-c "program build/zephyr/zephyr.elf verify reset exit"
