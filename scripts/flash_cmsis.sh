#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

# 解除 HID 驱动占用
HID_PATH="3-1:1.1"
if [ -e "/sys/bus/usb/drivers/usbhid/${HID_PATH}" ]; then
	echo "Unbinding HID driver from ${HID_PATH}..."
	echo "${HID_PATH}" | sudo tee /sys/bus/usb/drivers/usbhid/unbind >/dev/null 2>&1 || true
fi

sudo openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
	-c "adapter speed 500" \
	-c "program build/zephyr/zephyr.elf verify reset exit"
