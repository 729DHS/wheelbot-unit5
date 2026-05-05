#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

if command -v lsusb >/dev/null 2>&1; then
	if ! lsusb | grep -Eiq '0483:374[48bf]|0483:375[23]|STMicroelectronics.*ST-LINK'; then
		echo "WARNING: No ST-LINK detected by lsusb." >&2
		echo "Make sure ST-LINK is connected and passed through to Linux." >&2
	fi
fi

sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	-c "adapter speed 1000" \
	-c "program build/zephyr/zephyr.elf verify reset exit"
