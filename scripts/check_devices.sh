#!/usr/bin/env bash
set -euo pipefail

echo "== USB devices =="
if command -v lsusb >/dev/null 2>&1; then
	lsusb
else
	echo "lsusb is not installed."
fi

echo
echo "== ST-LINK check =="
if command -v lsusb >/dev/null 2>&1 && \
	lsusb | grep -Eiq '0483:374[48bf]|0483:375[23]|STMicroelectronics.*ST-LINK'; then
	echo "ST-LINK appears in lsusb."
else
	echo "No ST-LINK found. Plug in ST-LINK or pass it through to Linux."
fi

echo
echo "== Serial ports =="
ports="$(ls /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)"
if [ -n "${ports}" ]; then
	echo "${ports}"
else
	echo "No /dev/ttyUSB*, /dev/ttyACM*, or /dev/serial/by-id/* ports found."
fi

echo
echo "== Helpful commands =="
echo "Build:  ./scripts/build.sh"
echo "Flash:  ./scripts/flash.sh"
echo "Serial: ./scripts/serial.sh auto"
