#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-auto}"
BAUD="${2:-115200}"

if [ "${PORT}" = "auto" ]; then
	PORT="$(ls /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1 || true)"
fi

if [ -z "${PORT}" ] || [ ! -e "${PORT}" ]; then
	echo "No USB serial port found." >&2
	echo "Plug in the USB-TTL adapter, then run:" >&2
	echo "  ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* 2>/dev/null" >&2
	echo "Then open it, for example:" >&2
	echo "  ./scripts/serial.sh /dev/ttyUSB0" >&2
	exit 1
fi

if command -v picocom >/dev/null 2>&1; then
	exec picocom --echo -b "${BAUD}" "${PORT}"
fi

if command -v minicom >/dev/null 2>&1; then
	exec minicom -D "${PORT}" -b "${BAUD}"
fi

if command -v screen >/dev/null 2>&1; then
	exec screen "${PORT}" "${BAUD}"
fi

echo "No serial terminal found. Install picocom, minicom, or screen." >&2
exit 1
