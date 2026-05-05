# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F407IG" "--speed=4000")
board_runner_args(openocd --cmd-pre-init "source [find interface/cmsis-dap.cfg]")
board_runner_args(openocd --cmd-pre-init "source [find target/stm32f4x.cfg]")
board_runner_args(openocd --cmd-pre-init "adapter speed 1000")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
