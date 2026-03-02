# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Madison Gleydura <deepspace00@pm.me>

board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")

include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink_gdbserver.board.cmake)