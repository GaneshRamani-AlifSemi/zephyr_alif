# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=AB1C1F4M51820" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/alif_flash.board.cmake)
