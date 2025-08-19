# SPDX-License-Identifier: Apache-2.0


board_runner_args(jlink "--device=AE1C1F4051920" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/alif_flash.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
