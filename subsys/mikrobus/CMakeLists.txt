# SPDX-License-Identifier: BSD-3-Clause

include(../../cmake/manifesto.cmake)

zephyr_library()

zephyr_library_sources(
	w1-gpio.c
    mikrobus.c
)