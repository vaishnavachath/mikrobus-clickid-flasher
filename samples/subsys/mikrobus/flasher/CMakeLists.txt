# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(mikrobus-flasher)

zephyr_include_directories(../../../../../include)
include(../../../../cmake/manifesto.cmake)

mnfs_to_mnfb(
../../../../scripts/manifesto/manifests/${CONFIG_MIKROBUS_FLASHER_CLICK_NAME}.mnfs
${ZEPHYR_BINARY_DIR}/include/generated/mikrobus_click_manifest.mnfb
)

generate_inc_file_for_target(
app
${ZEPHYR_BINARY_DIR}/include/generated/mikrobus_click_manifest.mnfb
${ZEPHYR_BINARY_DIR}/include/generated/mikrobus_click_manifest.inc
)

target_sources(app PRIVATE src/main.c)
