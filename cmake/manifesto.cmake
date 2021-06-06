#
# Copyright (c) 2020 Friedt Professional Engineering Services, Inc
#
# SPDX-License-Identifier: BSD-3-Clause

set(G4Z_DIR ${CMAKE_CURRENT_LIST_DIR}/..)

function(mnfs_to_mnfb
    source_file    # The source file to be converted
    generated_file # The generated file
    )
  add_custom_command(
    OUTPUT ${generated_file}
    COMMAND
    ${PYTHON_EXECUTABLE}
    ${G4Z_DIR}/scripts/manifesto/manifesto
    -I mnfs
    -O mnfb
    -o ${generated_file}
    ${source_file}
    DEPENDS ${source_file}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endfunction()
