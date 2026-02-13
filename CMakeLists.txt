cmake_minimum_required(VERSION 3.22)

# Point to your toolchain file
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/gcc-arm-none-eabi.cmake")

# Project Name
set(CMAKE_PROJECT_NAME Instinct)
project(${CMAKE_PROJECT_NAME})

# Optimization: Strip absolute paths from __FILE__ macros to save Flash memory
add_compile_options("-fmacro-prefix-map=${CMAKE_SOURCE_DIR}=.")

# These variables allow sub-projects to find the SDK without hardcoded ".." paths
set(PLUMA_BOARD_PATH   ${CMAKE_SOURCE_DIR}/Board)
set(PLUMA_SDK_PATH     ${CMAKE_SOURCE_DIR}/SDK)

# A. Manufacturing Tool (The Flash Loader)
# add_subdirectory(Loader)

# B. First Stage Bootloader (FSBL)
# add_subdirectory(Boot)

# C. Main Application
add_subdirectory(Instinct)

