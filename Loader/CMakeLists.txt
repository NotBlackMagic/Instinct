cmake_minimum_required(VERSION 3.22)

# Project Name
set(CMAKE_PROJECT_NAME PlumaLoader)
project(${CMAKE_PROJECT_NAME})

# Setup compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(MCU_FLAGS  "-mcpu=cortex-m55 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -mcmse " )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}") 
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -Wl,--gc-sections -Wl,--print-memory-usage")

# Include toolchain file
include("../cmake/gcc-arm-none-eabi.cmake")

# Force optimization options that the toolchain might miss
# add_compile_options(${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti)
# add_link_options(${MCU_FLAGS} -Wl,--gc-sections -Wl,--print-memory-usage)

# Defines
add_compile_definitions(
	STM32N657xx
	USE_FULL_LL_DRIVER
	LOADER_BUILD       # Your custom flag to force Polling Mode in drivers
)

# Includes
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/Source
	${PLUMA_BOARD_PATH}                              # Board/
	${PLUMA_SDK_PATH}/MCU                            # SDK/MCU
	${PLUMA_SDK_PATH}/Drivers                        # SDK/Drivers
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Inc
	${PLUMA_SDK_PATH}/Middleware/CMSIS/Include
	${PLUMA_SDK_PATH}/Middleware/CMSIS/Device/ST/STM32N6xx/Include
)

# Sources
file(GLOB LOADER_SOURCES
	"Source/*.cpp"
	"Source/*.c"
)

set(BOARD_SOURCES 
	${PLUMA_BOARD_PATH}/PlumaN6.cpp
)

set(SDK_SOURCES
	${PLUMA_SDK_PATH}/MCU/system.cpp
	${PLUMA_SDK_PATH}/MCU/hyperbus.cpp
	${PLUMA_SDK_PATH}/Drivers/hyperFlash.cpp
)

set(ST_LL_SOURCES
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_gpio.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_rcc.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_utils.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_xspi.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pwr.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pwr_ex.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_dma.c
)

# Create executable
add_executable(${CMAKE_PROJECT_NAME} 
	${LOADER_SOURCES} 
	${BOARD_SOURCES} 
	${SDK_SOURCES}
	${ST_LL_SOURCES}
)

# Linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/stm32n6xx_extmemloader_stm32cube.ld")
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE 
	-T${LINKER_SCRIPT}
	-Wl,--undefined=Init   # KEEP the Init symbol even if main doesn't call it
)

# Signing & Header Generation
# Following code from: https://community.st.com/t5/stm32-mcus/how-to-add-post-build-commands-in-vs-code/ta-p/827930
set(SIGNING_TOOL "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_SigningTool_CLI.exe")
# set(SIGNING_TOOL "C:/ST/STM32CubeProgrammer/bin/STM32_SigningTool_CLI.exe")

set(TARGET_ELF      "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf")
set(TARGET_BIN      "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")
set(TARGET_SIGNED   "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_Signed.bin")
set(TARGET_FINAL    "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.stldr")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_ELF} ${TARGET_BIN}
	COMMAND ${CMAKE_COMMAND} -E echo y > input.txt
	COMMAND "${SIGNING_TOOL}" -bin "${TARGET_BIN}" -nk -of 0x80000000 -t fsbl -o "${TARGET_SIGNED}" -hv 2.3 -dump "${TARGET_SIGNED}" < input.txt
	COMMAND ${CMAKE_COMMAND} -E copy "${TARGET_SIGNED}" "${TARGET_FINAL}"
	COMMENT "Signing Loader binary and generating ${CMAKE_PROJECT_NAME}.stldr"
)

# Convert the signed ExtLoader .bin back to .elf
# add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B arm --change-addresses=0x80000000 "${TARGET_SIGNED}" "${TARGET_FINAL}"
#     COMMENT "Wrapping signed binary into ELF format"
# )
