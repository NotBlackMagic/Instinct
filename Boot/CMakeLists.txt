cmake_minimum_required(VERSION 3.22)

# Project Name
set(CMAKE_PROJECT_NAME Boot)
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)

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
)

# Generate firmware version info
string(TIMESTAMP BUILD_DATE "%Y-%m-%d")
string(TIMESTAMP BUILD_TIME "%H:%M:%S")

find_package(Git)
if(GIT_FOUND)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
		WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
		OUTPUT_VARIABLE GIT_HASH
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
else()
	set(GIT_HASH "Unknown")
endif()

set(CMAKE_PROJECT_VERSION "0.1.0")

# Includes
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/Source
	${PLUMA_BOARD_PATH}
	${PLUMA_SDK_PATH}/MCU
	${PLUMA_SDK_PATH}/Drivers
	${PLUMA_SDK_PATH}/Drivers/Storage
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Inc
	${PLUMA_SDK_PATH}/Middleware/CMSIS/Include
	${PLUMA_SDK_PATH}/Middleware/CMSIS/Device/ST/STM32N6xx/Include
)

# Sources
file(GLOB BOOT_SOURCES
	"Source/*.cpp"
	"Source/*.c"
	"Source/*.S"	# Gets startup.s and tx_initialize_low_level.S
)

set(BOARD_SOURCES 
	${PLUMA_BOARD_PATH}/PlumaN6.cpp
)

set(SDK_SOURCES
	${PLUMA_SDK_PATH}/MCU/system.cpp
	${PLUMA_SDK_PATH}/MCU/hyperbus.cpp
	${PLUMA_SDK_PATH}/Drivers/Storage/hyperFlash.cpp
	${PLUMA_SDK_PATH}/Drivers/Storage/hyperRAM.cpp
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
	${BOOT_SOURCES} 
	${BOARD_SOURCES} 
	${SDK_SOURCES}
	${ST_LL_SOURCES}
)

# Linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32N657XX_AXISRAM2_fsbl.ld")
	target_link_options(${CMAKE_PROJECT_NAME} PRIVATE 
	-T${LINKER_SCRIPT}
)

# Signing & Header Generation
# Following code from: https://community.st.com/t5/stm32-mcus/how-to-add-post-build-commands-in-vs-code/ta-p/827930
if(EXISTS "${SIGNING_TOOL_EXE}")
	set(TARGET_ELF			"${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf")
	set(TARGET_ELF_SIGNED	"${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_Signed.elf")
	set(TARGET_BIN			"${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")
	set(TARGET_SIGNED		"${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_Signed.bin")
	set(TARGET_FINAL		"${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.stldr")

	add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET_ELF} ${TARGET_BIN}
		COMMAND ${CMAKE_COMMAND} -E echo y > input.txt
		COMMAND "${SIGNING_TOOL_EXE}" -bin "${TARGET_BIN}" -nk -of 0x80000000 -t fsbl -o "${TARGET_SIGNED}" -hv 2.3 -align -dump "${TARGET_SIGNED}" < input.txt
		COMMAND ${CMAKE_COMMAND} -E copy "${TARGET_SIGNED}" "${TARGET_FINAL}"
		COMMENT "Signing Boot binary and generating ${CMAKE_PROJECT_NAME}.stldr"
	)

	# Convert the signed Boot .bin back to .elf
	add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B arm --change-addresses=0x70000000 "${TARGET_SIGNED}" "${TARGET_ELF_SIGNED}"
		COMMENT "Converting signed FSBL .bin back to .elf in address [0x70000000]"
	)
else()
	message(WARNING "Signing tool not found at ${SIGNING_TOOL_EXE}. Skipping signature generation.")
endif()