cmake_minimum_required(VERSION 3.22)

# Project Name
set(CMAKE_PROJECT_NAME Instinct)
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
	USE_HAL_DRIVER
	STM32N657xx
	USE_FULL_LL_DRIVER
	USE_RTOS
	TX_INCLUDE_USER_DEFINE_FILE
	TX_SINGLE_MODE_SECURE=1
	FX_INCLUDE_USER_DEFINE_FILE

	ZENOH_THREADX_STM32=1        # Use the built-in ThreadX memory/time hooks
	Z_FEATURE_NETWORK_CUSTOM=1   # Disable POSIX sockets, use your custom serial link
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

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/Config/Version.hpp.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Config/Version.hpp"
)

# Includes
include_directories(
	#
	${CMAKE_CURRENT_BINARY_DIR}/Config
	
	# App Includes
	${CMAKE_CURRENT_SOURCE_DIR}/Config
	${CMAKE_CURRENT_SOURCE_DIR}/Source

	${CMAKE_CURRENT_SOURCE_DIR}/Modules
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Acquisition
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Control
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Estimator
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Messaging
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Messaging/PubSub
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Messaging/PubSub/Messages
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/System
	${CMAKE_CURRENT_SOURCE_DIR}/Modules/Vision

	# Board Includes
	${PLUMA_BOARD_PATH}

	# SDK Includes
	${PLUMA_SDK_PATH}/MCU
	${PLUMA_SDK_PATH}/Drivers
	${PLUMA_SDK_PATH}/Drivers/Radio
	${PLUMA_SDK_PATH}/Drivers/Sensors
	${PLUMA_SDK_PATH}/Drivers/Storage
	${PLUMA_SDK_PATH}/Drivers/USB
	${PLUMA_SDK_PATH}/Drivers/Video

	# Third Party Libraries
	${PLUMA_SDK_PATH}/Middleware/MAVLink

	# Middleware Includes (ThreadX & CMSIS)
	${PLUMA_SDK_PATH}/Middleware/ThreadX/common/inc
	${PLUMA_SDK_PATH}/Middleware/ThreadX/ports/cortex_m55/gnu/inc
	${PLUMA_SDK_PATH}/Middleware/FileX/common/inc
	${PLUMA_SDK_PATH}/Middleware/FileX/ports/inc
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Inc
	${PLUMA_SDK_PATH}/Middleware/CMSIS/Include
	${PLUMA_SDK_PATH}/Middleware/CMSIS/Device/ST/STM32N6xx/Include
	${PLUMA_SDK_PATH}/Middleware/VideoEncoder/inc
	${PLUMA_SDK_PATH}/Middleware/VideoEncoder/src/common
	${PLUMA_SDK_PATH}/Middleware/VideoEncoder/src/h264
	${PLUMA_SDK_PATH}/Middleware/VideoEncoder/src/jpeg
	${PLUMA_SDK_PATH}/Middleware/ZenohPico/include
	${PLUMA_SDK_PATH}/Middleware/ZenohPico/platforms/common
	${PLUMA_SDK_PATH}/Middleware/ZenohPico/platforms/threadx/stm32
)

# Sources
file(GLOB_RECURSE APP_SOURCES 
	"Source/*.cpp" 
	"Source/*.c" 
	"Source/*.S"	# Gets startup.s and tx_initialize_low_level.S
	"Modules/*.cpp"
)

set(BOARD_SOURCES 
	${PLUMA_BOARD_PATH}/PlumaN6.cpp
)

file(GLOB SDK_SOURCES
	"${PLUMA_SDK_PATH}/MCU/*.cpp"
	"${PLUMA_SDK_PATH}/Drivers/*.cpp"
	"${PLUMA_SDK_PATH}/Drivers/Radio/*.cpp"
	"${PLUMA_SDK_PATH}/Drivers/Sensors/*.cpp"
	"${PLUMA_SDK_PATH}/Drivers/Storage/*.cpp"
	"${PLUMA_SDK_PATH}/Drivers/USB/*.cpp"
	"${PLUMA_SDK_PATH}/Drivers/Video/*.cpp"
)

# STM32 HAL/LL Drivers
set(ST_DRIVER_SOURCES
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_exti.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_gpio.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_rcc.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_utils.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_rcc.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_rcc_ex.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_gpio.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_dma.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_dma_ex.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pwr.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_pwr_ex.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_cortex.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_exti.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_usart.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_dma.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_hal_xspi.c
	${PLUMA_SDK_PATH}/Middleware/ST/STM32N6xx_HAL_Driver/Src/stm32n6xx_ll_i2c.c
)

# ThreadX Sources (C and ASM)
set(THREADX_ROOT "${PLUMA_SDK_PATH}/Middleware/ThreadX")
file(GLOB THREADX_SOURCES
	"${THREADX_ROOT}/common/src/*.c"
	"${THREADX_ROOT}/ports/cortex_m55/gnu/src/*.c"
	"${THREADX_ROOT}/ports/cortex_m55/gnu/src/*.S" 
)
list(FILTER THREADX_SOURCES EXCLUDE REGEX "tx_initialize_low_level.S")

# FileX Sources
set(FILEX_ROOT "${PLUMA_SDK_PATH}/Middleware/FileX")
file(GLOB FILEX_SOURCES
	"${FILEX_ROOT}/common/src/*.c"
	"${FILEX_ROOT}/ports/src/*.cpp"
)

# VideoEncoder (VENC) Sources
set(VENC_ROOT "${PLUMA_SDK_PATH}/Middleware/VideoEncoder")
file(GLOB VENC_SOURCES
	"${VENC_ROOT}/src/common/*.c"
	"${VENC_ROOT}/src/h264/*.c"
	"${VENC_ROOT}/src/jpeg/*.c"
)

# Zenoh-Pico Sources
set(ZENOH_ROOT "${PLUMA_SDK_PATH}/Middleware/ZenohPico")
file(GLOB_RECURSE ZENOH_CORE_SOURCES "${ZENOH_ROOT}/src/*.c")
file(GLOB ZENOH_PLATFORM_SOURCES
	"${ZENOH_ROOT}/platforms/common/*.c"
	"${ZENOH_ROOT}/platforms/threadx/stm32/*.c"
)
file(GLOB ZENOH_PORT_SOURCES "${ZENOH_ROOT}/ports/*.cpp")
set(ZENOH_SOURCES ${ZENOH_CORE_SOURCES} ${ZENOH_PLATFORM_SOURCES} ${ZENOH_PORT_SOURCES})

add_executable(${CMAKE_PROJECT_NAME}
	${APP_SOURCES}
	${BOARD_SOURCES}
	${SDK_SOURCES}
	${ST_DRIVER_SOURCES}
	${THREADX_SOURCES}
	${FILEX_SOURCES}
	${VENC_SOURCES}
	${ZENOH_SOURCES}
)

# Linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32N657XX_AXISRAM2_fsbl.ld")
# set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32N657XX_ROMxspi2.ld")
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
		COMMENT "Signing Instinct Firmware binary and generating ${CMAKE_PROJECT_NAME}.stldr"
	)

	# Convert the signed Instinct Firmware .bin back to .elf
	add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B arm --change-addresses=0x70100000 "${TARGET_SIGNED}" "${TARGET_ELF_SIGNED}"
		COMMENT "Converting signed .bin back to .elf in address [0x70100000]"
	)
else()
	message(WARNING "Signing tool not found at ${SIGNING_TOOL_EXE}. Skipping signature generation.")
endif()