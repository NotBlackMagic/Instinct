set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

include("${CMAKE_CURRENT_LIST_DIR}/UserConfig.cmake" OPTIONAL RESULT_VARIABLE USER_CONFIG_LOADED)

if(NOT USER_CONFIG_LOADED)
    message(WARNING "UserConfig.cmake not found!")
endif()

set(TOOLCHAIN_PREFIX  "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-")

set(CMAKE_C_COMPILER                "${TOOLCHAIN_PREFIX}gcc.exe")
set(CMAKE_ASM_COMPILER              "${TOOLCHAIN_PREFIX}gcc.exe")
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}g++.exe")
set(CMAKE_LINKER                    "${TOOLCHAIN_PREFIX}g++.exe")
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}objcopy.exe")
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}size.exe")

set(CMAKE_FIND_ROOT_PATH            "${TOOLCHAIN_BIN_DIR}/..")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "${STM32_MCU_FLAGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")

#set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/${STM32_LINKER_SCRIPT}\"")

set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs -u _printf_float")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")
#set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} ${STM32_LINKER_OPTION}")
set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")
