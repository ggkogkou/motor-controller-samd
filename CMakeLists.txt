cmake_minimum_required(VERSION 3.20)

# Set cross-compilation tools
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Compiler settings
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Project definition
project(motor-controller-samd21 LANGUAGES C ASM)

# MCU specific settings
set(MCU_FAMILY "samd21")
set(MCU_MODEL "ATSAMD21J18A")
set(MCU_ARCH "cortex-m0plus")

# Compiler flags
set(COMMON_FLAGS "-mcpu=${MCU_ARCH} -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections -Wall -Wextra -g3")

# C specific flags
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99" CACHE INTERNAL "C compiler flags")

# ASM specific flags
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "ASM compiler flags")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS 
    "-mcpu=${MCU_ARCH} -mthumb -mfloat-abi=soft -specs=nano.specs -T${CMAKE_CURRENT_SOURCE_DIR}/src/config/sam_d21_xpro/ATSAMD21J18A.ld -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--no-warn-mismatch"
    CACHE INTERNAL "Linker flags"
)

# Preprocessor definitions
add_compile_definitions(
    __SAMD21J18A__
    DONT_USE_PREDEFINED_CORE_HANDLERS
    DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS
)

# Include directories
include_directories(
    src
    src/config/sam_d21_xpro
    src/config/sam_d21_xpro/peripheral/clock
    src/config/sam_d21_xpro/peripheral/evsys
    src/config/sam_d21_xpro/peripheral/nvic
    src/config/sam_d21_xpro/peripheral/nvmctrl
    src/config/sam_d21_xpro/peripheral/port
    src/config/sam_d21_xpro/peripheral/systick
    src/config/sam_d21_xpro/peripheral/tcc
    src/packs/ATSAMD21J18A_DFP
    src/packs/ATSAMD21J18A_DFP/component
    src/packs/ATSAMD21J18A_DFP/instance
    src/packs/ATSAMD21J18A_DFP/pio
    src/packs/CMSIS/CMSIS/Core/Include
)

# Source files
set(SOURCES
    # Main application
    src/main.c
    
    # System initialization and configuration
    src/config/sam_d21_xpro/initialization.c
    src/config/sam_d21_xpro/exceptions.c
    src/config/sam_d21_xpro/interrupts.c
    src/config/sam_d21_xpro/startup_gcc.c
    src/config/sam_d21_xpro/libc_syscalls.c
    
    # Peripheral libraries
    src/config/sam_d21_xpro/peripheral/clock/plib_clock.c
    src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c
    src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c
    src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c
    src/config/sam_d21_xpro/peripheral/port/plib_port.c
    src/config/sam_d21_xpro/peripheral/systick/plib_systick.c
    src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c
    
    # Standard I/O (optional)
    src/config/sam_d21_xpro/stdio/xc32_monitor.c
)

# Create executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Custom target to create hex file
add_custom_target(${PROJECT_NAME}.hex ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Creating HEX file"
)

# Custom target to create binary file
add_custom_target(${PROJECT_NAME}.bin ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Creating BIN file"
)

# Custom target to show size information
add_custom_target(size ALL
    DEPENDS ${PROJECT_NAME}.elf
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Size information:"
)

# Print build configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Target MCU: ${MCU_MODEL}")
message(STATUS "Target architecture: ${MCU_ARCH}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Linker script: ${CMAKE_CURRENT_SOURCE_DIR}/src/config/sam_d21_xpro/ATSAMD21J18A.ld")
