cmake_minimum_required(VERSION 3.20)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_ASM_COMPILER_FORCED TRUE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(motor-controller-samd21 LANGUAGES C CXX ASM)

set(MCU_FAMILY "samd21")
set(MCU_ARCH "cortex-m0plus")
set(MCU "" CACHE STRING "Device variant: G18A or J18A")

string(TOUPPER "${MCU}" MCU_U)

if (MCU_U STREQUAL "G18A")
    set(MCU_MODEL "ATSAMD21G18A")
    set(DEVICE_DEFINE "__SAMD21G18A__")
    set(LD_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/config/sam_d21_xpro/ATSAMD21G18A.ld")
elseif (MCU_U STREQUAL "J18A")
    set(MCU_MODEL "ATSAMD21J18A")
    set(DEVICE_DEFINE "__SAMD21J18A__")
    set(LD_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/src/config/sam_d21_xpro/ATSAMD21J18A.ld")
else ()
    message(FATAL_ERROR "Set -DMCU=G18A or -DMCU=J18A (or use presets).")
endif ()

# ------------------------------------------------------------------------------
# Flags
# ------------------------------------------------------------------------------
set(COMMON_FLAGS "-mcpu=${MCU_ARCH} -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections -Wall -Wextra -g3")
set(OPT_FLAGS "-O3 -flto")

set(CMAKE_C_FLAGS
        "${COMMON_FLAGS} ${OPT_FLAGS} -std=gnu99"
        CACHE INTERNAL "C compiler flags" FORCE
)

set(CMAKE_CXX_FLAGS
        "${COMMON_FLAGS} ${OPT_FLAGS} -std=c++20 -fno-exceptions -fno-rtti -fno-threadsafe-statics"
        CACHE INTERNAL "C++ compiler flags" FORCE
)

set(CMAKE_ASM_FLAGS
        "${COMMON_FLAGS} ${OPT_FLAGS} -x assembler-with-cpp"
        CACHE INTERNAL "ASM compiler flags" FORCE
)

set(CMAKE_EXE_LINKER_FLAGS
        "-mcpu=${MCU_ARCH} -mthumb -mfloat-abi=soft -specs=nano.specs -specs=nosys.specs -T${LD_SCRIPT} -flto \
     -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--no-warn-mismatch"
        CACHE INTERNAL "Linker flags" FORCE
)

# ------------------------------------------------------------------------------
# Defines
# ------------------------------------------------------------------------------
add_compile_definitions(
        ${DEVICE_DEFINE}
        DONT_USE_PREDEFINED_CORE_HANDLERS
        DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS
        ETL_NO_STL
        ETL_TARGET_DEVICE_ARM
        ETL_COMPILER_GCC
        ETL_CPP20_SUPPORTED
)

# ------------------------------------------------------------------------------
# ETL (optional)
# ------------------------------------------------------------------------------
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/etl")
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/etl/CMakeLists.txt")
        add_subdirectory(lib/etl)
    endif ()

    set(ETL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/etl/include")
    if (EXISTS "${ETL_INCLUDE_DIR}")
        message(STATUS "Found ETL at: ${ETL_INCLUDE_DIR}")
    else ()
        message(WARNING "ETL include directory not found at: ${ETL_INCLUDE_DIR}")
    endif ()
else ()
    message(WARNING "ETL directory not found at: ${CMAKE_CURRENT_SOURCE_DIR}/lib/etl")
endif ()

# ------------------------------------------------------------------------------
# Includes
# ------------------------------------------------------------------------------
include_directories(
        src/
        src/config/sam_d21_xpro
        src/config/sam_d21_xpro/peripheral/adc
        src/config/sam_d21_xpro/peripheral/clock
        src/config/sam_d21_xpro/peripheral/dmac
        src/config/sam_d21_xpro/peripheral/eic
        src/config/sam_d21_xpro/peripheral/evsys
        src/config/sam_d21_xpro/peripheral/nvic
        src/config/sam_d21_xpro/peripheral/nvmctrl
        src/config/sam_d21_xpro/peripheral/port
        src/config/sam_d21_xpro/peripheral/sercom/usart
        src/config/sam_d21_xpro/peripheral/sercom/spi_master
        src/config/sam_d21_xpro/peripheral/systick
        src/config/sam_d21_xpro/peripheral/tc
        src/config/sam_d21_xpro/peripheral/tcc

        src/packs/ATSAMD21_DFP
        src/packs/ATSAMD21_DFP/component
        src/packs/ATSAMD21_DFP/instance
        src/packs/ATSAMD21_DFP/pio

        src/packs/CMSIS/CMSIS/Core/Include

        drivers

        ${ETL_INCLUDE_DIR}
)

# ------------------------------------------------------------------------------
# Sources
# ------------------------------------------------------------------------------
set(SOURCES
        # Main application
        src/main.cpp
        src/svpwm.cpp
        src/pid.cpp
        src/pmsm_controller.cpp
        src/spi_buffer.cpp
        src/USART_TxStream.cpp
        src/SAMD21_FOC.cpp
        src/Telemetry.cpp

        # System initialization and configuration
        src/config/sam_d21_xpro/initialization.c
        src/config/sam_d21_xpro/exceptions.c
        src/config/sam_d21_xpro/interrupts.c
        src/config/sam_d21_xpro/startup_gcc.c
        src/config/sam_d21_xpro/libc_syscalls.c

        # Peripheral libraries
        src/config/sam_d21_xpro/peripheral/adc/plib_adc.c
        src/config/sam_d21_xpro/peripheral/clock/plib_clock.c
        src/config/sam_d21_xpro/peripheral/dmac/plib_dmac.c
        src/config/sam_d21_xpro/peripheral/eic/plib_eic.c
        src/config/sam_d21_xpro/peripheral/evsys/plib_evsys.c
        src/config/sam_d21_xpro/peripheral/nvic/plib_nvic.c
        src/config/sam_d21_xpro/peripheral/nvmctrl/plib_nvmctrl.c
        src/config/sam_d21_xpro/peripheral/port/plib_port.c
        src/config/sam_d21_xpro/peripheral/systick/plib_systick.c
        src/config/sam_d21_xpro/peripheral/sercom/usart/plib_sercom3_usart.c
        src/config/sam_d21_xpro/peripheral/sercom/spi_master/plib_sercom5_spi_master.c
        src/config/sam_d21_xpro/peripheral/tc/plib_tc3.c
        src/config/sam_d21_xpro/peripheral/tcc/plib_tcc0.c

        drivers/as5047p.cpp
        drivers/drv8316.cpp
)

# ------------------------------------------------------------------------------
# Targets
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}.elf ${SOURCES})

if (TARGET etl)
    target_link_libraries(${PROJECT_NAME}.elf PRIVATE etl)
    message(STATUS "Linking with ETL target")
endif ()

find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size REQUIRED)

add_custom_target(${PROJECT_NAME}.hex ALL
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
        COMMENT "Creating HEX file"
)

add_custom_target(${PROJECT_NAME}.bin ALL
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
        COMMENT "Creating BIN file"
)

add_custom_target(size ALL
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
        COMMENT "Size information:"
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Target MCU: ${MCU_MODEL}")
message(STATUS "Target architecture: ${MCU_ARCH}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "ASM Compiler: ${CMAKE_ASM_COMPILER}")
message(STATUS "Linker script: ${LD_SCRIPT}")
