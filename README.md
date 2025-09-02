# Motor Controller SAMD21 - CMake Migration

This project has been migrated from MPLAB X IDE to a cross-platform CMake build system for the ATSAMD21J18A microcontroller.

## Overview

This is a TCC (Timer Counter for Control) synchronous PWM channels demonstration project for the SAMD21 microcontroller. The project generates PWM signals on multiple channels with incrementing duty cycles.

### PWM Pin Mapping
- Channel 0 PWMH - PA08
- Channel 0 PWML - PB10
- Channel 1 PWMH - PA09
- Channel 1 PWML - PB11
- Channel 2 PWMH - PA10
- Channel 2 PWML - PB12

## Prerequisites

### Required Tools
1. **ARM GCC Toolchain**: arm-none-eabi-gcc
   - Ubuntu/Debian: `sudo apt install gcc-arm-none-eabi`
   - Arch Linux: `sudo pacman -S arm-none-eabi-gcc`
   - macOS: `brew install arm-none-eabi-gcc`
   - Windows: Download from [ARM Developer](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)

2. **CMake**: Version 3.20 or higher
   - Ubuntu/Debian: `sudo apt install cmake`
   - Arch Linux: `sudo pacman -S cmake`
   - macOS: `brew install cmake`
   - Windows: Download from [cmake.org](https://cmake.org/download/)

3. **Make**: Build tool
   - Usually pre-installed on Linux/macOS
   - Windows: Use MinGW or WSL

## Building the Project

### Quick Build (Recommended)
```bash
./build.sh
```

### Manual Build
```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake .. -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)
```

### Build Types
- `Release`: Optimized for size and speed
- `Debug`: Debug symbols included, no optimization

## Output Files

After successful build, you'll find:
- `motor-controller-samd21.elf` - ELF executable for debugging
- `motor-controller-samd21.hex` - Intel HEX format for programming
- `motor-controller-samd21.bin` - Raw binary format
- `motor-controller-samd21.map` - Memory map file

## Project Structure

```
firmware/
├── CMakeLists.txt              # Main CMake configuration
├── arm-none-eabi.cmake         # ARM toolchain file
├── build.sh                    # Quick build script
├── README.md                   # This file
└── src/
    ├── main.c                  # Main application
    ├── config/sam_d21_xpro/    # Target configuration
    │   ├── definitions.h       # System definitions
    │   ├── initialization.c    # System initialization
    │   ├── ATSAMD21J18A.ld     # Linker script
    │   └── peripheral/         # Peripheral libraries
    └── packs/                  # Device support files
```

## Migration Notes

### Changes from MPLAB X
1. **Build System**: Migrated from MPLAB X proprietary build to CMake
2. **Toolchain**: Uses standard ARM GCC instead of XC32
3. **Cross-platform**: Can be built on Linux, macOS, and Windows
4. **IDE Independent**: Can be used with any IDE or text editor

### Maintained Compatibility
- All original source code preserved
- Same peripheral library structure
- Identical linker script
- Same compiler flags and optimizations

## Programming the Device

Use your preferred programming tool:
- **EDBG/CMSIS-DAP**: `openocd` or `edbg`
- **J-Link**: `JLinkExe`
- **ST-Link**: `st-flash` (with appropriate adapter)

Example with OpenOCD:
```bash
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg -c "program motor-controller-samd21.hex verify reset exit"
```

## Development

### Adding Source Files
Edit `CMakeLists.txt` and add your source files to the `SOURCES` variable.

### Adding Include Directories
Use `include_directories()` or `target_include_directories()` in `CMakeLists.txt`.

### Preprocessor Definitions
Add definitions using `add_compile_definitions()` or `target_compile_definitions()`.

## Troubleshooting

### Common Issues
1. **Toolchain not found**: Ensure ARM GCC is in your PATH
2. **CMake version**: Requires CMake 3.20+
3. **Missing dependencies**: Install build-essential or equivalent

### Clean Build
```bash
rm -rf build
./build.sh
```

## Contributing

When adding features:
1. Keep the CMake configuration clean and documented
2. Test builds on multiple platforms if possible
3. Update this README if adding new dependencies or build options
