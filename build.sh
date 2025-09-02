#!/bin/bash

# Build script for motor-controller-samd21 project

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building motor-controller-samd21 project${NC}"

# Check if ARM toolchain is available
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo -e "${RED}Error: ARM GCC toolchain not found!${NC}"
    echo "Please install arm-none-eabi-gcc toolchain:"
    echo "  Ubuntu/Debian: sudo apt install gcc-arm-none-eabi"
    echo "  Arch Linux: sudo pacman -S arm-none-eabi-gcc"
    echo "  macOS: brew install arm-none-eabi-gcc"
    exit 1
fi

# Create build directory
BUILD_DIR="build"
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Creating build directory...${NC}"
    mkdir -p "$BUILD_DIR"
fi

# Change to build directory
cd "$BUILD_DIR"

echo -e "${YELLOW}Configuring project with CMake...${NC}"
cmake .. -DCMAKE_TOOLCHAIN_FILE=../arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release

echo -e "${YELLOW}Building project...${NC}"
make -j$(nproc)

echo -e "${GREEN}Build completed successfully!${NC}"
echo ""
echo "Generated files:"
echo "  - motor-controller-samd21.elf (ELF executable)"
echo "  - motor-controller-samd21.hex (Intel HEX format)"
echo "  - motor-controller-samd21.bin (Binary format)"
echo "  - motor-controller-samd21.map (Memory map)"

# Show size information
echo ""
echo -e "${YELLOW}Size information:${NC}"
arm-none-eabi-size motor-controller-samd21.elf
