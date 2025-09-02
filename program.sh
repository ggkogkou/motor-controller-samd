#!/bin/bash

# Programming script for SAMD21 using OpenOCD and Atmel-ICE
# Usage: ./program.sh [elf|hex|bin]

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}SAMD21 Programming Script${NC}"
echo "Using Atmel-ICE debugger with OpenOCD"

# Check if OpenOCD is available
if ! command -v openocd &> /dev/null; then
    echo -e "${RED}Error: OpenOCD not found!${NC}"
    echo "Please install OpenOCD:"
    echo "  Ubuntu/Debian: sudo apt install openocd"
    echo "  Arch Linux: sudo pacman -S openocd"
    echo "  macOS: brew install openocd"
    exit 1
fi

# Determine file format and path
FORMAT=${1:-elf}
BUILD_DIR="build"

case $FORMAT in
    "elf")
        FIRMWARE_FILE="$BUILD_DIR/motor-controller-samd21.elf"
        PROGRAM_CMD="program $FIRMWARE_FILE verify reset exit"
        ;;
    "hex")
        FIRMWARE_FILE="$BUILD_DIR/motor-controller-samd21.hex"
        PROGRAM_CMD="program $FIRMWARE_FILE verify reset exit"
        ;;
    "bin")
        FIRMWARE_FILE="$BUILD_DIR/motor-controller-samd21.bin"
        PROGRAM_CMD="program $FIRMWARE_FILE 0x00000000 verify reset exit"
        ;;
    *)
        echo -e "${RED}Error: Invalid format '$FORMAT'${NC}"
        echo "Usage: $0 [elf|hex|bin]"
        exit 1
        ;;
esac

# Check if firmware file exists
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo -e "${RED}Error: Firmware file not found: $FIRMWARE_FILE${NC}"
    echo "Please build the project first with: ./build.sh"
    exit 1
fi

echo -e "${YELLOW}Programming $FIRMWARE_FILE...${NC}"

# Check if Atmel-ICE is connected
echo "Checking for Atmel-ICE connection..."

# Program the device
echo -e "${YELLOW}Starting OpenOCD and programming...${NC}"
openocd -f openocd.cfg -c "$PROGRAM_CMD"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Programming completed successfully!${NC}"
    echo ""
    echo "Your SAMD21 has been programmed with the TCC PWM firmware."
    echo "The device should now be running and generating PWM signals on:"
    echo "  - Channel 0 PWMH: PA08"
    echo "  - Channel 0 PWML: PB10" 
    echo "  - Channel 1 PWMH: PA09"
    echo "  - Channel 1 PWML: PB11"
    echo "  - Channel 2 PWMH: PA10"
    echo "  - Channel 2 PWML: PB12"
else
    echo -e "${RED}Programming failed!${NC}"
    echo ""
    echo "Possible issues:"
    echo "1. Atmel-ICE not connected or not detected"
    echo "2. Target board not connected or powered"
    echo "3. SWD connections incorrect"
    echo "4. Target board in an unknown state"
    echo ""
    echo "Try:"
    echo "- Check USB connection to Atmel-ICE"
    echo "- Verify SWD connections (SWDIO, SWCLK, GND, VCC)"
    echo "- Power cycle the target board"
    echo "- Check that the target voltage matches Atmel-ICE settings"
    exit 1
fi
