#!/bin/bash

# Simple programming script using the basic OpenOCD config
# Usage: ./program_simple.sh

set -e

echo "Programming SAMD21 with motor-controller-samd21.elf"
echo "Make sure your Atmel-ICE is connected to USB and target board"

# Check if ELF file exists
if [ ! -f "build/motor-controller-samd21.elf" ]; then
    echo "Error: ELF file not found. Run ./build.sh first"
    exit 1
fi

# Program using OpenOCD
openocd -f openocd_simple.cfg -c "program build/motor-controller-samd21.elf verify reset exit"

echo "Programming completed!"
