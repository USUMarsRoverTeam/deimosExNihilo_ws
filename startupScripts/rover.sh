#!/bin/bash
set -e

echo "=== Starting ROS Jazzy Teleop ==="

# Check if running with sudo (for CPU performance settings)
if [ "$EUID" -ne 0 ]; then 
    echo "Error: Please run with sudo"
    exit 1
fi

# CPU performance settings (optional - remove if you don't need these)
echo "Configuring CPU performance..."
wrmsr -a 0x1a0 0x850089 2>/dev/null || echo "Warning: wrmsr failed (this is usually fine)"
tee /sys/devices/system/cpu/intel_pstate/no_turbo <<< 0 2>/dev/null || echo "Warning: CPU turbo setting failed (this is usually fine)"

# Get the actual user (since we're using sudo)
ACTUAL_USER=$SUDO_USER
USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)

# Detect and set permissions for serial devices
echo "Checking for serial devices..."
for dev in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyACM1; do
    if [ -e "$dev" ]; then
        chmod 666 $dev
        echo "✓ Found and enabled: $dev"
    fi
done

# Source ROS 2 workspace
echo "Sourcing ROS 2 workspace..."
cd $USER_HOME/deimosExNihilo_ws  # Or wherever your workspace is
source install/setup.bash