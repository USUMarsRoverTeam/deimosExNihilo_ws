#!/bin/bash
set -e

echo "=== Starting ROS Jazzy Teleop (Native) ==="

# CPU performance settings (commented out - uncomment if needed)
# sudo wrmsr -a 0x1a0 0x850089
# sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo <<< 0

# Get user home directory
USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)

# Set ROS domain ID
export ROS_DOMAIN_ID=0

# Fix serial device permissions
echo "Setting up serial devices..."
sudo chmod 666 /dev/ttyUSB0 2>/dev/null && echo "✓ Enabled /dev/ttyUSB0" || echo "⚠ /dev/ttyUSB0 not found"
sudo chmod 666 /dev/ttyACM0 2>/dev/null && echo "✓ Enabled /dev/ttyACM0" || echo "⚠ /dev/ttyACM0 not found"

# Source ROS 2 workspace
echo "Sourcing ROS 2 workspace..."
cd $USER_HOME/deimosExNihilo_ws  # Change to your workspace path if different
source install/setup.bash