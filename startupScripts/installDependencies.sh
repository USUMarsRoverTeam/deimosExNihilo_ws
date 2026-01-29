#!/bin/bash
set -e

echo "=== Installing ROS 2 Jazzy Rover Teleop Dependencies ==="

# Update package list
echo "Updating package list..."
sudo apt update

# Install ROS 2 packages
echo "Installing ROS 2 packages..."
sudo apt install -y \
    ros-jazzy-joy \
    ros-jazzy-micro-ros-agent \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-cmake-python \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs

# Install system tools
echo "Installing system tools..."
sudo apt install -y \
    python3-pip \
    setserial \
    build-essential

# Optional: CPU performance tools
read -p "Install CPU performance tools (msr-tools)? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo apt install -y msr-tools
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install numpy

# Set up user permissions
echo "Setting up user permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G input $USER

echo ""
echo "=== Installation Complete! ==="
echo ""
echo "⚠️  IMPORTANT: You must LOG OUT and LOG BACK IN for group changes to take effect!"
echo ""
echo "After logging back in, verify with:"
echo "  groups"
echo "You should see 'dialout' and 'input' in the list."
echo ""
echo "Then build your workspace:"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select rover_teleop"
echo "  source install/setup.bash"