#!/bin/bash
"""
Setup script for Orbitron MAVLink Interface

This script installs dependencies and sets up the development environment
for the autonomous drone software stack.
"""

set -e  # Exit on any error

echo "=============================================="
echo "Orbitron MAVLink Interface Setup"
echo "=============================================="

# Check Python version
python_version=$(python3 --version 2>&1 | cut -d' ' -f2)
echo "Python version: $python_version"

# Check if we're in a virtual environment
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "Virtual environment: $VIRTUAL_ENV"
else
    echo "Warning: Not in a virtual environment"
    echo "Consider creating one with: python3 -m venv venv && source venv/bin/activate"
fi

# Install system dependencies (if on Debian/Ubuntu)
if command -v apt-get &> /dev/null; then
    echo "Installing system dependencies..."
    sudo apt-get update
    sudo apt-get install -y \
        python3-dev \
        python3-pip \
        build-essential \
        cmake \
        pkg-config \
        libopencv-dev \
        python3-opencv \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        git \
        curl
fi

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Create necessary directories
echo "Creating directory structure..."
mkdir -p logs
mkdir -p data/missions
mkdir -p data/telemetry
mkdir -p data/models
mkdir -p configs

# Set up environment configuration
echo "Setting up environment configuration..."
if [ ! -f ".env" ]; then
    cat > .env << EOF
# Orbitron Environment Configuration
ORBITRON_ENV=companion_computer
MAVLINK_CONNECTION_URL=serial:///dev/ttyAMA0:921600
LOG_LEVEL=INFO
DEBUG_MODE=false
MOCK_HARDWARE=false

# Flight Parameters
DEFAULT_TAKEOFF_ALTITUDE_M=10.0
MAX_FLIGHT_ALTITUDE_M=120.0
MAX_SPEED_MS=15.0

# Safety Settings
GEOFENCE_RADIUS_M=500.0
LOW_BATTERY_THRESHOLD=20.0
CRITICAL_BATTERY_THRESHOLD=10.0

# Hardware Settings
CAMERA_DEVICE=/dev/video0
GPS_DEVICE=/dev/ttyUSB0
LIDAR_DEVICE=/dev/ttyUSB1
EOF
    echo "Created default .env configuration file"
else
    echo ".env file already exists"
fi

# Check for MAVLink simulation
echo "Checking for MAVLink simulation tools..."
if command -v px4_sitl &> /dev/null; then
    echo "PX4 SITL found"
elif command -v ArduPilot &> /dev/null; then
    echo "ArduPilot SITL found"
else
    echo "No MAVLink simulation found"
    echo "For testing, consider installing:"
    echo "  - PX4 SITL: https://docs.px4.io/main/en/simulation/"
    echo "  - ArduPilot SITL: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html"
fi

# Test basic imports
echo "Testing Python imports..."
python3 -c "
try:
    from mavsdk import System
    print('✓ MAVSDK import successful')
except ImportError as e:
    print('✗ MAVSDK import failed:', e)

try:
    from pymavlink import mavutil
    print('✓ PyMAVLink import successful')
except ImportError as e:
    print('✗ PyMAVLink import failed:', e)

try:
    import cv2
    print('✓ OpenCV import successful')
except ImportError as e:
    print('✗ OpenCV import failed:', e)

try:
    import numpy as np
    print('✓ NumPy import successful')
except ImportError as e:
    print('✗ NumPy import failed:', e)
"

# Check hardware access
echo "Checking hardware access..."
if [ -e "/dev/ttyAMA0" ]; then
    echo "✓ Serial port /dev/ttyAMA0 found"
else
    echo "⚠ Serial port /dev/ttyAMA0 not found"
fi

if [ -e "/dev/video0" ]; then
    echo "✓ Camera /dev/video0 found"
else
    echo "⚠ Camera /dev/video0 not found"
fi

# Create test script
echo "Creating test script..."
cat > test_setup.py << 'EOF'
#!/usr/bin/env python3
"""Quick setup test script"""

import asyncio
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

async def test_setup():
    """Test basic setup"""
    print("Testing Orbitron setup...")
    
    try:
        # Test configuration
        from orbitron.config import get_environment_config, validate_config
        
        config = get_environment_config()
        print(f"✓ Configuration loaded: {config.name}")
        
        try:
            validate_config()
            print("✓ Configuration validation passed")
        except ValueError as e:
            print(f"⚠ Configuration warnings: {e}")
        
        # Test MAVLink interface import
        from orbitron.communication.mavlink_interface import MAVLinkInterface
        print("✓ MAVLink interface import successful")
        
        # Test drone controller import
        from orbitron.core.drone_controller import DroneController
        print("✓ Drone controller import successful")
        
        print("\n✓ Setup test completed successfully!")
        return True
        
    except Exception as e:
        print(f"✗ Setup test failed: {e}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_setup())
    sys.exit(0 if success else 1)
EOF

chmod +x test_setup.py

# Run basic test
echo "Running setup test..."
python3 test_setup.py

echo ""
echo "=============================================="
echo "Setup completed!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "1. Review the .env configuration file"
echo "2. Connect your flight controller"
echo "3. Run the demo: python3 examples/mavlink_demo.py"
echo "4. Check the documentation in docs/"
echo ""
echo "For PX4 simulation testing:"
echo "1. Start PX4 SITL: make px4_sitl gazebo"
echo "2. Set ORBITRON_ENV=simulation in .env"
echo "3. Run tests with simulation environment"
echo ""
