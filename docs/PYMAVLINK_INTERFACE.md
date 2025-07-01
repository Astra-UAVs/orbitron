# PyMAVLink Interface - Complete Implementation Guide

This document describes the complete PyMAVLink-only implementation for Orbitron's flight controller interface, providing high-performance MAVLink2 communication for autonomous drone operations.

## Overview

The refactored `mavlink_interface.py` provides:
- **Pure PyMAVLink implementation** - No MAVSDK dependencies
- **MAVLink2 protocol support** - Latest protocol with improved performance
- **Async/await pattern** - Non-blocking operations for better performance
- **Real-time telemetry streaming** - High-frequency data updates
- **Comprehensive flight operations** - Arm, takeoff, land, missions, offboard control
- **Parameter management** - Get/set flight controller parameters
- **Emergency procedures** - Safety-first design

## Features Implemented

### ✅ Connection Management
- [x] MAVLink2 protocol connection
- [x] Serial, TCP, UDP connection support
- [x] Automatic heartbeat monitoring
- [x] Connection health checking
- [x] Graceful disconnect handling

### ✅ Telemetry Monitoring
- [x] Real-time position updates (GPS, altitude)
- [x] Attitude data (roll, pitch, yaw)
- [x] Battery status monitoring
- [x] IMU data streaming
- [x] GPS status tracking
- [x] Mission progress monitoring
- [x] System health status

### ✅ Flight Operations
- [x] Arm/disarm commands
- [x] Takeoff with altitude control
- [x] Landing procedures
- [x] Return to launch (RTL)
- [x] Flight mode changes
- [x] Position hold/loiter

### ✅ Offboard Control
- [x] Position control (NED frame)
- [x] Velocity control (NED/Body frame)
- [x] Attitude control
- [x] Offboard mode management

### ✅ Mission Management
- [x] Mission upload to flight controller
- [x] Mission start/pause/clear
- [x] Mission progress monitoring
- [x] Waypoint navigation

### ✅ Parameter Management
- [x] Get individual parameters
- [x] Set parameters with validation
- [x] Bulk parameter retrieval
- [x] Parameter type handling (int/float)

### ✅ Utility Functions
- [x] Distance calculations
- [x] Flight readiness checks
- [x] Emergency procedures
- [x] Telemetry callback system

## Quick Start

### Basic Connection and Flight

```python
import asyncio
from orbitron.communication.mavlink_interface import mavlink_connection

async def basic_flight():
    # Connect to flight controller
    async with mavlink_connection("tcp:127.0.0.1:5760") as drone:
        print(f"Connected! Status: {drone.status}")
        
        # Wait for ready
        while not drone.is_ready_for_flight():
            await asyncio.sleep(1)
        
        # Basic flight sequence
        await drone.set_flight_mode('GUIDED')
        await drone.arm()
        await drone.takeoff(10.0)  # 10 meters
        await asyncio.sleep(10)    # Hold for 10 seconds
        await drone.land()

# Run the example
asyncio.run(basic_flight())
```

### Telemetry Monitoring

```python
async def monitor_telemetry():
    def on_position(msg):
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1e3
        print(f"Position: {lat:.6f}, {lon:.6f}, {alt:.1f}m")
    
    async with mavlink_connection("tcp:127.0.0.1:5760") as drone:
        drone.register_telemetry_callback('position', on_position)
        await asyncio.sleep(30)  # Monitor for 30 seconds
```

### Offboard Control

```python
async def offboard_example():
    async with mavlink_connection("tcp:127.0.0.1:5760") as drone:
        # Setup
        await drone.set_flight_mode('GUIDED')
        await drone.arm()
        await drone.takeoff(10.0)
        
        # Start offboard control
        await drone.start_offboard_control()
        
        # Fly in a square pattern
        await drone.set_velocity_ned(2.0, 0, 0)  # North 2 m/s
        await asyncio.sleep(5)
        await drone.set_velocity_ned(0, 2.0, 0)  # East 2 m/s
        await asyncio.sleep(5)
        await drone.set_velocity_ned(-2.0, 0, 0) # South 2 m/s
        await asyncio.sleep(5)
        await drone.set_velocity_ned(0, -2.0, 0) # West 2 m/s
        await asyncio.sleep(5)
        
        # Stop and land
        await drone.set_velocity_ned(0, 0, 0)
        await drone.stop_offboard_control()
        await drone.land()
```

## Connection Strings

The interface supports various connection methods:

```python
# Serial connection (hardware)
"serial:/dev/ttyAMA0:921600"

# TCP connection (SITL or network)
"tcp:127.0.0.1:5760"

# UDP connection
"udp:127.0.0.1:14550"

# USB serial
"serial:/dev/ttyUSB0:57600"
```

## Testing with SITL

### 1. Install ArduPilot SITL

```bash
# Install ArduPilot development environment
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build for copter
./waf configure --board sitl
./waf copter
```

### 2. Run SITL

```bash
# Start ArduCopter SITL
cd ardupilot/ArduCopter
sim_vehicle.py --vehicle=copter --out=tcp:127.0.0.1:5760
```

### 3. Run Tests

```bash
# Run the comprehensive test suite
cd /home/retro/Projects/orbitron
python test_pymavlink_interface.py

# Or with automatic SITL startup
python test_pymavlink_interface.py --start-sitl
```

### 4. Run Examples

```bash
# Interactive examples
python examples/pymavlink_examples.py
```

## Performance Optimizations

### High-Frequency Data Streams

The interface requests high-frequency data streams for optimal performance:

- **RAW_SENSORS**: 10 Hz (IMU, GPS)
- **EXTENDED_STATUS**: 5 Hz (Health, battery)
- **POSITION**: 10 Hz (Position, attitude)
- **RC_CHANNELS**: 5 Hz (RC inputs)

### Async Design

- All operations are non-blocking using `async/await`
- Background telemetry monitoring with dedicated tasks
- Thread pool for blocking PyMAVLink operations
- Efficient callback system for telemetry events

### Memory Management

- Circular buffers for telemetry data
- Configurable data retention periods
- Automatic cleanup of old data
- Minimal memory footprint

## Hardware Integration

### Raspberry Pi Configuration

```bash
# Enable UART for serial communication
sudo raspi-config
# Interface Options -> Serial Port -> No (login shell) -> Yes (hardware enabled)

# Add to /boot/config.txt
enable_uart=1
dtoverlay=uart0
```

### Pixhawk/Cube Orange Connection

```python
# Serial connection (TELEM2 port typical)
connection_string = "serial:/dev/ttyAMA0:921600"

# Create interface with hardware-specific settings
interface = MAVLinkInterface(
    connection_string=connection_string,
    system_id=255,        # Ground station ID
    target_system=1,      # Pixhawk system ID
    target_component=1,   # Autopilot component
    use_mavlink2=True     # Enable MAVLink2
)
```

## Error Handling

The interface implements comprehensive error handling:

```python
async def robust_flight():
    try:
        async with mavlink_connection("tcp:127.0.0.1:5760") as drone:
            # Check if ready
            if not drone.is_ready_for_flight():
                print("Drone not ready for flight")
                return
            
            # Perform operations with error checking
            if not await drone.arm():
                print("Failed to arm")
                return
            
            if not await drone.takeoff(10.0):
                print("Takeoff failed")
                await drone.emergency_land()
                return
                
    except ConnectionError:
        print("Failed to connect to flight controller")
    except Exception as e:
        print(f"Flight error: {e}")
        # Emergency procedures would be triggered here
```

## Safety Features

### Emergency Procedures

```python
# Emergency stop (immediate disarm)
await drone.emergency_stop()

# Emergency land at current position
await drone.emergency_land()

# Return to launch with safety checks
await drone.return_to_launch()
```

### Health Monitoring

```python
# Check flight readiness
if drone.is_ready_for_flight():
    print("✓ All systems ready")
else:
    health = drone.status.health
    if not health.is_gyrometer_calibration_ok:
        print("✗ Gyro calibration needed")
    if not health.is_global_position_ok:
        print("✗ GPS fix required")
```

### Heartbeat Monitoring

The interface continuously monitors the heartbeat:
- Automatic detection of connection loss
- Heartbeat age tracking
- Automatic failsafe triggering

## Advanced Usage

### Custom Message Handling

```python
# Send custom MAVLink messages
message = drone.connection.mav.command_long_encode(
    target_system, target_component,
    mavlink2.MAV_CMD_DO_SET_SERVO, 0,
    servo_number, pwm_value, 0, 0, 0, 0, 0
)
await drone.send_mavlink_message(message)
```

### Parameter Tuning

```python
# Get current parameter values
alt_hold_p = await drone.get_parameter("PSC_POSZ_P")
print(f"Altitude hold P gain: {alt_hold_p}")

# Set new parameter value
await drone.set_parameter("PSC_POSZ_P", 1.5)

# Bulk parameter operations
all_params = await drone.get_all_parameters()
print(f"Retrieved {len(all_params)} parameters")
```

## Migration from MAVSDK

If migrating from MAVSDK, key differences:

| MAVSDK | PyMAVLink Interface |
|--------|-------------------|
| `mavsdk.System()` | `mavlink_connection()` |
| `drone.action.arm()` | `await drone.arm()` |
| `drone.telemetry.position()` | `drone.status.position` |
| `drone.mission.upload_mission()` | `await drone.upload_mission()` |
| `drone.offboard.set_velocity_ned()` | `await drone.set_velocity_ned()` |

## Troubleshooting

### Common Issues

1. **Connection timeout**
   ```python
   # Increase timeout for slow connections
   interface = MAVLinkInterface(timeout_s=60.0)
   ```

2. **Permission denied on serial**
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

3. **No telemetry data**
   ```python
   # Check data stream requests
   await drone.request_data_stream(mavlink2.MAV_DATA_STREAM_POSITION, 10)
   ```

4. **GPS issues in SITL**
   ```bash
   # Set home position in SITL
   # In MAVProxy: param set GPS_TYPE 1
   ```

### Debug Mode

Enable debug logging for detailed information:

```python
import logging
logging.getLogger('orbitron.communication.mavlink_interface').setLevel(logging.DEBUG)
```

## Performance Metrics

The interface tracks performance metrics:

```python
status = drone.status
print(f"Telemetry rate: {status.telemetry_rate_hz:.1f} Hz")
print(f"Command latency: {status.command_latency_ms:.1f} ms")
print(f"Heartbeat age: {status.heartbeat_age:.1f} s")
```

## Future Enhancements

Planned improvements:
- [ ] Advanced mission planning with complex waypoints
- [ ] Swarm coordination support
- [ ] Enhanced sensor fusion integration
- [ ] Real-time flight path optimization
- [ ] Advanced failure detection and recovery

## Contributing

To contribute to the PyMAVLink interface:

1. Follow the existing async/await patterns
2. Add comprehensive error handling
3. Include unit tests for new features
4. Update documentation for any API changes
5. Test with both SITL and real hardware

## License

This implementation is part of the Orbitron project. See the main project license for details.
