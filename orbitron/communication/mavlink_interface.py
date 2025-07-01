#!/usr/bin/env python3
"""
PyMAVLink2 Flight Controller Interface for Autonomous Drone Operations

This module provides a comprehensive interface to flight controllers via MAVLink2 protocol
using PyMAVLink for high-performance, low-latency communication.

Features:
- Full MAVLink2 protocol support
- Async/await pattern for non-blocking operations
- Real-time telemetry streaming
- Mission planning and execution
- Parameter management
- Offboard control modes
- Custom message handling
- Failsafe and emergency procedures
- Multi-vehicle support
- High-frequency data streams
"""

import asyncio
import logging
import time
import math
import json
import struct
from typing import (
    Dict, List, Optional, Tuple, Callable, Any, Union, 
    AsyncGenerator, NamedTuple, Awaitable
)
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from contextlib import asynccontextmanager
from collections import defaultdict, deque
import threading
from concurrent.futures import ThreadPoolExecutor

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DroneState(Enum):
    """Drone operational states"""
    UNKNOWN = "unknown"
    INITIALIZING = "initializing"
    READY = "ready"
    ARMED = "armed"
    TAKING_OFF = "taking_off"
    IN_FLIGHT = "in_flight"
    LANDING = "landing"
    LANDED = "landed"
    EMERGENCY = "emergency"
    ERROR = "error"


class FlightMode(Enum):
    """Flight modes"""
    UNKNOWN = 0
    MANUAL = 1
    STABILIZE = 2
    ACRO = 3
    ALT_HOLD = 4
    LOITER = 5
    AUTO = 6
    RTL = 7
    LAND = 8
    GUIDED = 9
    OFFBOARD = 10


@dataclass
class Position:
    """Position data structure"""
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    absolute_altitude_m: float = 0.0
    relative_altitude_m: float = 0.0


@dataclass
class VelocityNed:
    """Velocity in NED frame"""
    north_ms: float = 0.0
    east_ms: float = 0.0
    down_ms: float = 0.0


@dataclass
class EulerAngle:
    """Euler angles"""
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class Quaternion:
    """Quaternion orientation"""
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Health:
    """Vehicle health status"""
    is_gyrometer_calibration_ok: bool = False
    is_accelerometer_calibration_ok: bool = False
    is_magnetometer_calibration_ok: bool = False
    is_level_calibration_ok: bool = False
    is_local_position_ok: bool = False
    is_global_position_ok: bool = False
    is_home_position_ok: bool = False


@dataclass
class Battery:
    """Battery status"""
    voltage_v: float = 0.0
    remaining_percent: float = 0.0
    current_a: float = 0.0


@dataclass
class GpsInfo:
    """GPS information"""
    num_satellites: int = 0
    fix_type: int = 0


@dataclass
class ImuData:
    """IMU data structure"""
    acceleration_ms2: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity_rads: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    magnetic_field_gauss: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    temperature_degc: float = 0.0


@dataclass
class DroneStatus:
    """Comprehensive drone status"""
    # Connection
    connected: bool = False
    heartbeat_age: float = 0.0
    
    # Flight state
    state: DroneState = DroneState.UNKNOWN
    flight_mode: FlightMode = FlightMode.UNKNOWN
    armed: bool = False
    in_air: bool = False
    
    # Position and orientation
    position: Optional[Position] = None
    velocity: Optional[VelocityNed] = None
    attitude: Optional[EulerAngle] = None
    quaternion: Optional[Quaternion] = None
    home_position: Optional[Position] = None
    
    # Health and sensors
    health: Optional[Health] = None
    gps_info: Optional[GpsInfo] = None
    battery: Optional[Battery] = None
    imu_data: Optional[ImuData] = None
    
    # Mission
    mission_current: int = 0
    mission_count: int = 0
    current_mission: Optional[str] = None
    
    # Performance metrics
    telemetry_rate_hz: float = 0.0
    command_latency_ms: float = 0.0


@dataclass
class FlightParameters:
    """Flight configuration parameters"""
    max_speed_ms: float = 15.0
    max_acceleration_ms2: float = 5.0
    max_climb_rate_ms: float = 5.0
    max_descent_rate_ms: float = 3.0
    cruise_altitude_m: float = 50.0
    rtl_altitude_m: float = 100.0
    landing_speed_ms: float = 1.0
    takeoff_speed_ms: float = 2.5
    position_hold_threshold_m: float = 1.0
    yaw_rate_limit_degs: float = 45.0


class MAVLinkInterface:
    """
    PyMAVLink Flight Controller Interface
    """
    
    def __init__(
        self, 
        connection_string: str = "tcp:127.0.0.1:5760",
        system_id: int = 255,
        component_id: int = 1,
        target_system: int = 1,
        target_component: int = 1,
        timeout_s: float = 30.0,
        use_mavlink2: bool = True
    ):
        """
        Initialize MAVLink interface
        
        Args:
            connection_string: Connection string (tcp:host:port, udp:host:port, serial:device:baud)
            system_id: Our system ID
            component_id: Our component ID
            target_system: Target system ID (flight controller)
            target_component: Target component ID
            timeout_s: Connection timeout
            use_mavlink2: Use MAVLink2 protocol
        """
        self.connection_string = connection_string
        self.system_id = system_id
        self.component_id = component_id
        self.target_system = target_system
        self.target_component = target_component
        self.timeout_s = timeout_s
        self.use_mavlink2 = use_mavlink2
        
        # MAVLink connection
        self.connection: Optional[mavutil.mavlink_connection] = None
        
        # Status and monitoring
        self.status = DroneStatus()
        self.flight_params = FlightParameters()
        self._monitoring_tasks: List[asyncio.Task] = []
        self._telemetry_callbacks: Dict[str, List[Callable]] = defaultdict(list)
        
        # Performance tracking
        self._telemetry_timestamps = deque(maxlen=100)
        self._command_timestamps: Dict[str, float] = {}
        self._last_heartbeat_time = 0.0
        
        # Thread pool for blocking operations
        self._executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="mavlink")
        
        # Event synchronization
        self._connected_event = asyncio.Event()
        self._ready_event = asyncio.Event()
        
        # Message sequence tracking
        self._sequence = 0
        
        # Offboard control state
        self._offboard_active = False
        
        logger.info(f"MAVLink interface initialized for system {system_id}")

    async def __aenter__(self):
        """Async context manager entry"""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        await self.disconnect()

    # ============================================================================
    # CONNECTION MANAGEMENT
    # ============================================================================

    async def connect(self) -> bool:
        """
        Establish connection to flight controller
        
        Returns:
            bool: True if connection successful
        """
        if self.status.connected:
            logger.warning("Already connected")
            return True
        
        logger.info(f"Connecting to flight controller: {self.connection_string}")
        
        try:
            # Create connection
            self.connection = await asyncio.get_event_loop().run_in_executor(
                self._executor,
                self._create_connection
            )
            
            if not self.connection:
                raise ConnectionError("Failed to create MAVLink connection")
            
            # Wait for heartbeat
            await self._wait_for_heartbeat()
            
            # Request data streams
            await self._request_data_streams()
            
            # Start monitoring
            await self._start_monitoring()
            
            # Wait for initial telemetry
            await asyncio.wait_for(self._wait_for_initial_telemetry(), timeout=10.0)
            
            self.status.connected = True
            self._connected_event.set()
            
            logger.info("Flight controller connected and ready")
            return True
            
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            await self.disconnect()
            return False

    def _create_connection(self) -> Optional[mavutil.mavlink_connection]:
        try:
            conn_str = self._parse_connection_string()
            connection = mavutil.mavlink_connection(
                conn_str,
                source_system=self.system_id,
                source_component=self.component_id,
                use_native=False,
                force_connected=True
            )
            if self.use_mavlink2:
                connection.mav.srcSystem = self.system_id
                connection.mav.srcComponent = self.component_id
            return connection
        except Exception as e:
            logger.error(f"Failed to create connection: {e}")
            return None

    def _parse_connection_string(self) -> str:
        if self.connection_string.startswith("tcp://"):
            return self.connection_string.replace("tcp://", "tcp:")
        elif self.connection_string.startswith("udp://"):
            return self.connection_string.replace("udp://", "udp:")
        elif self.connection_string.startswith("serial://"):
            parts = self.connection_string.replace("serial://", "").split(":")
            if len(parts) >= 2:
                return f"{parts[0]}:{parts[1]}"
            return parts[0]
        else:
            return self.connection_string

    async def _wait_for_heartbeat(self):
        logger.info("Waiting for heartbeat...")
        
        start_time = time.time()
        while time.time() - start_time < self.timeout_s:
            msg = await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.recv_match(type='HEARTBEAT', blocking=False, timeout=1.0)
            )
            
            if msg:
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                self._last_heartbeat_time = time.time()
                logger.info(f"Heartbeat received from system {self.target_system} component {self.target_component}")
                return
            
            await asyncio.sleep(0.1)
        
        raise TimeoutError("No heartbeat received")

    async def _request_data_streams(self):
        """Request high-frequency data streams"""
        logger.info("Requesting data streams...")        
        streams = [
            (mavlink2.MAV_DATA_STREAM_RAW_SENSORS, 10),      # IMU, GPS
            (mavlink2.MAV_DATA_STREAM_EXTENDED_STATUS, 5),   # Health, battery
            (mavlink2.MAV_DATA_STREAM_RC_CHANNELS, 5),       # RC inputs
            (mavlink2.MAV_DATA_STREAM_POSITION, 10),         # Position, attitude
            (mavlink2.MAV_DATA_STREAM_EXTRA1, 10),           # Attitude
            (mavlink2.MAV_DATA_STREAM_EXTRA2, 5),            # VFR_HUD
            (mavlink2.MAV_DATA_STREAM_EXTRA3, 2),            # AHRS, etc.
        ]

        for stream_id, rate_hz in streams:
            await self._send_data_stream_request(stream_id, rate_hz)
            await asyncio.sleep(0.1)
        
    async def _send_data_stream_request(self, stream_id: int, rate_hz: int, start: bool = True):
        """Send data stream request"""
        try:
            await self._send_command_long(
                mavlink2.MAV_CMD_REQUEST_DATA_STREAM,
                param1=stream_id,
                param2=rate_hz,
                param3=1 if start else 0
            )
        except Exception as e:
            logger.error(f"Failed to request data stream {stream_id}: {e}")

    async def _wait_for_initial_telemetry(self):
        """Wait for initial telemetry data"""
        logger.info("Waiting for initial telemetry...")
        
        required_messages = {'SYS_STATUS', 'GLOBAL_POSITION_INT', 'ATTITUDE'}
        received_messages = set()
        
        start_time = time.time()
        while len(received_messages) < len(required_messages) and time.time() - start_time < 10.0:
            msg = await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.recv_match(blocking=False, timeout=0.1)
            )
            
            if msg:
                msg_type = msg.get_type()
                if msg_type in required_messages:
                    received_messages.add(msg_type)
                    logger.debug(f"Received {msg_type}")
            
            await asyncio.sleep(0.1)
        
        logger.info(f"Initial telemetry received: {received_messages}")

    async def disconnect(self):
        """Disconnect from flight controller"""
        logger.info("Disconnecting from flight controller")
        for task in self._monitoring_tasks:
            task.cancel()
        
        if self._monitoring_tasks:
            await asyncio.gather(*self._monitoring_tasks, return_exceptions=True)
        if self.connection:
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                self.connection.close
            )
            self.connection = None
        
        self.status.connected = False
        self._connected_event.clear()
        self._ready_event.clear()
        
        logger.info("Disconnected from flight controller")

    # ============================================================================
    # TELEMETRY MONITORING
    # ============================================================================

    async def _start_monitoring(self):
        """Start telemetry monitoring tasks"""
        logger.info("Starting telemetry monitoring")
        
        monitors = [
            self._monitor_heartbeat(),
            self._monitor_sys_status(),
            self._monitor_global_position(),
            self._monitor_attitude(),
            self._monitor_battery(),
            self._monitor_imu(),
            self._monitor_gps(),
            self._monitor_mission_current(),
        ]
        
        self._monitoring_tasks = [
            asyncio.create_task(monitor) for monitor in monitors
        ]

    async def _monitor_heartbeat(self):
        """Monitor heartbeat messages"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='HEARTBEAT', blocking=False, timeout=1.0)
                )
                
                if msg:
                    self._last_heartbeat_time = time.time()
                    self.status.heartbeat_age = 0.0
                    
                    # Update flight mode and armed state
                    self.status.flight_mode = FlightMode(msg.custom_mode) if msg.custom_mode < 11 else FlightMode.UNKNOWN
                    self.status.armed = bool(msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED)
                    
                    await self._notify_callbacks("heartbeat", msg)
                else:
                    self.status.heartbeat_age = time.time() - self._last_heartbeat_time
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Heartbeat monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_sys_status(self):
        """Monitor system status"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='SYS_STATUS', blocking=False, timeout=1.0)
                )
                
                if msg:
                    if not self.status.battery:
                        self.status.battery = Battery()
                    
                    self.status.battery.voltage_v = msg.voltage_battery / 1000.0
                    self.status.battery.current_a = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                    self.status.battery.remaining_percent = msg.battery_remaining
                    
                    await self._notify_callbacks("sys_status", msg)
                
                await asyncio.sleep(0.2)
                
            except Exception as e:
                logger.error(f"SYS_STATUS monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_global_position(self):
        """Monitor global position"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=1.0)
                )
                
                if msg:
                    self.status.position = Position(
                        latitude_deg=msg.lat / 1e7,
                        longitude_deg=msg.lon / 1e7,
                        absolute_altitude_m=msg.alt / 1e3,
                        relative_altitude_m=msg.relative_alt / 1e3
                    )                    
                    self.status.velocity = VelocityNed(
                        north_ms=msg.vx / 100.0,
                        east_ms=msg.vy / 100.0,
                        down_ms=msg.vz / 100.0
                    )
                    
                    self._update_telemetry_rate()
                    await self._notify_callbacks("position", msg)
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Global position monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_attitude(self):
        """Monitor attitude"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='ATTITUDE', blocking=False, timeout=1.0)
                )
                
                if msg:
                    # Update attitude
                    self.status.attitude = EulerAngle(
                        roll_deg=math.degrees(msg.roll),
                        pitch_deg=math.degrees(msg.pitch),
                        yaw_deg=math.degrees(msg.yaw)
                    )
                    
                    await self._notify_callbacks("attitude", msg)
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"Attitude monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_battery(self):
        """Monitor battery status"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='BATTERY_STATUS', blocking=False, timeout=1.0)
                )
                
                if msg:
                    if not self.status.battery:
                        self.status.battery = Battery()
                    
                    # Update with more detailed battery info if available
                    if len(msg.voltages) > 0 and msg.voltages[0] != 65535:
                        self.status.battery.voltage_v = msg.voltages[0] / 1000.0
                    
                    if msg.current_battery != -1:
                        self.status.battery.current_a = msg.current_battery / 100.0
                    
                    if msg.battery_remaining != -1:
                        self.status.battery.remaining_percent = msg.battery_remaining
                    
                    await self._notify_callbacks("battery", msg)
                
                await asyncio.sleep(0.5)
                
            except Exception as e:
                logger.error(f"Battery monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_imu(self):
        """Monitor IMU data"""
        while self.status.connected:
            try:
                # Monitor RAW_IMU for high-frequency IMU data
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='RAW_IMU', blocking=False, timeout=1.0)
                )
                
                if msg:
                    self.status.imu_data = ImuData(
                        acceleration_ms2=(msg.xacc / 1000.0, msg.yacc / 1000.0, msg.zacc / 1000.0),
                        angular_velocity_rads=(msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0),
                        magnetic_field_gauss=(msg.xmag / 1000.0, msg.ymag / 1000.0, msg.zmag / 1000.0)
                    )
                    
                    await self._notify_callbacks("imu", msg)
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                logger.error(f"IMU monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_gps(self):
        """Monitor GPS status"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='GPS_RAW_INT', blocking=False, timeout=1.0)
                )
                
                if msg:
                    self.status.gps_info = GpsInfo(
                        num_satellites=msg.satellites_visible,
                        fix_type=msg.fix_type
                    )
                    await self._notify_callbacks("gps", msg)
                await asyncio.sleep(0.5)
                
            except Exception as e:
                logger.error(f"GPS monitoring error: {e}")
                await asyncio.sleep(1.0)

    async def _monitor_mission_current(self):
        """Monitor mission progress"""
        while self.status.connected:
            try:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='MISSION_CURRENT', blocking=False, timeout=1.0)
                )
                
                if msg:
                    self.status.mission_current = msg.seq
                    await self._notify_callbacks("mission_current", msg)
                
                await asyncio.sleep(0.5)
                
            except Exception as e:
                logger.error(f"Mission monitoring error: {e}")
                await asyncio.sleep(1.0)

    def _update_telemetry_rate(self):
        """Update telemetry rate calculation"""
        now = time.time()
        self._telemetry_timestamps.append(now)
        
        if len(self._telemetry_timestamps) >= 2:
            time_span = self._telemetry_timestamps[-1] - self._telemetry_timestamps[0]
            if time_span > 0:
                self.status.telemetry_rate_hz = (len(self._telemetry_timestamps) - 1) / time_span

    async def _notify_callbacks(self, event_type: str, data: Any):
        """Notify registered callbacks"""
        for callback in self._telemetry_callbacks[event_type]:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(data)
                else:
                    callback(data)
            except Exception as e:
                logger.error(f"Callback error for {event_type}: {e}")

    def register_telemetry_callback(self, event_type: str, callback: Callable):
        """Register callback for telemetry events"""
        self._telemetry_callbacks[event_type].append(callback)

    # ============================================================================
    # MESSAGE SENDING
    # ============================================================================

    async def _send_command_long(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        confirmation: int = 0
    ) -> bool:
        """Send COMMAND_LONG message"""
        try:
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    command,
                    confirmation,
                    param1, param2, param3, param4, param5, param6, param7
                )
            )
            return True
        except Exception as e:
            logger.error(f"Failed to send command {command}: {e}")
            return False

    async def _send_set_position_target_local_ned(
        self,
        x: float,
        y: float,
        z: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw: float = 0.0,
        coordinate_frame: int = mavlink2.MAV_FRAME_LOCAL_NED,
        type_mask: int = 0
    ):
        """Send SET_POSITION_TARGET_LOCAL_NED message"""
        try:
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.set_position_target_local_ned_send(
                    int(time.time() * 1000),  # time_boot_ms
                    self.target_system,
                    self.target_component,
                    coordinate_frame,
                    type_mask,
                    x, y, z,  # position
                    vx, vy, vz,  # velocity
                    0, 0, 0,  # acceleration (not used)
                    yaw, 0  # yaw, yaw_rate
                )
            )
            return True
        except Exception as e:
            logger.error(f"Failed to send position target: {e}")
            return False

    async def _send_set_attitude_target(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        roll_rate: float = 0.0,
        pitch_rate: float = 0.0,
        yaw_rate: float = 0.0,
        thrust: float = 0.5
    ):
        """Send SET_ATTITUDE_TARGET message"""
        try:
            # Convert to quaternion
            q = self._euler_to_quaternion(roll, pitch, yaw)
            
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.set_attitude_target_send(
                    int(time.time() * 1000),  # time_boot_ms
                    self.target_system,
                    self.target_component,
                    0,  # type_mask
                    q,  # quaternion [w, x, y, z]
                    roll_rate, pitch_rate, yaw_rate,  # body roll rate, pitch rate, yaw rate
                    thrust  # thrust
                )
            )
            return True
        except Exception as e:
            logger.error(f"Failed to send attitude target: {e}")
            return False

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]

    # ============================================================================
    # FLIGHT OPERATIONS
    # ============================================================================

    async def arm(self) -> bool:
        """Arm the drone"""
        logger.info("Arming drone")
        return await self._send_command_long(
            mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1  # Arm
        )

    async def disarm(self) -> bool:
        """Disarm the drone"""
        logger.info("Disarming drone")
        return await self._send_command_long(
            mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0  # Disarm
        )

    async def takeoff(self, altitude_m: float = None) -> bool:
        """
        Takeoff to specified altitude
        
        Args:
            altitude_m: Target altitude (default from flight params)
        """
        if altitude_m is None:
            altitude_m = self.flight_params.cruise_altitude_m
        
        logger.info(f"Taking off to {altitude_m}m")
        
        return await self._send_command_long(
            mavlink2.MAV_CMD_NAV_TAKEOFF,
            param7=altitude_m
        )

    async def land(self) -> bool:
        """Land the drone"""
        logger.info("Landing drone")
        return await self._send_command_long(mavlink2.MAV_CMD_NAV_LAND)

    async def return_to_launch(self) -> bool:
        """Return to launch position"""
        logger.info("Returning to launch")
        return await self._send_command_long(mavlink2.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    async def goto_position(
        self, 
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        yaw_deg: float = 0.0
    ) -> bool:
        """
        Go to specified global position
        
        Args:
            latitude_deg: Target latitude
            longitude_deg: Target longitude  
            altitude_m: Target altitude (relative to home)
            yaw_deg: Target yaw angle
        """
        logger.info(f"Going to position: {latitude_deg:.6f}, {longitude_deg:.6f}, {altitude_m}m")
        
        return await self._send_command_long(
            mavlink2.MAV_CMD_NAV_WAYPOINT,
            param4=yaw_deg,
            param5=latitude_deg,
            param6=longitude_deg,
            param7=altitude_m
        )

    async def set_flight_mode(self, mode: str) -> bool:
        """Set flight mode"""
        logger.info(f"Setting flight mode to {mode}")
        
        # Map mode strings to MAVLink modes (ArduPilot specific)
        mode_map = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'CIRCLE': 7,
            'POSITION': 8,
            'LAND': 9,
            'OF_LOITER': 10,
            'DRIFT': 11,
            'SPORT': 13,
            'FLIP': 14,
            'AUTOTUNE': 15,
            'POSHOLD': 16,
            'BRAKE': 17,
            'THROW': 18,
            'AVOID_ADSB': 19,
            'GUIDED_NOGPS': 20,
            'SMART_RTL': 21,
            'FLOWHOLD': 22,
            'FOLLOW': 23,
            'ZIGZAG': 24,
            'SYSTEMID': 25,
            'AUTOROTATE': 26,
            'AUTO_RTL': 27
        }
        
        if mode not in mode_map:
            logger.error(f"Unknown flight mode: {mode}")
            return False
        
        return await self._send_command_long(
            mavlink2.MAV_CMD_DO_SET_MODE,
            param1=mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=mode_map[mode]
        )

    async def hold_position(self) -> bool:
        """Hold current position"""
        logger.info("Holding position")
        return await self.set_flight_mode('LOITER')

    # ============================================================================
    # OFFBOARD CONTROL
    # ============================================================================

    async def start_offboard_control(self) -> bool:
        """Start offboard control mode"""
        logger.info("Starting offboard control")
        
        # Send initial setpoint
        success = await self._send_set_position_target_local_ned(0, 0, 0)
        if not success:
            return False
        
        # Enable offboard mode
        success = await self.set_flight_mode('GUIDED')
        if success:
            self._offboard_active = True
            logger.info("Offboard control active")
        
        return success

    async def stop_offboard_control(self) -> bool:
        """Stop offboard control mode"""
        logger.info("Stopping offboard control")
        
        success = await self.set_flight_mode('LOITER')
        if success:
            self._offboard_active = False
            logger.info("Offboard control stopped")
        
        return success

    async def set_velocity_ned(
        self, 
        north_ms: float, 
        east_ms: float, 
        down_ms: float, 
        yaw_deg: float = 0.0
    ) -> bool:
        """Set velocity in NED frame"""
        if not self._offboard_active:
            logger.warning("Offboard control not active")
            return False
        
        # Use velocity control with position target message
        type_mask = (
            mavlink2.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        
        return await self._send_set_position_target_local_ned(
            0, 0, 0,  # position (ignored)
            north_ms, east_ms, down_ms,  # velocity
            math.radians(yaw_deg),  # yaw
            type_mask=type_mask
        )

    async def set_velocity_body(
        self,
        forward_ms: float,
        right_ms: float, 
        down_ms: float,
        yawspeed_degs: float = 0.0
    ) -> bool:
        """Set velocity in body frame"""
        if not self._offboard_active:
            logger.warning("Offboard control not active")
            return False
        
        # Convert body frame to NED frame using current yaw
        if not self.status.attitude:
            logger.warning("No attitude data available for body frame conversion")
            return False
        
        yaw = math.radians(self.status.attitude.yaw_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Transform body velocities to NED
        north_ms = forward_ms * cos_yaw - right_ms * sin_yaw
        east_ms = forward_ms * sin_yaw + right_ms * cos_yaw
        
        return await self.set_velocity_ned(north_ms, east_ms, down_ms, yawspeed_degs)

    async def set_position_ned(
        self,
        north_m: float,
        east_m: float, 
        down_m: float,
        yaw_deg: float = 0.0
    ) -> bool:
        """Set position in NED frame"""
        if not self._offboard_active:
            logger.warning("Offboard control not active")
            return False
        
        # Use position control
        type_mask = (
            mavlink2.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavlink2.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        
        return await self._send_set_position_target_local_ned(
            north_m, east_m, down_m,  # position
            0, 0, 0,  # velocity (ignored)
            math.radians(yaw_deg),  # yaw
            type_mask=type_mask
        )

    # ============================================================================
    # MISSION MANAGEMENT
    # ============================================================================

    async def upload_mission(self, mission_items: List[Dict[str, Any]]) -> bool:
        """Upload mission to flight controller"""
        logger.info(f"Uploading mission with {len(mission_items)} items")
        
        try:
            # Clear existing mission first
            await self._send_command_long(mavlink2.MAV_CMD_MISSION_CLEAR_ALL)
            
            # Set mission count
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.mission_count_send(
                    self.target_system,
                    self.target_component,
                    len(mission_items)
                )
            )
            
            # Upload each mission item
            for i, item in enumerate(mission_items):
                await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.mav.mission_item_int_send(
                        self.target_system,
                        self.target_component,
                        i,  # seq
                        item.get('frame', mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
                        item.get('command', mavlink2.MAV_CMD_NAV_WAYPOINT),
                        0 if i == 0 else 0,  # current
                        1,  # autocontinue
                        item.get('param1', 0),
                        item.get('param2', 0),
                        item.get('param3', 0),
                        item.get('param4', 0),
                        int(item.get('x', 0) * 1e7),  # latitude
                        int(item.get('y', 0) * 1e7),  # longitude
                        item.get('z', 0),  # altitude
                        mavlink2.MAV_MISSION_TYPE_MISSION
                    )
                )
            
            logger.info("Mission uploaded successfully")
            return True
        except Exception as e:
            logger.error(f"Mission upload failed: {e}")
            return False

    async def start_mission(self) -> bool:
        """Start uploaded mission"""
        logger.info("Starting mission")
        try:
            return await self._send_command_long(mavlink2.MAV_CMD_MISSION_START)
        except Exception as e:
            logger.error(f"Mission start failed: {e}")
            return False

    async def pause_mission(self) -> bool:
        """Pause current mission"""
        logger.info("Pausing mission")
        try:
            return await self.set_flight_mode('LOITER')
        except Exception as e:
            logger.error(f"Mission pause failed: {e}")
            return False

    async def clear_mission(self) -> bool:
        """Clear uploaded mission"""
        logger.info("Clearing mission")
        try:
            return await self._send_command_long(mavlink2.MAV_CMD_MISSION_CLEAR_ALL)
        except Exception as e:
            logger.error(f"Mission clear failed: {e}")
            return False

    # ============================================================================
    # PARAMETER MANAGEMENT
    # ============================================================================

    async def get_parameter(self, name: str) -> Optional[Union[int, float]]:
        """Get parameter value"""
        try:
            # Request parameter
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.param_request_read_send(
                    self.target_system,
                    self.target_component,
                    name.encode('ascii'),
                    -1  # param_index
                )
            )
            
            # Wait for response
            start_time = time.time()
            while time.time() - start_time < 5.0:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.1)
                )
                
                if msg and msg.param_id.decode('ascii').rstrip('\x00') == name:
                    return msg.param_value
                
                await asyncio.sleep(0.01)
            
            logger.error(f"Parameter {name} not received")
            return None
            
        except Exception as e:
            logger.error(f"Get parameter {name} failed: {e}")
            return None

    async def set_parameter(self, name: str, value: Union[int, float]) -> bool:
        """Set parameter value"""
        try:
            # Determine parameter type
            param_type = mavlink2.MAV_PARAM_TYPE_REAL32 if isinstance(value, float) else mavlink2.MAV_PARAM_TYPE_INT32
            
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.param_set_send(
                    self.target_system,
                    self.target_component,
                    name.encode('ascii'),
                    float(value),
                    param_type
                )
            )
            
            # Wait for acknowledgment
            start_time = time.time()
            while time.time() - start_time < 5.0:
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.1)
                )
                
                if msg and msg.param_id.decode('ascii').rstrip('\x00') == name:
                    logger.info(f"Parameter {name} set to {value}")
                    return True
                
                await asyncio.sleep(0.01)
            
            logger.error(f"Parameter {name} set confirmation not received")
            return False
            
        except Exception as e:
            logger.error(f"Set parameter {name} failed: {e}")
            return False

    async def get_all_parameters(self) -> Dict[str, Union[int, float]]:
        """Get all parameters"""
        try:
            # Request all parameters
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                lambda: self.connection.mav.param_request_list_send(
                    self.target_system,
                    self.target_component
                )
            )
            
            parameters = {}
            start_time = time.time()
            
            while time.time() - start_time < 30.0:  # 30 second timeout
                msg = await asyncio.get_event_loop().run_in_executor(
                    self._executor,
                    lambda: self.connection.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.1)
                )
                
                if msg:
                    param_name = msg.param_id.decode('ascii').rstrip('\x00')
                    parameters[param_name] = msg.param_value
                    
                    # Check if we have all parameters
                    if len(parameters) >= msg.param_count:
                        break
                
                await asyncio.sleep(0.01)
            
            logger.info(f"Retrieved {len(parameters)} parameters")
            return parameters
            
        except Exception as e:
            logger.error(f"Get all parameters failed: {e}")
            return {}

    # ============================================================================
    # LOW-LEVEL MAVLINK ACCESS
    # ============================================================================

    async def send_mavlink_message(self, message) -> bool:
        """Send custom MAVLink message"""
        if not self.connection:
            logger.error("PyMAVLink connection not available")
            return False
        
        try:
            await asyncio.get_event_loop().run_in_executor(
                self._executor,
                self.connection.mav.send,
                message
            )
            return True
        except Exception as e:
            logger.error(f"Send MAVLink message failed: {e}")
            return False

    async def request_data_stream(
        self, 
        stream_id: int, 
        rate_hz: int, 
        start: bool = True
    ) -> bool:
        """Request data stream at specified rate"""
        if not self.connection:
            return False
        
        try:
            message = self.connection.mav.request_data_stream_encode(
                self.target_system,  # target_system
                self.target_component,  # target_component
                stream_id,  # req_stream_id
                rate_hz,  # req_message_rate
                1 if start else 0  # start_stop
            )
            return await self.send_mavlink_message(message)
        except Exception as e:
            logger.error(f"Request data stream failed: {e}")
            return False

    async def set_message_interval(self, message_id: int, interval_us: int) -> bool:
        """Set message interval for specific message"""
        if not self.connection:
            return False
        
        try:
            return await self._send_command_long(
                mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL,
                param1=message_id,
                param2=interval_us
            )
        except Exception as e:
            logger.error(f"Set message interval failed: {e}")
            return False

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================

    async def _wait_for_altitude(self, target_altitude: float, timeout_s: float = 60.0):
        """Wait for drone to reach target altitude"""
        start_time = time.time()
        
        while time.time() - start_time < timeout_s:
            if self.status.position:
                current_alt = self.status.position.relative_altitude_m
                if abs(current_alt - target_altitude) < 1.0:
                    return
            await asyncio.sleep(0.5)
        
        raise TimeoutError(f"Altitude {target_altitude}m not reached within {timeout_s}s")

    async def _wait_for_disarmed(self, timeout_s: float = 120.0):
        """Wait for drone to disarm"""
        start_time = time.time()
        
        while time.time() - start_time < timeout_s:
            if not self.status.armed:
                return
            await asyncio.sleep(0.5)
        
        raise TimeoutError(f"Drone not disarmed within {timeout_s}s")

    def is_ready_for_flight(self) -> bool:
        """Check if drone is ready for flight"""
        if not self.status.connected or not self.status.health:
            return False
        
        health = self.status.health
        return (
            health.is_gyrometer_calibration_ok and
            health.is_accelerometer_calibration_ok and
            health.is_magnetometer_calibration_ok and
            health.is_local_position_ok and
            health.is_global_position_ok and
            health.is_home_position_ok
        )

    def get_distance_to_home(self) -> Optional[float]:
        """Get distance to home position in meters"""
        if not (self.status.position and self.status.home_position):
            return None
        
        return self._calculate_distance(
            self.status.position.latitude_deg,
            self.status.position.longitude_deg,
            self.status.home_position.latitude_deg,
            self.status.home_position.longitude_deg
        )

    @staticmethod
    def _calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS coordinates"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    # ============================================================================
    # EMERGENCY PROCEDURES
    # ============================================================================

    async def emergency_stop(self) -> bool:
        """Emergency stop - immediate disarm"""
        logger.warning("EMERGENCY STOP ACTIVATED")
        try:
            return await self._send_command_long(
                mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
                param1=0,  # Disarm
                param2=21196  # Force disarm magic number
            )
        except Exception as e:
            logger.error(f"Emergency stop failed: {e}")
            return False

    async def emergency_land(self) -> bool:
        """Emergency land at current position"""
        logger.warning("EMERGENCY LAND ACTIVATED")
        try:
            return await self._send_command_long(mavlink2.MAV_CMD_NAV_LAND)
        except Exception as e:
            logger.error(f"Emergency land failed: {e}")
            return False

    # ============================================================================
    # CLEANUP
    # ============================================================================

    def __del__(self):
        """Cleanup on deletion"""
        if hasattr(self, '_executor'):
            self._executor.shutdown(wait=False)


# ============================================================================
# FACTORY AND CONVENIENCE FUNCTIONS
# ============================================================================

async def create_mavlink_interface(
    connection_string: str = "serial:///dev/ttyAMA0:921600",
    **kwargs
) -> MAVLinkInterface:
    """
    Factory function to create and connect MAVLink interface
    
    Args:
        connection_string: MAVLink connection string
        **kwargs: Additional arguments for MAVLinkInterface
    
    Returns:
        Connected MAVLinkInterface instance
    """
    interface = MAVLinkInterface(connection_string, **kwargs)
    
    if not await interface.connect():
        raise ConnectionError("Failed to connect to flight controller")
    
    return interface


@asynccontextmanager
async def mavlink_connection(connection_string: str = "serial:///dev/ttyAMA0:921600", **kwargs):
    """
    Async context manager for MAVLink connection
    
    Usage:
        async with mavlink_connection() as drone:
            await drone.arm()
            await drone.takeoff()
    """
    interface = None
    try:
        interface = await create_mavlink_interface(connection_string, **kwargs)
        yield interface
    finally:
        if interface:
            await interface.disconnect()


# Example usage
if __name__ == "__main__":
    async def main():
        async with mavlink_connection("tcp:127.0.0.1:5760") as drone:
            print(f"Connected to drone, status: {drone.status}")
            while not drone.is_ready_for_flight():
                await asyncio.sleep(1)
            
            # Basic flight
            await drone.arm()
            await drone.takeoff(20.0)
            await asyncio.sleep(10)
            await drone.land()
    
    asyncio.run(main())
