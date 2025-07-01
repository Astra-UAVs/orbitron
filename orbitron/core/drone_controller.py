#!/usr/bin/env python3
"""
Enhanced Drone Controller using Advanced MAVLink Interface

This module provides high-level drone control capabilities built on top of
the comprehensive MAVLink interface.
"""

import asyncio
import logging
import time
from typing import Optional, Dict, List, Callable, Any
from dataclasses import dataclass
from enum import Enum

from ..communication.mavlink_interface import MAVLinkInterface, DroneStatus, FlightParameters
from ..config import get_mavlink_url, get_environment_config

logger = logging.getLogger(__name__)


class AutonomousMode(Enum):
    """Autonomous operation modes"""
    MANUAL = "manual"
    ASSISTED = "assisted"
    AUTONOMOUS = "autonomous"
    EMERGENCY = "emergency"


@dataclass
class FlightPlan:
    """Flight plan definition"""
    waypoints: List[Dict[str, float]]
    mission_type: str
    parameters: Dict[str, Any]
    safety_constraints: Dict[str, float]


class DroneController:
    """
    High-level drone controller with autonomous capabilities
    
    Provides intelligent flight control, mission management, and safety features
    built on the MAVLink interface.
    """
    
    def __init__(
        self,
        connection_url: Optional[str] = None,
        auto_connect: bool = True
    ):
        self.connection_url = connection_url or get_mavlink_url()
        self.mavlink = MAVLinkInterface(self.connection_url)
        
        # Controller state
        self.autonomous_mode = AutonomousMode.MANUAL
        self.current_mission: Optional[FlightPlan] = None
        self.safety_checks_enabled = True
        
        # Callbacks
        self._mission_callbacks: List[Callable] = []
        self._safety_callbacks: List[Callable] = []
        
        # Performance monitoring
        self._last_command_time = 0.0
        self._command_history: List[Dict] = []
        
        # Initialize telemetry callbacks
        self._setup_telemetry_callbacks()
        
        logger.info(f"Drone controller initialized with connection: {self.connection_url}")
        
        if auto_connect:
            asyncio.create_task(self._auto_connect())

    async def _auto_connect(self):
        """Automatically connect to flight controller"""
        try:
            await self.connect()
        except Exception as e:
            logger.error(f"Auto-connect failed: {e}")

    def _setup_telemetry_callbacks(self):
        """Setup telemetry monitoring callbacks"""
        self.mavlink.register_telemetry_callback("battery", self._monitor_battery)
        self.mavlink.register_telemetry_callback("health", self._monitor_health)
        self.mavlink.register_telemetry_callback("position", self._monitor_position)
        self.mavlink.register_telemetry_callback("mission_progress", self._monitor_mission)

    # ============================================================================
    # CONNECTION MANAGEMENT
    # ============================================================================

    async def connect(self) -> bool:
        """Connect to flight controller"""
        logger.info("Connecting drone controller...")
        
        success = await self.mavlink.connect()
        if success:
            logger.info("Drone controller connected and ready")
            await self._perform_initial_checks()
        
        return success

    async def disconnect(self):
        """Disconnect from flight controller"""
        logger.info("Disconnecting drone controller...")
        await self.mavlink.disconnect()

    async def _perform_initial_checks(self):
        """Perform initial system checks"""
        logger.info("Performing initial system checks...")
        
        # Wait for telemetry
        max_wait = 10.0
        start_time = time.time()
        
        while time.time() - start_time < max_wait:
            if self.mavlink.status.health and self.mavlink.status.position:
                break
            await asyncio.sleep(0.5)
        
        # Check system health
        if not self.is_system_healthy():
            logger.warning("System health checks failed")
        
        # Check readiness
        if self.mavlink.is_ready_for_flight():
            logger.info("Drone is ready for flight operations")
        else:
            logger.warning("Drone not ready for flight - check sensors and calibration")

    # ============================================================================
    # STATUS AND MONITORING
    # ============================================================================

    @property
    def status(self) -> DroneStatus:
        """Get current drone status"""
        return self.mavlink.status

    @property
    def is_connected(self) -> bool:
        """Check if connected to flight controller"""
        return self.mavlink.status.connected

    @property
    def is_armed(self) -> bool:
        """Check if drone is armed"""
        return self.mavlink.status.armed

    @property
    def is_in_air(self) -> bool:
        """Check if drone is airborne"""
        return self.mavlink.status.in_air

    def is_system_healthy(self) -> bool:
        """Check overall system health"""
        if not self.status.health:
            return False
        
        health = self.status.health
        critical_checks = [
            health.is_gyrometer_calibration_ok,
            health.is_accelerometer_calibration_ok,
            health.is_magnetometer_calibration_ok,
            health.is_local_position_ok,
            health.is_global_position_ok
        ]
        
        return all(critical_checks)

    def get_battery_status(self) -> Dict[str, float]:
        """Get detailed battery status"""
        if not self.status.battery:
            return {}
        
        battery = self.status.battery
        return {
            "voltage_v": battery.voltage_v,
            "remaining_percent": battery.remaining_percent,
            "current_a": getattr(battery, 'current_battery_a', 0.0),
            "temperature_c": getattr(battery, 'temperature_degc', 0.0)
        }

    def get_position_info(self) -> Dict[str, float]:
        """Get detailed position information"""
        if not self.status.position:
            return {}
        
        pos = self.status.position
        return {
            "latitude_deg": pos.latitude_deg,
            "longitude_deg": pos.longitude_deg,
            "absolute_altitude_m": pos.absolute_altitude_m,
            "relative_altitude_m": pos.relative_altitude_m
        }

    # ============================================================================
    # BASIC FLIGHT OPERATIONS
    # ============================================================================

    async def arm(self, force: bool = False) -> bool:
        """
        Arm the drone
        
        Args:
            force: Force arming even if pre-arm checks fail
        """
        if not force and not self._pre_arm_checks():
            logger.error("Pre-arm checks failed")
            return False
        
        logger.info("Arming drone...")
        success = await self.mavlink.arm()
        
        if success:
            self._log_command("arm")
            await self._notify_safety_callbacks("armed", True)
        
        return success

    async def disarm(self, force: bool = False) -> bool:
        """
        Disarm the drone
        
        Args:
            force: Force disarming even if in air
        """
        if not force and self.is_in_air:
            logger.error("Cannot disarm while in air (use force=True to override)")
            return False
        
        logger.info("Disarming drone...")
        success = await self.mavlink.disarm()
        
        if success:
            self._log_command("disarm")
            await self._notify_safety_callbacks("armed", False)
        
        return success

    async def takeoff(self, altitude_m: float = 10.0, wait_for_completion: bool = True) -> bool:
        """
        Takeoff to specified altitude
        
        Args:
            altitude_m: Target altitude in meters
            wait_for_completion: Wait for takeoff to complete
        """
        if not self._pre_flight_checks():
            logger.error("Pre-flight checks failed")
            return False
        
        logger.info(f"Taking off to {altitude_m}m altitude...")
        
        # Arm if not already armed
        if not self.is_armed:
            if not await self.arm():
                return False
        
        success = await self.mavlink.takeoff(altitude_m)
        
        if success:
            self._log_command("takeoff", {"altitude_m": altitude_m})
            
            if wait_for_completion:
                await self._wait_for_takeoff_completion(altitude_m)
        
        return success

    async def land(self, wait_for_completion: bool = True) -> bool:
        """
        Land the drone
        
        Args:
            wait_for_completion: Wait for landing to complete
        """
        logger.info("Landing drone...")
        
        success = await self.mavlink.land()
        
        if success:
            self._log_command("land")
            
            if wait_for_completion:
                await self._wait_for_landing_completion()
        
        return success

    async def return_to_launch(self) -> bool:
        """Return to launch position"""
        logger.info("Returning to launch...")
        
        success = await self.mavlink.return_to_launch()
        
        if success:
            self._log_command("rtl")
        
        return success

    async def hold_position(self) -> bool:
        """Hold current position"""
        logger.info("Holding position...")
        
        success = await self.mavlink.hold_position()
        
        if success:
            self._log_command("hold")
        
        return success

    # ============================================================================
    # AUTONOMOUS FLIGHT OPERATIONS
    # ============================================================================

    async def goto_location(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float,
        yaw_deg: Optional[float] = None
    ) -> bool:
        """
        Fly to specified GPS location
        
        Args:
            latitude_deg: Target latitude
            longitude_deg: Target longitude
            altitude_m: Target altitude (relative to home)
            yaw_deg: Target yaw angle (optional)
        """
        if not self._autonomous_flight_checks():
            return False
        
        yaw = yaw_deg if yaw_deg is not None else float('nan')
        
        logger.info(f"Flying to location: {latitude_deg:.6f}, {longitude_deg:.6f}, {altitude_m}m")
        
        success = await self.mavlink.goto_position(
            latitude_deg, longitude_deg, altitude_m, yaw
        )
        
        if success:
            self._log_command("goto", {
                "latitude_deg": latitude_deg,
                "longitude_deg": longitude_deg,
                "altitude_m": altitude_m,
                "yaw_deg": yaw_deg
            })
        
        return success

    async def follow_path(self, waypoints: List[Dict[str, float]]) -> bool:
        """
        Follow a path defined by waypoints
        
        Args:
            waypoints: List of waypoint dictionaries with lat, lon, alt keys
        """
        if not waypoints:
            logger.error("Empty waypoint list")
            return False
        
        if not self._autonomous_flight_checks():
            return False
        
        logger.info(f"Following path with {len(waypoints)} waypoints")
        
        for i, waypoint in enumerate(waypoints):
            logger.info(f"Flying to waypoint {i + 1}/{len(waypoints)}")
            
            success = await self.goto_location(
                waypoint["lat"],
                waypoint["lon"],
                waypoint["alt"],
                waypoint.get("yaw")
            )
            
            if not success:
                logger.error(f"Failed to reach waypoint {i + 1}")
                return False
            
            # Wait for arrival
            await self._wait_for_position_reached(waypoint)
        
        logger.info("Path following completed")
        return True

    async def orbit_location(
        self,
        center_lat: float,
        center_lon: float,
        radius_m: float,
        altitude_m: float,
        speed_ms: float = 5.0,
        clockwise: bool = True,
        orbits: int = 1
    ) -> bool:
        """
        Orbit around a specified location
        
        Args:
            center_lat: Center latitude
            center_lon: Center longitude
            radius_m: Orbit radius in meters
            altitude_m: Orbit altitude
            speed_ms: Orbit speed
            clockwise: Orbit direction
            orbits: Number of orbits to complete
        """
        if not self._autonomous_flight_checks():
            return False
        
        logger.info(f"Orbiting location {center_lat:.6f}, {center_lon:.6f} at {radius_m}m radius")
        
        # Generate orbit waypoints
        import math
        waypoints = []
        points_per_orbit = 16
        angle_step = 2 * math.pi / points_per_orbit
        
        for orbit in range(orbits):
            for i in range(points_per_orbit):
                angle = i * angle_step
                if not clockwise:
                    angle = -angle
                
                # Calculate waypoint position
                lat_offset = radius_m * math.cos(angle) / 111320.0  # Approximate meters per degree
                lon_offset = radius_m * math.sin(angle) / (111320.0 * math.cos(math.radians(center_lat)))
                
                waypoints.append({
                    "lat": center_lat + lat_offset,
                    "lon": center_lon + lon_offset,
                    "alt": altitude_m
                })
        
        return await self.follow_path(waypoints)

    # ============================================================================
    # MISSION MANAGEMENT
    # ============================================================================

    async def load_mission(self, mission_plan: FlightPlan) -> bool:
        """Load a mission plan"""
        logger.info(f"Loading mission: {mission_plan.mission_type}")
        
        # Validate mission
        if not self._validate_mission(mission_plan):
            logger.error("Mission validation failed")
            return False
        
        self.current_mission = mission_plan
        
        # Convert to MAVLink mission items
        mission_items = self._convert_to_mission_items(mission_plan)
        
        success = await self.mavlink.upload_mission(mission_items)
        
        if success:
            logger.info("Mission loaded successfully")
            await self._notify_mission_callbacks("mission_loaded", mission_plan)
        
        return success

    async def start_mission(self) -> bool:
        """Start the loaded mission"""
        if not self.current_mission:
            logger.error("No mission loaded")
            return False
        
        logger.info("Starting mission...")
        
        success = await self.mavlink.start_mission()
        
        if success:
            await self._notify_mission_callbacks("mission_started", self.current_mission)
        
        return success

    async def pause_mission(self) -> bool:
        """Pause current mission"""
        logger.info("Pausing mission...")
        
        success = await self.mavlink.pause_mission()
        
        if success:
            await self._notify_mission_callbacks("mission_paused", self.current_mission)
        
        return success

    async def resume_mission(self) -> bool:
        """Resume paused mission"""
        logger.info("Resuming mission...")
        
        success = await self.mavlink.start_mission()
        
        if success:
            await self._notify_mission_callbacks("mission_resumed", self.current_mission)
        
        return success

    # ============================================================================
    # SAFETY AND EMERGENCY PROCEDURES
    # ============================================================================

    async def emergency_stop(self) -> bool:
        """Emergency stop - immediate disarm"""
        logger.critical("EMERGENCY STOP ACTIVATED")
        
        # Switch to emergency mode
        self.autonomous_mode = AutonomousMode.EMERGENCY
        
        success = await self.mavlink.emergency_stop()
        
        if success:
            await self._notify_safety_callbacks("emergency_stop", True)
        
        return success

    async def emergency_land(self) -> bool:
        """Emergency land at current position"""
        logger.critical("EMERGENCY LAND ACTIVATED")
        
        # Switch to emergency mode
        self.autonomous_mode = AutonomousMode.EMERGENCY
        
        success = await self.mavlink.emergency_land()
        
        if success:
            await self._notify_safety_callbacks("emergency_land", True)
        
        return success

    async def failsafe_return(self) -> bool:
        """Failsafe return to launch"""
        logger.warning("FAILSAFE RETURN ACTIVATED")
        
        success = await self.return_to_launch()
        
        if success:
            await self._notify_safety_callbacks("failsafe_return", True)
        
        return success

    # ============================================================================
    # VALIDATION AND CHECKS
    # ============================================================================

    def _pre_arm_checks(self) -> bool:
        """Perform pre-arm safety checks"""
        if not self.safety_checks_enabled:
            return True
        
        checks = {
            "Connected": self.is_connected,
            "System healthy": self.is_system_healthy(),
            "GPS fix": self.status.gps_info and self.status.gps_info.num_satellites >= 6,
            "Battery level": self.status.battery and self.status.battery.remaining_percent > 20.0
        }
        
        failed_checks = [name for name, passed in checks.items() if not passed]
        
        if failed_checks:
            logger.error(f"Pre-arm checks failed: {', '.join(failed_checks)}")
            return False
        
        return True

    def _pre_flight_checks(self) -> bool:
        """Perform pre-flight safety checks"""
        if not self._pre_arm_checks():
            return False
        
        additional_checks = {
            "Ready for flight": self.mavlink.is_ready_for_flight(),
            "Home position set": self.status.health and self.status.health.is_home_position_ok
        }
        
        failed_checks = [name for name, passed in additional_checks.items() if not passed]
        
        if failed_checks:
            logger.error(f"Pre-flight checks failed: {', '.join(failed_checks)}")
            return False
        
        return True

    def _autonomous_flight_checks(self) -> bool:
        """Perform autonomous flight safety checks"""
        if not self._pre_flight_checks():
            return False
        
        if not self.is_armed:
            logger.error("Drone must be armed for autonomous flight")
            return False
        
        if self.autonomous_mode == AutonomousMode.MANUAL:
            logger.warning("Autonomous flight requested in manual mode")
        
        return True

    def _validate_mission(self, mission: FlightPlan) -> bool:
        """Validate mission plan"""
        if not mission.waypoints:
            logger.error("Mission has no waypoints")
            return False
        
        # Check waypoint format
        for i, wp in enumerate(mission.waypoints):
            required_keys = ["lat", "lon", "alt"]
            if not all(key in wp for key in required_keys):
                logger.error(f"Waypoint {i} missing required keys: {required_keys}")
                return False
        
        # Check altitude limits
        max_alt = max(wp["alt"] for wp in mission.waypoints)
        if max_alt > 120.0:  # Regulatory limit for most countries
            logger.error(f"Mission altitude {max_alt}m exceeds regulatory limits")
            return False
        
        return True

    # ============================================================================
    # UTILITY METHODS
    # ============================================================================

    def _log_command(self, command: str, params: Optional[Dict] = None):
        """Log command execution"""
        self._last_command_time = time.time()
        
        command_log = {
            "timestamp": self._last_command_time,
            "command": command,
            "parameters": params or {}
        }
        
        self._command_history.append(command_log)
        
        # Keep only last 100 commands
        if len(self._command_history) > 100:
            self._command_history.pop(0)

    async def _wait_for_takeoff_completion(self, target_altitude: float):
        """Wait for takeoff to complete"""
        logger.info("Waiting for takeoff completion...")
        
        timeout_s = 60.0
        start_time = time.time()
        
        while time.time() - start_time < timeout_s:
            if self.status.position:
                current_alt = self.status.position.relative_altitude_m
                if current_alt >= target_altitude * 0.95:
                    logger.info(f"Takeoff complete at {current_alt:.1f}m")
                    return
            await asyncio.sleep(1.0)
        
        logger.warning("Takeoff completion timeout")

    async def _wait_for_landing_completion(self):
        """Wait for landing to complete"""
        logger.info("Waiting for landing completion...")
        
        timeout_s = 120.0
        start_time = time.time()
        
        while time.time() - start_time < timeout_s:
            if not self.is_armed and not self.is_in_air:
                logger.info("Landing complete")
                return
            await asyncio.sleep(1.0)
        
        logger.warning("Landing completion timeout")

    async def _wait_for_position_reached(self, target: Dict[str, float], tolerance_m: float = 2.0):
        """Wait for position to be reached"""
        timeout_s = 60.0
        start_time = time.time()
        
        while time.time() - start_time < timeout_s:
            if self.status.position:
                distance = self._calculate_distance_to_target(target)
                if distance <= tolerance_m:
                    return
            await asyncio.sleep(1.0)
        
        logger.warning(f"Position reach timeout for {target}")

    def _calculate_distance_to_target(self, target: Dict[str, float]) -> float:
        """Calculate distance to target position"""
        if not self.status.position:
            return float('inf')
        
        return self.mavlink._calculate_distance(
            self.status.position.latitude_deg,
            self.status.position.longitude_deg,
            target["lat"],
            target["lon"]
        )

    def _convert_to_mission_items(self, mission: FlightPlan):
        """Convert flight plan to MAVLink mission items"""
        from mavsdk.mission import MissionItem
        
        items = []
        
        for i, wp in enumerate(mission.waypoints):
            item = MissionItem(
                sequence=i,
                frame=1,  # Global relative frame
                command=16,  # NAV_WAYPOINT
                current=i == 0,
                autocontinue=True,
                param1=0,  # Hold time
                param2=0,  # Acceptance radius
                param3=0,  # Pass radius
                param4=float('nan'),  # Yaw
                x=int(wp["lat"] * 1e7),
                y=int(wp["lon"] * 1e7),
                z=wp["alt"],
                mission_type=1
            )
            items.append(item)
        
        return items

    # ============================================================================
    # TELEMETRY CALLBACKS
    # ============================================================================

    async def _monitor_battery(self, battery):
        """Monitor battery status"""
        if battery.remaining_percent < 20.0:
            logger.warning(f"Low battery: {battery.remaining_percent:.1f}%")
            
            if battery.remaining_percent < 10.0:
                logger.critical(f"Critical battery: {battery.remaining_percent:.1f}%")
                await self._notify_safety_callbacks("critical_battery", battery)

    async def _monitor_health(self, health):
        """Monitor system health"""
        if not health.is_gyrometer_calibration_ok:
            logger.error("Gyroscope calibration error")
        
        if not health.is_accelerometer_calibration_ok:
            logger.error("Accelerometer calibration error")
        
        if not health.is_magnetometer_calibration_ok:
            logger.error("Magnetometer calibration error")

    async def _monitor_position(self, position):
        """Monitor position updates"""
        # Check altitude limits
        if position.relative_altitude_m > 120.0:
            logger.warning(f"High altitude: {position.relative_altitude_m:.1f}m")

    async def _monitor_mission(self, progress):
        """Monitor mission progress"""
        logger.info(f"Mission progress: {progress.current}/{progress.total}")

    # ============================================================================
    # CALLBACK MANAGEMENT
    # ============================================================================

    def register_mission_callback(self, callback: Callable):
        """Register mission event callback"""
        self._mission_callbacks.append(callback)

    def register_safety_callback(self, callback: Callable):
        """Register safety event callback"""
        self._safety_callbacks.append(callback)

    async def _notify_mission_callbacks(self, event: str, data: Any):
        """Notify mission callbacks"""
        for callback in self._mission_callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(event, data)
                else:
                    callback(event, data)
            except Exception as e:
                logger.error(f"Mission callback error: {e}")

    async def _notify_safety_callbacks(self, event: str, data: Any):
        """Notify safety callbacks"""
        for callback in self._safety_callbacks:
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(event, data)
                else:
                    callback(event, data)
            except Exception as e:
                logger.error(f"Safety callback error: {e}")

    # ============================================================================
    # CONTEXT MANAGER SUPPORT
    # ============================================================================

    async def __aenter__(self):
        """Async context manager entry"""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        await self.disconnect()


# ============================================================================
# FACTORY FUNCTIONS
# ============================================================================

async def create_drone_controller(connection_url: Optional[str] = None) -> DroneController:
    """
    Factory function to create and connect drone controller
    
    Args:
        connection_url: MAVLink connection URL (optional)
    
    Returns:
        Connected DroneController instance
    """
    controller = DroneController(connection_url, auto_connect=False)
    
    if not await controller.connect():
        raise ConnectionError("Failed to connect to flight controller")
    
    return controller


# Example usage
if __name__ == "__main__":
    async def main():
        # Create drone controller
        async with DroneController() as drone:
            print(f"Connected to drone: {drone.status}")
            
            # Basic flight sequence
            if drone.is_system_healthy():
                await drone.takeoff(10.0)
                await asyncio.sleep(5)
                await drone.goto_location(47.397742, 8.545594, 10.0)
                await asyncio.sleep(5)
                await drone.land()
    
    asyncio.run(main())
