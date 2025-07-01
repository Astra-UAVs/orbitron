#!/usr/bin/env python3
"""
Configuration settings for the Orbitron autonomous drone software stack
"""

import os
from dataclasses import dataclass
from typing import Dict, Any, Optional

# ============================================================================
# MAVLINK CONFIGURATION
# ============================================================================

# Default MAVLink connection settings
MAVLINK_CONNECTION_URL = os.getenv("MAVLINK_CONNECTION_URL", "serial:///dev/ttyAMA0:921600")
MAVLINK_HEARTBEAT_TIMEOUT_S = float(os.getenv("MAVLINK_HEARTBEAT_TIMEOUT_S", "5.0"))
FC_TARGET_SYSTEM_ID = int(os.getenv("FC_TARGET_SYSTEM_ID", "1"))
FC_TARGET_COMPONENT_ID = int(os.getenv("FC_TARGET_COMPONENT_ID", "1"))

# Flight parameters
DEFAULT_TAKEOFF_ALTITUDE_M = float(os.getenv("DEFAULT_TAKEOFF_ALTITUDE_M", "10.0"))
MAX_FLIGHT_ALTITUDE_M = float(os.getenv("MAX_FLIGHT_ALTITUDE_M", "120.0"))
RTL_ALTITUDE_M = float(os.getenv("RTL_ALTITUDE_M", "50.0"))
MAX_SPEED_MS = float(os.getenv("MAX_SPEED_MS", "15.0"))
CRUISE_SPEED_MS = float(os.getenv("CRUISE_SPEED_MS", "8.0"))

# Safety settings
GEOFENCE_RADIUS_M = float(os.getenv("GEOFENCE_RADIUS_M", "500.0"))
LOW_BATTERY_THRESHOLD = float(os.getenv("LOW_BATTERY_THRESHOLD", "20.0"))
CRITICAL_BATTERY_THRESHOLD = float(os.getenv("CRITICAL_BATTERY_THRESHOLD", "10.0"))

# ============================================================================
# HARDWARE CONFIGURATION
# ============================================================================

# Camera settings
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "1920"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "1080"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))

# Sensor settings
IMU_DEVICE = os.getenv("IMU_DEVICE", "/dev/i2c-1")
GPS_DEVICE = os.getenv("GPS_DEVICE", "/dev/ttyUSB0")
LIDAR_DEVICE = os.getenv("LIDAR_DEVICE", "/dev/ttyUSB1")

# ============================================================================
# AI/ML CONFIGURATION
# ============================================================================

# Object detection
OBJECT_DETECTION_MODEL = os.getenv("OBJECT_DETECTION_MODEL", "yolov8n.pt")
DETECTION_CONFIDENCE_THRESHOLD = float(os.getenv("DETECTION_CONFIDENCE_THRESHOLD", "0.5"))
DETECTION_IOU_THRESHOLD = float(os.getenv("DETECTION_IOU_THRESHOLD", "0.45"))

# Navigation
SLAM_ALGORITHM = os.getenv("SLAM_ALGORITHM", "ORB_SLAM3")
PATH_PLANNING_ALGORITHM = os.getenv("PATH_PLANNING_ALGORITHM", "RRT_STAR")

# ============================================================================
# COMMUNICATION CONFIGURATION
# ============================================================================

# Ground control station
GCS_HOST = os.getenv("GCS_HOST", "192.168.1.100")
GCS_PORT = int(os.getenv("GCS_PORT", "14550"))

# Mesh networking
MESH_NETWORK_ID = os.getenv("MESH_NETWORK_ID", "orbitron_mesh")
MESH_CHANNEL = int(os.getenv("MESH_CHANNEL", "11"))
MESH_POWER_DBM = int(os.getenv("MESH_POWER_DBM", "20"))

# Cloud connectivity
CLOUD_ENDPOINT = os.getenv("CLOUD_ENDPOINT", "")
CLOUD_API_KEY = os.getenv("CLOUD_API_KEY", "")

# ============================================================================
# LOGGING CONFIGURATION
# ============================================================================

LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
LOG_FILE = os.getenv("LOG_FILE", "/var/log/orbitron/orbitron.log")
TELEMETRY_LOG_FILE = os.getenv("TELEMETRY_LOG_FILE", "/var/log/orbitron/telemetry.log")

# ============================================================================
# ENVIRONMENT SPECIFIC SETTINGS
# ============================================================================

@dataclass
class EnvironmentConfig:
    """Environment-specific configuration"""
    name: str
    mavlink_url: str
    simulation: bool = False
    debug: bool = False
    extra_params: Dict[str, Any] = None

# Predefined environments
ENVIRONMENTS = {
    "simulation": EnvironmentConfig(
        name="Simulation",
        mavlink_url="udp://:14540",
        simulation=True,
        debug=True,
        extra_params={
            "physics_step_size": 0.004,
            "lockstep": True
        }
    ),
    "pixhawk_usb": EnvironmentConfig(
        name="Pixhawk USB",
        mavlink_url="serial:///dev/ttyACM0:57600",
        simulation=False,
        debug=False
    ),
    "pixhawk_telemetry": EnvironmentConfig(
        name="Pixhawk Telemetry",
        mavlink_url="serial:///dev/ttyAMA0:57600",
        simulation=False,
        debug=False
    ),
    "companion_computer": EnvironmentConfig(
        name="Companion Computer",
        mavlink_url="serial:///dev/ttyAMA0:921600",
        simulation=False,
        debug=False
    ),
    "udp_gcs": EnvironmentConfig(
        name="UDP GCS",
        mavlink_url="udp://192.168.1.100:14550",
        simulation=False,
        debug=True
    )
}

# Current environment
CURRENT_ENVIRONMENT = os.getenv("ORBITRON_ENV", "companion_computer")

def get_environment_config() -> EnvironmentConfig:
    """Get current environment configuration"""
    return ENVIRONMENTS.get(CURRENT_ENVIRONMENT, ENVIRONMENTS["companion_computer"])

def get_mavlink_url() -> str:
    """Get MAVLink URL for current environment"""
    if MAVLINK_CONNECTION_URL != "serial:///dev/ttyAMA0:921600":
        return MAVLINK_CONNECTION_URL
    return get_environment_config().mavlink_url

# ============================================================================
# MISSION CONFIGURATION
# ============================================================================

# Mission parameters
WAYPOINT_ACCEPTANCE_RADIUS_M = float(os.getenv("WAYPOINT_ACCEPTANCE_RADIUS_M", "1.0"))
MISSION_ITEM_TIMEOUT_S = float(os.getenv("MISSION_ITEM_TIMEOUT_S", "60.0"))
AUTO_CONTINUE_MISSION = os.getenv("AUTO_CONTINUE_MISSION", "true").lower() == "true"

# Search and rescue
SAR_SEARCH_PATTERN = os.getenv("SAR_SEARCH_PATTERN", "spiral")
SAR_SEARCH_ALTITUDE_M = float(os.getenv("SAR_SEARCH_ALTITUDE_M", "30.0"))
SAR_SEARCH_SPEED_MS = float(os.getenv("SAR_SEARCH_SPEED_MS", "5.0"))

# ============================================================================
# PERFORMANCE CONFIGURATION
# ============================================================================

# System resources
MAX_CPU_USAGE_PERCENT = float(os.getenv("MAX_CPU_USAGE_PERCENT", "80.0"))
MAX_MEMORY_USAGE_PERCENT = float(os.getenv("MAX_MEMORY_USAGE_PERCENT", "80.0"))
MAX_TEMPERATURE_C = float(os.getenv("MAX_TEMPERATURE_C", "70.0"))

# Processing rates
VISION_PROCESSING_FPS = int(os.getenv("VISION_PROCESSING_FPS", "30"))
TELEMETRY_RATE_HZ = int(os.getenv("TELEMETRY_RATE_HZ", "50"))
CONTROL_LOOP_RATE_HZ = int(os.getenv("CONTROL_LOOP_RATE_HZ", "100"))

# ============================================================================
# DEVELOPMENT CONFIGURATION
# ============================================================================

# Debug settings
DEBUG_MODE = os.getenv("DEBUG_MODE", "false").lower() == "true"
VERBOSE_LOGGING = os.getenv("VERBOSE_LOGGING", "false").lower() == "true"
ENABLE_PROFILING = os.getenv("ENABLE_PROFILING", "false").lower() == "true"

# Testing
ENABLE_HARDWARE_TESTS = os.getenv("ENABLE_HARDWARE_TESTS", "true").lower() == "true"
MOCK_HARDWARE = os.getenv("MOCK_HARDWARE", "false").lower() == "true"

# Validation functions
def validate_config():
    """Validate configuration settings"""
    errors = []
    
    # Check critical parameters
    if MAX_FLIGHT_ALTITUDE_M > 400:  # FAA limit for most countries
        errors.append("MAX_FLIGHT_ALTITUDE_M exceeds regulatory limits")
    
    if LOW_BATTERY_THRESHOLD <= CRITICAL_BATTERY_THRESHOLD:
        errors.append("LOW_BATTERY_THRESHOLD must be higher than CRITICAL_BATTERY_THRESHOLD")
    
    if GEOFENCE_RADIUS_M < 10:
        errors.append("GEOFENCE_RADIUS_M too small for safe operation")
    
    # Check hardware devices exist (if not mocking)
    if not MOCK_HARDWARE:
        import os
        devices = [CAMERA_DEVICE, GPS_DEVICE, LIDAR_DEVICE]
        for device in devices:
            if device and not os.path.exists(device):
                errors.append(f"Hardware device not found: {device}")
    
    if errors:
        raise ValueError("Configuration validation failed:\n" + "\n".join(f"- {e}" for e in errors))

# Load and validate configuration on import
if __name__ != "__main__":
    try:
        validate_config()
    except ValueError as e:
        print(f"Configuration warning: {e}")
