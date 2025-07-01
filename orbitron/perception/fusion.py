import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple, List

import numpy as np

# Assuming these interfaces are available in your project structure
from orbitron.communication.mavlink_interface import MAVLinkInterface, DroneStatus
from orbitron.hardware.fc.range_finder import LidarTFminiI2C
from orbitron.hardware.monocam.camera_stream import CameraStream, FrameData


# Data structures for sensor readings
@dataclass
class LidarData:
    timestamp: float
    distance_m: Optional[float] = None
    strength: Optional[int] = None
    temperature_c: Optional[float] = None
    is_valid: bool = False

@dataclass
class FusedSensorData:
    timestamp: float = field(default_factory=time.time)
    
    # From MAVLinkInterface (Flight Controller)
    fc_status: Optional[DroneStatus] = None
    
    # Rangefinders
    lidar_front: Optional[LidarData] = None
    lidar_right: Optional[LidarData] = None
    lidar_left: Optional[LidarData] = None
    lidar_down: Optional[LidarData] = None
    
    # Cameras
    front_camera_frame: Optional[Any] = None # e.g., FrameData or np.ndarray
    down_camera_frame: Optional[Any] = None  # e.g., FrameData or np.ndarray
    
    # Other processed data (examples)
    obstacle_map_2d: Optional[np.ndarray] = None # Example: 2D grid around drone
    estimated_pose_local: Optional[Dict[str, float]] = None # x, y, z, roll, pitch, yaw
    environment_classification: Optional[str] = None # e.g., "open_field", "urban_canyon"

class SensorFusionEngine:
    def __init__(self, mavlink_interface: MAVLinkInterface, config: Optional[Dict[str, Any]] = None):
        self.logger = logging.getLogger(__name__)
        self.mavlink_interface = mavlink_interface
        self.config = config if config else {}

        self._running = False
        self._fusion_task: Optional[asyncio.Task] = None
        self.latest_fused_data = FusedSensorData()

        # Initialize sensors (replace Mocks with actual implementations)
        # LiDARs
        self.lidar_front = LidarTFminiI2C(sensor_name="FrontLidar")
        self.lidar_right = LidarTFminiI2C(sensor_name="RightLidar")
        self.lidar_left = LidarTFminiI2C(sensor_name="LeftLidar")
        self.lidar_down = LidarTFminiI2C(sensor_name="DownLidar")
        self.lidars = {
            "front": self.lidar_front,
            "right": self.lidar_right,
            "left": self.lidar_left,
            "down": self.lidar_down,
        }

        # Cameras
        self.front_camera = CameraStream(camera_name="FrontCam")
        self.down_camera = CameraStream(camera_name="DownCam")
        self.cameras = {
            "front": self.front_camera,
            "down": self.down_camera,
        }
        
        # Data buffers for raw sensor inputs
        self._raw_data_buffers = {
            "fc_status": None,
            "lidar_front": None,
            "lidar_right": None,
            "lidar_left": None,
            "lidar_down": None,
            "front_camera_frame": None,
            "down_camera_frame": None,
        }
        self._data_locks: Dict[str, asyncio.Lock] = {key: asyncio.Lock() for key in self._raw_data_buffers}

    async def _initialize_sensors(self):
        """Initialize all connected sensors."""
        self.logger.info("Initializing external sensors...")
        # Initialize cameras
        for name, cam in self.cameras.items():
            if hasattr(cam, 'initialize'):
                success = await cam.initialize()
                if success and hasattr(cam, 'start_streaming'):
                    await cam.start_streaming()
                    self.logger.info(f"Camera {name} initialized and streaming.")
                elif success:
                     self.logger.info(f"Camera {name} initialized (not streaming by default).")
                else:
                    self.logger.error(f"Failed to initialize camera {name}.")
        # LiDARs are typically ready on power-up or __init__ for I2C/Serial
        self.logger.info("External sensors initialization attempt complete.")

    async def start(self):
        if self._running:
            self.logger.warning("Sensor fusion engine already running.")
            return
        
        await self._initialize_sensors()
        
        self._running = True
        self._fusion_task = asyncio.create_task(self._fusion_loop())
        self.logger.info("Sensor fusion engine started.")

    async def stop(self):
        if not self._running:
            self.logger.warning("Sensor fusion engine not running.")
            return
        
        self._running = False
        if self._fusion_task:
            self._fusion_task.cancel()
            try:
                await self._fusion_task
            except asyncio.CancelledError:
                self.logger.info("Fusion loop cancelled.")
        
        # Clean up sensors
        for cam in self.cameras.values():
            if hasattr(cam, 'close'):
                await cam.close()
        for lidar in self.lidars.values():
            if hasattr(lidar, 'close'):
                await lidar.close() # Assuming lidars have an async close
                
        self.logger.info("Sensor fusion engine stopped.")

    async def _update_fc_data(self):
        """Fetch latest data from MAVLinkInterface."""
        async with self._data_locks["fc_status"]:
            # The MAVLinkInterface internally updates its `status` attribute
            self._raw_data_buffers["fc_status"] = self.mavlink_interface.status
        # self.logger.debug(f"FC Data updated: Armed={self.mavlink_interface.status.armed}, Alt={self.mavlink_interface.status.position.relative_altitude_m if self.mavlink_interface.status.position else 'N/A'}")


    async def _update_lidar_reading(self, name: str, lidar_instance: Any):
        """Fetch latest data from a single LiDAR."""
        try:
            reading = await lidar_instance.read_frame()
            timestamp = time.time()
            if reading:
                dist, strength, temp = reading
                # Basic validation (can be more sophisticated)
                is_valid = (dist is not None and 
                            0.1 < dist < 12.0 and # TFmini-S typical range
                            strength > 20) # Example strength threshold
                
                async with self._data_locks[f"lidar_{name}"]:
                    self._raw_data_buffers[f"lidar_{name}"] = LidarData(
                        timestamp=timestamp,
                        distance_m=dist,
                        strength=strength,
                        temperature_c=temp,
                        is_valid=is_valid
                    )
            else:
                async with self._data_locks[f"lidar_{name}"]:
                     self._raw_data_buffers[f"lidar_{name}"] = LidarData(timestamp=timestamp, is_valid=False)
        except Exception as e:
            self.logger.error(f"Error reading LiDAR {name}: {e}")
            async with self._data_locks[f"lidar_{name}"]:
                self._raw_data_buffers[f"lidar_{name}"] = LidarData(timestamp=time.time(), is_valid=False)

    async def _update_camera_frame(self, name: str, camera_instance: Any):
        """Fetch latest frame from a single camera."""
        try:
            frame_data = await camera_instance.get_latest_frame() # Assuming async get_latest_frame
            async with self._data_locks[f"{name}_camera_frame"]:
                self._raw_data_buffers[f"{name}_camera_frame"] = frame_data
        except Exception as e:
            self.logger.error(f"Error reading Camera {name}: {e}")
            async with self._data_locks[f"{name}_camera_frame"]:
                 self._raw_data_buffers[f"{name}_camera_frame"] = None


    async def _gather_all_sensor_data(self):
        """Concurrently gather data from all sensors."""
        tasks = [self._update_fc_data()]
        
        for name, lidar_instance in self.lidars.items():
            tasks.append(self._update_lidar_reading(name, lidar_instance))
            
        for name, camera_instance in self.cameras.items():
            tasks.append(self._update_camera_frame(name, camera_instance))
            
        await asyncio.gather(*tasks)
        # self.logger.debug("All sensor data gathering cycle complete.")

    def _perform_fusion(self):
        """
        Core sensor fusion logic.
        This is where algorithms like EKF, UKF, occupancy grids, object detection, etc.,
        would be implemented. This template provides a placeholder.
        """
        current_time = time.time()
        fused_data = FusedSensorData(timestamp=current_time)

        # 1. Populate with raw/slightly processed data
        async def populate_data(): # Helper to run async lock access in sync method
            for key, lock in self._data_locks.items():
                async with lock:
                    if key == "fc_status": fused_data.fc_status = self._raw_data_buffers[key]
                    elif key == "lidar_front": fused_data.lidar_front = self._raw_data_buffers[key]
                    elif key == "lidar_right": fused_data.lidar_right = self._raw_data_buffers[key]
                    elif key == "lidar_left": fused_data.lidar_left = self._raw_data_buffers[key]
                    elif key == "lidar_down": fused_data.lidar_down = self._raw_data_buffers[key]
                    elif key == "front_camera_frame": fused_data.front_camera_frame = self._raw_data_buffers[key]
                    elif key == "down_camera_frame": fused_data.down_camera_frame = self._raw_data_buffers[key]
        
        # This is a bit of a hack to call async code from sync.
        # In a real scenario, _perform_fusion might also be async or data is passed differently.
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                asyncio.ensure_future(populate_data()) # If loop is running
            else:
                loop.run_until_complete(populate_data()) # If called outside running loop (less ideal)
        except RuntimeError: # If no current event loop
             asyncio.run(populate_data())


        # 2. Time Synchronization and Data Validation
        #    - Check timestamps of all data.
        #    - Discard stale data based on `current_time` and sensor-specific thresholds.
        #    - Validate data quality (e.g., LiDAR strength, GPS fix type).

        # 3. State Estimation (Pose and Velocity)
        #    - Example: Use an Extended Kalman Filter (EKF)
        #    - Inputs: IMU (from fc_status), GPS (fc_status), Altitude (fc_status),
        #              Optical Flow (from downward_camera_frame if available),
        #              LiDAR (for altitude updates or feature tracking).
        #    - Output: fused_data.estimated_pose_local (more precise x,y,z, roll,pitch,yaw)
        if fused_data.fc_status and fused_data.fc_status.position and fused_data.fc_status.attitude:
            fused_data.estimated_pose_local = {
                "x": fused_data.fc_status.position.latitude_deg, # This is GPS lat, needs conversion to local frame
                "y": fused_data.fc_status.position.longitude_deg, # This is GPS lon, needs conversion to local frame
                "z": fused_data.fc_status.position.relative_altitude_m,
                "roll": fused_data.fc_status.attitude.roll_deg,
                "pitch": fused_data.fc_status.attitude.pitch_deg,
                "yaw": fused_data.fc_status.attitude.yaw_deg,
            }
            # self.logger.debug(f"Placeholder pose: Z={fused_data.estimated_pose_local['z']:.2f}m, Yaw={fused_data.estimated_pose_local['yaw']:.1f}deg")


        # 4. Environment Mapping / Obstacle Detection
        #    - Example: Create a 2D or 3D occupancy grid.
        #    - Inputs: LiDAR (front, left, right, down), Front Camera (depth estimation or object detection).
        #    - Transform LiDAR points into a common frame (e.g., drone body or local NED).
        #    - Project camera-based obstacles into the map.
        #    - Output: fused_data.obstacle_map_2d or a 3D point cloud.
        # Example: simple obstacle indication from front LiDAR
        if fused_data.lidar_front and fused_data.lidar_front.is_valid and fused_data.lidar_front.distance_m < 2.0:
             self.logger.info(f"FUSION: Potential obstacle DETECTED by front LiDAR at {fused_data.lidar_front.distance_m:.2f}m")


        # 5. Scene Understanding / Environment Classification
        #    - Inputs: Camera feeds, LiDAR data.
        #    - Use CV models on camera frames for object recognition (people, cars, buildings).
        #    - Analyze LiDAR point cloud patterns.
        #    - Output: fused_data.environment_classification.

        self.latest_fused_data = fused_data
        # self.logger.info(f"Fusion complete. Timestamp: {fused_data.timestamp}")


    async def _fusion_loop(self):
        """Main loop for periodically gathering and fusing sensor data."""
        fusion_interval = self.config.get("FUSION_INTERVAL_S", 0.1) # e.g., 10 Hz
        while self._running:
            start_time = time.perf_counter()
            
            await self._gather_all_sensor_data()
            self._perform_fusion() # This is synchronous in the template
            
            # Optionally, emit an event or call a callback with self.latest_fused_data
            # event_system.publish("fused_data_updated", self.latest_fused_data)

            elapsed_time = time.perf_counter() - start_time
            await asyncio.sleep(max(0, fusion_interval - elapsed_time))

    def get_latest_fused_data(self) -> FusedSensorData:
        return self.latest_fused_data

async def example_usage():
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    # Initialize MAVLinkInterface (assuming it connects successfully)
    # In a real scenario, you'd pass a real, connected mavlink_interface instance
    class MockMAVLinkInterface:
        def __init__(self):
            self.status = DroneStatus(connected=True, armed=False)
            self.logger = logging.getLogger("MockMAVLinkInterface")
            self.logger.info("Mock MAVLink Interface Initialized.")
            asyncio.create_task(self._simulate_updates())

        async def _simulate_updates(self):
            while True:
                self.status.armed = not self.status.armed # Toggle armed state
                if self.status.position is None:
                    from mavsdk.telemetry import Position # type: ignore
                    self.status.position = Position(0.0,0.0,10.0,0.0)
                self.status.position.relative_altitude_m = np.random.uniform(9.5, 10.5)
                
                if self.status.attitude is None:
                    from mavsdk.telemetry import EulerAngle # type: ignore
                    self.status.attitude = EulerAngle(0.0,0.0,0.0,0.0)
                self.status.attitude.yaw_deg = np.random.uniform(0,360)
                
                self.logger.debug(f"Simulated FC update: Armed={self.status.armed}, Alt={self.status.position.relative_altitude_m:.2f}")
                await asyncio.sleep(1) # Simulate telemetry update rate

        async def connect(self): return True # Mock
        async def disconnect(self): pass # Mock

    mock_mavlink_iface = MockMAVLinkInterface()
    
    fusion_config = {"FUSION_INTERVAL_S": 0.2} # Run fusion at 5 Hz
    engine = SensorFusionEngine(mavlink_interface=mock_mavlink_iface, config=fusion_config)
    
    try:
        await engine.start()
        for i in range(20): # Run for a few cycles
            await asyncio.sleep(0.5)
            fused_data = engine.get_latest_fused_data()
            logger.info(f"Cycle {i}: Fused Data Timestamp: {fused_data.timestamp:.2f}")
            if fused_data.fc_status and fused_data.fc_status.position:
                logger.info(f"  FC Altitude: {fused_data.fc_status.position.relative_altitude_m:.2f}m")
            if fused_data.lidar_front:
                logger.info(f"  Front LiDAR: {fused_data.lidar_front.distance_m:.2f}m (Valid: {fused_data.lidar_front.is_valid})")
            if fused_data.front_camera_frame:
                 logger.info(f"  Front Cam Frame ID: {fused_data.front_camera_frame.frame_id}, Shape: {fused_data.front_camera_frame.shape}")

    except asyncio.CancelledError:
        logger.info("Example usage cancelled.")
    finally:
        await engine.stop()

if __name__ == "__main__":
    asyncio.run(example_usage())