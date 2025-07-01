import numpy as np

class Configuration:
    """
    Central configuration settings for the Fused Perception Engine (FPE).
    Adjust these values based on your specific hardware and requirements.
    """
    # --- Camera Settings ---
    CAMERA_INDEX = 0         # Default camera index (e.g., 0, 1, or GStreamer pipeline string)
    # Lower resolution for better performance on SBCs
    # CAMERA_WIDTH = 1280
    # CAMERA_HEIGHT = 720
    CAMERA_WIDTH = 640       # Example SBC-friendly resolution
    CAMERA_HEIGHT = 480      # Example SBC-friendly resolution
    CAMERA_FPS = 30          # Target FPS (camera might not achieve it)
    RESIZED_WIDTH = 640      # Resize frames for processing (can be same as CAMERA_WIDTH)
    RESIZED_HEIGHT = 480     # Resize frames for processing

    # --- IMU Settings ---
    # Specific settings depend on the IMU and connection method
    # Example: MPU6050 I2C address
    # IMU_ADDRESS = 0x68
    IMU_TYPE = "MAVSDK" # Change to 'MPU6050', 'ICM20948', etc.
    IMU_SAMPLE_RATE_HZ = 200 # Target sample rate (actual rate depends on sensor/driver)

    # --- Stabilization Settings ---
    STABILIZATION_METHOD = "IMU_HOMOGRAPHY" # 'IMU_HOMOGRAPHY' or 'NONE'
    # Gyro sensitivity factor (adjust based on IMU calibration/units)
    # Might need tuning depending on whether units are deg/s or rad/s
    GYRO_SENSITIVITY = 1.0 # Example: Assume rad/s input

    # --- Camera Intrinsics (CRITICAL - Calibrate your camera!) ---
    # These are EXAMPLE values, replace with your actual calibration results
    # fx, fy: focal lengths in pixels
    # cx, cy: principal point (image center)
    FX = 500.0
    FY = 500.0
    CX = RESIZED_WIDTH / 2
    CY = RESIZED_HEIGHT / 2
    # Distortion coefficients (k1, k2, p1, p2, [k3]) - often needed for accuracy
    # Set to zeros if not using or unknown, but calibration is recommended
    DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    # --- Derived Intrinsics ---
    CAMERA_MATRIX = np.array([
        [FX, 0, CX],
        [0, FY, CY],
        [0, 0, 1]
    ], dtype=np.float32)

    # --- Feature Tracking (Part 2) ---
    MAX_FEATURES = 200
    FEATURE_QUALITY = 0.01
    MIN_FEATURE_DISTANCE = 10

    # --- Object Detection (Part 3) ---
    # DETECTOR_MODEL_PATH = "models/yolov5s.onnx" # Example
    DETECTION_CONF_THRESHOLD = 0.4
    DETECTION_NMS_THRESHOLD = 0.5
    DETECTION_INTERVAL = 5 # Run detection every N frames

    # --- Optical Flow (Part 4) ---
    # Flow settings (if using RAFT/PWCNet, model paths might go here)

    # --- Depth Estimation (Part 5) ---
    # Depth model settings

    # --- VIO / 3D Tracking (Part 6+) ---
    # Settings for VIO algorithm, EKF, etc.

    # --- Display Settings ---
    DISPLAY_SCALE = 1.0 # Scale factor for visualization window
    SHOW_STABILIZED = True
    SHOW_FEATURES = False # Enable later
    SHOW_DETECTIONS = False # Enable later


# Create a single instance for easy access
config = Configuration()

# --- Helper function for Camera Intrinsics based on Config ---
def get_camera_matrix():
    """Returns the camera intrinsic matrix based on config."""
    return np.array([
        [config.FX, 0, config.CX],
        [0, config.FY, config.CY],
        [0, 0, 1]
    ], dtype=np.float32)

def get_dist_coeffs():
    """Returns the distortion coefficients based on config."""
    return config.DIST_COEFFS