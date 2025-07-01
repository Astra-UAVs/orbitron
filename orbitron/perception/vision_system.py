#!/usr/bin/env python3

import time
import asyncio
import logging
import cv2 # OpenCV
import numpy as np
from picamera2 import Picamera2
from threading import Thread, Lock
from concurrent.futures import ThreadPoolExecutor

try:
	from tflite_runtime.interpreter import Interpreter
except ImportError:
	logging.error("TensorFlow Lite Runtime not found. Object detection will be disabled.")
	Interpreter = None

from config import (
	FRONT_CAMERA_INDEX,
	DOWN_CAMERA_INDEX,
	VISION_PROCESSING_WIDTH,
	VISION_PROCESSING_HEIGHT,
	VISION_FRAMERATE,
)

log = logging.getLogger(__name__)

class VisionSystem:

	def __init__(self, loop: asyncio.AbstractEventLoop):
		"""Initializes the VisionSystem"""
		log.info("Initializing VisionSystem...")
		self.loop = loop
		self.picam2 = Picamera2()

		self.front_cam_config = self._create_cam_config("front")
		self.down_cam_config = self._create_cam_config("down")
		self.picam2.configure([self.front_cam_config, self.down_cam_config])

		log.info(f"Picamera2 configured for {len(self.picam2.camera_controls)} cameras")
		log.info(f"Front Cam Sensor Resolution: {self.picam2.sensor_resolution[FRONT_CAMERA_INDEX]}")
		log.info(f"Down Cam Sensor Resolution: {self.picam2.sensor_resolution[DOWN_CAMERA_INDEX]}")

		self._running = False
		self._process_threads = []
		self._thread_executor = ThreadPoolExecutor(max_workers=4)
		self._data_lock = Lock()

		self.od_model_path = "models/ssd_mobilenet_v2_coco_quant_postprocess.tflite" # <--- CHANGE MODEL PATH
		self.od_labels_path = "models/coco_labels.txt" # <--- CHANGE LABELS PATH
		self.od_threshold = 0.5 # Confidence threshold
		self.od_interpreter = None
		self.od_labels = None
		self.od_input_details = None
		self.od_output_details = None
		self.latest_detected_objects = []

		self.prev_down_gray = None
		self.latest_flow_velocity = (0.0, 0.0)

		# --- Landing Marker Detection (Down Cam) ---
		# ArUco markers
		self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Example dictionary
		self.aruco_params = cv2.aruco.DetectorParameters()
		# Placeholder for camera matrix and distortion coeffs (NEEDS CALIBRATION!)
		self.camera_matrix = np.eye(3) # Identity matrix - REPLACE WITH CALIBRATED VALUES
		self.dist_coeffs = np.zeros(5) # Zero distortion - REPLACE WITH CALIBRATED VALUES
		self.marker_size_m = 0.15 # Size of the marker side in meters <--- CHANGE AS NEEDED
		self.latest_marker_pose = None # Relative pose: {'id': int, 'rvec': ndarray, 'tvec': ndarray}

		self._load_object_detection_model()

	def _create_camera_config(self, name: str):
		cam_index = FRONT_CAMERA_INDEX if name == "front" else DOWN_CAMERA_INDEX
		config = self.picam2.create_preview_configuration(
			main={"size": (VISION_PROCESSING_WIDTH, VISION_PROCESSING_HEIGHT), "format": "RGB888"},
			controls={"FrameRate": VISION_FRAMERATE},
			queue=True,
			camera_num=cam_index
		)
		log.info(f"Created config for '{name}' camera (Index: {cam_index}) @ {VISION_PROCESSING_WIDTH}x{VISION_PROCESSING_HEIGHT}, {VISION_FRAMERATE} FPS")
		return config

	def _load_object_detection_model(self):
		"""Loads the Object Detection model and labels"""
		if Interpreter is None:
			log.warning("TFLite runtime not available. Cannot load OD model.")
			return

		try:
			log.info(f"Loading object detection model: {self.od_model_path}")
			self.od_interpreter = Interpreter(model_path=self.od_model_path)

			self.od_interpreter.allocate_tensors()
			self.od_input_details = self.od_interpreter.get_input_details()
			self.od_output_details = self.od_interpreter.get_output_details()

			height = self.od_input_details[0]['shape'][1]
			width = self.od_input_details[0]['shape'][2]

			log.info(f"OD model loaded. Expected input size: {width}x{height}")

			log.info(f"Loading Labels: {self.od_labels_path}")
			self.od_labels = load_labels(self.od_labels_path)
			if not self.od_labels:
				log.error("Failed to load OD labels. Detection will lack class names.")

		except ValueError as e:
			log.error(f"Error loading TFLite model: {e}. Is the model file valid/compatible?")
			self.od_interpreter = None
		except Exception as e:
			log.error(f"UNEXPECTED error loading OD model or labels: {e}")
			self.od_interpreter = None

	def _camera_read_loop(self, cam_index: int, process_func: Callbable):
		"""Read frames from a specific camera stream and schedules processing"""
		cam_name = "Front" if cam_index == FRONT_CAMERA_INDEX else "Down"
		log.info(f"Starting read loop for {cam_name} camera (Index: {cam_index})...")
		while self._running:
			try:
				request = self.picam2.capture_request(stream_name=self.picam2.camera_configuration()[cam_index]["stream"], camera_num=cam_index)
				if request:
					image = request.make_array(self.picam2.camera_configuration()[cam_index]["main"]["stream"])
					request.release()

					if image is not None:
						self.loop.call_soon_threadsafe(asyncio.create_task, process_func(image))
					else:
						log.warning(f"{cam_name} camera request did not contain image data.")
				else:
					log.warning(f"{cam_name} camera capture_request returned None.")
					time.sleep(0.05)
			except Exception as e:
				log.error(f"Error in {cam_name} camera read loop: {e}", exc_info=True)
				time.sleep(0.5)
		log.info(f"Exiting read loop for {cam_name} camera.")


	async def _process_front_frame_async(self, frame_rgb):
		try:
			future = self.loop.run_in_executor(
				self._thread_executor,
				self._detect_objects,
				frame_rgb.copy(),
				self.od_interpreter,
				self.od_input_details,
				self.od_output_details,
				self.od_threshold
			)
			detected_objects = await future

			with self._data_lock:
				self.latest_detected_objects = detected_objects

			# TODO: Extract features for VIO/SLAM here
			features = await self.loop.run_in_executor(self._thread_executor, self._extract_features, frame_rgb)
			self.latest_front_features = features

		except Exception as e:
			log.error(f"Error processing front frame: {e}", exc_info=True)

	async def _process_down_frame_async(self, frame_rgb):
		try:
			frame_gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

			# Submit tasks to executor
			future_flow = self.loop.run_in_executor(
				self._thread_executor,
				self._calculate_opt_flow,
				frame_gray.copy(),
				self.prev_down_gray
			)
			future_marker = self.loop.run_in_executor(
				self._thread_executor,
				self._detect_landing_marker,
				frame_gray,
				self.aruco_dict,
				self.aruco_params,
				self.camera_matrix,
				self.dist_coeffs,
				self.marker_size_m
			)

			flow_result = await future_flow
			marker_pose = await future_marker

			with self._data_lock:
				if flow_result is not None:
					self.latest_flow_velocity = flow_result
					self.prev_down_gray = frame_gray
				self.latest_marker_pose = marker_pose

		except Exception as e:
			log.error(f"Error processing down frame: {e}", exc_info=True)
			self.prev_down_gray = None

	def _detect_objects(self, frame_rgb, interpreter, input_details, output_details, threshold):
		"""Performs object detection on a single frame"""
		if interpreter is None:
			return []

		try:
			input_height = input_details[0]['shape'][1]
			input_width = input_details[0]['shape'][2]

			# Resize frame to model input size
			resized_frame = cv2.resize(frame_rgb, (input_width, input_height))
			# Add batch dimension and normalize if required by the model
			# Example assumes uint8 input:
			input_data = np.expand_dims(resized_frame, axis=0)
			# If model expects float input: input_data = (np.float32(input_data) - 127.5) / 127.5

			# Perform inference
			interpreter.set_tensor(input_details[0]['index'], input_data)
			interpreter.invoke()

			# Retrieve detection results - output format depends heavily on the model
			# Example assumes SSD Mobilenet output format:
			# output_details[0]: Bounding boxes (normalized)
			# output_details[1]: Class IDs
			# output_details[2]: Scores
			# output_details[3]: Number of detections
			boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Normalized [ymin, xmin, ymax, xmax]
			classes = interpreter.get_tensor(output_details[1]['index'])[0]
			scores = interpreter.get_tensor(output_details[2]['index'])[0]
			# num_detections = int(interpreter.get_tensor(output_details[3]['index'])[0]) # Some models have this

			results = []
			frame_height, frame_width, _ = frame_rgb.shape
			for i in range(len(scores)):
				if scores[i] > threshold:
					# Denormalize bounding box
					ymin, xmin, ymax, xmax = boxes[i]
					bbox = [
						int(xmin * frame_width),
						int(ymin * frame_height),
						int((xmax - xmin) * frame_width),
						int((ymax - ymin) * frame_height)
					]
					results.append({
						'bbox': bbox, # [x, y, w, h] in original frame coordinates
						'class_id': int(classes[i]),
						'score': float(scores[i])
					})
			return results

		except Exception as e:
			log.error(f"Error during object detection inference: {e}", exc_info=True)
			return []

	def _calculate_opt_flow(self, current_gray, prev_gray):
		"""Calculates sparse optical flow using Lucas-Kanade."""
		if prev_gray is None or current_gray is None:
			return None

		try:
			feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
			# Parameters for Lucas-Kanade optical flow
			lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

			# Find good features to track in the previous frame
			p0 = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
			if p0 is None:
				return (0.0, 0.0)

			# Calculate optical flow
			p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, current_gray, p0, None, **lk_params)

			# Select good points
			good_new = p1[st == 1]
			good_old = p0[st == 1]

			if len(good_new) < 5:
				return (0.0, 0.0)

			# Calculate the average displacement vector
			# Flow vector is (new_pos - old_pos)
			displacement = good_new - good_old
			avg_displacement = np.mean(displacement, axis=0)

			# This displacement is in pixels per frame. Needs scaling by altitude and time
			# to become velocity (m/s). Scaling happens later using Lidar altitude.
			# Assuming down camera has minimal roll/pitch for this simple calculation.
			vx = avg_displacement[0]
			vy = avg_displacement[1]

			return (vx, vy)
			
		except Exception as e:
			log.error(f"Error calculating optical flow: {e}", exc_info=True)
			return None

	def _detect_landing_marker(self, frame_gray, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_size_m):
		"""Detects ArUco markers and estimates their pose"""
		try:
			corners, ids, rejected = cv2.aruco.detectMarkers(frame_gray, aruco_dict, parameters=aruco_params)

			if ids is not None and len(ids) > 0:
				# Marker detected, estimate pose for the first detected marker
				# Requires camera calibration (camera_matrix, dist_coeffs)
				rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_m, camera_matrix, dist_coeffs)

				# Store pose of the first detected marker
				marker_id = int(ids[0][0])
				rvec = rvecs[0][0]
				tvec = tvecs[0][0]

				return {'id': marker_id, 'rvec': rvec, 'tvec': tvec}
			else:
				return None
		except Exception as e:
			log.error(f"Error detection ArUco marker: {e}", exc_info=True)
			return None

	def get_detected_objects(self) -> list:
		"""Returns the latest list of detected objects."""
		with self._data_lock:
			return self.latest_detected_objects[:] # Return a copy

	def get_optical_flow_velocity(self) -> tuple:
		"""Returns the latest estimated optical flow vector (vx, vy) in pixels/frame."""
		with self._data_lock:
			return self.latest_flow_velocity

	def get_landing_marker_pose(self) -> Optional[dict]:
		"""Returns the pose {'id', 'rvec', 'tvec'} of the first detected marker, or None."""
		with self._data_lock:
			return self.latest_marker_pose.copy() if self.latest_marker_pose else None