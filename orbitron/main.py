
from src.video_processing import VideoProcessing
from src.drone_control import Drone
from dronekit import VehicleMode

import time
import numpy as np
import pathlib

CAM_ASPECT_RATIO = 16.0 / 9.0
INPUT_IMG_SIZE = 224

gstream_pipeline = (
	"nvarguscamerasrc ! "
	"video/x-raw(memory:NVMM), "
	"width=(int){capture_width:d}, height=(int){capture_height:d}, "
	"format=(string)NV12, framerate=(fraction){framerate:d}/1 ! "
	"nvvidconv top={crop_top:d} bottom={crop_bottom:d} left={crop_left:d} right={crop_right:d} flip-method={flip_method:d} ! "
	"video/x-raw, width=(int){display_width:d}, height=(int){display_height:d}, format=(string)BGRx ! "
	"videoconvert ! "
	"video/x-raw, format=(string)BGR ! appsink".format(
		capture_width=int(INPUT_IMG_SIZE * CAM_ASPECT_RATIO),
		capture_height=INPUT_IMG_SIZE,
		framerate=60,
		crop_top=0,
		crop_bottom=INPUT_IMG_SIZE,
		crop_left=int(INPUT_IMG_SIZE * (CAM_ASPECT_RATIO - 1) / 2),
		crop_right=int(INPUT_IMG_SIZE * (CAM_ASPECT_RATIO + 1) / 2),
		flip_method=0,
		display_width=INPUT_IMG_SIZE,
		display_height=INPUT_IMG_SIZE,
	)
)

print(gstream_pipeline)

MODELS_PATH = pathlib.Path(".") / "drone-models" / "models"


