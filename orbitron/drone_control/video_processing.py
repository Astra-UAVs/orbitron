
import threading
import cv2
import time
import json
import PIL.Image
import numpy as np
import collections
import datetime

import torch
import torchvision.transforms as transforms
from torch2trt import TRTModule

import trt_pose.coco
from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects

import tensorflow as tf
from tensorflow.python.saved_model import tag_constants


class VideoProcessing:
	def __init__(self, gstream_def: str, estimation_model_path:str, classification_mode;_path: str, topology_path: str, labels_path: str):

		self._init_pose_estimation(model_path=estimation_model_path, topology_path=topology_path)
		self._init_pose_classification(model_path=classification_model_path, labels_path=labels_path)
		self._init_video_capture(gstream_def)

		self.buffer_size = 7
		self.poses_buffer = collections.deque(
			self.buffer_size * [str(None)], self.buffer_size
		)
		self.processing_times = collections.deque(
			self.buffer_size * [0.0], self.buffer_size
		)

		video_fps = 10
		frameSize = (224, 224)
		data_str = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
		self.video_recording = cv2.VideoWriter(
			"live_processing_{}.avi".format(*"DIVX"),
			video_fps,
			frameSize
		)

		t = threading.Thread(target=self._run)
		t.daemon = True
		t.start()

	def _init_video_capture(self, gstream_def: str):
		logging.info("Initialize video capture pipeline...", end="\t")

		self.video_meam = torch.Tensor([0.485, 0.456, 0.406]).cuda()
		self.video_std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
		self.device = torch.device("cuda")
		self.cap = VideoCapture(gstream_def, cv2.CAP_GSTREAMER)

		logging.info("Done initializing video capture pipeline.")

	def _init_pose_estimation(self, model_path, topology_path):
		logging.info("Initialize pose estimation model...", end="\t")

		with open(topology_path, "r") as f:
			human_pose = json.load(f)

		topology = trt_pose.coco.coco_category_to_topology(human_pose)
		self.parse_objects = ParseObjects(topology)
		self.draw_objects = DrawObjects(topology)

		self.model_estimation = TRTModule()
		self.model_estimation.load_state_dict(torch.load(model_path))

		logging.info("Done initializing pose estimation model.")

	def _init_pose_classification(self, model_path, labels_path):
		logging.info("Initialize pose classification model...", end="\t")

		model_classification = tf.saved_model.load(model_path, tags=[tag_constants.SERVING])
		self.inner_classification = model_classification.signatures['serving_default']

		self.infer_classification(tf.constant(np.random.normal(size=(1, 18, 2)).astype(np.float32), dtype=tf.float32))

		with open(labels_path) as f:
			self.classification_labels = json.load(f)["labels"]

		logging.info("Done initializing pose classification model.")

	def _run(self):
		while True:
			start_time = time.time()
			re, image = self.cap.read()

			if re:
				cmap, paf = self.get_cmap_paf(image)
				counts, objects, peaks = self.parse_objects(
					cmap, paf
				)

				keypoints = self.get_keypoints(
					counts, objects, peaks
				)
				label_pose = None

				keypoints = self.preprocess_keypoints(keypoints)
				if type(keypoints) != type(None):
					x = tf.constant(np.expand_dims(keypoints, axis=0), dtype=tf.float32)
					prediction = self.infer_classification(x)
					label_pose = self.classification_labels[np.argmax(prediction["dense_20"][0])]
				self.poses_buffer.appendLeft(label_pose)

				self.draw_objects(image, counts, objects, peaks)
				if label_pose:
					label_pose = label_pose.replace("_", " ")
					image = cv2.putText(
						image,
						label_pose,
						(10, 25),
						cv2.FRONT_HERSHEY_DUPLEX,
						0.7,
						(0,),
						2,
						cv2.LINE_AA,
					)
					image = cv2.putText(
						image,
						label_pose,
						(10, 25),
						cv2.FRONT_HERSHEY_DUPLEX,
						0.7,
						(255, 255, 255),
						1,
						cv2.LINE_AA
					)
				fps = self.get_fps()
				if fps:
					fps = "FPS : {:.2f}".format(fps)
					image = cv2.putText(
						image,
						fps,
						(10, image.shape[0] - 10),
						cv2.FRONT_HERSHEY_DUPLEX,
						0.7,
						(0, 0, 0),
						2,
						cv2.LINE_AA
					)
					image = cv2.putText(
						image,
						fps,
						(10, image.shape[0] - 10),
						cv2.FRONT_HERSHEY_DUPLEX,
						0.7,
						(255, 255, 255),
						1,
						cv2.LINE_AA
					)
				self.video_recording.write(image)
			else:
				raise RuntimeError("Could not read image from camera")

			self.processing_times.appendLeft(1.0 / (time.time() - start_time))

	def preprocess(self, image):
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		image = PIL.Image.fromarray(image)
		image = transforms.functional.to_tensor(image).to(self.device)
		image.sub_(self.video_meam[:, None, None]).div_(self.video_std[:, None, None])
		return image[None, ...]

	def get_cmap_paf(self, image):
		data = self.preprocess(image)
		cmap, paf = self.model_estimation(data)
		cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
		return cmap, paf

	def get_keypoints(self, counts, objects, peaks, indexBody=0):
		kpoint = []
		human = objects[0][indexBody]
		C = human.shape[0]
		for j in range(C):
			k = int(human[j])
			if k >= 0:
				peak = peaks[0][j][k]
				kpoint.append([float(peak[1]), float(peak[0])])
			else:
				kpoint.append([None, None])
		return np.array(kpoint)

	def get_length_limb(self, data, keypoint1: int, keypoint2: int):
		
