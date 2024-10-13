import cv2
import queue
import logging
import threading

class VideoCapture:

	def __init__(self, stream, api_preference):
		self.cap = cv2.VideoCapture(stream, api_preference)
		self.q = queue.Queue()
		t = threading.Thread(target=self._reader)
		t.daemon = True
		t.start()

	def _reader(self):
		""" Read frame as soon as they are available, keeping only the most recent one """
		try:
			while True:
				ret, frame = self.cap.read()
				if not ret:
					break
				if not self.q.empty():
					try:
						self.q.get_nowait()
					except queue.Empty:
						pass
				self.q.put((ret, frame))
		except:
			self.cap.release()
			logging.info("Video processing stopped")

	def read(self):
		return self.q.get()

	def release(self):
		return self.cap.release()