from threading import Thread, Event
from gi.repository import GLib
import frameRenderer
import cv2
import time

class frameUpdater:
	def __init__(self, main_frame):
		self.main_frame = main_frame
		self.__running = False
		self.__frame_renderer = frameRenderer.cortarCampo()
	
	def stop(self):
		self.__running = False
	
	def run(self):
		self.__running = True
		self.thread = Thread(target=self.__loop__)
		self.thread.start()
		
	def set_frame_renderer(self, frame_renderer):
		self.__frame_renderer = frame_renderer
	
	def __loop__(self):
		#cap = cv2.VideoCapture(0)
		#cap.release()
		#cap = cv2.VideoCapture(0)
		frame = cv2.imread("frame.png")
	
		#while(cap.isOpened() and self.__running):
		while(self.__running):
			#ret, frame = cap.read()
			frame_processed = self.__frame_renderer.transformFrame(frame)
			
			image_to_render = cv2.resize(frame_processed, (700,round(frame_processed.shape[0]/frame_processed.shape[1]*700)))
			
			height, width, depth = image_to_render.shape
			GLib.idle_add(self.main_frame.do_update_frame, (image_to_render, width, height, depth))
			
			time.sleep(0.03)
