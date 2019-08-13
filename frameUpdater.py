from threading import Thread, Event
from gi.repository import GLib
import frameRenderer
import cv2
import time
import cameras
import configFile

class frameUpdater():
	
	def __init__(self, main_frame):
		# Private
		self.__running = False
		self.__camera_changed = False
		self.__frame_renderer = frameRenderer.cortarCampo()
		
		self.main_frame = main_frame
		self.camera_index = 0
		self.cap = None
		
		# Load configuration file
		config = configFile.getConfig()
		if(config.get("camera")):
			self.camera_index = config["camera"]
		else: config["camera"] = 0
		configFile.saveConfig(config)
		
		
	
	def stop(self):
		self.__running = False
	
	def run(self):
		self.__running = True
		self.thread = Thread(target=self.__loop__)
		self.thread.start()
		
	def setCamera(self, index):
		if self.camera_index == index: return
		self.camera_index = index
		self.__camera_changed = True
		config = configFile.getConfig()
		config["camera"] = self.camera_index
		configFile.saveConfig(config)
		
	def set_frame_renderer(self, frame_renderer):
		self.__frame_renderer = frame_renderer
		
	def __init_cap__(self):
		if self.cap: self.cap.release()
		while self.__running:
			self.cap = cv2.VideoCapture(self.camera_index)
			if self.cap.isOpened(): break
			time.sleep(0.03)
	
	def __loop__(self):
		#frame = cv2.imread("frame.png")
		self.__init_cap__()
		
		while(self.__running):
			
			while self.cap.isOpened() and self.__running:
				tw = time.time()
			
				try:
					ret, frame = self.cap.read()
					#frame_resized = cv2.resize(frame, (round(frame.shape[1]/frame.shape[0]*600),600))
					t0 = time.time()
					frame_processed = self.__frame_renderer.transformFrame(frame, frame)
					print("processamento: {0}".format(time.time()-t0))
					
					height, width, depth = frame_processed.shape
					GLib.idle_add(self.main_frame.do_update_frame, (frame_processed, width, height, depth))
					
					#time.sleep(0.03)
					
					if self.__camera_changed:
						self.__camera_changed = False
						GLib.idle_add(self.main_frame.clear_image)
						self.__init_cap__()
						break
					
				except:
					pass
				print("loop: {0}".format(time.time()-tw))
			
