from threading import Thread, Event
from gi.repository import GLib
import frameRenderer
import cv2
import time
import cameras
import configFile
import mainWindow

class frameUpdater():
	
	def __init__(self, main_frame):
		# Private
		self.__running = False
		self.__camera_changed = False
		self.__frame_renderer = frameRenderer.cortarCampo()
		
		self.main_frame = main_frame
		self.cap = None
		
		# Load configuration file
		self.camera_index = configFile.getValue("camera", 0)
		
	
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
		configFile.setValue("camera", self.camera_index)
	
	def update_stats(self, processing_time, loop_time):
		mainWindow.MainWindow().getObject("stats_label").set_text("Tempo de processamento: {:3.0f} ms\nTempo de loop: {:3.0f} ms".format(processing_time*1000, loop_time*1000))
		
	def set_frame_renderer(self, frame_renderer):
		self.__frame_renderer = frame_renderer
		
	def __init_cap__(self):
		if self.cap: self.cap.release()
		while self.__running:
			self.cap = cv2.VideoCapture(self.camera_index)
			if self.cap.isOpened(): break
			time.sleep(0.03)
	
	def __loop__(self):
		frame_ = cv2.imread("frame.png")
		#self.__init_cap__()
		
		while(self.__running):
			
			#while self.cap.isOpened() and self.__running:
				loop_time = time.time()
			
				#ret, frame = self.cap.read()
				frame = frame_.copy()
				#frame_resized = cv2.resize(frame, (round(frame.shape[1]/frame.shape[0]*600),600))
				processing_time = time.time()
				frame_processed = self.__frame_renderer.transformFrame(frame, frame)
				processing_time = time.time()-processing_time
				
				height, width, depth = frame_processed.shape
				GLib.idle_add(self.main_frame.do_update_frame, (frame_processed, width, height, depth))
				
				#time.sleep(0.03)
				
				if self.__camera_changed:
					self.__camera_changed = False
					GLib.idle_add(self.main_frame.clear_image)
					self.__init_cap__()
					break
					
				loop_time = time.time()-loop_time
				GLib.idle_add(self.update_stats, processing_time, loop_time)
			
