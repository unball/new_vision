import configFile
import numpy as np
import mainWindow
import cv2
import singleton

class renderIdentity():
	def transformFrame(self, frame):
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

class cortarCampo(metaclass=singleton.Singleton):
	
	def __init__(self):
		# Variables
		self.points = []
		self.homography = None
		self.frame_shape = None
		self.pointer_position = None
		
		# Load configuration file
		config = configFile.getConfig()
		if(config.get("points")):
			self.points = config["points"]
		else: config["points"] = []
		configFile.saveConfig(config)
	
	def getHomography(self):
		if self.homography is None:
			if len(self.points) == 4:
				self.updateHomography()
				return self.homography
			else:
				return None
		else:
			return self.homography
	
	def sortPoints(self,points):
		if len(points) == 4:
			points.sort(key=sum)
			if points[1][0] > points[2][0]:
				tmp = points[1]
				points[1] = points[2]
				points[2] = tmp
		return points
	
	def updateHomography(self):
		self.homography = self.findHomography(self.frame_shape)
		config = configFile.getConfig()
		config["points"] = self.points
		configFile.saveConfig(config)
	
	def findHomography(self,shape):
		height, width, _ = shape
		frame_points = np.array([[0,0],[0, height],[width,0],[width,height]])
		h, mask = cv2.findHomography(np.array(self.sortPoints(self.points.copy())), frame_points, cv2.RANSAC)
		return h
		
	def update_points(self, point):
		if len(self.points) >= 4:
			self.points.clear()
			
		if len(self.points) < 4:
			self.points.append(point)
			
		if len(self.points) == 4:
			self.updateHomography()
	
	def set_pointer_position(self, position):
		self.pointer_position = position
	
	def transformFrame(self, frame):
		frame = cv2.resize(frame, (700,round(frame.shape[0]/frame.shape[1]*700)))
		self.frame_shape = frame.shape
		
		color = (0,255,0) if len(self.points) == 4 else (255,255,255)
		
		# Draw line for each pair of points
		if len(self.points) > 1:
			for i in range(len(self.points)-1):
				cv2.line(frame, (self.points[i][0], self.points[i][1]), (self.points[i+1][0], self.points[i+1][1]), color, thickness=2)
		
		# Closes rectangle
		if len(self.points) == 4:
			cv2.line(frame, (self.points[0][0], self.points[0][1]), (self.points[3][0], self.points[3][1]), color, thickness=2)
		
		# Draw helping line from last chosen point to current mouse position
		if self.pointer_position and len(self.points) > 0 and len(self.points) < 4:
			cv2.line(frame, (self.points[-1][0], self.points[-1][1]), (self.pointer_position[0], self.pointer_position[1]), (255,255,255))
			# Draw extra line from first chosen point to current position when it's the last point to be chosen
			if len(self.points) == 3:
				cv2.line(frame, (self.points[0][0], self.points[0][1]), (self.pointer_position[0], self.pointer_position[1]), color)
		
		for point in self.points:
			cv2.circle(frame, (point[0], point[1]), 5, color, thickness=-1)
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		

class segmentarPreto(metaclass=singleton.Singleton):

	def __init__(self):
		# Variables
		self.ui_elements = ["fundo_hmin", "fundo_smin", "fundo_vmin", "fundo_hmax", "fundo_smax", "fundo_vmax"]
		self.default_hsv_interval = [0,94,163,360,360,360]
		self.hsv_interval = None
		
		# Load configuration file
		config = configFile.getConfig()
		if(config.get("preto_hsv_interval")):
			self.hsv_interval = np.array(config["preto_hsv_interval"])
		else:
			self.hsv_interval = np.array(self.default_hsv_interval)
			config["preto_hsv_interval"] = self.default_hsv_interval
		configFile.saveConfig(config)
		
		# Set initial scrollbars position
		for index,id in enumerate(self.ui_elements):
			mainWindow.MainWindow().getObject(id).set_value(self.hsv_interval[index])
	
	def update_hsv_interval(self, value, index):
		self.hsv_interval[index] = value
		config = configFile.getConfig()
		config["preto_hsv_interval"][index] = value
		configFile.saveConfig(config)
	
	def transformFrame(self, frame):
		frame = cv2.resize(frame, (700,round(frame.shape[0]/frame.shape[1]*700)))
		homography_matrix = cortarCampo().getHomography()
		if(homography_matrix is not None): frame = cv2.warpPerspective(frame, homography_matrix, (frame.shape[1], frame.shape[0]))
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, self.hsv_interval[0:3], self.hsv_interval[3:6])
		
		return cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)


class segmentarTime(metaclass=singleton.Singleton):
	def __init__(self):
		# Variables
		self.ui_elements = ["time_hmin", "time_smin", "time_vmin", "time_hmax", "time_smax", "time_vmax"]
		self.default_hsv_interval = [13,0,0,32,360,360]
		self.hsv_interval = None
		
		# Load configuration file
		config = configFile.getConfig()
		if(config.get("time_hsv_interval")):
			self.hsv_interval = np.array(config["time_hsv_interval"])
		else:
			self.hsv_interval = np.array(self.default_hsv_interval)
			config["time_hsv_interval"] = self.default_hsv_interval
		configFile.saveConfig(config)
	
		# Set initial scrollbars position
		for index,id in enumerate(self.ui_elements):
			mainWindow.MainWindow().getObject(id).set_value(self.hsv_interval[index])
	
	def update_hsv_interval(self, value, index):
		self.hsv_interval[index] = value
		config = configFile.getConfig()
		config["time_hsv_interval"][index] = value
		configFile.saveConfig(config)
	
	def transformFrame(self, frame):
		frame = cv2.resize(frame, (700,round(frame.shape[0]/frame.shape[1]*700)))
		homography_matrix = cortarCampo().getHomography()
		if(homography_matrix is not None): frame = cv2.warpPerspective(frame, homography_matrix, (frame.shape[1], frame.shape[0]))
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, segmentarPreto().hsv_interval[0:3], segmentarPreto().hsv_interval[3:6])
		mask2 = mask & cv2.inRange(img_hsv, self.hsv_interval[0:3], self.hsv_interval[3:6])
		
		return cv2.cvtColor(mask2, cv2.COLOR_GRAY2RGB)
