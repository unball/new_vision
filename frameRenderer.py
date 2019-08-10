from gi.repository import Gtk, GLib
import configFile
import numpy as np
import mainWindow
import cv2
import singleton
import time

class renderIdentity():
	def transformFrame(self, frame, originalFrame):
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

class cortarCampo(metaclass=singleton.Singleton):
	
	def __init__(self):
		# Variables
		self.points = []
		self.homography = None
		self.frame_shape = None
		self.pointer_position = None
		self.show_warpped = False
		
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
		if self.show_warpped: return
		
		if len(self.points) >= 4:
			self.points.clear()
			
		if len(self.points) < 4:
			self.points.append(point)
			
		if len(self.points) == 4:
			self.updateHomography()
	
	def set_pointer_position(self, position):
		self.pointer_position = position
	
	def set_show_mode(self, value):
		self.show_warpped = value
		
	def warp(self, frame):
		homography_matrix = self.getHomography()
		if(homography_matrix is not None): return cv2.warpPerspective(frame, homography_matrix, (frame.shape[1], frame.shape[0]))
		else: return frame
	
	def transformFrame(self, frame, originalFrame):
		self.frame_shape = frame.shape
		
		if(self.show_warpped and self.getHomography() is not None):
			frame = cv2.warpPerspective(frame, self.getHomography(), (frame.shape[1], frame.shape[0]))
			return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		
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
	
	def transformFrame(self, frame, originalFrame):
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
	
	def transformFrame(self, frame, originalFrame):
		homography_matrix = cortarCampo().getHomography()
		if(homography_matrix is not None): frame = cv2.warpPerspective(frame, homography_matrix, (frame.shape[1], frame.shape[0]))
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, segmentarPreto().hsv_interval[0:3], segmentarPreto().hsv_interval[3:6])
		mask2 = mask & cv2.inRange(img_hsv, self.hsv_interval[0:3], self.hsv_interval[3:6])
		
		return cv2.cvtColor(mask2, cv2.COLOR_GRAY2RGB)

class RoboAdversario():
	def __init__(self, centro, angulo):
		self.centro = centro
		self.angulo = angulo
		
class RoboAliado():
	def __init__(self, identificador):
		self.identificador = identificador
		self.centro = (-1,-1)
		self.angulo = -1
		self.estado = "Não-Identificado"
		self.ui = None

class identificarRobos(metaclass=singleton.Singleton):
	
	def __init__(self):
		self.angles = np.array([0, 90, 180, -90, -180])
		self.robosAliados = [
			RoboAliado((1,3)),
			RoboAliado((2,3)),
			RoboAliado((1,4)),
			RoboAliado((2,4)),
			RoboAliado((3,4))
		]
		self.robosAdversarios = []
	
	
	def definePoly(self, countor):
		perimetro = cv2.arcLength(countor, True)
		points = cv2.approxPolyDP(countor, 0.05*perimetro, True)
		return len(points)
		
	def atualizarRobos(self, robosIdentificados):
		for robo in self.robosAliados:
			identificado = False
			for roboIdentificado in robosIdentificados:
				if roboIdentificado[0] == robo.identificador:
					robo.centro = roboIdentificado[1]
					robo.angulo = roboIdentificado[2]["calc"]
					robo.estado = "Identificado"
					identificado = True
					break
			if not identificado: robo.estado = "Não-Identificado"
	
	def updateRobotsInfo(self, robos):
		timeFlow = mainWindow.MainWindow().getObject("time_flow")
		for robo in robos:
			if robo.ui:
				robo.ui["idLabel"].set_text("({0},{1})".format(robo.identificador[0], robo.identificador[1]))
				robo.ui["posicaoLabel"].set_text("Posição: (x: {:.1f}, y: {:.1f})".format(robo.centro[0], robo.centro[1]))
				robo.ui["anguloLabel"].set_text("Ângulo: {:.1f}º".format(robo.angulo))
				robo.ui["estadoLabel"].set_text("Estado: " + robo.estado)
			else:
				flowBoxChild = Gtk.FlowBoxChild()
				Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboRow")
				columnBox = Gtk.Box()
				idBox = Gtk.Box()
				idBox.set_orientation(Gtk.Orientation.VERTICAL)
				idBox.set_margin_left(10)
				idBox.set_margin_right(10)
				idBox.set_margin_top(10)
				idBox.set_margin_bottom(10)
				roboLabel = Gtk.Label("Robô")
				idLabel = Gtk.Label("({0},{1})".format(robo.identificador[0], robo.identificador[1]))
				Gtk.StyleContext.add_class(idLabel.get_style_context(), "roboId")
				idLabel.set_size_request(80,-1)
				idBox.add(roboLabel)
				idBox.add(idLabel)
				infoBox = Gtk.Box()
				estadoLabel = Gtk.Label("Estado: " + robo.estado)
				estadoLabel.set_halign(Gtk.Align.START)
				posicaoLabel = Gtk.Label("Posição: (x: {:.1f}, y: {:.1f})".format(robo.centro[0], robo.centro[1]))
				posicaoLabel.set_halign(Gtk.Align.START)
				anguloLabel = Gtk.Label("Ângulo: {:.1f}º".format(robo.angulo))
				anguloLabel.set_halign(Gtk.Align.START)
				infoBox.set_orientation(Gtk.Orientation.VERTICAL)
				infoBox.set_valign(Gtk.Align.CENTER)
				infoBox.add(estadoLabel)
				infoBox.add(posicaoLabel)
				infoBox.add(anguloLabel)
				columnBox.add(idBox)
				columnBox.add(infoBox)
				flowBoxChild.add(columnBox)
				timeFlow.add(flowBoxChild)
				robo.ui = {"idLabel": idLabel, "posicaoLabel": posicaoLabel, "anguloLabel": anguloLabel, "estadoLabel": estadoLabel}
			timeFlow.show_all()
	
	def isOwm(self, img, mask):
		_,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = sorted(contours, key=cv2.contourArea)
		contour = contours[-1]
		rectangle = cv2.minAreaRect(contour)
		img2 = img.copy()

		center = rectangle[0]
		angle = rectangle[-1]
		
		box = cv2.boxPoints(rectangle) 
		box = np.int0(box)
		cv2.drawContours(img2,[box],0,(255,0,255),1)

		img_filtered = cv2.GaussianBlur(img, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, segmentarTime().hsv_interval[0:3], segmentarTime().hsv_interval[3:6])

		_,countors,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
		countors = [countor for countor in countors if cv2.contourArea(countor)>10]
		countors = sorted(countors, key=cv2.contourArea)
		
		try:
			if len(countors) != 0:
				contornosInternos = len(countors)
				countor = countors[-1]
				angle1 = cv2.fitEllipse(countor)[-1]
				cv2.drawContours(img2,countor,0,(255,0,0),1)
				M = cv2.moments(countor)
				cX = M["m10"] / M["m00"]
				cY = M["m01"] / M["m00"]
				angle_c = 180.0/np.pi *np.arctan2(-(center[1]-cY), center[0]-cX)
				angles_p =  -angle + self.angles
				angles_p1 =  -angle1 + self.angles

				perimetro = cv2.arcLength(countor, True)
				points = cv2.approxPolyDP(countor, 0.05*perimetro, True)
				cv2.drawContours(img2, points, -1, (0,0,255), 4)

				poligono = self.definePoly(countors[-1])
				formaPrincipal = poligono if poligono < 4 else 4

				cv2.putText(img2, str((contornosInternos, formaPrincipal)), (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255,255,255))
				
				return (contornosInternos, formaPrincipal), center, {"calc": angle_c, 
					    "pred": angles_p[np.abs(angle_c -angles_p).argmin()], 
					    "pred2": angles_p1[np.abs(angle_c -angles_p1).argmin()]
					   }, img2
		except:
			return None, center, angle, None
		
		return None, center, angle, None
		
	
	def transformFrame(self, frame, originalFrame):
		a = time.time()
		# Corta o campo
		img_warpped = cortarCampo().warp(frame)
		
		# Segmenta o fundo
		img_filtered = cv2.GaussianBlur(img_warpped, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, segmentarPreto().hsv_interval[0:3], segmentarPreto().hsv_interval[3:6])
		
		# Encontra componentes conectados e aplica operações de abertura e dilatação
		num_components, components = cv2.connectedComponents(mask)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
		components = cv2.morphologyEx(np.uint8(components), cv2.MORPH_OPEN, kernel)
		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
		components = cv2.dilate(np.uint8(components), kernel, iterations=1)
		
		# Itera por cada elemento conectado
		processed_image = np.zeros(img_warpped.shape, np.uint8)
		aliados_identificados = []
		for label in np.unique(components)[1:]:
			component_mask = np.uint8(np.where(components == label, 255, 0))
			comp = cv2.bitwise_and(img_warpped, img_warpped, mask=component_mask)
			identificador, centro, angulo, component_image = self.isOwm(comp,component_mask)
			if component_image is not None: processed_image = cv2.add(processed_image, component_image)
			if identificador is not None: aliados_identificados.append((identificador, centro, angulo))
			
		self.atualizarRobos(aliados_identificados)
		
		#print(self.robosAliados)
		
		#print(time.time()-a)
		GLib.idle_add(self.updateRobotsInfo, self.robosAliados)
		
		return cv2.cvtColor(processed_image, cv2.COLOR_RGB2BGR)
