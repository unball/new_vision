from gi.repository import Gtk, GLib
import configFile
import numpy as np
import mainWindow
import cv2
import singleton
import time
import pixel2metric
import visao

class renderIdentity():
	def transformFrame(self, frame, originalFrame):
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

class cortarCampo(metaclass=singleton.Singleton):
	
	def __init__(self):
		# Variables
		self.pointer_position = None
		self.show_warpped = False
		self.frame_shape = None
		
		self.points = configFile.getValue("points", [])
		
	def sortPoints(self,points):
		if len(points) == 4:
			points.sort(key=sum)
			if points[1][0] > points[2][0]:
				tmp = points[1]
				points[1] = points[2]
				points[2] = tmp
		return points
	
	def update_points(self, point):
		if self.show_warpped: return
		
		if len(self.points) >= 4:
			self.points.clear()
			
		if len(self.points) < 4:
			self.points.append(point)
			
		if len(self.points) == 4 and self.frame_shape is not None:
			configFile.setValue("points", self.points)
			
			visao.Visao().updateHomography(self.sortPoints(self.points.copy()), self.frame_shape)
	
	def set_pointer_position(self, position):
		self.pointer_position = position
	
	def set_show_mode(self, value):
		self.show_warpped = value
		
	def transformFrame(self, frame, originalFrame):
		self.frame_shape = frame.shape
		
		if(self.show_warpped):
			return cv2.cvtColor(visao.Visao().warp(frame), cv2.COLOR_RGB2BGR)
		
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
		
		# Set initial scrollbars position
		for index,id in enumerate(self.ui_elements):
			mainWindow.MainWindow().getObject(id).set_value(visao.Visao().preto_hsv[index])
	
	def update_hsv_interval(self, value, index):
		visao.Visao().atualizarPretoHSV(value, index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = visao.Visao().warp(frame)
		return cv2.cvtColor(visao.Visao().segmentarFundo(img_warped), cv2.COLOR_GRAY2RGB)

class segmentarTime(metaclass=singleton.Singleton):
	def __init__(self):
		# Variables
		self.ui_elements = ["time_hmin", "time_smin", "time_vmin", "time_hmax", "time_smax", "time_vmax"]
	
		# Set initial scrollbars position
		for index,id in enumerate(self.ui_elements):
			mainWindow.MainWindow().getObject(id).set_value(visao.Visao().time_hsv[index])
	
	def update_hsv_interval(self, value, index):
		visao.Visao().atualizarTimeHSV(value, index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = visao.Visao().warp(frame)
		return cv2.cvtColor(visao.Visao().segmentarTime(img_warped), cv2.COLOR_GRAY2RGB)

class segmentarBola(metaclass=singleton.Singleton):
	def __init__(self):
		# Variables
		self.ui_elements = ["bola_hmin", "bola_smin", "bola_vmin", "bola_hmax", "bola_smax", "bola_vmax"]
	
		# Set initial scrollbars position
		for index,id in enumerate(self.ui_elements):
			mainWindow.MainWindow().getObject(id).set_value(visao.Visao().bola_hsv[index])
	
	def update_hsv_interval(self, value, index):
		visao.Visao().atualizarBolaHSV(value, index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = visao.Visao().warp(frame)
		return cv2.cvtColor(visao.Visao().segmentarBola(img_warped), cv2.COLOR_GRAY2RGB)

class identificarRobos(metaclass=singleton.Singleton):
	def __init__(self):
		pass
	
	def updateRobotsInfo(self, robos, bola):
		timeFlow = mainWindow.MainWindow().getObject("time_flow")
		bolaEstado = mainWindow.MainWindow().getObject("bola_estado")
		bolaPosicao = mainWindow.MainWindow().getObject("bola_posicao")
		for robo in robos:
			if robo.ui:
				robo.ui["idLabel"].set_text("{0}".format(robo.identificador))
				robo.ui["posicaoLabel"].set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(robo.centro[0], robo.centro[1]))
				robo.ui["anguloLabel"].set_text("Ângulo {:.1f}º".format(robo.angulo))
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
				idLabel = Gtk.Label("{0}".format(robo.identificador))
				Gtk.StyleContext.add_class(idLabel.get_style_context(), "roboId")
				idLabel.set_size_request(80,-1)
				idBox.add(roboLabel)
				idBox.add(idLabel)
				infoBox = Gtk.Box()
				estadoLabel = Gtk.Label("Estado: " + robo.estado)
				estadoLabel.set_halign(Gtk.Align.START)
				posicaoLabel = Gtk.Label("Posição: (x: {:.2f}, y: {:.2f})".format(robo.centro[0], robo.centro[1]))
				posicaoLabel.set_halign(Gtk.Align.START)
				anguloLabel = Gtk.Label("Ângulo {:.1f}º".format(robo.angulo))
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
			
		if bola is not None:
			bolaEstado.set_text("Estado: Identificada")
			bolaPosicao.set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(bola[0][0], bola[0][1]))
		else:
			bolaEstado.set_text("Estado: Não-Identificada")
			
	
	def transformFrame(self, frame, originalFrame):
		processed_frame = visao.Visao().atualizarRobosAliados(frame)
		
		robosAliados = visao.Visao().robosAliados
		bola = visao.Visao().bola
		
		GLib.idle_add(self.updateRobotsInfo, robosAliados, bola)
		
		return cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)
