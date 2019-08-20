import cv2
import singleton
import configFile
import numpy as np
import pixel2metric

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

class Visao(metaclass=singleton.Singleton):
	def __init__(self):
		self.robosAliados = [
			RoboAliado(0),
			RoboAliado(1),
			RoboAliado(2),
			RoboAliado(3),
			RoboAliado(4)
		]
		self.robosAdversarios = []
		self.bola = None
		self.angles = np.array([0, 90, 180, -90, -180])
		self.homography = None
		self.default_preto_hsv = [0,94,163,360,360,360]
		self.default_time_hsv = [13,0,0,32,360,360]
		self.default_bola_hsv = [0, 117, 0, 98, 360, 360]
		
		self.preto_hsv = np.array(configFile.getValue("preto_hsv_interval", self.default_preto_hsv))
		self.time_hsv = np.array(configFile.getValue("time_hsv_interval", self.default_time_hsv))
		self.bola_hsv = np.array(configFile.getValue("bola_hsv_interval", self.default_bola_hsv))
		self.homography = np.array(configFile.getValue("homography_matrix"))
	
	def atualizarPretoHSV(self, value, index):
		self.preto_hsv[index] = value
		config = configFile.getConfig()
		config["preto_hsv_interval"][index] = value
		configFile.saveConfig(config)
	
	def atualizarTimeHSV(self, value, index):
		self.time_hsv[index] = value
		config = configFile.getConfig()
		config["time_hsv_interval"][index] = value
		configFile.saveConfig(config)
	
	def atualizarBolaHSV(self, value, index):
		self.bola_hsv[index] = value
		config = configFile.getConfig()
		config["bola_hsv_interval"][index] = value
		configFile.saveConfig(config)
	
	def obterRobosAliados(self):
		return self.robosAliados
		
	def atualizarRobos(self, robosAliadosIdentificados, robosAdversariosIdentificados, bola):
		# Computa novos robos inimigos
		self.robosAdversarios = [RoboAdversario(robo[0], robo[1]) for robo in robosAdversariosIdentificados]
		
		# Atualiza posição da bola
		self.bola = bola
		
		for robo in self.robosAliados:
			identificado = False
			for roboIdentificado in robosAliadosIdentificados:
				if roboIdentificado[0] == robo.identificador:
					robo.centro = roboIdentificado[1]
					robo.angulo = roboIdentificado[2]
					robo.estado = "Identificado"
					identificado = True
					break
			if not identificado: robo.estado = "Não-Identificado"
	
	def updateHomography(self, points, shape):
		height, width, _ = shape
		frame_points = np.array([[0,0],[0, height],[width,0],[width,height]])
		h, mask = cv2.findHomography(np.array(points), frame_points, cv2.RANSAC)
		configFile.setValue("homography_matrix", h.tolist())
		
		self.homography = h
	
	def warp(self, frame):
		homography_matrix = self.homography
		if(homography_matrix is not None): return cv2.warpPerspective(frame, homography_matrix, (frame.shape[1], frame.shape[0]))
		else: return frame
		
	
	def definePoly(self, countor):
		rect = cv2.minAreaRect(countor)
		contourArea = cv2.contourArea(countor)
		rectArea = rect[1][0]*rect[1][1]
		
		return 4 if contourArea/rectArea > 0.7 else 3
	
	def detectarCamisa(self, component_mask):
		# Encontra um contorno para a camisa com base no maior contorno
		_,mainContours,_ = cv2.findContours(component_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		mainContour = sorted(mainContours, key=cv2.contourArea)[-1]
		
		# Encontra o menor retângulo que se inscreve na camisa
		rectangle = cv2.minAreaRect(mainContour)
		
		# Calcula a posição e ângulo parcial da camisa com base no retângulo
		center = rectangle[0]
		centerMeters = pixel2metric.pixel2meters(center, component_mask.shape)
		angle = rectangle[-1]
		
		return center, centerMeters, angle
		
	def detectarTime(self, img, componentMask, componentTeamMask):
		# Frame a ser renderizado
		renderFrame = img.copy()
		
		# Encontra os contornos internos com área maior que um certo limiar e ordena
		_,internalContours,_ = cv2.findContours(componentTeamMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
		internalContours = [countor for countor in internalContours if cv2.contourArea(countor)>10]
		internalContours = sorted(internalContours, key=cv2.contourArea)
		
		countInternalContours = len(internalContours)
			
		# Obtém centro e ângulo da camisa
		center, centerMeters, angle = self.detectarCamisa(componentMask)
		
		# Não é do nosso time
		if countInternalContours == 0:
			return None, centerMeters, angle, renderFrame
		
		# Seleciona a forma principal
		mainShape = internalContours[-1]
		
		# Desenha o contorno no frame
		cv2.drawContours(renderFrame, mainShape, 0, (255,0,0), 1)
		
		# Calcula o centro do contorno principal
		M = cv2.moments(mainShape)
		cX = M["m10"] / M["m00"]
		cY = M["m01"] / M["m00"]
		
		# Calcula o ângulo com base no vetor entre o centro do contorno principal e o centro da camisa
		calculatedAngle = 180.0/np.pi *np.arctan2(-(center[1]-cY), center[0]-cX)
		partialAngles =  -calculatedAngle + self.angles
		estimatedAngle = partialAngles[np.abs(calculatedAngle - partialAngles).argmin()]
		
		# Define qual o polígono da figura principal
		poligono = self.definePoly(mainShape)
		
		# Computa o identificador com base na forma e no número de contornos internos
		identificador = (0 if poligono == 3 else 2) + countInternalContours -1
		
		# Insere o número do robô no frame
		cv2.putText(renderFrame, str(identificador), (int(center[0])-10, int(center[1])+10), cv2.FONT_HERSHEY_TRIPLEX, 1, (0,0,0))
		
		return identificador, centerMeters, estimatedAngle, renderFrame
	
	def segmentarFundo(self, frame):
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		return cv2.inRange(img_hsv, self.preto_hsv[0:3], self.preto_hsv[3:6])
	
	def segmentarTime(self, frame):
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, self.preto_hsv[0:3], self.preto_hsv[3:6])
		return mask & cv2.inRange(img_hsv, self.time_hsv[0:3], self.time_hsv[3:6])
	
	def segmentarBola(self, frame):
		img_filtered = cv2.GaussianBlur(frame, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, self.preto_hsv[0:3], self.preto_hsv[3:6])
		return mask & cv2.inRange(img_hsv, self.bola_hsv[0:3], self.bola_hsv[3:6])
	
	def atualizarRobosAliados(self, frame):
		# Corta o campo
		img_warpped = self.warp(frame)
		
		# Segmenta o fundo
		img_filtered = cv2.GaussianBlur(img_warpped, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, self.preto_hsv[0:3], self.preto_hsv[3:6])
		
		# Segmenta o time
		teamMask = cv2.inRange(img_hsv, self.time_hsv[0:3], self.time_hsv[3:6])
		
		# Encontra componentes conectados e aplica operações de abertura e dilatação
		num_components, components = cv2.connectedComponents(mask)
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
		components = cv2.morphologyEx(np.uint8(components), cv2.MORPH_OPEN, kernel)
		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
		components = cv2.dilate(np.uint8(components), kernel, iterations=1)
		
		# Frame zerado a ser renderizado
		processed_image = np.zeros(img_warpped.shape, np.uint8)
		
		# Listas com aliados e inimigos
		robosAliadosIdentificados = []
		robosAdversariosIdentificados = []
		
		# Itera por cada elemento conectado
		for label in np.unique(components)[1:]:
			# Máscara do elemento
			componentMask = np.uint8(np.where(components == label, 255, 0))
			
			# Imagem com apenas o elemento
			comp = cv2.bitwise_and(img_warpped, img_warpped, mask=componentMask)
			
			# Máscara dos componentes internos do elemento
			componentTeamMask = componentMask & teamMask
			
			# Tenta detectar o elemento como sendo do nosso time ou adversário
			identificador, centro, angulo, component_image = self.detectarTime(comp, componentMask, componentTeamMask)
			
			# Adiciona às listas de aliados ou inimigos
			if identificador is not None:
				robosAliadosIdentificados.append((identificador, centro, angulo))
			else:
				robosAdversariosIdentificados.append((centro, angulo))
				
			if component_image is not None:
				processed_image = cv2.add(processed_image, component_image)

		# Segmenta a bola
		bolaMask = mask & cv2.inRange(img_hsv, self.bola_hsv[0:3], self.bola_hsv[3:6])
		_,bolaContours,_ = cv2.findContours(bolaMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		bolaContours = [countor for countor in bolaContours if cv2.contourArea(countor)>10]
		
		if len(bolaContours) != 0:
			bolaContour = max(bolaContours, key=cv2.contourArea)
			((x,y), radius) = cv2.minEnclosingCircle(bolaContour)
			cv2.circle(processed_image, (int(x),int(y)), int(radius), (0,255,0), 1)
			
			bola = (pixel2metric.pixel2meters((x,y), bolaMask.shape), radius)
		else: bola = None
			
		self.atualizarRobos(robosAliadosIdentificados, robosAdversariosIdentificados, bola)
		
		return processed_image
