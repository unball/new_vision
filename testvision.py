import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages' 
if ros_path in sys.path:
	sys.path.remove(ros_path)
import cv2 
import numpy as np

#cap = cv2.VideoCapture(3)

name = 'video'
radius = 5

clickedPoints = []
homography_matrix = None
height, width = (None, None)

def get_mouse_clicks(event, x, y, flags, params):
	if event == cv2.EVENT_LBUTTONDOWN:
		print ("Point clicked: ({},{})".format(x, y))
		clickedPoints.append([x,y])

def findHomography(img):
	global height, width
	if input("Quer usar os valores pré-definidos? [s/n] ") == 's':
		clickedPoints = [[60,36], [535,41], [57,459], [541,455]]
	else:
		cv2.imshow("nohomography", img)
		cv2.setMouseCallback("nohomography", get_mouse_clicks)
		cv2.waitKey(0)
	
	height, width, _ = img.shape
	points = np.array([[0,0],[width, 0],[0,height],[width,height]])
	h, mask = cv2.findHomography(np.array(clickedPoints), points, cv2.RANSAC)
	im1Reg = cv2.warpPerspective(img, h, (width, height))
	cv2.imshow("homography", im1Reg)
	cv2.waitKey(0)
	return h


lista_min = np.array([0,94,163])
lista_max = np.array([360,360,360])

lista_min_preto = lista_min.copy()
lista_max_preto = lista_max.copy()

lista_min_time = np.array([13,0,0])
lista_max_time = np.array([32,360,360])

frame = cv2.imread('frame.png')

def callback(arg):
	lista_min[0] = cv2.getTrackbarPos('hmin', name)
	lista_min[1] = cv2.getTrackbarPos('smin', name)
	lista_min[2] = cv2.getTrackbarPos('vmin', name)
	lista_max[0] = cv2.getTrackbarPos('hmax', name)
	lista_max[1] = cv2.getTrackbarPos('smax', name)
	lista_max[2] = cv2.getTrackbarPos('vmax', name)


def createConfigWindow():
	cv2.destroyWindow(name)
	cv2.namedWindow(name, cv2.WINDOW_NORMAL)
	cv2.createTrackbar('hmin', name, lista_min[0], 360, callback)
	cv2.createTrackbar('smin', name, lista_min[1], 360, callback)
	cv2.createTrackbar('vmin', name, lista_min[2], 360, callback)
	cv2.createTrackbar('hmax', name, lista_max[0], 360, callback)
	cv2.createTrackbar('smax', name, lista_max[1], 360, callback)
	cv2.createTrackbar('vmax', name, lista_max[2], 360, callback) 

def segmentarPreto():
	while(True):
		#ret,frame = cap.read()
		img_warpped = cv2.warpPerspective(frame, homography_matrix, (width, height))
		img_filtered = cv2.GaussianBlur(img_warpped, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, lista_min, lista_max)

		#cv2.imshow('raw',img_warpped)
		cv2.imshow(name, mask)
		cv2.imshow('aux', cv2.bitwise_not(mask))
		key = cv2.waitKey(1)

		if key == ord('q'):
			return True
	return False
	
def segmentarTime():
	while(True):
		#ret,frame = cap.read()
		img_warpped = cv2.warpPerspective(frame, homography_matrix, (width, height))
		img_filtered = cv2.GaussianBlur(img_warpped, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, lista_min_preto, lista_max_preto)
		mask2 = mask & cv2.inRange(img_hsv, lista_min, lista_max)

		cv2.imshow('raw', cv2.bitwise_and(img_warpped, img_warpped, mask=mask))
		cv2.imshow(name, mask2)
		key = cv2.waitKey(1)

		if key == ord('q'):
			return True
	return False


def findPolys():
	while(True):
		#ret,frame = cap.read()
		img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, lista_min, lista_max)
		
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius,radius))
		img_morph= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		img_morph = cv2.dilate(img_morph,kernel,iterations = 1)

		img2 = cv2.bitwise_and(frame,frame,mask=img_morph)


		countors,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for countor in countors:
			perimetro = cv2.arcLength(countor, True)
			points = cv2.approxPolyDP(countor, 0.1*perimetro, True)
			cv2.drawContours(img2, points, -1, (0,0,255), 4)

		cv2.imshow('raw',frame)
		cv2.imshow(name, img2)
		key = cv2.waitKey(1)

		if key == ord('q'):
			return True
	return False

def rectangle_angle_and_center(rectangle):
	center = rectangle[0]
	angle = -rectangle[-1]
	if rectangle[1][1] < rectangle[1][0]:
		angle = (angle + 90)%180 - 180
	return (center, angle)

def definePoly(countor):
	perimetro = cv2.arcLength(countor, True)
	points = cv2.approxPolyDP(countor, 0.05*perimetro, True)
	return len(points)

angles = np.array([0, 90, 180, -90, -180])
center_robot = np.array([0,0])
center_form = np.array([0,0])

def isOwm(img,mask):
	contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours = sorted(contours, key=cv2.contourArea)
	contour = contours[-1]
	rectangle = cv2.minAreaRect(contour)
	img2 = img.copy()

	center = rectangle[0]
	angle = rectangle[-1]
	print("centro {0}",format(rectangle[0]) )
	print("angulo {0}",format(rectangle[-1]) )
	
	box = cv2.boxPoints(rectangle) 
	box = np.int0(box)
	cv2.drawContours(img2,[box],0,(255,0,255),1)

	img_filtered = cv2.GaussianBlur(img, (5,5), 0)
	img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(img_hsv, lista_min_time, lista_max_time)

	countors,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
	countors = [countor for countor in countors if cv2.contourArea(countor)>10]
	countors = sorted(countors, key=cv2.contourArea)
	
	if len(countors)!=0:
		print(len(countors))
		countor = countors[-1]
		angle1 = cv2.fitEllipse(countor)[-1]
		cv2.drawContours(img2,countor,0,(255,0,0),1)
		M = cv2.moments(countor)
		cX = M["m10"] / M["m00"]
		cY = M["m01"] / M["m00"]
		angle_c = 180.0/np.pi *np.arctan2(-(center[1]-cY), center[0]-cX)
		print("angule calc {0}".format(angle_c))
		angles_p =  -angle + angles
		print("angule pred {0}".format(angles_p[np.abs(angle_c -angles_p).argmin()]))
		angles_p1 =  -angle1 + angles
		print("angule pred2 {0}".format(angles_p1[np.abs(angle_c -angles_p1).argmin()]))

		perimetro = cv2.arcLength(countor, True)
		points = cv2.approxPolyDP(countor, 0.05*perimetro, True)
		cv2.drawContours(img2, points, -1, (0,0,255), 4)

		print(definePoly(countors[-1]))
		print('--------')
		return img2
	return -1


def recognizeRobots(img, mask):
	countors,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	center, rad = cv2.minEnclosingCircle(points) 


	
	pass

def detectarRobos():
	while(True):
		#ret,frame = cap.read()
		img_warpped = cv2.warpPerspective(frame, homography_matrix, (width, height))
		img_filtered = cv2.GaussianBlur(img_warpped, (5,5), 0)
		img_hsv = cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, lista_min_preto, lista_max_preto)



		num_components, components = cv2.connectedComponents(mask)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
		components = cv2.morphologyEx(np.uint8(components), cv2.MORPH_OPEN, kernel)
		kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
		components = cv2.dilate(np.uint8(components), kernel, iterations=1)

		cv2.imshow("raw", frame)

		for label in np.unique(components)[1:]:
			component_mask = np.uint8(np.where(components == label, 255, 0))
			comp = cv2.bitwise_and(img_warpped, img_warpped, mask=component_mask)
			cv2.imshow("component", comp)
			cv2.waitKey(0)
			cv2.imshow("component", isOwm(comp,component_mask))
			cv2.waitKey(0)
		break

def findElip():
	while(True):
		#ret,frame = cap.read()
		if not ret:
			continue
		img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(img_hsv, lista_min, lista_max)
		
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius,radius))
		img_morph= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		img_morph = cv2.dilate(img_morph,kernel,iterations = 2)

		img2 = cv2.bitwise_and(frame,frame,mask=img_morph)


		_,countors,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		countors = sorted(countors, key=cv2.contourArea)
		
		for countor in countors:
			if len(countor)>5 and cv2.contourArea(countor, True)> 30:
				eli= cv2.fitEllipse(countor)
				print(cv2.contourArea(countor, True))
				cv2.ellipse(img2, eli, (0,0, 255))
		
		#cv2.imshow('raw',frame)
		cv2.imshow(name, img2)
		key = cv2.waitKey(1)

		if key == ord('q'):
			return True
	return False


while(True):
	# Cortar e retificar campo
	homography_matrix = findHomography(frame)
	
	# Segmentar background
	createConfigWindow()
	while not segmentarPreto():
		if  input('impossivel seguimentar, erro na leitura, quer tentar de novo? s/n')!='s':
			print(':(')
			exit()
	print("Ó você segmentou o preto!")
	lista_min_preto = lista_min.copy()
	lista_max_preto = lista_max.copy()
	
	# Segmentar time
	lista_min = lista_min_time.copy()
	lista_max = lista_max_time.copy()
	
	createConfigWindow()
	while not segmentarTime():
		if  input('impossivel seguimentar, erro na leitura, quer tentar de novo? s/n')!='s':
			print(':(')
			exit()
	print("Ó você segmentou o time!")
	lista_min_time = lista_min.copy()
	lista_max_time = lista_max.copy()
	
	detectarRobos()
	
	
	break

cv2.destroyAllWindows()
