import cv2
import numpy as np
from vision.msg import VisionMessage

class ImageTracker:
    def __init__(self, segm_limits, kernel):
        self.kernel = kernel
        self.frame = None
        self.vision_msg = VisionMessage()
        self.contours = None
        self.segm_limits = np.array(segm_limits)

        self.ball_position = [0, 0]
        self.buffer_ball = [0, 0]

        self.allies_shirt = []
        self.robotID_0 = []
        self.robotID_1 = []
        self.robotID_2 = []

        self.robots_found = np.array([[False, False], [False, False], [False, False], [False, False], [False, False], [False, False]])

        self.allies_position = [[0, 0], [0, 0], [0, 0]]
        self.allies_angles = [0, 0, 0]

        self.index = np.array((0, 0, 0)) # Index of a shirt compared to a robot. Ex.: index[0] = 1 means that the second value in M is the allie 0


    def set_frame(self, frame):
        self.frame = frame

    def find_colours(self):
        self.find_ball()
        self.find_team()
        #self.find_angles()
        self.mount_msg()

    def find_team(self):
        self.find_allies_shirt()
        self.findID_0()
        self.findID_1()
        self.findID_2()

        #self.allies_clr_id = np.asarray(self.allies_clr_id)
        #self.allies_shirt = np.asarray(self.allies_shirt)

        #self.find_respectives_indexes()

        #self.allies_position[0] = [(self.allies_clr_id[0][0] + self.allies_shirt[self.index[0]][0])/2, (self.allies_clr_id[0][1] + self.allies_shirt[self.index[0]][1])/2]
        #self.allies_position[1] = [(self.allies_clr_id[1][0] + self.allies_shirt[self.index[1]][0])/2, (self.allies_clr_id[1][1] + self.allies_shirt[self.index[1]][1])/2]
        #self.allies_position[2] = [(self.allies_clr_id[2][0] + self.allies_shirt[self.index[2]][0])/2, (self.allies_clr_id[2][1] + self.allies_shirt[self.index[2]][1])/2]


    def find_ball(self):
        mask = cv2.inRange(self.frame, self.segm_limits[0][0], self.segm_limits[0][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        if self.contours:
            M = cv2.moments(self.contours[0])
            if M['m00'] == 0:
                if self.buffer_ball  == [0, 0]:
                    self.ball_position = [0, 0]
                else:
                    self.ball_position = self.buffer_ball
            else:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.ball_position = [cx, cy]
                self.buffer_ball = [cx, cy]
        else:
            self.ball_position = self.buffer_ball


    def findID_0(self):
        mask = cv2.inRange(self.frame, self.segm_limits[1][0], self.segm_limits[1][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        #print len(self.contours)

        self.robotID_0 = []
        if self.contours:
            for contour in self.contours:
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    if (self.vision_msg.found[0] == False):
                        pass
                    else:
                        pass
                else:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.robotID_0.append([cx, cy])

    def findID_1(self):
        mask = cv2.inRange(self.frame, self.segm_limits[2][0], self.segm_limits[2][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        #print len(self.contours)

        self.robotID_1 = []
        if self.contours:
            for contour in self.contours:
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    if (self.vision_msg.found[1] == False):
                        pass
                    else:
                        pass
                else:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.robotID_1.append([cx, cy])

    def findID_2(self):
        mask = cv2.inRange(self.frame, self.segm_limits[3][0], self.segm_limits[3][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        #print len(self.contours)

        self.robotID_2 = []
        if self.contours:
            for contour in self.contours:
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    if (self.vision_msg.found[2] == False):
                        pass
                    else:
                        pass
                else:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.robotID_2.append([cx, cy])


    def find_allies_shirt(self):
        mask = cv2.inRange(self.frame, self.segm_limits[4][0], self.segm_limits[4][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        self.allies_shirt = []
        #print len(self.contours)
        if self.contours:
            for contour in self.contours:
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    self.allies_shirt.append([0, 0])
                else:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.allies_shirt.append([cx, cy])

    def find_respectives_indexes(self):

        robot_aux = np.array(self.robotID_0)
        shirt_aux = np.array(self.allies_shirt)
        for id_obj in robot_aux:
            for shirt_obj in shirt_aux:
                

        # Find index for robot 0
        aux_dist = np.linalg.norm(self.allies_clr_id[0] - self.allies_shirt[0])
        self.index[0] = 0
        for contour in range(len(self.allies_shirt)):
            if (np.linalg.norm(self.allies_clr_id[0] - self.allies_shirt[contour])) < aux_dist:
                aux_dist = np.linalg.norm(self.allies_clr_id[0] - self.allies_shirt[contour])
                self.index[0] = contour

            if (np.linalg.norm(self.allies_clr_id[0] - self.allies_shirt[2])) < aux_dist:
                aux_dist = np.linalg.norm(self.allies_clr_id[0] - self.allies_shirt[contour])
                self.index[0] = contour

        # Find index for robot 1
        aux_dist = np.linalg.norm(self.allies_clr_id[1] - self.allies_shirt[0])
        self.index[1] = 0
        for contour in range(len(self.allies_shirt)):
            if (np.linalg.norm(self.allies_clr_id[1] - self.allies_shirt[contour])) < aux_dist:
                aux_dist = np.linalg.norm(self.allies_clr_id[1] - self.allies_shirt[contour])
                self.index[1] = contour

            if (np.linalg.norm(self.allies_clr_id[1] - self.allies_shirt[2])) < aux_dist:
                aux_dist = np.linalg.norm(self.allies_clr_id[1] - self.allies_shirt[contour])
                self.index[1] = contour

        # Find index for robot 2
        aux_dist = np.linalg.norm(self.allies_clr_id[2] - self.allies_shirt[0])
        self.index[2] = 0
        for contour in range(len(self.allies_shirt)):
            if (np.linalg.norm(self.allies_clr_id[2] - self.allies_shirt[contour])) < aux_dist:
                aux_dist = np.linalg.norm(self.allies_clr_id[2] - self.allies_shirt[contour])
                self.index[2] = contour

            if (np.linalg.norm(self.allies_clr_id[2] - self.allies_shirt[2])) < aux_dist:
                aux_dist = np.linalg.norm(self.allies_clr_id[2] - self.allies_shirt[contour])
                self.index[2] = contour

    def find_angles(self):
        # Angle will be calculated as th = arctg(dy/dx)
        # dx = clr_id.cx - shirt.cx
        # dy = -(clr_id.cy - shirt.cy); -1 'cause the y axis of a image grows in down direction, so in fact we're doing dy = shirt.cy - clr_id.cy

        dx = self.allies_clr_id[0][0] - self.allies_position[0][0]
        dy = -1 * (self.allies_clr_id[0][1] - self.allies_position[0][1])
        self.allies_angles[0] = np.arctan(dy/dx)

        dx = self.allies_clr_id[1][0] - self.allies_position[1][0]
        dy = -1 * (self.allies_clr_id[1][1] - self.allies_position[1][1])
        self.allies_angles[1] = np.arctan(dy/dx)

        dx = self.allies_clr_id[2][0] - self.allies_position[2][0]
        dy = -1 * (self.allies_clr_id[2][1] - self.allies_position[2][1])
        self.allies_angles[2] = np.arctan(dy/dx)

    def mount_msg(self):
        for allie_robot in xrange(3):
            self.vision_msg.x[allie_robot] = self.allies_position[0][0]
            self.vision_msg.y[allie_robot] = self.allies_position[0][1]
            self.vision_msg.th[allie_robot] = self.allies_angles[allie_robot]

        for robot in xrange(3):
            if self.robots_found[robot][0] == True and self.robots_found[robot][1] == True:
                self.vision_msg.found[robot] = True

        self.vision_msg.ball_x = self.ball_position[0]
        self.vision_msg.ball_y = self.ball_position[1]

    def get_vision_msg(self):
        return self.vision_msg
