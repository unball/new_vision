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

        self.ball_position = []
        self.allies_clr_id = [[0, 0], [0, 0], [0, 0]]
        self.allies_shirt = []
        self.index = np.array((0, 0, 0)) # Index of a shirt compared to a robot. Ex.: index[0] = 1 means that the second value in M is the allie 0
        self.allies_position = [0, 0, 0]
        self.allies_angles = [0, 0, 0]

        self.buffer_ball = [0, 0]
        self.buffer_allies_clr = [[0, 0], [0, 0], [0, 0]]
        self.buffer_allies_shirt = [[0, 0], [0, 0], [0, 0]]

    def set_frame(self, frame):
        self.frame = frame

    def find_colours(self):
        self.find_ball()
        self.find_team()
        self.find_angles()
        self.mount_msg()

    def find_team(self):
        self.find_allie_0()
        self.find_allie_1()
        self.find_allie_2()
        self.find_allies_shirt()

        self.allies_clr_id = np.asarray(self.allies_clr_id)
        self.allies_shirt = np.asarray(self.allies_shirt)

        self.find_respectives_indexes()

        self.allies_position[0] = [(self.allies_clr_id[0][0] + self.allies_shirt[self.index[0]][0])/2, (self.allies_clr_id[0][1] + self.allies_shirt[self.index[0]][1])/2]
        self.allies_position[1] = [(self.allies_clr_id[1][0] + self.allies_shirt[self.index[1]][0])/2, (self.allies_clr_id[1][1] + self.allies_shirt[self.index[1]][1])/2]
        self.allies_position[2] = [(self.allies_clr_id[2][0] + self.allies_shirt[self.index[2]][0])/2, (self.allies_clr_id[2][1] + self.allies_shirt[self.index[2]][1])/2]


    def find_ball(self):

        mask = cv2.inRange(self.frame, self.segm_limits[0][0], self.segm_limits[0][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        if self.contours:
            M = cv2.moments(self.contours[0])

            if M['m00'] == 0:
                if self.buffer_ball == [0, 0]:
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


    def find_allie_0(self):
        mask = cv2.inRange(self.frame, self.segm_limits[1][0], self.segm_limits[1][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        if self.contours:
            M = cv2.moments(self.contours[0])

            if M['m00'] == 0:
                if (self.vision_msg.found[0] == False):
                    self.allies_clr_id[0] = [0, 0]
                else:
                    self.allies_clr_id[0] = self.buffer_allies_clr[0]
            else:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                self.allies_clr_id[0] = [cx, cy]
                self.buffer_allies_clr[0] = [cx, cy]

        else:
            self.allies_clr_id[0] = self.buffer_allies_clr[0]



    def find_allie_1(self):
        mask = cv2.inRange(self.frame, self.segm_limits[2][0], self.segm_limits[2][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        if self.contours:
            M = cv2.moments(self.contours[0])

            if M['m00'] == 0:
                if (self.vision_msg.found[1] == False):
                    self.allies_clr_id[1] = [0, 0]
                else:
                    self.allies_clr_id[1] = self.buffer_allies_clr[1]
            else:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                self.allies_clr_id[1] = [cx, cy]
                self.buffer_allies_clr[1] = [cx, cy]
        else:
            self.allies_clr_id[1] = self.buffer_allies_clr[1]

    def find_allie_2(self):
        mask = cv2.inRange(self.frame, self.segm_limits[3][0], self.segm_limits[3][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        if self.contours:
            M = cv2.moments(self.contours[0])

            if M['m00'] == 0:
                if (self.vision_msg.found[1] == False):
                    self.allies_clr_id[2] = [0, 0]
                else:
                    self.allies_clr_id[2] = self.buffer_allies_clr[2]
            else:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                self.allies_clr_id[2] = [cx, cy]
                self.buffer_allies_clr[2] = [cx, cy]
        else:
            self.allies_clr_id[2] = self.buffer_allies_clr[2]

    def find_allies_shirt(self):
        mask = cv2.inRange(self.frame, self.segm_limits[4][0], self.segm_limits[4][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

        if self.contours:

            for contour in range(len(self.contours)):

                M = cv2.moments(self.contours[contours])
                if M['m00'] == 0:
                    self.allies_shirt = self.buffer_allies_shirt

                else:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    self.allies_shirt.append([cx, cy])
                    self.buffer_allies_shirt.append([cx, cy])
        else:
            self.allies_shirt = self.buffer_allies_shirt


    def find_respectives_indexes(self):

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
        self.vision_msg.x[0] = self.allies_position[0][0]
        self.vision_msg.x[1] = self.allies_position[1][0]
        self.vision_msg.x[2] = self.allies_position[2][0]

        self.vision_msg.y[0] = self.allies_position[0][1]
        self.vision_msg.y[1] = self.allies_position[1][1]
        self.vision_msg.y[2] = self.allies_position[2][1]

        self.vision_msg.th[0] = self.allies_angles[0]
        self.vision_msg.th[1] = self.allies_angles[1]
        self.vision_msg.th[2] = self.allies_angles[2]

        if self.vision_msg.x[0]!=0 and self.vision_msg.x[0]!=0:
            self.vision_msg.found[0] = True

        if self.vision_msg.x[1]!=0 and self.vision_msg.x[1]!=0:
            self.vision_msg.found[1] = True

        if self.vision_msg.x[2]!=0 and self.vision_msg.x[2]!=0:
            self.vision_msg.found[2] = True

        self.vision_msg.ball_x = self.ball_position[0]
        self.vision_msg.ball_y = self.ball_position[1]


    def get_vision_msg(self):
        return self.vision_msg
