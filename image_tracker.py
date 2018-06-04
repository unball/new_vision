import cv2
import numpy as np
from vision.msg import VisionMessage

offset = 15

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
        self.allies_position = [[], [], []]
        self.allies_angles = [0, 0, 0]
        self.robots_found = [[False], [False], [False],  [False],  [False],  [False]]

    def set_frame(self, frame):
        self.frame = frame

    def find_colours(self):
        self.find_ball()
        self.find_team()
        self.mount_msg()

    def find_team(self):
        self.find_allies_shirt()
        self.find_robots()


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

    def find_robots(self):
        self.robots_found = np.array([[False], [False], [False],  [False],  [False],  [False]])
        self.allies_angles = [0, 0, 0]

        for position in self.allies_shirt:
            shirt_x = position[0]
            shirt_y = position[1]
            xi, xf = shirt_x - offset, shirt_x + offset
            yi, yf = shirt_y - offset, shirt_y + offset
            robot_roi = self.frame[yi:yf, xi:xf]

            # Find robot 0
            mask = cv2.inRange(robot_roi, self.segm_limits[1][0], self.segm_limits[1][1])
            if isinstance(mask, type(None)):
                pass
            else:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
                self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
                if self.contours:
                    M = cv2.moments(self.contours[0])
                    if M['m00'] == 0:
                        local_id_x, local_id_y = 0, 0
                    else:
                        local_id_x = int(M['m10']/M['m00'])
                        local_id_y = int(M['m01']/M['m00'])
                        global_id_x = local_id_x + shirt_x + offset
                        global_id_y = local_id_y + shirt_y + offset

                        true_x = (global_id_x/2) + shirt_x
                        true_y = (global_id_y/2) + shirt_y

                        self.allies_position[0] = [shirt_x, shirt_y]
                        self.robots_found[0] = True
                        self.allies_angles[0] = self.find_angle(local_id_x, local_id_y, offset/2, offset/2)
                else:
                    # Find robot 1
                    mask = cv2.inRange(robot_roi, self.segm_limits[2][0], self.segm_limits[2][1])
                    if isinstance(mask, type(None)):
                        pass
                    else:
                        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
                        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
                        if self.contours:
                            M = cv2.moments(self.contours[0])
                            if M['m00'] == 0:
                                local_id_x, local_id_y = 0, 0
                            else:
                                local_id_x = int(M['m10']/M['m00'])
                                local_id_y = int(M['m01']/M['m00'])
                                global_id_x = local_id_x + shirt_x + offset
                                global_id_y = local_id_y + shirt_y + offset

                                true_x = (global_id_x/2) + shirt_x
                                true_y = (global_id_y/2) + shirt_y

                                self.allies_position[1] = [shirt_x, shirt_y]
                                self.robots_found[1] = True
                                self.allies_angles[1] = self.find_angle(global_id_x, global_id_y, shirt_x, shirt_y)
                        else:
                            # Find robot 2
                            mask = cv2.inRange(robot_roi, self.segm_limits[3][0], self.segm_limits[3][1])
                            if isinstance(mask, type(None)):
                                pass
                            else:
                                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
                                self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
                                if self.contours:
                                    M = cv2.moments(self.contours[0])
                                    if M['m00'] == 0:
                                        local_id_x, local_id_y = 0, 0
                                    else:
                                        local_id_x = int(M['m10']/M['m00'])
                                        local_id_y = int(M['m01']/M['m00'])
                                        global_id_x = local_id_x + shirt_x + offset
                                        global_id_y = local_id_y + shirt_y + offset

                                        true_x = (global_id_x/2) + shirt_x
                                        true_y = (global_id_y/2) + shirt_y

                                        self.allies_position[2] = [shirt_x, shirt_y]
                                        self.robots_found[2] = True
                                        self.allies_angles[2] = self.find_angle(global_id_x, global_id_y, shirt_x, shirt_y)


    def find_allies_shirt(self):
        mask = cv2.inRange(self.frame, self.segm_limits[4][0], self.segm_limits[4][1])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        self.contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        #print len(self.contours)
        self.allies_shirt = []
        if self.contours:
            for contour in self.contours:
                M = cv2.moments(contour)
                if M['m00'] == 0:
                    self.allies_shirt.append([0, 0])
                else:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.allies_shirt.append([cx, cy])

    def find_angle(self, id_x, id_y, shirt_x, shirt_y):
        # Angle will be calculated as th = arctg(dy/dx)
        # -1 'cause the y axis of a image grows in down direction, so in fact we're doing dy = shirt.cy - clr_id.cy
        dx = id_x - shirt_x
        dy = -1 * (id_y - shirt_y)
        theta = np.arctan2(dy, dx)
        return theta

    def mount_msg(self):
        for robot in xrange(3):
            if self.robots_found[robot] == True:
                self.vision_msg.found[robot] = True
                self.vision_msg.x[robot] = self.allies_position[robot][0]
                self.vision_msg.y[robot] = self.allies_position[robot][1]
                self.vision_msg.th[robot] = self.allies_angles[robot]

        self.vision_msg.ball_x = self.ball_position[0]
        self.vision_msg.ball_y = self.ball_position[1]

    def get_vision_msg(self):
        return self.vision_msg
