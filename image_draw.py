import cv2
import numpy as np

class ImageDraw:
    def __init__ (self):
        self.frame = None
        self.ball_position = [0, 0]
        self.allies_position = [[0, 0], [0, 0], [0, 0]]
        self.allies_angles = [0, 0, 0]

    def set_info(self, vision_msg):
        self.ball_position[0] = vision_msg.ball_x
        self.ball_position[1] = vision_msg.ball_y

        for robot in range(3):
            self.allies_position[robot][0] = vision_msg.x[robot]
            self.allies_position[robot][1] = vision_msg.y[robot]

        for robot in range(3):
            self.allies_angles[robot] = vision_msg.th[robot]

    def set_frame(self, frame):
        self.frame = frame

    def draw_all_info(self):
        self.draw_ball()
        self.draw_allies()
        #self.draw_angles()

    def draw_ball(self):
        x = self.ball_position[0]
        y = self.ball_position[1]

        cv2.circle(self.frame, (x, y), 8, (255, 255, 255), 1)

    def draw_allies(self):

        x = self.allies_position[0][0]
        y = self.allies_position[0][1]
        xf = x + 15
        yf = y + 15
        cv2.rectangle(self.frame, (x, y), (xf, yf), (255, 0, 0), 1)

        x = self.allies_position[1][0]
        y = self.allies_position[1][1]
        xf = x + 15
        yf = y + 15
        cv2.rectangle(self.frame, (x, y), (xf, yf), (0, 255, 0), 1)

        x = self.allies_position[2][0]
        y = self.allies_position[2][1]
        xf = x + 15
        yf = y + 15
        cv2.rectangle(self.frame, (x, y), (xf,yf), (0, 0, 255), 1)

    def draw_angles(self):
        x = self.allies_position[0][0]
        y = self.allies_position[0][1]
        xf = int(x + np.cos(self.allies_angles[0]))
        yf = int(y + np.sin(self.allies_angles[0]))
        cv2.line(self.frame, (x, y), (xf, yf), (255, 0, 0), 1)

        x = self.allies_position[1][0]
        y = self.allies_position[1][1]
        xf = int(x + np.cos(self.allies_angles[1]))
        yf = int(y + np.sin(self.allies_angles[1]))
        cv2.line(self.frame, (x, y), (xf, yf), (0, 255, 0), 1)

        x = self.allies_position[2][0]
        y = self.allies_position[2][1]
        xf = int(x + np.cos(self.allies_angles[2]))
        yf = int(y + np.sin(self.allies_angles[2]))
        cv2.line(self.frame, (x, y), (xf, yf), (0, 255, 0), 1)

    def get_drawn_frame(self):
        return self.frame
