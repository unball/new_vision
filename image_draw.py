import cv2
import numpy as np

class ImageDraw:
    def __init__ (self):
        self.frame = None
        self.ball_position = []
        self.allies_position = []
        self.allies_angles = []

    def set_info(self, vision_msg):
        self.ball_position.append(vision_msg.ball_x)
        self.ball_position.append(vision_msg.ball_y)

        self.allies_position.append([vision_msg.x[0], vision_msg.y[0]])
        self.allies_position.append([vision_msg.x[1], vision_msg.y[1]])
        self.allies_position.append([vision_msg.x[2], vision_msg.y[2]])

        self.allies_angles.append(vision_msg.th[0])
        self.allies_angles.append(vision_msg.th[1])
        self.allies_angles.append(vision_msg.th[2])

    def set_frame(self, frame):
        self.frame = frame

    def draw_all_info(self):
        self.draw_ball()
        self.draw_allies()
        #self.draw_angles()

    def draw_ball(self):
        x = self.ball_position[0]
        y = self.ball_position[1]

        cv2.circle(self.frame, (x, y), 10, (255, 255, 255), 2)

    def draw_allies(self):

        x = self.allies_position[0][0]
        y = self.allies_position[0][1]
        xf = x + 15
        yf = y + 15
        cv2.rectangle(self.frame, (x, y), (xf, yf), (255, 0, 0), 2)

        x = self.allies_position[1][0]
        y = self.allies_position[1][1]
        xf = x + 15
        yf = y + 15
        cv2.rectangle(self.frame, (x, y), (xf, yf), (0, 255, 0), 2)

        x = self.allies_position[2][0]
        y = self.allies_position[2][1]
        xf = x + 15
        yf = y + 15
        cv2.rectangle(self.frame, (x, y), (xf,yf), (0, 0, 255), 2)

    def draw_angles(self):
        x = self.allies_position[0][0]
        y = self.allies_position[0][1]
        xf = int(x + np.cos(self.allies_angles[0]))
        yf = int(y + np.sin(self.allies_angles[0]))
        cv2.line(self.frame, (x, y), (xf, yf), (255, 0, 0), 2)

        x = self.allies_position[1][0]
        y = self.allies_position[1][1]
        xf = int(x + np.cos(self.allies_angles[1]))
        yf = int(y + np.sin(self.allies_angles[1]))
        cv2.line(self.frame, (x, y), (xf, yf), (0, 255, 0), 2)

        x = self.allies_position[2][0]
        y = self.allies_position[2][1]
        xf = int(x + np.cos(self.allies_angles[2]))
        yf = int(y + np.sin(self.allies_angles[2]))
        cv2.line(self.frame, (x, y), (xf, yf), (0, 255, 0), 2)

    def get_drawn_frame(self):
        return self.frame
