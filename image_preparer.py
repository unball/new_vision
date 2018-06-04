import cv2
import numpy as np


class ImagePreparer:
    def __init__(self, points, kernel):
        self.kernel = kernel
        self.points = points

    def set_frame(self, frame):
        self.frame = frame

    def apply_all_filters(self):
        self.geometrical_transformer()
        self.blur()
        #self.hsv_conversor()
        #self.improve_mask()

    def geometrical_transformer(self):
        rows = 480
        columns = 640

        pts1 = np.float32([[self.points[0][0], self.points[0][1]], [self.points[1][0], self.points[1][1]], [self.points[2][0], self.points[2][1]], [self.points[3][0], self.points[3][1]]])
        pts2 = np.float32([[0, 0], [columns, 0], [0, rows], [columns, rows]])

        M = cv2.getPerspectiveTransform(pts1, pts2)

        self.frame = cv2.warpPerspective(self.frame, M, (columns, rows))

    def blur(self):
        self.frame = cv2.GaussianBlur(self.frame, self.kernel, 0)

    def hsv_conversor(self):
        self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

    def improve_mask(self):
        self.frame = cv2.morphologyEx(self.frame, cv2.MORPH_OPEN, self.kernel)

    def get_processed_frame(self):
        return self.frame
