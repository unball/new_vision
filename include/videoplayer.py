import cv2

class VideoPlayer():
    def __init__(self, frame = None, eofr_flag = None):
        self.eofr_flag = eofr_flag
        self.frame = frame

        if (frame != None) and (eofr_flag != None): #TODO: ver essa comparacao aqui
            self.play(frame, eofr_flag)

    def play(self, frame, eofr_flag):
        if eofr_flag == True:
            cv2.imshow('raw_video', frame)
            cv2.waitKey(1)
