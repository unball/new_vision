import cv2

class VideoPlayer():
    def __init__(self, frame = None, eofr_flag = None):
        self.eofr_flag = eofr_flag
        self.frame = frame

        if (frame != None) and (eofr_flag != None):
            play(frame, eofr_flag)

    def play(self, frame = self.frame, eofr_flag = self.eofr_flag):
        if eofr_flag == True:
            cv2.imshow('raw_video', frame)

            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
