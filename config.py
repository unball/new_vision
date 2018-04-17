import json
import cv2
import numpy as np

#TODO: GET MOUSE EVENTS. CONFIGURE EACH HSV SEGMENTATION. WRITE CONFGS IN JSON FILE

with open('config.json') as f:
	config = json.load(f)

def configure_transformation():
    cap = cv2.VideoCapture(0)

    if cap.isOpened() == False:
        print "Error while trying to open video input. Check your webcam or your file and try again."
        exit()

    while True:
        ret, frame = cap.read()

        cv2.imshow("frame", frame)

        key = cv2.waitKey(1)

        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

def configure_segmentation():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        cv2.imshow("frame", frame)

        key = cv2.waitKey(1)

        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

        elif key == ord('b'):
            print "configuring ball"

        elif key == ord('0'):
            print "configuring first allie"

        elif key == ord('1'):
            print "configuring second allie"

        elif key == ord('2'):
            print "configuring third allie"

        elif key == ord('a'):
            print "configuring allie's shirt"

        elif key == ord('e'):
            print "configuring enemy's shirt"

if __name__ == '__main__':
    configure_transformation()
    #configure_segmentation()
    exit()
