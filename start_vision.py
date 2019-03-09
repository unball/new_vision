#!/usr/bin/env python
import cv2
import numpy as np

import rospy

from vision.msg import VisionMessage

from image_processor import ImageProcessor
from pixel2metric import pixels2meters


FREQUENCY = 20 #In Hz
CAMERA_INDEX = 0

# -------------------- FUNCTIONS SECTION  --------------------

def start():
    # output_msg_output will be the message build with the information extracted from the image
    output_msg = VisionMessage()
    raw_video = cv2.VideoCapture(CAMERA_INDEX)
    #print "publicou"
    processor = ImageProcessor()

    if raw_video.isOpened() == False:
        print "Error while trying to open video input. Check your webcam or file and try again."
        exit()

    # Instantiate the objects of the message and the message publisher
    #pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)      TODO: NEW MESSAGE
    global pub
    pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)

    # Starts the ros node
    rospy.init_node('vision_node')

    # Define the publishing frequency in Hz
    global rate
    rate = rospy.Rate(FREQUENCY)

    # -------------------- MAIN LOOP SECTION  --------------------
    while (raw_video.isOpened() == True):
        # raw_frame stores the current frame read by 'VideoCapture::read()'
        # ret stores the return of that same method. False if no frames has be grabbed
        ret, raw_frame = raw_video.read()

        # Responsible to show the frames
        # cv2.imshow("raw_frame", raw_frame)
        # cv2.waitKey(1)

        # -------------------- PROCESSING ARCHITECTURE SECTION  --------------------
        e1 = cv2.getTickCount()
        processor.process_frame(raw_frame)
        processed_frame = processor.get_processed_frame()
        cv2.imshow("processed_frame", processed_frame)
        cv2.waitKey(1)
        output_msg = processor.get_vision_msg()
        output_msg = pixels2meters(output_msg)
        #print output_msg
        e2 = cv2.getTickCount()

        if not rospy.is_shutdown():
            pub.publish(output_msg)
            rate.sleep()

        time = (e2 - e1)/ cv2.getTickFrequency()

        print "Tempo de processamento: {} segundos".format(time)


# -------------------- MAIN SECTION  --------------------

if __name__ == '__main__':
    print " ---------- vision_node started ---------- "

    start()
