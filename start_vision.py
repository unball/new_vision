#!/usr/bin/env python
import cv2
import numpy

import rospy
from vision.msg import VisionMessage

from image_processor import ImageProcessor


FREQUENCY = 10 #In Hz


#TODO: 1. Coloursr finder; 2. Switch that guarantees message publishing in 10Hz
# -------------------- FUNCTIONS SECTION  --------------------

def build_dummy_output_msg(aux_output):
    for robot in range(6):
        aux_output.x[robot] = 1
        aux_output.y[robot] = 1
        aux_output.th[robot] = 1
        aux_output.found[robot] = True
    aux_output.ball_x = 1
    aux_output.ball_y = 1

def assembly_msg(aux_output):
    output_msg = VisionMessage()

    for robot in range(6):
        output_msg.x[robot] = aux_output.x[robot]
        output_msg.y[robot] = aux_output.y[robot]
        output_msg.th[robot] = aux_output.th[robot]
        output_msg.found[robot] = aux_output.found[robot]
    output_msg.ball_x = aux_output.ball_x = 1
    output_msg.ball_y = aux_output.ball_y = 1

    return output_msg

def publish_msg(pub, rate, aux_output):
    if not rospy.is_shutdown():
        output_msg = assembly_msg(aux_output)
        pub.publish(output_msg)
        rate.sleep()

def start():
    # aux_output will be the message build with the information extracted from the image
    aux_output = VisionMessage()
    raw_video = cv2.VideoCapture(-1)
    #print "publicou"
    processor = ImageProcessor()

    if raw_video.isOpened() == False:
        print "Error while trying to open video input. Check your webcam or file and try again."
        exit()

    # -------------------- MAIN LOOP SECTION  --------------------
    while (raw_video.isOpened() == True):

        # Instantiate the objects of the message and the message publisher
        output_msg = VisionMessage()
        pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)

        # Starts the ros node
        rospy.init_node('vision_node')

        # Define the publishing frequency in Hz
        rate = rospy.Rate(FREQUENCY)

        # raw_frame stores the current frame read by 'VideoCapture::read()'
        # ret stores the return of that same method. False if no frames has be grabbed
        ret, raw_frame = raw_video.read()

        # Responsible to show the frames
        # cv2.imshow("raw_frame", raw_frame)
        # cv2.waitKey(1)

        # -------------------- PROCESSING ARCHITECTURE SECTION  --------------------
        processed_frame = processor.process_frame(raw_frame)
        cv2.imshow("processed_frame", processed_frame)
        cv2.waitKey(1)

        # Build a output message for tests
        # build_dummy_output_msg(aux_output)


        # Function that wraps the ros methods responsible to publish the message
        publish_msg(pub, rate, aux_output)


# -------------------- MAIN SECTION  --------------------

if __name__ == '__main__':
    print " ---------- vision_node started ---------- "
    start()
