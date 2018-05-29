#!/usr/bin/env python
import cv2
import numpy as np

import rospy
from vision.msg import VisionMessage

from image_processor import ImageProcessor


FREQUENCY = 100 #In Hz


#TODO list: 1. Bug with angle detection; 2. Bug with image draw
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
        # output_msg = assembly_msg(aux_output)
        pub.publish(aux_output)
        rate.sleep()

def start():
    # output_msg_output will be the message build with the information extracted from the image
    output_msg = VisionMessage()
    raw_video = cv2.VideoCapture(-1)
    #print "publicou"
    processor = ImageProcessor()

    if raw_video.isOpened() == False:
        print "Error while trying to open video input. Check your webcam or file and try again."
        exit()


    # -------------------- MAIN LOOP SECTION  --------------------
    while (raw_video.isOpened() == True):

        # Instantiate the objects of the message and the message publisher
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
        e1 = cv2.getTickCount()
        processor.process_frame(raw_frame)
        processed_frame = processor.get_processed_frame()
        cv2.imshow("processed_frame", processed_frame)
        cv2.waitKey(1)
        output_msg = processor.get_vision_msg()
        #output_msg = pixel2metric(output_msg)
        #print output_msg
        e2 = cv2.getTickCount()

        # Function that wraps the ros methods responsible to publish the message
        publish_msg(pub, rate, output_msg)

        time = (e2 - e1)/ cv2.getTickFrequency()

        #print "Tempo de processamento: {} segundos".format(time)


# -------------------- MAIN SECTION  --------------------

if __name__ == '__main__':
    print " ---------- vision_node started ---------- "

    start()
