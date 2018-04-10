#!/usr/bin/env python
import cv2
import numpy

import rospy
from vision.msg import VisionMessage

from lib.videoplayer import VideoPlayer

FREQUENCY = 10 #In Hz

#TODO: 1. Architecture of image processing; 2. Image processors; 3. Finders

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
    aux_output = VisionMessage()
    raw_video = cv2.VideoCapture(0)
    raw_player = VideoPlayer()

    #MAIN LOOP WHERE THE FRAMES ARE CATCH
    while (raw_video.isOpened() == True):

        #raw_frame stores the current frame read by 'VideoCapture::read()'
        #ret stores the return of that same method. False if no frames has be grabbed
        ret, raw_frame = raw_video.read()

        #Responsible to show the frames
        raw_player.play(raw_frame, ret)

        #Instantiate the objects of the message and the message publisher
        output_msg = VisionMessage()
        pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)

        #Starts the ros node
        rospy.init_node('vision_node')

        #Define the publishing frequency in Hz
        rate = rospy.Rate(FREQUENCY)

        #Build a output message for tests
        build_dummy_output_msg(aux_output)

        #Function that wraps the ros methods responsible to publish the message
        publish_msg(pub, rate, aux_output)


if __name__ == '__main__':
    print " ---------- vision_node started ---------- "
    start()
