#!/usr/bin/env python
import rospy
import numpy
import cv2
from include.videoplayer import VideoPlayer

def start():
    raw_video = cv2.VideoCapture('../videos/teste.mp4')

    while (raw_video.isOpened() == True): #MAIN LOOP WHERE THE FRAMES ARE CATCH
        ret, frame = raw_video.read()
        raw_player = VideoPlayer()

if __name__ == '__main__':
    start()
