#!/usr/bin/env python

import roslib; roslib.load_manifest('manual_override')
import rospy

import cv2

def camera():
    cv2.namedWindow('Camera View')
    vc = cv2.VideoCapture(0)

    if vc.isOpened():
        rval, frame = vc.read()
    else:
        print 'No camera'
        rval = False

    while rval:
        cv2.imshow('Camera View', frame)
        rval, frame = vc.read()
        key = cv2.waitKey(20)
        if key == 27:
            break

    cv2.destroyWindow('Camera View')

if __name__ == '__main__':
    rospy.init_node('watch_camera')
    camera()
