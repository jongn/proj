#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("io/internal_camera/head_camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(gray,25,255,1)

    contours,h = cv2.findContours(thresh,1,2)

    cv2.imshow('thresh', thresh)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print len(approx)
        if len(approx)==5:
            print "pentagon"
            cv2.drawContours(img,[cnt],0,255,-1)
        elif len(approx)==3:
            print "triangle"
            cv2.drawContours(img,[cnt],0,(0,255,0),-1)
        elif len(approx)==4:
            print "square"
            cv2.drawContours(img,[cnt],0,(0,0,255),-1)
        elif len(approx) == 9:
            print "half-circle"
            cv2.drawContours(img,[cnt],0,(255,255,0),-1)
        elif len(approx) > 15:
            print "circle"
            cv2.drawContours(img,[cnt],0,(0,255,255),-1)

    cv2.imshow('img',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)