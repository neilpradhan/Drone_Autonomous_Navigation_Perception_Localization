#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

c=0

class image_converter:
  
  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)  
    self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (640,480))

  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    global c
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # height, width = cv_image.shape[:2]
      # print('height',height)
      # print('width',width)

      #c=c+1

      #cv2.imwrite(str(c)+".png",cv_image)
      self.out.write(cv_image)

    except CvBridgeError as e:
      self.out.release()
      print(e)

  '''  # Convert BGR to HSV
          hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      
          # define range of the color we look for in the HSV space
          lower = np.array([0,0,250])
          upper = np.array([255,5,255])
      
          # Threshold the HSV image to get only the pixels in ranage
          mask = cv2.inRange(hsv, lower, upper)
      
          # Bitwise-AND mask and original image
          res = cv2.bitwise_and(cv_image, cv_image, mask= mask)
      
          # Publish the image
          try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
          except CvBridgeError as e:
            print(e)
      '''
def main(args):
  rospy.init_node('colorseg', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:

    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)