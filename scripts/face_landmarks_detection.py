#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('theatre')
import sys
import rospy
import rospkg
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import dlib
import glob


class image_converter:

  def __init__(self, predictor_path):
    self.detector = dlib.get_frontal_face_detector()
    self.predictor = dlib.shape_predictor(predictor_path)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
    self.image_pub = rospy.Publisher("image_landmarks",Image, queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    dbg_image = cv_image.copy()

    dets = self.detector(cv_image, 1)
    for k, d in enumerate(dets):
      shape = self.predictor(cv_image, d)

      for p_i in shape.parts():
        cv2.circle(dbg_image, (p_i.x,p_i.y), 1, (0,0,255))

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(dbg_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('face_landmarks_detection', anonymous=True)
  rospack = rospkg.RosPack()
  pkg_path = rospack.get_path('theatre')

  ic = image_converter(pkg_path+"/models/shape_predictor_68_face_landmarks.dat")

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

