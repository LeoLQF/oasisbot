#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('oasisbot_cv')
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    self.save_to_path = rospy.get_param("~save_to_path", "../data/rgb_image/")
    self.img_source = rospy.get_param("~img_source", "/camera/rgb/image_raw")

    self.image_sub = rospy.Subscriber(self.img_source, Image, self.callback)

    self.bridge = CvBridge()

    self.timer = rospy.Time.now().to_sec()

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    if (rospy.Time.now().to_sec() - self.timer)*1000>500:  # record rgb image per of 500ms
      self.timer = rospy.Time.now().to_sec()

      cv2.imwrite(self.save_to_path+"image" + str(rospy.Time.now()) + ".jpg", cv_image)
      cv2.putText(cv_image, 'save rgb image Succeed', (10, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 5)
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
      rospy.loginfo("save rgb image Succeed")
    else:
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)

def main(args):
  rospy.init_node('image_collection_from_image_stream', anonymous=True)
  image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
