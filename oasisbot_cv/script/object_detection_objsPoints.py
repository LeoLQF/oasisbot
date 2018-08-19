#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from sensor_msgs.msg import Image
from object_msgs.msg import ObjectsInBoxes
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import tf

# [515.4758986346594, 0,                 315.8145803008886,
#  0,                 517.6997276723781, 250.6995339983923,
#  0,                 0,                 1]

class ObjectDetection_objsPoints():

    def __init__(self):
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        input_objsBoxs = rospy.get_param("~input_objsBoxs", "/objBoxs")
        input_depth = rospy.get_param("~input_depth", "/camera/depth/image")

        self.height = 480                      # rows
        self.width = 640                       # columns
        self.camera_fx = 515.4758986346594
        self.camera_fy = 517.6997276723781
        self.camera_cx = 315.8145803008886
        self.camera_cy = 250.6995339983923
        self.camera_factor = 1

        self._cv_bridge = CvBridge()

        self.cv_dipth_img = np.zeros((self.height, self.width))

        self._sub = rospy.Subscriber(input_objsBoxs,
                                     ObjectsInBoxes,
                                     self.callback_objsBoxs,
                                     queue_size=1)

        self._sub = rospy.Subscriber(input_depth,
                                     Image,
                                     self.callback_depth,
                                     queue_size=1)

        self._pub_objsPoints = rospy.Publisher('objsPoints',
                                              PolygonStamped,
                                              queue_size=1)




    def callback_objsBoxs(self, objsBoxs_msg):

        objsPints = PolygonStamped()
        objsPints.header = objsBoxs_msg.header

        for i in range(len(objsBoxs_msg.objects_vector)):
            x_offset = objsBoxs_msg.objects_vector[i].roi.x_offset
            y_offset = objsBoxs_msg.objects_vector[i].roi.y_offset
            height = objsBoxs_msg.objects_vector[i].roi.height
            width = objsBoxs_msg.objects_vector[i].roi.width

            # u_center <-- x + w*1/2
            # v_center <-- y + h*1/2
            u_center = x_offset + width * 1 / 2
            v_center = y_offset + height * 1 / 2

            d = self.cv_dipth_img[v_center, u_center]
            if math.isnan(d):
                rospy.logwarn("=== d is nan ============")

            else:
                z = d / self.camera_factor
                x = (u_center - self.camera_cx) * z / self.camera_fx
                y = (v_center - self.camera_cy) * z / self.camera_fy

                objPint = Point32()
                objPint.x = x
                objPint.y = y
                objPint.z = z

                objsPints.polygon.points.append(objPint)

        if objsPints.polygon.points:
            self._pub_objsPoints.publish(objsPints)
            # rospy.loginfo(objsPints)
        else:
            self._pub_objsPoints.publish(objsPints)
            pass  #


    def callback_depth(self, depth_msg):

        try:
            depth_img = self._cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        except CvBridgeError as e:
            rospy.loginfo(e)

        # # 有待改进
        # # 简单的填充深度图中的孔洞
        # depth_img = np.array(depth_img)
        # # 均值滤波
        # a = np.where(np.isnan(depth_img))
        # depth_img[a] = 0.6                      # 目标平均高度
        # depth_mean = cv2.blur(depth_img, (10, 10))
        # depth_img[a] = depth_mean[a]

        self.cv_dipth_img = depth_img

    def shutdown(self):
        rospy.loginfo("Stopping the  object point publisher...")
        rospy.sleep(1)

if __name__ == '__main__':

    rospy.init_node('object_detection_objsPoints')


    try:
        ObjectDetection_objsPoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_detection_objsPoints has stoped.")
