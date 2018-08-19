#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from object_msgs.msg import Objs_topology
import numpy as np
import tf

from swiftpro.msg import position

MATH_PI = 3.141592653589793238463
MATH_TRANS = 57.2958
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_UPPER_LOWER = (MATH_UPPER_ARM / MATH_LOWER_ARM)
LOWER_ARM_MAX_ANGLE = 135.6
LOWER_ARM_MIN_ANGLE = 0
UPPER_ARM_MAX_ANGLE = 100.7
UPPER_ARM_MIN_ANGLE = 0
LOWER_UPPER_MAX_ANGLE = 151
LOWER_UPPER_MIN_ANGLE = 10

class objsPoins_to_armEnd():

    def __init__(self):
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        input_objsTopology = rospy.get_param("~input_objsTopology", "/objsTopology")
        input_arrived= rospy.get_param("~input_Arrived", "/Arrived")

        self._sub = rospy.Subscriber(input_objsTopology,
                                     Objs_topology,
                                     self.callback_objsTopology,
                                     queue_size=1)

        self._sub = rospy.Subscriber(input_arrived,
                                     Bool,
                                     self.callback_arrived,
                                     queue_size=1)

        self._pub_position = rospy.Publisher('position_write_topic',
                                     position,
                                     queue_size=1)

        self.listener_base = tf.TransformListener()
        self.listener_armEnd = tf.TransformListener()

        self.pos = position()

        self.arrived = True             # 准备就绪，可以发送下一个目标点位置

    def callback_arrived(self, arrived_msg):
        self.arrived = arrived_msg.data



    def callback_objsTopology(self, objsTopology_msg):
        objsTopology = objsTopology_msg

        reachability_objs_arr = []
        for i in range(len(objsTopology.objsPoints)):

            obj_point = PointStamped()
            obj_point.header.frame_id = '/camera_rgb_optical_frame'
            new = rospy.Time(0)
            obj_point.header.stamp = new
            self.listener_base.waitForTransform("/Base", "/camera_rgb_optical_frame", new, rospy.Duration(1.0))
            self.listener_armEnd.waitForTransform("/Link8", "/camera_rgb_optical_frame", new, rospy.Duration(1.0))
            obj_point.point.x = objsTopology.objsPoints[i].obj_point.x
            obj_point.point.y = objsTopology.objsPoints[i].obj_point.y
            obj_point.point.z = objsTopology.objsPoints[i].obj_point.z

            base_obj_point = self.listener_base.transformPoint("/Base", obj_point)
            # rospy.loginfo(base_obj_point)

            _pos = position()
            # m转换到mm
            _pos.x = base_obj_point.point.x * 1000
            _pos.y = base_obj_point.point.y * 1000
            _pos.z = base_obj_point.point.z * 1000

            if self.swift_pro_ik([_pos.x,
                                  _pos.y,
                                  _pos.z]):

                reachability = "reachable"
                # rospy.loginfo("可到达")

                reachability_objs_arr.append(_pos)

            else:
                reachability = "unreachable"
                # rospy.loginfo("不可到达")

            # 发布目标坐标变换
            armEnd_obj_point = self.listener_armEnd.transformPoint("/Link8", obj_point)
            br = tf.TransformBroadcaster()
            br.sendTransform((armEnd_obj_point.point.x,
                              armEnd_obj_point.point.y,
                              armEnd_obj_point.point.z),
                             tf.transformations.quaternion_from_euler(0, 0, 0),
                             rospy.Time.now(),
                             "obj_{}_{}".format(i, reachability),
                             "/Link8")

        # 计算与基座最近的目标位置  将最经位置的目标点发送给机械臂
        # rospy.loginfo(self.arrived)
        if self.arrived:
            if reachability_objs_arr:
                min_distance_indx = 0
                min_distance = np.inf
                for i in range(len(reachability_objs_arr)):
                    x = reachability_objs_arr[i].x
                    y = reachability_objs_arr[i].y
                    z = reachability_objs_arr[i].z

                    dis = np.sqrt(np.square(x)+np.square(y)+np.square(z))

                    if dis < min_distance:
                        min_distance =dis
                        min_distance_indx = i

                _pos = reachability_objs_arr[min_distance_indx]

                # rospy.loginfo(_pos)
                self._pub_position.publish(_pos)
                self.arrived = False

    # 机械臂逆运动学求解， 确定位置是否可到达
    # 参数  ：
    #   position ：目标位置 [x,y,x]，单位mm
    # 返回值：true 可到达
    #        false 不可到达
    def swift_pro_ik(self, position):    # 单位mm

        x = position[0]
        y = position[1]
        z = position[2]
        xIn, zIn, phi, rightAll, sqrtZX = [0.0, 0.0, 0.0, 0.0, 0.0]
        angleRot, angleLeft, angleRight = [0.0, 0.0, 0.0]

        z += 74.55
        zIn = (z - MATH_L1) / MATH_LOWER_ARM

        if x < 0.1:
            x = 0.1

        # calculate value of theta1: the rotation  angle
        if y == 0:
            angleRot = 90
        elif y < 0:
            angleRot = -np.arctan(x / y) * MATH_TRANS

        elif y > 0:
            angleRot = 180 - np.arctan(x / y) * MATH_TRANS

        xIn = (x / np.sin(angleRot / MATH_TRANS) - MATH_L2 - 56.55) / MATH_LOWER_ARM
        phi = np.arctan(zIn / xIn) * MATH_TRANS
        sqrtZX = np.sqrt(zIn * zIn + xIn * xIn)
        rightAll = (sqrtZX * sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER - 1) \
                   / (2 * MATH_UPPER_LOWER * sqrtZX)
        angleRight = np.arccos(rightAll) * MATH_TRANS

        # calculate value of theta2 and theta3
        rightAll = (sqrtZX * sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER) / (2 * sqrtZX)
        angleLeft = np.arccos(rightAll) * MATH_TRANS
        angleLeft = angleLeft + phi
        angleRight = angleRight - phi

        if np.isnan(angleRot) or np.isnan(angleLeft) or np.isnan(angleRight):
            return False
        #
        # angle[0] = angleRot
        # angle[1] = angleLeft
        # angle[2] = angleRight
        return True

    def shutdown(self):
        rospy.loginfo("Stopping the  objsPoins_to_armEnd")
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('objsPoins_to_armEnd')


    try:
        objsPoins_to_armEnd()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("objsPoins_to_armEnd has stoped.")
