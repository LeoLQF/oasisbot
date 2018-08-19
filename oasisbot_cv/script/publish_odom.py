#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Float32, Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, TransformStamped
import numpy as np
import tf

class publish_odom():

    def __init__(self):
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        input_objsPoints = rospy.get_param("~input_objsPoints", "/objsPoints")
        self.target_spacing_restriction = rospy.get_param("~target_spacing_restriction", 0.04)

        # 订阅目标点
        self._sub = rospy.Subscriber(input_objsPoints,
                                     PolygonStamped,
                                     self.callback_objsPints,
                                     queue_size=1)

        self._pub_odom = rospy.Publisher('odom',
                                         Odometry,
                                         queue_size=1)

        self.objsPoints_old = PolygonStamped()

        self.odom = Odometry()
        self.odom_trans = TransformStamped()
        self.odom_br = tf.TransformBroadcaster()

        self.odom_x, self.odom_y,  self.odom_z = [0, 0, 0]


    def callback_objsPints(self, objsPints_msg):

        objsPints = objsPints_msg

        diff = []
        if self.objsPoints_old.polygon.points:  # 非空

            for i in range(len(objsPints.polygon.points)):

                objPint = objsPints.polygon.points[i]

                for j in range(len(self.objsPoints_old.polygon.points)):

                    ibjPint_old = self.objsPoints_old.polygon.points[j]

                    d, v = self.p2p_distence_voter(ibjPint_old, objPint)

                    if d < self.target_spacing_restriction:
                        diff.append(v)
        else:  # 空
            pass

        if diff:
            rospy.loginfo("%d个目标被找到", len(diff))
            odom_diff = np.mean(diff, axis=0)
            # rospy.loginfo(odom_diff)

            self.odom_x += odom_diff[0]
            self.odom_y += odom_diff[1]
            self.odom_z += odom_diff[2]

            self.odom_trans.header.stamp = rospy.Time().now()
            self.odom_trans.header.frame_id = "odom"
            self.odom_trans.child_frame_id = "camera_link"

            self.odom_trans.transform.translation.x = self.odom_x
            self.odom_trans.transform.translation.y = self.odom_z
            self.odom_trans.transform.translation.z = self.odom_y
            self.odom_trans.transform.rotation = tf.transformations.quaternion_from_euler(0, 0, 0)

            self.odom_br.sendTransform((self.odom_x, self.odom_z, self.odom_y),
                                       self.odom_trans.transform.rotation,
                                       self.odom_trans.header.stamp,
                                       self.odom_trans.header.frame_id,
                                       self.odom_trans.child_frame_id)
            rospy.loginfo( self.odom_trans.transform.translation)

        else:
            rospy.loginfo("没有目标被找到")

        self.objsPoints_old = objsPints_msg


        # # 发布目标
        # br = tf.TransformBroadcaster()
        # for i in range(len(self.objs_topology.objsPoints)):
        #     x = self.objs_topology.objsPoints[i].obj_point.x
        #     y = self.objs_topology.objsPoints[i].obj_point.y
        #     z = self.objs_topology.objsPoints[i].obj_point.z
        #
        #     br.sendTransform((x, y, z),
        #                      tf.transformations.quaternion_from_euler(0, math.pi, 0),
        #                      rospy.Time.now(),
        #                      "obj_{}".format(self.objs_topology.objsPoints[i].obj_ID),
        #                      "camera_rgb_optical_frame")

    def p2p_distence_voter(self, point1, point2):
        v_x = point1.x - point2.x
        v_y = point1.y - point2.y
        v_z = point1.z - point2.z
        d = np.sqrt(np.square(v_x) + np.square(v_y) + np.square(v_z))
        return d, [v_x, v_y, v_z]

    def shutdown(self):
        rospy.loginfo("Stopping the  odometry_publisher")
        rospy.sleep(1)

if __name__ == '__main__':

    from utils import objsPoints_mamager_utils

    rospy.init_node('odometry_publisher')


    try:
        publish_odom()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("odometry_publisher has stoped.")
