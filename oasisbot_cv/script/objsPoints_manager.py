#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Float32, Time
from geometry_msgs.msg import PolygonStamped, Vector3, Point32
from object_msgs.msg import ObjPoint_info, Objs_topology
import numpy as np
import math
import tf

# [515.4758986346594, 0,                 315.8145803008886,
#  0,                 517.6997276723781, 250.6995339983923,
#  0,                 0,                 1]
# [fx, s,  cx]
# [0,  fy, cy]
# [0,  0,  1]

class objsPoints_manager():

    def __init__(self):
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        input_objsPoints = rospy.get_param("~input_objsPoints", "/objsPoints")
        self.target_spacing_restriction = rospy.get_param("~target_spacing_restriction", 0.06)

        # 订阅目标点
        self._sub = rospy.Subscriber(input_objsPoints,
                                     PolygonStamped,
                                     self.callback_objsPints,
                                     queue_size=1)

        # 维护一个当前目标点，并追踪目标
        self._pub_objs_topology = rospy.Publisher('objsTopology',
                                                  Objs_topology,
                                                  queue_size=1)

        self.objs_topology = Objs_topology()       # 保存当前目标拓扑表示
        self.objs_topology.target_spacing_restriction.data = self.target_spacing_restriction  # 目标间距限制4cm

        self.height = 480                      # rows   y
        self.width = 640                       # columns  x
        self.camera_fx = 515.4758986346594
        self.camera_fy = 517.6997276723781
        self.camera_cx = 315.8145803008886
        self.camera_cy = 250.6995339983923
        self.camera_factor = 1

        self.diff_FLAG = False

    def p2p_distence_voter(self, new_objPoint, old_objPoint):
        v_x = new_objPoint.x - old_objPoint.x
        v_y = new_objPoint.y - old_objPoint.y
        v_z = new_objPoint.z - old_objPoint.z
        d = np.sqrt(np.square(v_x) + np.square(v_y) + np.square(v_z))
        return d, [v_x, v_y, v_z]

    def callback_objsPints(self, objsPints_msg):
        objsPints = objsPints_msg
        self.objs_topology.header = objsPints.header

        # rospy.loginfo("接收目标点数%d", len(objsPints.polygon.points))

        # 所有的up_to_date目标点up_to_date置为False，表示待更新
        for i in range(len(self.objs_topology.objsPoints)):
            objsPoints_i_old = self.objs_topology.objsPoints[i]
            objsPoints_i_old.up_to_date.data = False

        # 更新被探测到的目标点
        diff = []
        for i in range(len(objsPints.polygon.points)):
            objPint = objsPints.polygon.points[i]
            # rospy.loginfo(objPint)

            distence_arr = []
            v_arr = []
            NEW_OBJ_FLAG = True
            if self.objs_topology.objsPoints:  # 非空：拓扑结构中有目标
                for j in range(len(self.objs_topology.objsPoints)):
                    objsPoints_j_old = self.objs_topology.objsPoints[j]

                    d, v = self.p2p_distence_voter(objPint, objsPoints_j_old.obj_point)  # 计算两点距离
                    distence_arr.append(d)
                    v_arr.append(v)

                # 计算最近距离目标
                min_at = np.where(distence_arr == np.min(distence_arr))
                min_val = np.min(distence_arr)
                if min_val <= self.target_spacing_restriction:  # 小于目标距离限制，认为是已有目标，并更新已有目标目标点，不更新obj_ID
                    self.objs_topology.objsPoints[[0][0]].obj_point.x = objPint.x
                    self.objs_topology.objsPoints[[0][0]].obj_point.y = objPint.y
                    self.objs_topology.objsPoints[[0][0]].obj_point.z = objPint.z
                    self.objs_topology.objsPoints[[0][0]].up_to_date.data = True  # 数据已更新

                    diff.append(v_arr[min_at[0][0]])  # 此处暂时只考虑平移
                    self.diff_FLAG = True
                    NEW_OBJ_FLAG = False
                    # rospy.loginfo("匹配已有目标.")

            if NEW_OBJ_FLAG:  # 新目标，加入目标拓扑
                objoint_info = ObjPoint_info()
                objoint_info.obj_ID.data = rospy.Time().now()
                objoint_info.obj_point = objPint
                objoint_info.up_to_date.data = True
                self.objs_topology.objsPoints.append(objoint_info)
                # rospy.loginfo("加入新目标.")

        # +++++++***********重点改进-diff*************++++++++++++
        # 推测并更新未被探测到的目标点
        if self.diff_FLAG:
            if diff:
                # rospy.loginfo("%d个目标被找到.", len(diff))
                diff = np.mean(diff, axis=0)

                for i in range(len(self.objs_topology.objsPoints)):

                    if self.objs_topology.objsPoints[i].up_to_date.data == False:
                        self.objs_topology.objsPoints[i].obj_point.x += diff[0]
                        self.objs_topology.objsPoints[i].obj_point.y += diff[1]
                        self.objs_topology.objsPoints[i].obj_point.z += diff[2]
                        self.objs_topology.objsPoints[i].up_to_date.data = True  # 数据已更新
            else:
                # rospy.loginfo("没有已存在的目标被找到.清空目标缓存.")
                # pop all
                self.objs_topology.objsPoints = []
                self.diff_FLAG = False

        # 超出探测范围的点目标点删除
        if self.objs_topology.objsPoints:
            for i in range(len(self.objs_topology.objsPoints)):

                x = self.objs_topology.objsPoints[i].obj_point.x
                y = self.objs_topology.objsPoints[i].obj_point.y
                z = self.objs_topology.objsPoints[i].obj_point.z

                u = x * self.camera_fx / z + self.camera_cx
                v = y * self.camera_fy / z + self.camera_cy

                # self.height = 480  # rows    y  v
                # self.width = 640  # columns  x  u
                if u < 0 or u >= self.width or v < 0 or v >= self.height:
                    self.objs_topology.objsPoints.pop(i)
                    break

        # rospy.loginfo("目标总数：%d.", len(self.objs_topology.objsPoints))
        if self.objs_topology.objsPoints:
            self._pub_objs_topology.publish(self.objs_topology)
            # rospy.loginfo(self.objs_topology)

        # 发布目标坐标变换
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


    def shutdown(self):
        rospy.loginfo("Stopping the  objsPoints_manager")
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('objsPoints_manager')


    try:
        objsPoints_manager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("objsPoints_manager has stoped.")
