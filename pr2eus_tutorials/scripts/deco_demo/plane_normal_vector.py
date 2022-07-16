#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Point, Pose, Quaternion, PoseArray

import numpy as np

class PolygonToVector(object):
    def __init__(self):
        self.point_msg = Point()
        self.quaternion_msg = Quaternion()
        self.pose_msg = Pose()
        self.pose_array_msg = PoseArray()

        self.pub = rospy.Publisher("~output", PoseArray, queue_size=1)
        rospy.Subscriber("~input", PolygonArray, self.plane_vec_cb)
        
    def polygon_to_vec(self, points_msg):
        # 同一平面上の点なので任意の3点で良い
        p1_msg = points_msg[0]
        p2_msg = points_msg[int(len(points_msg)/2)]
        p3_msg = points_msg[len(points_msg)-1]
        p1 = np.array([p1_msg.x, p1_msg.y, p1_msg.z])
        p2 = np.array([p2_msg.x, p2_msg.y, p2_msg.z])
        p3 = np.array([p3_msg.x, p3_msg.y, p3_msg.z])
        vec_1 = p1 - p2
        vec_2 = p3 - p2
        norm_vec = np.cross(vec_1, vec_2)
        vec_size = np.linalg.norm(norm_vec)
        return norm_vec / vec_size, p2
    
    def plane_vec_cb(self, msg):
        self.pose_array_msg.poses = []
        for polygon in msg.polygons:
            norm_vec, center_pos = self.polygon_to_vec(polygon.polygon.points)
            self.point_msg.x = center_pos[0]
            self.point_msg.y = center_pos[1]
            self.point_msg.z = center_pos[2]
            self.quaternion_msg.x = norm_vec[0]
            self.quaternion_msg.y = norm_vec[1]
            self.quaternion_msg.z = norm_vec[2]
            self.pose_msg.position = self.point_msg
            self.pose_msg.orientation = self.quaternion_msg
            self.pose_array_msg.poses.append(self.pose_msg)
        self.pub.publish(self.pose_array_msg)

if __name__ == '__main__':
    rospy.init_node("polygon_to_vector")
    point_to_vector = PolygonToVector()
    rospy.spin()
