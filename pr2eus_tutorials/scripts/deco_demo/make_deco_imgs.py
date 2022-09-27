#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

import cv2
import glob
import roslib.packages
import numpy as np
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel

DIR_PATH = roslib.packages.get_pkg_dir('pr2eus_tutorials') + "/scripts/deco_demo/"


def print_decoration_info(decos_img, decos_pos, decos_dims, decos_req_uv, bimg_lt_pos, bimg_rb_pos, planes_norm_vec):
    # deco_bboxes's frame_id -> "head_mount_kinect_rgb_optical_frame"
    # other's frame_id -> "base_footprint"
    bridge = CvBridge()
    decos_jpg = bridge.imgmsg_to_cv2(decos_img, desired_encoding="bgr8")
    cv2.imwrite(DIR_PATH + "/images/decos_img.jpg", decos_jpg)
    print("Back Img info->")
    print("lt: ", bimg_lt_pos.x, bimg_lt_pos.y, bimg_lt_pos.z)
    print("rb: ", bimg_rb_pos.x, bimg_rb_pos.y, bimg_rb_pos.z)
    print("planes_norm_vec: ", np.array(planes_norm_vec))
    print("= decos_pos")
    print(np.array(decos_pos))
    print("= decos_dims")
    print(np.array(decos_dims))
    print("= decos_req_uv")
    print(np.array(decos_req_uv))
