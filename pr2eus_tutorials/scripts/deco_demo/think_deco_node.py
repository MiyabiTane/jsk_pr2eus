#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import glob
import roslib.packages
import cv2

from think_deco import ThinkDecoration, think_with_trained_pix2pix, remove_dup_deco

from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pr2eus_tutorials.msg import DecoImages

from cv_bridge import CvBridge


class ThinkDecorationNode:
    def __init__(self):
        self.dir_path = roslib.packages.get_pkg_dir('pr2eus_tutorials') + "/scripts/deco_demo"
        self.deco_imgs = []
        self.deco_masks = []
        self.seq = -1
        self.input_img = None
        self.output_img = None
        self.bridge = CvBridge()
        self.output_arr = [(0, 0, 0, 0)]
        self.flag = 0

        self.pub = rospy.Publisher("~output", PoseArray, queue_size=1)
        self.pub_msg = PoseArray()

        rospy.Subscriber("~input", DecoImages, self.decos_cb)


    def decos_cb(self, msg):
        seq = msg.header.seq
        if seq - self.seq > 60:
            # 一定時間以上経ったら計算結果を無効とする
            self.flag = 0
        if seq != self.seq:
            print("=== THINK DECORATION START===")
            self.flag = 1
            self.seq = seq
            self.input_img = self.bridge.imgmsg_to_cv2(msg.back_img, desired_encoding="bgr8")
            """ ToDo
            for deco_msg in msg.deco_imgs:
                deco_img = self.bridge.imgmsg_to_cv2(deco_msg, desired_encoding="bgr8")
                self.deco_imgs.append(deco_img)
            for mask_msg in msg.mask_imgs:
                mask_img = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
                self.deco_masks.append(mask_img)
            """
            #### temporary ####
            files = glob.glob(self.dir_path + "/images/temp/input*.jpg")
            files = sorted(files, key=lambda x: int(x[-5]))
            for fi in files:
                deco_img = cv2.imread(fi)
                self.deco_imgs.append(deco_img)
            files = glob.glob(self.dir_path + "/images/temp/mask*.jpg")
            files = sorted(files, key=lambda x: int(x[-5]))
            for fi in files:
                mask_img = cv2.imread(fi, 0)
                self.deco_masks.append(mask_img)
            ####
            # make decoration img
            self.output_img = think_with_trained_pix2pix(self.input_img)
            # Visualize (for debug)
            # cv2.imwrite(self.dir_path + "/share/input.png", self.input_img)
            # cv2.imwrite(self.dir_path + "/share/output.png", self.output_img)
            # think placement of decorations
            self.deco_imgs, self.deco_masks, decorated_pos = remove_dup_deco(self.input_img, self.deco_imgs, self.deco_masks)
            think_deco = ThinkDecoration(self.deco_imgs, self.deco_masks, self.input_img, self.output_img, decorated_pos)
            self.output_arr = think_deco.GA_calc()

            # publish result
            # position.x -> center_x, position.y -> center_y, orientation.x -> width, orientation.y -> length
            # position.z -> flag
            self.pub_msg = PoseArray()
            for x, y, w, l in self.output_arr:
                pose_msg = Pose()
                point_msg = Point()
                quater_msg = Quaternion()
                point_msg.x = x + w / 2
                point_msg.y = y + l / 2
                point_msg.z = self.flag
                quater_msg.x = w
                quater_msg.y = l
                pose_msg.position = point_msg
                pose_msg.orientation = quater_msg
                self.pub_msg.poses.append(pose_msg)

        self.pub.publish(self.pub_msg)


if __name__ == '__main__':
    rospy.init_node("think_decoration")
    think_deco_node = ThinkDecorationNode()    
    rospy.spin()