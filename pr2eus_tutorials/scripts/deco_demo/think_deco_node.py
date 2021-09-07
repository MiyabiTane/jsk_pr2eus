#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import glob
import roslib.packages
import cv2

from think_deco import ThinkDecoration, think_with_trained_pix2pix

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pr2eus_tutorials.msg import DecoImages

from cv_bridge import CvBridge


class ThinkDecorationNode:
    def __init__(self):
        self.dir_path = roslib.packages.get_pkg_dir('pr2eus_tutorials') + "/scripts/deco_demo"
        self.deco_imgs = []
        self.deco_masks = []
        self.secs = -1
        self.input_img = None
        self.output_img = None
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("~output", PoseArray, queue_size=1)

        rospy.Subscriber("~input", DecoImages, self.decos_cb)


    def decos_cb(self, msg):
        secs = msg.header.stamp.secs
        if abs(secs - self.secs) > 1:
            print("=== THINK DECORATION START===")
            self.secs = secs
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
            cv2.imwrite(self.dir_path + "/images/input.png", self.input_img)
            cv2.imwrite(self.dir_path + "/images/output.png", self.output_img)
            # think placement of decorations
            think_deco = ThinkDecoration(self.deco_imgs, self.deco_masks, self.input_img, self.output_img)
            think_deco.GA_calc()


if __name__ == '__main__':
    rospy.init_node("think_decoration")
    think_deco_node = ThinkDecorationNode()    
    rospy.spin()