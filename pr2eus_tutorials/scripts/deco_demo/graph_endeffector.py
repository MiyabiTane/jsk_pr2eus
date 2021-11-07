#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped

import numpy as np
import matplotlib.pyplot as plt
import time

class WrenchGraph:

    def __init__(self, mode, save_name):
        # mode: force or torque
        self.x_lst = []
        self.y_lst = []
        self.z_lst = []
        self.norm_lst = []
        self.time_lst = []
        self.mode = mode
        self.save_name = save_name
        print("init")

    def shutdown(self):
        print("shutdown rospy")

    def cb(self, msg):
        if self.mode == "force":
            Force = msg.wrench.force
            f_norm = np.linalg.norm(np.array([Force.x, Force.y, Force.z]))
            self.time_lst.append(time.time())
            self.x_lst.append(Force.x)
            self.y_lst.append(Force.y)
            self.z_lst.append(Force.z)
            self.norm_lst.append(f_norm)
        elif self.mode == "torque":
            Torque = msg.wrench.torque
            t_norm = np.linalg.norm(np.array([Torque.x, Torque.y, Torque.z]))
            self.time_lst.append(time.time())
            self.x_lst.append(Torque.x)
            self.y_lst.append(Torque.y)
            self.z_lst.append(Torque.z)
            self.norm_lst.append(t_norm)
        # print(len(self.time_lst))
        if len(self.time_lst) > 2:
            if self.time_lst[-1] - self.time_lst[0] > 30:
                rospy.signal_shutdown(self.shutdown)
                self.make_graph()
        
    def get_info(self, mode):
        # mode: left or right
        rospy.init_node("wrench_cb")
        if mode == "left":
            rospy.Subscriber("/left_endeffector/wrench", WrenchStamped, self.cb)
        elif mode == "right":
            rospy.Subscriber("/right_endeffector/wrench", WrenchStamped, self.cb)
        rospy.spin()

    def make_graph(self):
        print("make graph")
        self.time_lst = np.array(self.time_lst) - np.array([self.time_lst[0]] * len(self.time_lst))
        plt.plot(self.time_lst, self.x_lst, label = self.mode + "_x")
        plt.plot(self.time_lst, self.y_lst, label = self.mode + "_y")
        plt.plot(self.time_lst, self.z_lst, label = self.mode + "_z")
        plt.plot(self.time_lst, self.norm_lst, label = self.mode + "_norm")
        plt.xlabel("time[s]")
        plt.ylabel(self.mode)
        plt.legend()
        plt.savefig(self.save_name)


wrench_graph = WrenchGraph("torque", "right_torque.png")
wrench_graph.get_info("right")

