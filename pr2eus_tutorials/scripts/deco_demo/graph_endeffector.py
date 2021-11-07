#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import WrenchStamped

import numpy as np
import matplotlib.pyplot as plt
import time

class WrenchGraph:

    def __init__(self, mode, lr_mode, measure_mode, save_name):
        # mode: force or torque
        # lr_mode: left or right
        # measure_momde: original or diff
        self.x_lst = []
        self.y_lst = []
        self.z_lst = []
        self.norm_lst = []
        self.time_lst = []
        self.mode = mode
        self.lr_mode = lr_mode
        self.m_mode = measure_mode
        self.save_name = save_name
        self.sub = None
        print("init")

    def shutdown(self):
        print("shutdown rospy")

    def cb(self, msg):
        flg = True
        cur_time = time.time()
        if self.m_mode == "diff" and len(self.time_lst) > 0:
            if cur_time - self.time_lst[-1] < 1:
                flg = False
        if self.mode == "force":
            Force = msg.wrench.force
            f_norm = np.linalg.norm(np.array([Force.x, Force.y, Force.z]))
            if flg:
                self.time_lst.append(cur_time)
                self.x_lst.append(Force.x)
                self.y_lst.append(Force.y)
                self.z_lst.append(Force.z)
                self.norm_lst.append(f_norm)
        elif self.mode == "torque":
            Torque = msg.wrench.torque
            t_norm = np.linalg.norm(np.array([Torque.x, Torque.y, Torque.z]))
            if flg:
                self.time_lst.append(cur_time)
                self.x_lst.append(Torque.x)
                self.y_lst.append(Torque.y)
                self.z_lst.append(Torque.z)
                self.norm_lst.append(t_norm)
        # print(len(self.time_lst))
        if len(self.time_lst) > 2:
            if self.time_lst[-1] - self.time_lst[0] > 30:
                self.sub.unregister()
                self.make_graph()
                rospy.signal_shutdown(self.shutdown)
                
    def get_info(self):
        # mode: left or right
        rospy.init_node("wrench_cb")
        if self.lr_mode == "left":
            self.sub = rospy.Subscriber("/left_endeffector/wrench", WrenchStamped, self.cb)
        elif self.lr_mode == "right":
            self.sub = rospy.Subscriber("/right_endeffector/wrench", WrenchStamped, self.cb)
        rospy.spin()

    def make_graph(self):
        print("make graph")
        self.time_lst = np.array(self.time_lst) - np.array([self.time_lst[0]] * len(self.time_lst))
        if self.m_mode == "original":
            plt.plot(self.time_lst, self.x_lst, label = self.mode + "_x")
            plt.plot(self.time_lst, self.y_lst, label = self.mode + "_y")
            plt.plot(self.time_lst, self.z_lst, label = self.mode + "_z")
            plt.plot(self.time_lst, self.norm_lst, label = self.mode + "_norm")
        elif self.m_mode == "diff":
            self.norm_lst = [abs(self.norm_lst[i] - self.norm_lst[i-1]) for i in range(len(self.norm_lst)) if i > 0]
            self.norm_lst = [0] + self.norm_lst
            plt.plot(self.time_lst, self.norm_lst, label = self.mode + "_norm_diff")
        plt.xlabel("time[s]")
        plt.ylabel(self.mode)
        plt.legend()
        plt.savefig(self.save_name)


wrench_graph = WrenchGraph("force", "right", "diff", "right_force_diff.png")
wrench_graph.get_info()

