#!/usr/bin/env python
# -*- coding: utf-8 -*-
# /speech_to_text -> /request 
import rospy

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String, Bool

class RelaySoundTopic(object):
    def __init__(self):
        self.pub = rospy.Publisher("~output", String, queue_size=1)
        self.flag = False

        rospy.Subscriber("~input", SpeechRecognitionCandidates, self.relay_cb)
        rospy.Subscriber("~input/flag", Bool, self.flag_cb)

    def flag_cb(self, msg):
        self.flag = msg.data

    def relay_cb(self, msg):
        if self.flag:
            texts = ""
            for text in msg.transcript:
                texts += text + " "
            texts = texts[:-1]
            pub_msg = String()
            pub_msg.data = texts
            self.pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node("relay_node")
    relaysoundtopic = RelaySoundTopic()
    rospy.spin()


