#!/usr/bin/env python
# -*- coding: utf-8 -*-
# /speech_to_text -> /request 
import rospy

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from topic_tools.srv import MuxSelect

class RelaySoundTopic(object):
    def __init__(self):
        self.pub = rospy.Publisher("~output", String, queue_size=1)
        self.c_flag = False
        self.m_flag = True

        rospy.wait_for_service("/speech_to_text_mux/select")
        set_speech_mode = rospy.ServiceProxy("/speech_to_text_mux/select", MuxSelect)
        self.prev_speech_mode = set_speech_mode("speech_to_text_other").prev_topic

        rospy.Subscriber("~input", SpeechRecognitionCandidates, self.relay_cb)
        rospy.Subscriber("~input/chaplus_flag", Bool, self.flag_cb)
        rospy.Subscriber("~input/mux_flag", Bool, self.mode_cb)
        # Ctrl+Cなどでプログラムを止めた時にmuxを元に戻すため、常にsubできるトピックを指定
        # rospy.Subscriber("~input/interrupt", Image, self.interrupt_cb)

    def flag_cb(self, msg):
        self.c_flag = msg.data

    def mode_cb(self, msg):
        # PR2はデフォルトでdialogfowが走る
        # こんにちは、ねえねえなどのhotwordを一度言うと勝手に喋り出してしまうので
        # chaplusの応答を使う時は/speech_to_textのdefaultを"speech_to_text_other"にしておく
        if msg.data:
            rospy.wait_for_service("/speech_to_text_mux/select")
            set_speech_mode = rospy.ServiceProxy("/speech_to_text_mux/select", MuxSelect)
            set_speech_mode("speech_to_text_other")

        if not msg.data:
            rospy.wait_for_service("/speech_to_text_mux/select")
            set_speech_mode = rospy.ServiceProxy("/speech_to_text_mux/select", MuxSelect)
            set_speech_mode(self.prev_speech_mode)

    def intterupt_cb(self, msg):
        try:
            pass
        except rospy.ROSInterruptException:
            rospy.wait_for_service("/speech_to_text_mux/select")
            set_speech_mode = rospy.ServiceProxy("/speech_to_text_mux/select", MuxSelect)
            set_speech_mode(self.prev_speech_mode)

    def relay_cb(self, msg):
        if self.c_flag:
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

