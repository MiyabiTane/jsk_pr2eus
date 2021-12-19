#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from topic_tools.srv import MuxSelect

rospy.wait_for_service("/speech_to_text_mux/select")
set_speech_mode = rospy.ServiceProxy("/speech_to_text_mux/select", MuxSelect)
response = set_speech_mode("speech_to_text_other")
print(response.prev_topic)