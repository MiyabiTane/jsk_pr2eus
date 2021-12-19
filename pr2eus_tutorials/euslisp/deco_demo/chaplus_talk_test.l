#!/usr/bin/env roseus

;; MuxSelectに:topicは無いと言って怒られる。現状このコードは動かない。

;; init
(ros::roseus "pr2_deco_demo_node")

;; load pkg
(ros::load-ros-manifest "topic_tools")

(defun call-mux-service ()
  (ros::wait-for-service "/speech_to_text_mux/select")
  (format t "CALLED SERVICE~%")
  (setq req_msg (instance topic_tools::MuxSelect :init))
  (format t "req_msg: ~A~%" req_msg)
  (send req_msg :topic "speech_to_text_other")
  (format t "== ~A~%" (stringp (send req_msg :topic)))
  ;; (setq req_msg "topic: '/speech_to_text_julius'")
  (setq prev_msg (ros::service-call "/speech_to_text_mux/select" req_msg))
  (format t "prev_msg is: ~A~%" (send prev_msg :prev_topic))
  )


(call-mux-service)
