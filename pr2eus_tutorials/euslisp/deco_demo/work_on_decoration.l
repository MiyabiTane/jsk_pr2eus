#!/usr/bin/env roseus

;;set pub sub topic
(ros::load-ros-package "speech_recognition_msgs")
(ros::load-ros-package "geometry_msgs")
(setq *speech-sub-topic* "/speech_to_text")
(setq *l-force-sub-topic* "/left_endeffector/wrench")
(setq *r-force-sub-topic* "/right_endeffector/wrench")
(ros::rate 10)


(defun before-catch-pose ()
  (setq *target-rcoords* (make-cascoords
			  :pos (float-vector 600 -300 900)
			  :rpy (float-vector 1.486e-06 1.384e-06 -1.571)))
  (send *pr2* :rarm :inverse-kinematics *target-rcoords* :rotation-axis t)
  (setq *target-lcoords* (make-cascoords
			  :pos (float-vector 600 300 900)
			  :rpy (float-vector 1.486e-06 1.384e-06 1.571)))
  (send *pr2* :larm :inverse-kinematics *target-lcoords* :rotation-axis t)
  (send-robot 5000)
  )


(defun speech-cb (msg)
  (setq text (send msg :transcript))
  (format t "got words: ~A~%" text)
  (setq flag 1)
  )


(defun vforce-cb (msg)
  ;; (setq force_data (send (send msg :wrench) :force))
  ;; (setq force_vector (float-vector (send force_data :x) (send force_data :y) (send force_data :z)))
  ;; (setq force_norm (norm force_vector))
  (setq torque_data (send (send msg :wrench) :torque))
  (setq torque_vector (float-vector (send torque_data :x) (send torque_data :y) (send torque_data :z)))
  (setq torque_norm (norm torque_vector))
  )


(defun catch-balloon ()
  ;; ToDo[ここは対話を使うのではなく、自分で認識して取る] --------
  (send *ri* :speak-jp "風船を渡して下さい")
  (format t "speak: 風船を渡して下さい~%")
  (setq flag 0)
  (ros::subscribe *speech-sub-topic* speech_recognition_msgs::SpeechRecognitionCandidates #'speech-cb)
  (while (= flag 0)
    (ros::spin)
    (ros::sleep))
  (ros::unsubscribe *speech-sub-topic*)
  (send *ri* :speak-jp "ありがとうございます")
  ;; ---------------------------------------------
  ;; start to catch
  (setq l_prev_norm 0)
  (setq r_prev_norm 0)
  (setq l_norm_diff 0)
  (setq r_norm_diff 0)
  (ros::subscribe *l-force-sub-topic* geometry_msgs::WrenchStamped #'l-force-cb)
  (ros::subscribe *r-force-sub-topic* geometry_msgs::WrenchStamped #'r-force-cb)
  (while (and (< l_norm_diff 2) (< r_norm_diff 2))
    (setq l_force_msg (one-shot-subscribe *l-force-sub-topic*
					 geometry_msgs::WrenchStamped
					 :after-stamp (ros::time-now)))
    (vforce-cb l_force_msg)
    (setq l_torque_norm torque_norm)
    (setq r_force_msg (one-shot-subscribe *r-force-sub-topic*
					  geometry_msgs::WrenchStamped
					  :after-stamp (ros::time-now))) 
    (vforce-cb r_force_msg)
    (setq r_torque_norm torque_norm)
    (when (and (= l_prev_norm 0) (= r_prev_norm 0))
      (setq l_prev_norm l_torque_norm)
      (setq r_prev_norm r_torque_norm))
    (setq l_norm_diff (abs (- l_torque_norm l_prev_norm)))
    (setq r_norm_diff (abs (- r_torque_norm r_prev_norm)))
    (format t "diff left: ~A, right: ~A~%" l_norm_diff r_norm_diff)
    ;; bring arm closer
    (send *pr2* :rarm :move-end-pos #f(0 10 0) :world)
    (send *pr2* :larm :move-end-pos #f(0 -10 0) :world))
  (ros::unsubscribe *l-force-sub-topic*)
  (ros::unsubscribe *r-force-sub-topic*)
  )


;; --------- make pose ----------
(defun make-before-catch-pose ()
  ;; rarm
  (send *pr2* :rarm :inverse-kinematics (make-coords :pos (float-vector 700 -200 900)))
  (setq *r-end-coords* (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq *r-end-coords* (make-cascoords :pos (v+ *r-end-coords* #f(100 0 0))))
  (send (send *pr2* :rarm :end-coords) :assoc *r-end-coords*)
  (setq *target-coords* (send *r-end-coords* :copy-worldcoords))
  (send *target-coords* :rotate (* (/ pi 2) -1) :x :world)
  (objects (list *pr2* *r-end-coords* *target-coords*))
  (send *pr2* :rarm :inverse-kinematics *target-coords* :move-target *r-end-coords* :debug-view t)
  (format t "rarm-pose: ~A~%" (send *pr2* :rarm :end-coords))
  ;; init poseを元に左右の対称性をみた
  )
