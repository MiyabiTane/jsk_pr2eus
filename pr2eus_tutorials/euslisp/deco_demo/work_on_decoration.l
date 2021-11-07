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
  (send *pr2* :rarm :move-end-pos #f(0 150 0) :world)
  (send *pr2* :larm :move-end-pos #f(0 -150 0) :world)
  (send-robot 5000)
  )


(defun speech-cb (msg)
  (setq text (send msg :transcript))
  (format t "got words: ~A~%" text)
  (setq flag 1)
  )


(defun vforce-cb (msg)
  (setq force_data (send (send msg :wrench) :force))
  (setq force_vector (float-vector (send force_data :x) (send force_data :y) (send force_data :z)))
  (setq force_norm (norm force_vector))
  )


(defun catch-balloon ()
  ;; ToDo[ここは対話を使うのではなく、自分で認識して取る] --------
  (send *ri* :speak-jp "風船を渡して下さい~%")
  (ros::duration-sleep 5)
  (send *ri* :speak-jp "ありがとうございます~%")
  ;; ---------------------------------------------
  ;; start to catch
  (setq l_prev_norm -1)
  (setq r_prev_norm -1)
  (setq l_norm_diff 0)
  (setq r_norm_diff 0)
  (while (and (< l_norm_diff 3) (< r_norm_diff 3))
    ;; bring arm closer
    (send *pr2* :rarm :move-end-pos #f(0 10 0) :world)
    (send *pr2* :larm :move-end-pos #f(0 -10 0) :world)
    (send-robot 4000)
    ;; subscribe force info
    (setq l_force_msg (one-shot-subscribe *l-force-sub-topic*
					 geometry_msgs::WrenchStamped
					 :after-stamp (ros::time-now)))
    (vforce-cb l_force_msg)
    (setq l_force_norm force_norm)
    (setq r_force_msg (one-shot-subscribe *r-force-sub-topic*
					  geometry_msgs::WrenchStamped
					  :after-stamp (ros::time-now))) 
    (vforce-cb r_force_msg)
    (setq r_force_norm force_norm)
    (when (and (= l_prev_norm -1) (= r_prev_norm -1))
      (setq l_prev_norm l_force_norm)
      (setq r_prev_norm r_force_norm))
    (setq l_norm_diff (abs (- l_force_norm l_prev_norm)))
    (setq r_norm_diff (abs (- r_force_norm r_prev_norm)))
    (formar r "diff left: ~A, right: ~A~%" l_norm_diff r_norm_diff))
  (ros::unsubscribe *l-force-sub-topic*)
  (ros::unsubscribe *r-force-sub-topic*)
  )


(defun work-on-decoration ()
  ;; 壁との干渉計算
  (setq pre_x -1) ;; if not execute funcs in think_decoration.l
  (convert-2D->3D 320 240)  ;;func in think_decoration.l
  (setq wall_x cur_x)  ;; 壁との距離
  (format t "wall_x: ~A~%" wall_x)
  (setq *wall* (make-cube 100 4000 4000))
  (send *wall* :locate (float-vector cur_x 0 2000))
  ;; (setq *co* (instance collision-object-publisher :init))
  ;; (send *co* :add-object *wall* :frame-id "map" :relative-pose (make-coords :pos #f(-2200.0 6300.0 0.0)))
  ;; (objects (list *pr2* *wall*))
  
  ;; (send *co* :wipe-all)
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
