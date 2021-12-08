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
  (setq *target-hcoords* (scale 0.5 (v+ (send *target-rcoords* :pos) (send *target-lcoords* :pos))))
  (send *pr2* :head :look-at (v- *target-hcoords* #f(0 0 100)))
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
  )


(defun catch-balloon ()
  ;; ToDo[ここは対話を使うのではなく、自分で認識して取る] --------
  (send *ri* :speak-jp "風船を渡して下さい~%")
  (ros::duration-sleep 5)
  (send *ri* :speak-jp "ありがとうございます~%")
  ;; ---------------------------------------------
  ;; start to catch
  (setq l_prev_norm (float-vector 0 0 0))
  (setq r_prev_norm (float-vector 0 0 0))
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
    (setq l_force_vec force_vector)
    (setq r_force_msg (one-shot-subscribe *r-force-sub-topic*
					  geometry_msgs::WrenchStamped
					  :after-stamp (ros::time-now))) 
    (vforce-cb r_force_msg)
    (setq r_force_vec force_vector)
    (when (and (= (norm l_prev_vec) 0) (= (norm r_prev_vec) 0))
      (setq l_prev_norm l_force_norm)
      (setq r_prev_norm r_force_norm))
    (setq l_norm_diff (norm (v- l_force_vec l_prev_vec)))
    (setq r_norm_diff (norm (v- r_force_vec r_prev_vec)))
    (formar r "diff left: ~A, right: ~A~%" l_norm_diff r_norm_diff))
  (ros::unsubscribe *l-force-sub-topic*)
  (ros::unsubscribe *r-force-sub-topic*)
  )


;;(defun ik-same-pose (lpos rpos)
;; (setq *pr2_l_pose* (send (send *pr2* :larm :end-coords) :copy-worldcoords))
;; (setq diff (v- lpos (send *pr2_l_pose* :pos)))
;; (send *pr2_l_pose* :translate diff :world)
;; (send *pr2* :larm :inverse-kinematics *pr2_l_pose* :debug-view t)
;; (send *pr2_r_pose* (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
;; (setq diff (v- rpos (send *pr2_r_pose* :pos)))
;; (send *pr2* :rarm :inverse-kinematics *pr2_r_pose* :debug-view t)
;;  )


(defun set-balloon (goal_pos)
  ;; 手先の座標はベースリンク基準であることに注意
  (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq cur_lpos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :pos))
  (setq arm_end_coords (scale 0.5 (v+ cur_rpos cur_lpos)))
  ;; 壁に近づく
  (setq move_vec (float-vector (elt goal_pos 0) (elt goal_pos 1) 0))
  (setq move_vec (v- move_vec (float-vector (elt arm_end_coords 0) (elt arm_end_coords 1) 0)))
  (setq move_vec (v- move_vec #f(300 0 0)))
  (setq move_length (norm move_vec))
  (format t "MOVE_VEC: ~A ~A~%" move_vec move_length)
  (send *ri* :go-velocity 0 0 (atan (/ y x)))
  (send *ri* :wait-interpolation)
  (while (> move_length 0)
    (if (>= move_length 300)
	(send *ri* :go-velocity 300 0 0)
      (send *ri* :go-velocity move_length 0 0))
    (setq move_length (- move_length 300))
    (send *ri* :wait-interpolation)
    )
  (send *ri* :go-velocity 0 0 (* -1 (atan (/ y x))))
  (send *ri* :wait-interpolation)
  ;; (send *ri* :go-velocity (elt move_vec 0) (elt move_vec 1) 0)
  ;; 腕を動かす
  (setq balloon_width (- (elt cur_lpos 1) (elt cur_rpos 1)))
  (setq goal_rpos (v- goal_pos (float-vector 0 (/ balloon_width 2) 0)))
  (setq goal_lpos (v+ goal_pos (float-vector 0 (/ balloon_width 2) 0)))
  (setq goal_rpos (v- goal_rpos move_vec))
  (setq goal_lpos (v- goal_lpos move_vec))
  ;; デバッグ用
  (setq r_target (make-cube 40 40 40))
  (send r_target :locate goal_rpos)
  (setq l_target (make-cube 40 40 40))
  (send l_target :locate goal_lpos)
  (objects (list *pr2* r_target l_target))

  (format t "RPOS: ~A -> ~A~%" cur_rpos goal_rpos)
  (format t "LPOS: ~A -> ~A~%" cur_lpos goal_lpos)

  (setq up_length (- (- (elt goal_rpos 2) (elt cur_rpos 2)) 100))
  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) up_length))
  (send *pr2* :rarm :move-end-pos (v- goal_rpos cur_rpos) :world)
  (send *pr2* :larm :move-end-pos (v- goal_lpos cur_lpos) :world)
  (send-robot 4000)
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
  (objects (list *pr2* *wall*))
  ;; 飾り付け
  (dotimes (i (length place_pos_lst))
    (if (= i 0)
	(set-balloon (elt place_pos_lst i)))
    )
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

;; move-to
;; (setq *spot* (make-coords :pos #f(1000 0 0)))
;;  (send *ri* :move-to *spot*)
