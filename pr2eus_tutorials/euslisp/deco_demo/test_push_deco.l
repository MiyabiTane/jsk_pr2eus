#!/usr/bin/env roseus

;; init
(ros::roseus "pr2_deco_demo_node")

;; robot model
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;; use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

;; load packages
(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "geometry_msgs")
(ros::load-ros-package "std_msgs")
(setq *mux-flag-topic* "/relay_sound/input/mux_flag")
(ros::advertise *mux-flag-topic* std_msgs::Bool 1)
(ros::rate 10)


(defun send-robot (wait_time)
  (send *ri* :angle-vector (send *pr2* :angle-vector) wait_time)
  (send *ri* :wait-interpolation)
  )

(defun set-init-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :torso_lift_joint :joint-angle (- (send *pr2* :torso_lift_joint :joint-angle) 100))
  (send *pr2* :head :look-at #f(500 0 1400))
  (send-robot 5000)
  )

;; global variables
(setq place_pos_lst '())

;; set robot
(pr2-init)
(objects (list *pr2*))

(load "think_decoration.l")
(load "work_on_decoration.l")

;; ---------- main -------------
(defun rotate-rhand (ref_coords radi axis)
  (send (send *pr2* :rarm :end-coords) :assoc ref_coords)
  (setq target_coords (send ref_coords :copy-worldcoords))
  (cond ((string= axis "y")
	 (send target_coords :rotate radi :y :world))
	((string= axis "x")
	 (send target_coords :rotate radi :x :world))
	((string= axis "z")
	 (send target_coords :rotate radi :z :world)))
  ;; (objects (list *pr2* ref_coords target-coords))
  (send *pr2* :rarm :inverse-kinematics target_coords :move-target ref_coords)
  )

(defun rotate-lhand (ref_coords radi axis)
  (send (send *pr2* :larm :end-coords) :assoc ref_coords)
  (setq target_coords (send ref_coords :copy-worldcoords))
  (cond ((string= axis "y")
	 (send target_coords :rotate radi :y :world))
	((string= axis "x")
	 (send target_coords :rotate radi :x :world))
	((string= axis "z")
	 (send target_coords :rotate radi :z :world)))
  ;; (objects (list *pr2* ref_coords target-coords))
  (send *pr2* :larm :inverse-kinematics target_coords :move-target ref_coords)
  )

(defun close-wall (offset)
  (setq pre_x -1)
  (convert-2D->3D 10 10)  ;;func in think_decoration.l
  (format t "wall_x ~A~%" cur_x)
  (setq wall_dist (- cur_x (elt (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos) 0)))
  (format t "wall_dist ~A~%" wall_dist)
  (send *ri* :go-velocity (- wall_dist offset) 0 0)
  (send *ri* :wait-interpolation)
  )

(defun push-balloon-pose ()
  (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq cur_lpos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :pos))
  (setq balloon_width (- (elt cur_lpos 1) (elt cur_rpos 1)))
  (setq arm_end_coords (make-cascoords :pos (scale 0.5 (v+ cur_rpos cur_lpos))))
  (setq r_ik_res (rotate-rhand arm_end_coords (/ pi 3) "y"))
  (setq l_ik_res (rotate-lhand arm_end_coords (/ pi 3) "y"))
  (send-robot 4000)
  (while (not (and r_ik_res l_ik_res))
    (format t "RETRY IK~%")
    (send *pr2* :rarm :move-end-pos #f(-50 0 0) :world)
    (send *pr2* :larm :move-end-pos #f(-50 0 0) :world)
    (setq r_ik_res (rotate-rhand arm_end_coords (/ pi 3) "y"))
    (setq l_ik_res (rotate-lhand arm_end_coords (/ pi 3) "y")))
  (send-robot 4000)
  ;; 壁に近づく
  (send *pr2* :rarm :move-end-pos #f(-100 0 0) :world)
  (send *pr2* :larm :move-end-pos #f(-100 0 0) :world)
  (send *pr2* :head :look-at (float-vector 500 0 (+ 1400 up_length))) ;;up_length<-set-balloon
  (send-robot 4000)
  (close-wall 0)
  )

(defun push-balloon-more-pose ()
  (send *pr2* :rarm :move-end-pos #f(0 -100 0) :world)
  (send *pr2* :larm :move-end-pos #f(0 100 0) :world)
  (send-robot 4000)
  (send *pr2* :rarm :move-end-pos #f(-100 0 0) :world)
  (send *pr2* :larm :move-end-pos #f(-100 0 0) :world)
  (send-robot 4000)
  (send *ri* :go-velocity -200 0 0)
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector #f(300.894 31.0882 20.6352 91.8899 -69.0461 -348.564 -50.7958 281.54 -31.7928 30.9266 -69.7889 -97.2702 303.286 -40.9615 -326.799 3.184338e-07 35.0618))
  (send *pr2* :larm :move-end-pos #f(0 0 -100) :world)
  (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq up_length (- (- (elt goal_rpos 2) (elt cur_rpos 2)) 100))
  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) up_length))
  (send-robot 4000)
  (close-wall 100)
  )

(defun push-ballon (thresh)
  ;; start to push
  (setq l_prev_vec (float-vector 0 0 0))
  (setq r_prev_vec (float-vector 0 0 0))
  (setq l_norm_diff 0)
  (setq r_norm_diff 0)
  (while (or (< l_norm_diff 3) (< r_norm_diff 3))
    ;; bring arm closer
    (send *pr2* :rarm :move-end-pos #f(10 0 0) :world)
    (send *pr2* :larm :move-end-pos #f(10 0 0) :world)
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
      (setq l_prev_vec l_force_vec)
      (setq r_prev_vec r_force_vec))
    (setq l_norm_diff (norm (v- l_force_vec l_prev_vec)))
    (setq r_norm_diff (norm (v- r_force_vec r_prev_vec)))
    (format t "Left ~A ~A~%" l_prev_vec l_force_vec)
    (format t "Right ~A ~A~%" r_prev_vec r_force_vec)
    (format t "diff left: ~A, right: ~A~%" l_norm_diff r_norm_diff))
  )

(defun push-balloon-r (thresh)
  ;; start to push
  (send *pr2* :rarm :move-end-pos #f(100 0 0) :world)
  (send-robot 4000)
  (setq prev_vec (float-vector 0 0 0))
  (setq norm_diff 0)
  (while (< norm_diff thresh)
    (send *pr2* :rarm :move-end-pos #f(10 0 0) :world)
    (send-robot 4000)
    (setq force_msg (one-shot-subscribe *r-force-sub-topic*
					geometry_msgs::WrenchStamped
					:after-stamp (ros::time-now)))
    (vforce-cb force_msg) ;; func in work_on_decoration.l
    (when (= (norm prev_vec) 0)
      (setq prev_vec force_vector))
    (setq norm_diff (norm (v- prev_vec force_vector)))
    (format t "diff: ~A, ~A ~A~%" norm_diff prev_vec force_vector))
  )

(defun main ()
  (send *pr2* :head :look-at #f(500 0 1400))
  (send-robot 4000)
  (setq pre_x -1)
  (convert-2D->3D 320 240)  ;;func in think_decoration.l
  (setq wall_x cur_x)
  (format t "wall_x: ~A~%" wall_x)

  (before-catch-pose) ;; func in work_on_decoration.l
  (catch-balloon)
  ;; (send *pr2* :rarm :move-end-pos #f(0 100 0) :world)
  ;; (send *pr2* :larm :move-end-pos #f(0 -100 0) :world)
  ;; (send-robot 4000)

  (set-balloon goal_pos nil) ;; func in work_on_decoration.l
  (send *pr2* :rarm :move-end-pos #f(-100 0 0) :world)
  (send *pr2* :larm :move-end-pos #f(-100 0 0) :world)
  (send-robot 4000)
  (push-balloon-pose)
  (push-balloon 25)
  ;; (send *pr2* :rarm :move-end-pos #f(150 0 0) :world)
  ;; (send *pr2* :larm :move-end-pos #f(150 0 0) :world)
  ;; (send-robot 4000)

  (push-balloon-more-pose)
  (push-balloon-r 25)
  ;; (send *pr2* :rarm :move-end-pos #f(150 0 0) :world)
  ;; (send-robot 4000)

  (send *ri* :go-velovity -200 0 0)
  (send *ri* :wait-interpolation)
  )

(setq goal_pos #f(1429.53 -100.688 1000.87))
;; (main)


;; --------------------
;; (defun push-balloon-more-pose ()
;;   (send *pr2* :rarm :move-end-pos #f(0 -100 0) :world)
;;   (send *pr2* :larm :move-end-pos #f(0 100 0) :world)
;;   (send-robot 4000)
;;   (rotate-rhand arm_end_coords (* -1 (/ pi 3)) "y")
;;   (rotate-lhand arm_end_coords (* -1 (/ pi 3)) "y")
;;   (send-robot 4000)
;;   (send *pr2* :larm :move-end-pos #f(0 0 -100) :world)
;;   (send-robot 4000)
;;   (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
;;   (setq cur_lpos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :pos))
;;   (setq cur_rcoords (make-cascoords :pos cur_rpos))
;;   (setq cur_lcoords (make-cascoords :pos cur_lpos))
;;   (rotate-lhand cur_lcoords (* -1 (/ pi 2)) "x")
;;   (send-robot 4000)
;;   (rotate-lhand cur_lcoords (* -1 (/ pi 2)) "z")
;;   (send-robot 4000)
;;   (send *pr2* :larm :move-end-pos (float-vector 0 (* -1 (/ balloon_width 2)) 0) :world)
;;   (send-robot 5000)
;;   (rotate-rhand (make-cascoords :pos cur_rpos) (/ pi 2) "z")
;;   (send-robot 4000)
;;   (send *pr2* :rarm :move-end-pos #f(-200 0 0) :world)
;;   (send-robot 4000)
;;   (send *pr2* :rarm :move-end-pos (float-vector 0 (/ balloon_width 2) 0) :world)
;;   (send-robot 4000)
;;   )
