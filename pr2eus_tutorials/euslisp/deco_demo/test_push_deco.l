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

(defun push-balloon ()
  (send *pr2* :rarm :move-end-pos #f(0 50 0) :world)
  (send *pr2* :larm :move-end-pos #f(0 -50 0) :world)
  (send-robot 5000)
  (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq cur_lpos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :pos))
  ;; (setq balloon_width (- (elt cur_lpos 2) (elt cur_rpos 2)))
  (setq arm_end_coords (make-cascoords :pos (scale 0.5 (v+ cur_rpos cur_lpos))))
  (rotate-rhand arm_end_coords (/ pi 3) "y")
  (rotate-lhand arm_end_coords (/ pi 3) "y")
  (send-robot 5000)
  ;; 風船を壁につける
  (setq pre_x -1)
  ;; (convert-2D->3D 10 10)  ;;func in think_decoration.l
  (setq cur_x 300)
  (setq wall_dist (- cur_x (elt (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos) 0)))
  ;; (send *ri* :go-velocity (- wall_dist 100) 0 0)
  ;; (send *ri* :wait-interpolation)
  (send *pr2* :rarm :move-end-pos #f(50 0 0) :world)
  (send *pr2* :larm :move-end-pos #f(50 0 0) :world)
  (send-robot 5000)
  ;; 風船を押し付ける
  (send *pr2* :rarm :move-end-pos #f(0 -100 0) :world)
  (send *pr2* :larm :move-end-pos #f(0 100 0) :world)
  (send-robot 5000)
  (rotate-rhand arm_end_coords (* -1 (/ pi 3)) "y")
  (rotate-lhand arm_end_coords (* -1 (/ pi 3)) "y")
  (send-robot 5000)
  (send *pr2* :larm :move-end-pos #f(0 0 -100) :world)
  (send-robot 5000)
  (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
  (setq cur_lpos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :pos))
  (setq cur_rcoords (make-cascoords :pos cur_rpos))
  (setq cur_lcoords (make-cascoords :pos cur_lpos))
  (rotate-lhand cur_lcoords (* -1 (/ pi 2)) "x")
  (send-robot 5000)
  (rotate-lhand cur_lcoords (* -1 (/ pi 2)) "z")
  (send-robot 5000)
  ;; (send *pr2* :larm :move-end-pos (float-vector 0 (/ balloon_width 2) 0) :world)
  ;; (send-robot 5000)
  ;; (rotate-rhand (make-cascoords :pos cur_rpos) (/ pi 2) "z")
  )

(defun main ()
  (setq pre_x -1)
  (convert-2D->3D 320 240)  ;;func in think_decoration.l
  (setq wall_x cur_x)
  (format t "wall_x: ~A~%" wall_x)

  (before-catch-pose) ;; func in work_on_decoration.l
  ;; (catch-balloon)
  (set-balloon goal_pos)
  (push-balloon)
  )


;; (set-init-pose)
(setq goal_pos #f(300 -264.688 1218.87)) ;; #f(1429.53 -264.688 1218.87))

