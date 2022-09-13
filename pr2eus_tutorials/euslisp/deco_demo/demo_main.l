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
(ros::load-ros-package "jsk_recognition_msgs")
(ros::roseus-add-msgs "pr2eus_tutorials")

(defvar *camera-frame-id* "head_mount_kinect_rgb_optical_frame")
(defvar *base-frame-id* "/base_footprint")
(setq *mux-flag-topic* "/relay_sound/input/mux_flag")
(setq *plane-norm-topic* "/plane_normal_vector/output")

(ros::advertise *mux-flag-topic* std_msgs::Bool 1)
(ros::rate 10)

;; global variables
(setq deco_image_lst '())
(setq deco_image_pos_lst '())
(setq place_pos_lst '())
(setq dimg_pub_msg (instance pr2eus_tutorials::DecoImages :init))
(setq pre_x -1)

;; set robot
(pr2-init)
(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)

;; load files
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "common_func.l")
(load "recognize_wall.l")
(load "recognize_decoration.l")
(load "think_decoration.l")
(load "work_on_decoration.l")

;; main ------------
(recognize-wall)
(recognize-decoration)

;; (set-init-pose)
;; (format t "set pose ~%")
;; (get-place-pos)
;; (setq place_pos_lst (list #f(1431.12 480.798 1000.08) #f(1429.85 -126.574 1507.49) #f(1430.41 164.55 1262.37) #f(1429.52 -266.222 1173.37)))

;; (before-catch-pose)
;; (catch-balloon)
;; (work-on-decoration)

;; -----------------

;; switch mux
;; (format t "== FINISH DEMO ==~%")
;; (setq pub_mux_msg (instance std_msgs::Bool :init))
;; (send pub_mux_msg :data nil)
;; (dotimes (i 100)
;;   (ros::publish *mux-flag-topic* pub_mux_msg)
;;   (ros::spin-once)
;;   (ros::sleep)
;;   )


