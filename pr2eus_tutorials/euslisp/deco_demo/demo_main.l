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


(defun send-robot (wait_time)
  (send *ri* :angle-vector (send *pr2* :angle-vector) wait_time)
  (send *ri* :wait-interpolation)
  )

(defun set-init-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :head :look-at #f(500 0 1500))
  (send-robot 5000)
  )

;; global variables
(setq place_pos_lst '())

;; set robot
(pr2-init)
(set-init-pose)
(format t "set pose ~%")

;; load files
;; (load "recognize_decoration.l")
(load "think_decoration.l")
(load "work_on_decoration.l")

;; main
(get-place-pos)

(before-catch-pose)
(catch-balloon)
