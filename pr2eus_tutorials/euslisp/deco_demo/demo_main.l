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
;; (set-init-pose)
(format t "set pose ~%")

;; load files
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
;; (load "recognize_decoration.l")
(load "think_decoration.l")
(load "work_on_decoration.l")

;; main ------------
;;(get-place-pos)

;; (setq place_pos_lst (list #f(1430.55 240.193 1074.58) #f(1430.08 -5.14256 1019.39) #f(1429.59 -245.31 1239.8) #f(1430.77 338.217 1351.63) #f(1429.54 -296.461 1155.97) #f(1430.44 179.398 1397.42) #f(1429.91 -75.6795 1455.58) #f(1430.09 -5.34563 1142.01)))
(setq place_pos_lst (list #f(1430.55 240.193 1074.58)))

(before-catch-pose)
;; (catch-balloon)
(work-on-decoration)

;; -----------------

;; switch mux
(format t "== FINISH DEMO ==~%")
(setq pub_mux_msg (instance std_msgs::Bool :init))
(send pub_mux_msg :data nil)
(dotimes (i 100)
  (ros::publish *mux-flag-topic* pub_mux_msg)
  (ros::spin-once)
  (ros::sleep)
  )
