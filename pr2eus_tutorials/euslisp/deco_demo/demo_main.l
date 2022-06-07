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
  ;; (send *pr2* :reset-manip-pose)
  ;; (send *pr2* :torso_lift_joint :joint-angle (- (send *pr2* :torso_lift_joint :joint-angle) 100))
  ;; (send *pr2* :head :look-at #f(500 0 1400))
  (send *pr2* :angle-vector #f(200.0 75.0 50.0 110.0 -110.0 -20.0 -10.0 -10.0 -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0 0.0 11.8786))
  (send-robot 5000)
  )

;; global variables
(setq deco_pos_lst '())
(setq deco_size_lst '())
(setq place_pos_lst '())

;; set robot
(pr2-init)

;; load files
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "recognize_decoration.l")
(load "think_decoration.l")
(load "work_on_decoration.l")

;; main ------------
;; (set-init-pose)
(before-recog-pose)
(find-decoration)

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
