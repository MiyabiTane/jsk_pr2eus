#!/usr/bin/env roseus

;; init
(ros::roseus "pr2_deco_demo_node")

;; robot model
(require :pr2-interface "package://pr2eus/pr2-interface.l")

;;use tf
(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))

(defun set-init-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :head :look-at #f(500 0 1500))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

;; global variables
(setq place_pos '())

(pr2-init)
(set-init-pose)
(format t "set pose ~%")

;;(load "recognize_decoration.l")
(load "think_decoration.l")
