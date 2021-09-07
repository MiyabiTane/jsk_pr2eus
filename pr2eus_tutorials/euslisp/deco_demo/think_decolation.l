#!/usr/bin/env roseus

;; init
(ros::roseus "pr2_deco_demo_node")

;; set pub topic
(ros::roseus-add-msgs "pr2eus_tutorials")
(setq *deco-pub-topic* "/think_deco/input")
(ros::advertise *deco-pub-topic* pr2eus_tutorials::DecoImages 1)
(ros::rate 10)

(ros::load-ros-package "sensor_msgs")

;; robot model
(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defun set-init-pose ()
  (send *pr2* :reset-manip-pose)
  (send *pr2* :head :look-at #f(500 0 1500))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun pub-img ()
  (setq pub-msg (instance pr2eus_tutorials::DecoImages :init))
  ;; Subscribe image_raw
  (setq *image_raw-msg*
	(one-shot-subscribe "/head_mount_kinect/rgb/image_raw"
			    sensor_msgs::Image
			    :after-stamp (ros::time-now)))
  (send pub-msg :header :stamp (ros::time-now))
  (send pub-msg :back_img *image_raw-msg*)
  (while (ros::ok)
    (ros::publish *deco-pub-topic* pub-msg)
    (unix:usleep (* 100 1000))
    (ros::spin-once)
    )
  )

(pr2-init)
(set-init-pose)
(format t "set pose ~%")
(pub-img)
(ros::exit)
