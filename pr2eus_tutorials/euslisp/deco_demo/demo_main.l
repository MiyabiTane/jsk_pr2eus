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

(defvar *camera-frame-id* "head_mount_kinect_rgb_optical_frame")
(defvar *base-frame-id* "/base_footprint")
(setq *mux-flag-topic* "/relay_sound/input/mux_flag")
(setq *plane-norm-topic* "/plane_normal_vector/output")


(ros::advertise *mux-flag-topic* std_msgs::Bool 1)
(ros::rate 10)


(defun send-robot (wait_time)
  (send *ri* :angle-vector (send *pr2* :angle-vector) wait_time)
  (send *ri* :wait-interpolation)
  )

(defun set-arm-end-pos ()
  (let* ((arm_end_pos #f(0 0 0)))
    (setq cur_rpos (send (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :pos))
    (setq cur_lpos (send (send (send *pr2* :larm :end-coords) :copy-worldcoords) :pos))
    (setq arm_end_pos (scale 0.5 (v+ cur_rpos cur_lpos)))
    arm_end_pos)
  )

(defun plane-norm-cb (msg)
  (setq poses_msg (send msg :poses))
  (when (> (length poses_msg) 0)
    (setq pose_msg (elt poses_msg 0))
    (setq pos_msg (send pose_msg :position))
    (setq vec_msg (send pose_msg :orientation))
    (setq norm_coords (send (send *tfl* :lookup-transform *base-frame-id* *camera-frame-id* (ros::time 0)) 			  :transform (ros::tf-pose->coords pose_msg)))
    ;; (format t "norm_coords: ~A~%" norm_coords)
    ;; (format t "pos: ~A, rot: ~A~%" (send norm_coords :pos) (send norm_coords :rot))
    (setq norm_rpy (elt (rpy-angle (send norm_coords :rot)) 0))
    ;; (format t "norm_rpy: ~A~%" norm_rpy)
    )
  )

(defun go-vel-rotate (move_rad)
  ;;warning!! if the degree is large, this funcdoes not correct
  ;; memo: (/ pi 6) -> 2100, (/ pi 4) -> 2650
  (setq move_time 2650)
  (cond ((< (abs move_rad) (/ pi 6))
 	 (setq move_time (abs (* (/ move_rad (/ pi 6)) 2100))))
 	((< (abs move_rad) (/ pi 4))
 	 (setq move_time (+ 2100 (* 550 (/ (- move_rad (/ pi 6)) (/ pi 12)))))))
  (send *ri* :go-velocity 0 0 move_rad move_time :wait t)
  )

(defun rotate-ref-box ()
  ;; (send norm_coords_cp :locate #f(500 0 0) :world)
  (setq base_box (make-cube 50 50 50))
  (setq virtual_box (make-cube 50 50 50))
  (send virtual_box :locate #f(0 0 500) :world)
  (send base_box :assoc virtual_box)
  (send base_box :newcoords (make-coords :rpy norm_rpy))
  ;; (objects (list *pr2* base_box virtual_box))
  (setq vbox_pos (send (send virtual_box :copy-worldcoords) :pos))
  (format t "vbox_pos ~A~%" vbox_pos)
  (setq vbox_xpos (elt vbox_pos 0))
  (setq vbox_ypos (elt vbox_pos 1))
  ;; (go-vel-rotate (atan (/ vbox_ypos vbox_xpos)))
  (if (> vbox_ypos 0)
      (send *ri* :go-velocity 0 0 (/ pi 20) 500 :wait t)
    (send *ri* :go-velocity 0 0 (* -1 (/ pi 20)) 500 :wait t))
  )

(defun rotate-ref-wall ()
  (ros::subscribe *plane-norm-topic* geometry_msgs::PoseArray #'plane-norm-cb)
  (setq vbox_ypos 100)
  (while (> (abs vbox_ypos) 50)
    (setq pose_msg nil)
    (while (not pose_msg)
      (ros::spin-once)
      (ros::sleep))
    (rotate-ref-box))
  (ros::unsubscribe *plane-norm-topic*)
  )

(defun go-vel-linear (move_x)
  ;; 818.54 -> 3000ms (818.54 -> 820)
  (setq move_time (abs (* (/ move_x 820.0) 3000)))
  ;; (format t "go-vel-linear move: ~A, time: ~A~%" move_x move_time)
  (send *ri* :go-velocity move_x 0 0 move_time :wait t)
  ;; (send *ri* :wait-interpolation)
  )

(defun set-init-pose ()
  ;; (send *pr2* :reset-manip-pose)
  ;; (send *pr2* :torso_lift_joint :joint-angle (- (send *pr2* :torso_lift_joint :joint-angle) 100))
  ;; (send *pr2* :head :look-at #f(500 0 1400))
  (send *pr2* :angle-vector #f(200.0 75.0 50.0 110.0 -110.0 -20.0 -10.0 -10.0 -75.0 50.0 -110.0 -110.0 20.0 -10.0 -10.0 0.0 11.8786))
  (send-robot 5000)
  )

;; global variables
(setq deco_image_lst '())
(setq deco_image_pos_lst '())
(setq place_pos_lst '())

;; set robot
(pr2-init)
(send *ri* :start-grasp :rarm)
(send *ri* :start-grasp :larm)

;; load files
(load "package://pr2eus_moveit/euslisp/collision-object-publisher.l")
(load "recognize_decoration.l")
(load "think_decoration.l")
(load "work_on_decoration.l")

;; main ------------
(set-init-pose)
(before-recog-pose)
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


