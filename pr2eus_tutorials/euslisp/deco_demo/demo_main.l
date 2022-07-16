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
  (setq pose_msg (elt (send msg :poses) 0))
  (setq pos_msg (send pose_msg :position))
  (setq vec_msg (send pose_msg :orientation))
  (format t "pos: ~A ~A ~A, vec: ~A ~A ~A%"
	  (send pos_msg :x) (send pos_msg :y) (send pos_msg :z)
	  (send vec_msg :x) (send vec_msg :y) (send vec_msg :z))
  ;; (setq norm_coords
  ;;	(make-cascoords
  ;;	 :pos (float-vector (send pos_msg :x) (send pos_msg :y) (send pos_msg :z))
  ;;	 :rpy (float-vector (send vec_msg :x) (send vec_msg :y) (send vec_msg :z))))
  ;; (setq norm_coords (send (send *tfl* :lookup-transform *base-frame-id* *camera-frame-id* (ros::time 0)) 			  :transform norm_coords))
  (setq norm_coords (send (send *tfl* :lookup-transform *base-frame-id* *camera-frame-id* (ros::time 0)) 			  :transform (ros::tf-pose->coords pose_msg)))
  )

(defun rotate-ref-wall ()
  (setq vec_msg nil)
  (ros::subscribe *plane-norm-topic* geometry_msgs::PoseArray #'plane-norm-cb)
  (while (not vec_msg)
    (ros::spin-once)
    (ros::sleep)
    )
  (ros::unsubscribe *plane-norm-topic*)
  (objects (list *pr2* norm_coords))
  ;; ToDo go-vel
  )

(defun go-vel-linear (move_x)
  ;; 818.54 -> 3000ms (818.54 -> 820)
  (setq move_time (abs (* (/ move_x 820.0) 3000)))
  (send *ri* :go-velocity move_x 0 0 move_time :wait t)
  ;; (send *ri* :wait-interpolation)
  ;; memo: (/ pi 6) -> 2100, (/ pi 4) -> 2650
  )

(defun test-go-vel (time)
  (setq pre_x 0)
  (convert-2D->3D 320 240)
  (setq wall_x cur_x)
  (setq arm_end_pos (set-arm-end-pos))
  (setq move_x (- cur_x (elt arm_end_pos 0)))
  (format t "move_x ~A~%" move_x)
  (send *ri* :go-velocity move_x 0 0 time :wait t)
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
;; (set-init-pose)
;; (before-recog-pose)
;; (find-decoration)

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


