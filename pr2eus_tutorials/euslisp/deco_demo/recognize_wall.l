#!/usr/bin/env roseus

(defun get-backimg-pos ()
  (let* ((point_msg (instance geometry_msgs::Point :init)))
    (convert-2D->3D 200 200)
    (format t "left_top pos: ~A~%" result_3d_pos)
    (send point_msg :x (elt result_3d_pos 0))
    (send point_msg :y (elt result_3d_pos 1))
    (send point_msg :z (elt result_3d_pos 2))
    (send dimg_pub_msg :bimg_lt_pos point_msg)
    (convert-2D->3D 300 300)
    (format t "right_bottom pos: ~A~%" result_3d_pos)
    (send point_msg :x (elt result_3d_pos 0))
    (send point_msg :y (elt result_3d_pos 1))
    (send point_msg :z (elt result_3d_pos 2))
    (send dimg_pub_msg :bimg_rb_pos point_msg)
    )
  )

(defun recognize-wall ()
  (move-arm-out-of-view #f(500 0 1200))
  (send-robot 4000)
  (rotate-ref-wall)
  (format t "finish rotate~%")
  (setq *image-raw-msg*
	(one-shot-subscribe "/kinect_head/rgb/throttled/image_rect_color"
			    sensor_msgs::Image
			    :after-stamp (ros::time-now)))
  (send dimg_pub_msg :back_img *image-raw-msg*)
  (get-backimg-pos)
  )