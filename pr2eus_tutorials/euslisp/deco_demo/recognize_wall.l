#!/usr/bin/env roseus

(defun get-backimg-pos ()
  (let* ((point_msg (instance geometry_msgs::Point :init)))
    (setq *wall-pcl*
	  (one-shot-subscribe "/kinect_head/depth_registered/throttled/points"
			      sensor_msgs::PointCloud2
			      :after-stamp (ros::time-now)))
    (convert-2D->3D 0 0 *wall-pcl*)
    (format t "left_top pos: ~A~%" result_3d_pos)
    (send point_msg :x (elt result_3d_pos 0))
    (send point_msg :y (elt result_3d_pos 1))
    (send point_msg :z (elt result_3d_pos 2))
    (send dimg_req_msg :bimg_lt_pos point_msg)
    (setq point_msg (instance geometry_msgs::Point :init))
    (convert-2D->3D 640 480 *wall-pcl*)
    (format t "right_bottom pos: ~A~%" result_3d_pos)
    (send point_msg :x (elt result_3d_pos 0))
    (send point_msg :y (elt result_3d_pos 1))
    (send point_msg :z (elt result_3d_pos 2))
    (send dimg_req_msg :bimg_rb_pos point_msg)
    ;; set wall_x
    (convert-2D->3D 320 240 *wall-pcl*)
    (format t "center pos: ~A~%" result_3d_pos)
    (setq wall_x (elt result_3d_pos 0))
    )
  )

(defun recognize-wall ()
  (move-arm-out-of-view *look-at-pos*)
  (send-robot 4000)
  (rotate-ref-wall)
  (format t "finish rotate~%")
  (setq *image-raw-msg*
	(one-shot-subscribe "/kinect_head/rgb/throttled/image_rect_color"
			    sensor_msgs::Image
			    :after-stamp (ros::time-now)))
  (send dimg_req_msg :back_img *image-raw-msg*)
  (get-backimg-pos)
  )
