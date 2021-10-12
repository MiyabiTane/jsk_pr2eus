;; set pub topic
(ros::roseus-add-msgs "pr2eus_tutorials")
(setq *deco-pub-topic* "/think_deco/input")
(setq *deco-sub-topic* "/think_deco/output")
(setq *screenpoint-pub-topic* "/kinect_head/rgb/throttled/image_rect_color/screenpoint")
(setq *screenpoint-sub-topic* "/pointcloud_screenpoint_nodelet/output_point")
(setq *camera-frame* "head_mount_kinect_rgb_optical_frame")
(setq *base-frame* "base_footprint")
;;(setq *base-frame* "odom")
(ros::advertise *deco-pub-topic* pr2eus_tutorials::DecoImages 1)
(ros::advertise *screenpoint-pub-topic* geometry_msgs::PointStamped 1)
(ros::rate 10)

(ros::load-ros-package "sensor_msgs")
(ros::load-ros-package "geometry_msgs")


(defun pub-img ()
  (setq pub-msg (instance pr2eus_tutorials::DecoImages :init))
  ;; Subscribe image_raw
  (setq *image_raw-msg*
	(one-shot-subscribe "/head_mount_kinect/rgb/image_raw"
			    sensor_msgs::Image
			    :after-stamp (ros::time-now)))
  (send pub-msg :header :stamp (ros::time-now))
  (send pub-msg :back_img *image_raw-msg*)
  )


(defun screen-pos-cb (msg)
  (setq coords (make-coords :pos (ros::tf-point->pos (send msg :point))))
  (send coords :transform
	(send *tfl* :lookup-transform *base-frame* *camera-frame* (ros::time 0)) :world)
  (setq place_pos (send coords :pos))
  (setq cur_x (elt place_pos 0))
  (when (not (= pre_x cur_x))
    (setq flag 1)
    (setq pre_x cur_x))
  )


(defun convert-2D->3D (x y)
  ;; use pointcloud_screenpoint_nodelet
  (ros::subscribe *screenpoint-sub-topic* geometry_msgs::PointStamped #'screen-pos-cb)
  (setq pub-msg (instance geometry_msgs::PointStamped :init))
  (send pub-msg :header :stamp (ros::time-now))
  (send pub-msg :header :frame_id *camera-frame*)
  (send (send pub-msg :point) :x x)
  (send (send pub-msg :point) :y y)
  (setq flag 0)
  (while (= flag 0)
    (ros::publish *screenpoint-pub-topic* pub-msg)
    (ros::spin-once)
    (ros::sleep))
  ;; result 3D pos: place_pos
  )


(defun deco-pos-cb (msg)
  (setq msg_arr (send msg :poses))
  (when (= (send (send (elt msg_arr 0) :position) :z) 1)
    ;; check to see if the information has been updated
    (if (= (length place_pos_lst) 0)
	(setq pre_x -1)
      (progn
	(setq pre_x (elt (elt place_pos_lst 0) 0))
	(setq place_pos '())))
    (dotimes (i (length msg_arr))
      (setq x (send (send (elt msg_arr i) :position) :x))
      (setq y (send (send (elt msg_arr i) :position) :y))
      (convert-2D->3D x y)
      (setq place_pos_lst (append place_pos_lst (list place_pos)))
      )
    (ros::unsubscribe *screenpoint-sub-topic*)
    (setq flag 1)
    (format t "place_pos_lst: ~A~%" place_pos_lst)
    )
  )


(defun get-place-pos ()
  (pub-img)
  (setq flag 0)
  (ros::subscribe *deco-sub-topic* geometry_msgs::PoseArray #'deco-pos-cb)
  (while (= flag 0)
    (ros::publish *deco-pub-topic* pub-msg)
    (unix:usleep (* 100 1000))
    (ros::spin-once))
  (ros::unsubscribe *deco-sub-topic*)
  )


(get-place-pos)
;; (ros::exit)
