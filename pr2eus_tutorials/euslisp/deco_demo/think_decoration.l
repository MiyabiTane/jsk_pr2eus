;; set pub topic
(ros::roseus-add-msgs "pr2eus_tutorials")
(setq *deco-pub-topic* "/think_deco/input")
(setq *deco-sub-topic* "/think_deco/output")
(ros::advertise *deco-pub-topic* pr2eus_tutorials::DecoImages 1)
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


(defun pos_cb (msg)
  (setq first_msg (elt (send msg :poses) 0))
  (when (= (send (send first_msg :position) :z) 1)
    (format t "called ~%")
    (setq flag 1))
  )


(defun get-place-pos ()
  (pub-img)
  (setq flag 0)
  (while (= flag 0)
    (ros::publish *deco-pub-topic* pub-msg)
    (ros::subscribe *deco-sub-topic* geometry_msgs::PoseArray #'pos_cb)
    (unix:usleep (* 100 1000))
    (ros::spin-once))
  (ros::unsubscribe *deco-sub-topic*)
  )


(get-place-pos)
;; (ros::exit)
