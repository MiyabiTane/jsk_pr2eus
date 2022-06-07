;; set pub sub topics
(ros::load-ros-package "jsk_recognition_msgs")
(defvar *base-frame-id* "/base_footprint")
(defvar *bounding-box-list* "/segmentation_decomposer/boxes")

(defun before-recog-pose ()
  (setq cur_torso_angle (send *pr2* :torso_lift_joint :joint-angle))
  (before-catch-pose)
  (send *pr2* :head :look-at #f(500 0 1400))
  (send *pr2* :torso_lift_joint :joint-angle cur_torso_angle)
  (send-robot 5000)
  )

(defun tf-pose->coords (frame_id pose)
  (send (send *tfl* :lookup-transform *base-frame-id* frame_id (ros::time 0))
        :transform (ros::tf-pose->coords pose)))

(defun bbox->cube (bbox)
  (let* ((dims (ros::tf-point->pos (send bbox :dimensions)))
         (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
    (send bx :newcoords
          (tf-pose->coords (send bbox :header :frame_id) (send bbox :pose)))
    bx))

(defun debug_vis_boxes (i)
  (setq debug_lst '())
  (setq dims (elt deco_size_lst i))
  (setq pos (elt deco_pos_lst i))
  (setq debug_cube (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
  (send debug_cube :move-to pos :world)
  (objects (list *pr2* debug_cube))
  )

(defun bbox->lst (bbox)
  (setq dims (ros::tf-point->pos (send bbox :dimensions)))
  (setq deco_size_lst (list-insert dims (length deco_size_lst) deco_size_lst))
  (setq pos (tf-pose->coords (send bbox :header :frame_id) (send bbox :pose)))
  (setq deco_pos_lst (list-insert pos (length deco_pos_lst) deco_pos_lst))
  )

(defun find-boxes ()
  (ros::duration-sleep 1.0)
  (setq bboxes_msg (one-shot-subscribe *bounding-box-list*
				       jsk_recognition_msgs::BoundingBoxArray
				       :after-stamp (ros::time-now)))
  (setq bboxes (send bboxes_msg :boxes))
  )

(defun get-boxes-coords (bboxes)
  (dotimes (i (length (send bboxes_msg :boxes)))
    (setq bbox (elt (send bboxes_msg :boxes) i))
    (setq bbox (bbox->lst bbox))
    (format t "deco_size_lst ~A~%" deco_size_lst)
    (format t "deco_pos_lst ~A~%" deco_pos_lst)
    )
  )

(defun find-decoration ()
  (dotimes (i 6)
    (send *ri* :go-velocity 0 0 (/ pi 6))
    (send *ri* :wait-interpolation))
  (find-boxes)
  (while (< (length bboxes) 3)
    (send *ri* :go-velocity 0 0 (/ pi 6))
    (send *ri* :wait-interpolation)
    (find-boxes))
  (get-boxes-coords bboxes)
  )

;; send images to think_deco node
(defun get-deco-imgs ()
  )
