#!/usr/bin/env roseus

;; Initialization
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs") ;; like #include in C
(load "package://pr2eus/pr2.l")

;;;
;;; Pr2 model 
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(if (not (boundp '*pr2*)) (pr2))
(objects (list *pr2*))

;;;
;;; Client node defination
(ros::roseus "tabletop_objectdetection_client")

;;publication of coordinates
(ros::advertise "tabletop/coodinates" tabletop-object-ik::coordinates 10)


;; Subcribe to Objectdetection
;; Run (objectdetection-cb) whenever msgs comes
;; set callback function
(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

;; main loop
(ros::rate 10)
(while (ros::ok)
  (setq c (tf-object-update))
  (ros::spin-once) ; for call callback function
  (send *irtviewer* :draw-objects)
  (send-all c :draw-on :flush t)
  (x::window-main-one)
  (ros::sleep)
  )

;; 
;; 
(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))



(defun tf-object-update (&optional (root-frame-id "/base_footprint"))
  (let (c obj-frame-id ret)
    (dolist (obj-frame-id (list "/tabletop0" "/tabletop1" "/tabletop2"))
      ;; advertise
      (when (send *tfl* :get-latest-common-time root-frame-id obj-frame-id)
	(setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
	(when c
	  (ros::ros-info "~A ~A" obj-frame-id c)
	  (push c ret))))
    ret))

(defun objectdetection-cb (msg)
  (let ((frame-id (send msg :header :frame_id)) type ret)
    (print msg)
    (unless (eq (char frame-id 0) #\/) (setq frame-id (concatenate string "/" frame-id)))
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send *tfb* :send-transform ret frame-id type)
      (ros::ros-info "~A ~A ~A" ret frame-id type))
    ))

(defun average-z (c)
  (/ (+ (elt (elt c 0) 2) (elt (elt c 1) 2) (elt (elt c 2) 2)) 3))

(defun transform-coords-to-table nil
  (let (c ave-z)
	



(defun stroke-r (obj s-list)
  (progn (send obj :rarm :inverse-kinematics
			   (car s-list)
			   :link-list (send obj :link-list (send obj :rarm :end-coords :parent))
			   :move-target (send obj :rarm :end-coords)
			   :rotation-axis :z
			   :debug-view nil)
		 (send *irtviewer* :draw-objects)
		 (stroke-r obj (cdr s-list))))

(defun stroke (obj)
  (progn (stroke-r (obj *j0*))
		 (stroke-r (obj *j1*))
		 (stroke-r (obj *s0*))
		 (stroke-r (obj *k0*))
		 (stroke-r (obj *k1*)))
 
