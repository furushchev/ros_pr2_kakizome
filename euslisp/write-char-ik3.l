#!/usr/bin/env roseus

;;Initialization
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "pr2_kakizome")
(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(load "jsk-stroke.l")
(load "sikaku.l")
(load "eustf.l")
(load "package://roseus/euslisp/roseus-utils.l")

(make-jsk)

;;make pen
(setq *pen* (make-cylinder 3 150))

;;PR2 model
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))
(objects (list *pr2* *pen*))

(defun tf-object-update (&optional (root-frame-id "/base_footprint"))
  (let (c obj-frame-id ret)
	(setq obj-frame-id  "/table")
      ;; advertise
	(when (send *tfl* :get-latest-common-time root-frame-id obj-frame-id)
	(setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
	(when c
	  (ros::ros-info "~A ~A" obj-frame-id c)
;	  (setq tc c)
	  (push c ret)))
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

(defun hang-pen nil
  (send *pr2*  :inverse-kinematics
		(make-coords :pos (float-vector 550 0 (+ 700 150)))
		:rotation-axis t
		:move-target (send *pr2* :rarm :end-coords)
		:link-list (send *pr2* :link-list
						 (send *pr2* :rarm :end-coords :parent)
						 (send *pr2* :torso_lift_link))
		:debug-view nil)
  (send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm)
  (send *ri* :wait-interpolation)
  (unix:sleep 5)
  (send *ri* :start-grasp :rarm))


#|
(defun stroke-r (obj s-list c)
  (let (transformed-s (car s-list) table-z roll-axis-vector rot)
	(return-from stroke-r nil)
;;;	(setq table-z ((send c :worldrot) :matrix-column 2))
	(setq roll-axis-vector (v* table-z #f(0 0 1))
	(setq rot (abs (/ (v. table-z #f(0 0 1)) (norm table-z))))
	(send transformed-s 
	(progn (send obj :rarm :inverse-kinematics
				 
				 :link-list (send obj :link-list (send obj :rarm :end-coords :parent))
				 :move-target (send obj :rarm :end-coords)
				 :rotation-axis :z
				 :debug-view nil)
		   (when *irtviewer* (send *irtviewer* :draw-objects))
		   (unix:sleep 5)
		   (stroke-r obj (cdr s-list)))))))
|#

(defun stroke (point-cloud-list table-coords &optional (debug nil))
  (warn "stroke called~%")
  (let (p point-vector point-vector-transformed table-z-rot rotate-axis-vector rot
		  point-vector-list angle-vector-list)
	(when (not table-coords)
	  (setq table-coords (make-coords :pos #f(550 0 700)
									  :rot #2f((1 0 0.01) (0 1 0) (0.0 0 1.0)))))
	(print table-coords)
	(setq table-z-rot (matrix-column (send table-coords :worldrot) 2))
	(print table-z-rot)
	(setq rotate-axis-vector (v* table-z-rot #f(0 0 1)))
	(setq rot (acos (abs (/ (v. table-z-rot #f(0 0 1)) (norm table-z-rot)))))
	(print rotate-axis-vector)
	(print (rad2deg rot))
	(dolist (p point-cloud-list) ;;１画ごと
	  (send p :rotate rot rotate-axis-vector)
	  (setq angle-vector-list nil)
	  (dotimes (i (elt (array-dimensions (send p :points)) 0))
		(setq point-vector (scale 250 (matrix-row (send p :points) i)))
		(warn "pv~%")
		(print point-vector)
		;;(send point-vector :draw-on :flush t)
;		(push (float-vector (+ (elt point-vector 0) 150)
;							(elt point-vector 1)
;		(push (float-vector (- 50 (elt point-vector 1))
		(setq point-vector-transformed (float-vector (+ 550.0 (elt point-vector 1))
													 (- 0.0 (* (elt point-vector 0) -1))
													 (+ (elt point-vector 2) (elt (send table-coords :worldpos) 2) 120))) ;150はペンの長さ
;							 point-vector-list)
		(push point-vector-transformed point-vector-list)
		(send-all point-vector-list :draw-on :flush t :size 10)
		(setq ik-ret
			  (send *pr2*  :inverse-kinematics
;					(send (make-coords :pos (float-vector 650 0 (+ (elt (send table-coords :worldpos) 2) 150)))
					(send (make-coords :pos (float-vector 0 0 0))
						  :translate point-vector-transformed)
					:rotation-axis t
					:move-target (send *pr2* :rarm :end-coords)
					:link-list (send *pr2* :link-list
									 (send *pr2* :rarm :end-coords :parent)
									 (send *pr2* :torso_lift_link))
					:debug-view nil))
		(send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
		(when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
		(when (boundp '*ri*)
		  (if debug
			  (progn (print (list (send *pr2* :rarm :end-coords :worldpos) ik-ret))
;					 (unix:usleep (* 200 1000))
					 (send-all point-vector-list :draw-on :flush t))
;			(progn (send *ri* :angle-vector (send *pr2* :angle-vector) 500)
;				   (send *ri* :wait-interpolation)
;				   (print (send *pr2* :rarm :end-coords :worldpos))
			(progn (push (send *pr2* :angle-vector) angle-vector-list)
				   (print (send *pr2* :rarm :end-coords :worldpos))
			))
		  )
		)
	  (when (and (boundp '*ri*) (not debug))
		(print angle-vector-list)
		(print "pre")
		(setq angle-vector-list (reverse angle-vector-list))
		(send *ri* :angle-vector (car angle-vector-list))
		(send *ri* :wait-interpolation)
		(print "start")
		(send *ri* :angle-vector-sequence angle-vector-list (make-sequence 'list (length angle-vector-list) :initial-element 100)) ;;TODO; 今は全部ikを解いてからだが、一筆ごとにして次の画に映るときに腕を少し上げてから移動するようにする。
		(print "sending")
		(send *ri* :wait-interpolation)
		(send *pr2* :rarm :move-end-pos (float-vector 0 0 30))
		(send *ri* :angle-vector (send *pr2* :angle-vector))
		(send *ri* :wait-interpolation)
		(print "end")
		))
	))

(defun s-debug nil (stroke *pr2* (list *sikaku*) nil t))
(defun rot-debug (point-cloud)
  (let (table-coords table-z-rot rotate-axis-vector rot)
	(setq table-coords (make-coords :pos #f(800 0 700)
									:rot #2f((1 0 0.3) (0 1 0) (0.0 0 1.0))))
	(print table-coords)
	(setq table-z-rot (matrix-column (send table-coords :worldrot) 2))
	(print table-z-rot)
	(setq rotate-axis-vector (v* table-z-rot #f(0 0 1)))
	(setq rot (acos (abs (/ (v. table-z-rot #f(0 0 1)) (norm table-z-rot)))))
	(print rotate-axis-vector)
	(print (rad2deg rot))
	(send point-cloud :rotate rot rotate-axis-vector)
	(objects (list point-cloud))
))

;;client node definition
(ros::roseus "tabletop_objectdetection_client")

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))

;;subscribe to object detection
(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

;;callback function
(defun get-touch-pos-cloud-msg (msg)
  (let (point-cloud-list pc c)
;  (print msg)
;  (print (make-eus-pointcloud-from-ros-msg1 (elt (send msg :pointsList) 0)))
;  (unix:sleep 3)
	(dolist (pc-msg (send msg :pointsList))
	  (setq pc (make-eus-pointcloud-from-ros-msg1 pc-msg))
	  (print (send pc :points))
	  (push pc point-cloud-list))
	(setq c (tf-object-update))
	(when c
	  (stroke (reverse point-cloud-list) (elt c 0)))
  )
  (hang-pen)
)

;;Subscribe ROS CANVAS touch point cloud
(ros::subscribe "touch_pos_cloud" pr2_kakizome::PointCloudList #'get-touch-pos-cloud-msg)

;;main loop

  (ros::rate 10)
  (setq count 0)
  (while (ros::ok)
;	(setq c (tf-object-update))
	(ros::spin-once) ; for call callback function
	(when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
;	(send-all c :draw-on :flush t)
;	(x::window-main-one)
	(ros::sleep)
	(when (eq count 0) (hang-pen))
;	(when c
;	  (if (eq count 0)
;		  (progn 
			(print "loop")
;			(print (elt c 0))
;			(stroke *jsk* (elt c 0))
;		  (progn (stroke (list *sikaku*) (elt c 0))
			(setq count (+ 1 count))
;			)))
	)

