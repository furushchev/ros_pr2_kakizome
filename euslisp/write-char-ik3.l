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

;;make pen
(setq *pen* (make-cylinder 3 150))

;;PR2 model
;(if (not (boundp '*irtviewer*)) (make-irtviewer))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))
(if (boundp '*irtviewer*) (objects (list *pr2* *pen*)))

(defun tf-object-update (&optional (root-frame-id "/base_footprint"))
  (let (c obj-frame-id ret)
    (setq obj-frame-id  "/table")
      ;; advertise
    (when (send *tfl* :get-latest-common-time root-frame-id obj-frame-id)
      (setq c (send *tfl* :lookup-transform root-frame-id obj-frame-id (ros::time 0)))
      (when c
        (ros::ros-info "~A ~A" obj-frame-id c)
        (push c ret)))
    ret))

(defun objectdetection-cb (msg)
  (let ((frame-id (send msg :header :frame_id)) type ret)
;    (print msg)
    (unless (eq (char frame-id 0) #\/) (setq frame-id (concatenate string "/" frame-id)))
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send *tfb* :send-transform ret frame-id type)
;      (ros::ros-info "~A ~A ~A" ret frame-id type)
    )
    (tf-object-update)
))

(defun hang-pen ()
  (let (rarm-end-coords
        avec tvec)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq rarm-end-coords  (send (send *pr2* :rarm :end-coords) :copy-worldcoords))
    (send *pr2* :inverse-kinematics
          (send rarm-end-coords :translate #f(0 0 100) :world)
          :rotation-axis :z
          :move-target (send *pr2* :rarm :end-coords)
          :link-list (send *pr2* :link-list
                           (send *pr2* :rarm :end-coords :parent))
          :look-at-target t
          :debug-view nil)
    (push (send *pr2* :angle-vector) avec)
    (send *pr2*  :inverse-kinematics
          (make-coords :pos (float-vector 550 0 (+ 700 250)) :rpy (list 0 0 0))
          :rotation-axis t
          :move-target (send *pr2* :rarm :end-coords)
          :link-list (send *pr2* :link-list
                           (send *pr2* :rarm :end-coords :parent))
                                        ;                         (send *pr2* :torso_lift_link))
          :look-at-target t
          :debug-view nil)
    (push (send *pr2* :angle-vector) avec)
    (dotimes (i (length avec))
      (push 3000 tvec))
;  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :angle-vector-sequence (reverse avec) tvec)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm :wait t)
;  (unix:sleep 5)
  (send *ri* :start-grasp :rarm)
  ))

(defun stroke (point-cloud-list table-coords &optional (debug nil))
  (warn "stroke called~%")
  (let (p point-vector point-vector-transformed table-z-rot rotate-axis-vector rot
          point-vector-list angle-vector-list)
    (when (not table-coords)
      (setq table-coords (make-coords :pos #f(550 0 700)
                                      :rot #2f((1 0 0.01) (0 1 0) (0.0 0 1.0)))))
    (warn "table-coords: ~A~%" table-coords)
    (setq table-z-rot (matrix-column (send table-coords :worldrot) 2))
    (warn "table-z-rot: ~A~%" table-z-rot)
    (setq rotate-axis-vector (v* table-z-rot #f(0 0 1)))
    (setq rot (acos (abs (/ (v. table-z-rot #f(0 0 1)) (norm table-z-rot)))))
    (warn "rotate-axis-vector: ~A~%" rotate-axis-vector)
    (warn "rot(deg): ~A~%" (rad2deg rot))
    (dolist (p point-cloud-list) ;;１画ごと
      (send p :rotate rot rotate-axis-vector)
      (setq angle-vector-list nil)
      (dotimes (i (elt (array-dimensions (send p :points)) 0))
        (setq point-vector (scale 0.45 (matrix-row (send p :points) i)))
        (warn "pv~A~%" point-vector)
        (setq point-vector-transformed (float-vector (+ 900.0 (* (elt point-vector 1) -1))
                                                     (+ 225.0 (* (elt point-vector 0) -1))
                                                     (+ (elt point-vector 2) (elt (send table-coords :worldpos) 2) 120))) ;120はペンの長さ
        (push point-vector-transformed point-vector-list)
        (if (boundp '*irtviewer*) (send-all point-vector-list :draw-on :flush t :size 10))
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        (setq ik-ret
              (send *pr2*  :inverse-kinematics
                    (make-coords :pos point-vector-transformed)
                    :rotation-axis :z
                    :move-target (send *pr2* :rarm :end-coords)
                    :link-list (send *pr2* :link-list
                                     (send *pr2* :rarm :end-coords :parent)
                                     (send *pr2* :torso_lift_link)
                                     )
                    :look-at-target t
;                    :revert-if-fail nil
                    :debug-view nil))
        (when (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
        (when (boundp '*ri*)
          (if debug
              (progn (print (list (send *pr2* :rarm :end-coords :worldpos) ik-ret))
                     (send-all point-vector-list :draw-on :flush t))
            (progn (push (send *pr2* :angle-vector) angle-vector-list)
                   (warn "rarm end pos: ~A~%" (send *pr2* :rarm :end-coords :worldpos))
                   ))
          )
        )
      (when (and (boundp '*ri*) (not debug))
        (print "pre")
        (setq angle-vector-list (reverse angle-vector-list))
        (warn "angle-vector-list: ~A~%" angle-vector-list)
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
  (hang-pen)
  (let (point-cloud-list pc c)
    (dolist (pc-msg (send msg :pointsList))
      (setq pc (make-eus-pointcloud-from-ros-msg1 pc-msg))
      (warn "points: ~A~%" (send pc :points))
      (push pc point-cloud-list))
    (setq c (tf-object-update))
    (when c
      (stroke (reverse point-cloud-list) (elt c 0)))
    )
;  (hang-pen)
  )

;;Subscribe ROS CANVAS touch point cloud
(ros::subscribe "touch_pos_cloud" pr2_kakizome::PointCloudList #'get-touch-pos-cloud-msg)

(defun pr2-init-pose ()
  (pr2-reset-pose)
  (send *pr2* :head :neck-p :joint-angle 50)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation))

;;main loop
(defun main ()
  (ros::rate 10)
  (pr2-init-pose)
  (while (ros::ok)
    (ros::spin-once) ; for call callback function
    (when (boundp '*irtviewer*)
      (send *irtviewer* :draw-objects))
    (ros::sleep)
    )
  )