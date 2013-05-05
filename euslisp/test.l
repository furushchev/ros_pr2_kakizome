(load "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")

(if (not (boundp '*irtviewer*)) (make-irtviewer))
(if (not (boundp '*pr2*)) (pr2))
(if (not (boundp '*ri*)) (setq *ri* (instance pr2-interface :init)))

(objects (list *pr2*))

(setq table-coords (make-coords :pos #f(650 0 700)
								:rot #2f((1 0 0.01) (0 1 0) (0.0 0 1.0)))))

(ros::rate 10)
(when (ros::ok)
  (send *pr2*  :inverse-kinematics
		(make-coords :pos (float-vector 650 0 (+ (elt (send table-coords :worldpos) 2) 150)))
		:rotation-axis t
		:move-target (send *pr2* :rarm :end-coords)
		:link-list (send *pr2* :link-list
						 (send *pr2* :rarm :end-coords :parent)
						 (send *pr2* :torso_lift_link))
		:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :rarm)
  (print "stoped")
  (send *ri* :wait-interpolation)
  (print "interpolated")
  (unix:sleep 3)
  (send *ri* :start-grasp :rarm)
  (print "start")
)
