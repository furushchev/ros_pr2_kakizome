(defclass words-points
  :super pointcloud
  :slot (pc-list curr-num))

(defmethod words-points
  (:init
   (&optional (pc))
   (setq pc-list pc)
   (setq curr-num )
   self)
  (:itr-move-to-point (word-num 0)
	 (if (< (length (elt pc-list word-num)) i)
		 (return nil))
	 
	   
