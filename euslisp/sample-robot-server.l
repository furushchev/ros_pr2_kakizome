;;
;; robot-server��simulation���륯�饹�Υ���ץ�
;;
(load "robot-server-simulator.l")
(load "maze.l")
(load "subsumption.l")
(load "models/room73b2-scene.l")

;;
;; server��ޤ����̤�eus��Ω���夲�Ƥ���
;;  server¦�Ǥϡ���ϩ��㳲ʪ�ʤɤ�����ʪ�Τ򤳤���Τ褦�˺��������Ϥ�
;;  ���ξ�硢*obstacle1*, *obstacle2*���Ϥ���server��Ω���夲��
;; 
(warn "(init-server)~%")
(warn "(init-server 'kadai1)~%") ; kadai1
(warn "(init-server 'maze)~%")

(defun init-server (&optional (world 'maze))
  ()
  (let (objects init-coords maze)
    (case
     world
     ('maze
      ;;make red cone for goal
      (setq goal (make-cone (float-vector 0 0 300) 100))
      (setf (get goal :face-color) :red)
      (send goal :locate (float-vector (* 470 5.5) (* 470 4.5) 0))
      ;;��ϩ���֥������Ȥ���(maze.l�򻲾ȤΤ���)
      (setq maze 
	    (make-maze *m* ;;��ϩ�ꥹ��
		       :remove-wall-list (list '((5 4) (5 5))) ;;��������ɥ���ǥå����λ���
		       :maze-length 470  ;;�ºݤ��ɤ�regulation�˽ॺ��
		       :maze-height 200))
      (setq objects (list maze goal))
      (setq init-coords (make-coords :pos
                                     (generate-coords-from-nodenum 1)
                                     :rpy (float-vector pi/2 0 0))))
     ('kadai1
      ;;������������object����������
;      (setq *obstacle1* (make-cube 100 100 200))
	  (setq *obstacle1* (room73b2-knife))
      (send *obstacle1* :locate #f(850 0 100))
	  (send *obstacle1* :rotate (deg2rad 180) :z)
;      (setq *obstacle2* (make-cube 100 100 200))
	  (setq *obstacle2* (room73b2-mug-cup))
      (send *obstacle2* :locate #f(-550 0 100))
      (setq *obstacle3* (make-cube 2500 10 200))
      (send *obstacle3* :locate #f(0 1250 100))
      (setq *obstacle4* (make-cube 2500 10 200))
      (send *obstacle4* :locate #f(0 -1250 100))
      (setq *obstacle5* (make-cube 10 2500 200))
      (send *obstacle5* :locate #f(1250 0 100))
      (setq *obstacle6* (make-cube 10 2500 200))
      (send *obstacle6* :locate #f(-1250 0 100))
      
;      (setq objects (list *obstacle1*
;                          *obstacle2*
;                          *obstacle3*
	  (setq objects (list *obstacle3*
                          *obstacle4*
                          *obstacle5*
                          *obstacle6*))
      (send-all objects :set-color :lightslateblue)
	  (setq objects (append objects (list *obstacle1* *obstacle2*)))
;      (send *obstacle1* :set-color :blue)
;      (send *obstacle2* :set-color :red)
      (setq init-coords (make-coords))
      )
     (t
      ;;������������object����������
      (setq *obstacle1* (make-cube 100 200 200))
      (send *obstacle1* :locate #f(600 0 100))
      (setq *obstacle2* (make-cube 100 200 200))
      (send *obstacle2* :locate #f(-400 0 100))
      (send *obstacle1* :set-color :red)
      (send *obstacle2* :set-color :red)
      (setq objects (list *obstacle1* *obstacle2*))
      (setq init-coords (make-coords)))
      )
    (start-robotsim-server objects :init-coords init-coords)
    ))
(defun m () (init-server 'maze))
(defun km () (init-server 'kadai1))

;;
;; ��Ҥ�(init-server)��Ω���夲��server��������֤�
;; (demo)��¹Ԥ���ȡ�
;; connect-robotsim-server�� simulation-server����³����
;; �ʹߤϡ�connect-robotsim-server������줿 *rs-sim*
;; ��µ��Υ��ܥå�server *rs* ������Ʊ�ͤˡʰ���̤����method���뤬)
;; ���ܥåȤ�ư�������Ȥ��Ǥ���
;;
;; robot-programming-2009/client/eus-client/sample.l��Ʊ��interface��
;; ���Ҥ���Ƥ���Τ��狼��Ȼפ�
;;
;; (connect-robot-server)�Τ����� (connect-robotsim-server)��Ȥ�����
;;
(defun go-pos (x y theta)
  (unless *rs-sim*
    (warn ";; ERROR : ~%")
    (warn ";; you need to invoke~%")
    (warn ";; (connect-robotsim-server)~%")
    (warn ";; (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))~%")
    (warn ";; before this function~%")
    (return-from go-pos nil))
  (when (> (abs theta) 1)
    (send *rs-sim* :daisya-velocity-vector (float-vector 0 0 (* 0.5 theta)))
    (unix::usleep (round (* 69 (abs theta) 1000)))
    (let ((tmp y)) (setq y x) (setq x (abs tmp))))
  (send *rs-sim* :daisya-velocity-vector (float-vector x y 0))
  (unix::usleep (round (* 6.5 (max (abs x) (abs y)) 1000)))
  (send *rs-sim* :daisya-velocity-vector #f(0 0 0))
  (unix::usleep (* 100 1000))
  t)

(defun move-arm (vel &optional (tm 1000))
  (unless *rs-sim*
    (warn ";; ERROR : ~%")
    (warn ";; you need to invoke~%")
    (warn ";; (connect-robotsim-server)~%")
    (warn ";; (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))~%")
    (warn ";; before this function~%")
    (return-from move-arm))
  (when (not (boundp '*daisya*))
    (setq *daisya* (instance daisyaclass :init))
    (send *daisya* :arm-angle-vector #f(80 20 70 0 0)))
  (unless (send *daisya* :solve-ik
                (make-cascoords :pos (v+ (send *daisya* :end-coords :worldpos) vel))
                )
    (warn "ERROR :: could not solve IK~%")
    (warn "      :: try smaller velocity~%")
    (return-from move-arm nil))
  (send *rs-sim* :arm-angle-vector (send *daisya* :arm-angle-vector) tm)
  (unix:usleep (* tm 1000))
  t)

(warn "(demo0) ;; plan->action~%")
(defun demo0
  (&key (start 1) (goal 25))
  (let ((path (maze-search start goal *m*))
        (prev-coords (make-coords :pos #f(200 200 0)
                                  :rpy (float-vector pi/2 0 0)))
        curr-coords pos rot c v bmp-vec)
    (when (not *rs-sim*) (connect-robotsim-server))
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (warn "solved path is ~a~%" path)
    (dolist (p (cdr path))
;	  (setq bmp-vec (send *rs-sim* :bumper-vector))
;	  (send *rs-sim* :daisya-velocity-vector #f(10 0 0))
;	  (if (or (= (aref bmp-vec 0) 1) (= (aref bmp-vec 5) 1))
;		  (progn (send *rs-sim* :daisya-velocity-vector #f(-100 0 0))
;		(unix:usleep (* 1000 500))
;		(send *rs-sim* :daisya-velocity-vector #f(1 0 0))))
      (setq c (generate-coords-from-nodenum p))
      (setq pos (send prev-coords :inverse-transform-vector
                      (float-vector (elt c 0) (elt c 1) 0)))
      (setq curr-coords (send prev-coords :copy-worldcoords))
      (send curr-coords :translate pos)
      (if (> (abs (elt pos 1)) (abs (elt pos 0)))
          (setq rot (if (> (elt pos 0) (elt pos 1)) -90 90))
        (setq rot 0))
      (send curr-coords :rotate (deg2rad rot) :z)
      (print (list p c pos rot))
;	  (do-until-key nil)
      (go-pos (elt pos 0) (elt pos 1) rot)
;      (unix:usleep (* 1000 5000))
	  (unix:sleep 1)
      (setq prev-coords curr-coords)
      )
    (dotimes (i 3)
      (move-arm #f(0 100 0) 200)
      (move-arm #f(0 -100 0) 200))
    ))
;;

(warn "(demo0.5) ;; kadai 2. ~%")
(defun demo0.5
  (&optional (time 200))
  (let (bmp-vec psd-vec)
	(when (not *rs-sim*) (connect-robotsim-server))
	(send *rs-sim* :arm-poweron-vector #f(1 1 1 1 0))

	;;�Ȥꤢ��������Ϥ��
	(send *rs-sim* :daisya-velocity-vector #f(100 0 0))

	(do-until-key
	 (setq bmp-vec (send *rs-sim* :bumper-vector))
	 (setq psd-vec (send *rs-sim* :psd-vector))
	 (send *rs-sim* :daisya-velocity-vector #f(0 0 0))
	 (if (or (= (aref bmp-vec 0) 1) (< 50 (aref psd-vec 0) 200))
		 (progn (go-pos 0 0 -90)
				(setq psd-vec (send *rs-sim* :psd-vector))
			(if (< 50 (aref psd-vec 0) 200)
			  (progn (go-pos 0 0 -180)
;					 (send *rs-sim* :daisya-velocity-vector #f(100 0 0)))
					 (go-pos 150 0 0))
;			  (send *rs-sim* :daisya-velocity-vector #f(100 0 0)))))
			  (go-pos 150 0 0)))
;		 (send *rs-sim* :daisya-velocity-vector #f(100 0 0)))
	   (go-pos 150 0 0))
	 (unix:sleep 1))
))

(warn "(demo2) ;; demo1 using subsumption~%")
(defun demo2
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid) ts)
    ;;robot-server(simulator)�ȤĤʤ�
    (when (not *rs-sim*) (connect-robotsim-server))
    ;;
    ;; subsumption �����
    (push
     (instance
      subsumption-task :init :name "Back bumper" :priority 100
      :check-func
      #'(lambda (s)
	  (let ((bmp (cdr (assoc :bumper s))))
	    (if bmp
		(or (= (aref bmp 2) 1) (= (aref bmp 3) 1)))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(100 0 0))
          (warn "!!forward~%")))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front bumper" :priority 90
      :check-func
      #'(lambda (s)
	  (let ((bmp (cdr (assoc :bumper s))))
	    (if bmp
		(or (= (aref bmp 0) 1) (= (aref bmp 5) 1)))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-100 0 0))
          (warn "!!backward~%")))
     ts)

    (setq s (instance subsumption :init ts :debug nil))

    ;;�Ӥ�servo�򤤤��
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 1))
    ;;���ᤷ���Ӥ�ư�����Ƥߤ�
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;�Ȥꤢ��������Ϥ����
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))

    ;; subsumption �򥹥�����
    (send s :start-clients)
    ;;key�򲡤����ޤ�³����
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      (format t ";; bmp ~A~%" bmp-vec)
      (send s :sensor-vector (list (cons :bumper bmp-vec)))
      (unix:usleep (* time 1000)))
    ;; subsumption ��ߤ��
    (send s :stop-clients)
    
    ;;��֤�ߤ��
    (send *rs-sim* :wheel-reset)
    ;;�Ӥ˻��������
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;�Ǹ��arm��servo�򤭤�
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    )
  )

(warn "(demo3) ;; use vision for maze~%")
(defun demo3
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid))
    ;;robot-server(simulator)�ȤĤʤ�
    (when (not *rs-sim*) (connect-robotsim-server))
    (when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;;�Ӥ�servo�򤤤��
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 1))
    ;;���ᤷ���Ӥ�ư�����Ƥߤ�
    (send *rs-sim* :arm-angle-vector #f(80 -20 -70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;�Ȥꤢ��������Ϥ����
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))
    ;;key�򲡤����ޤ�³����
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      (setq psd-vec (send *rs-sim* :psd-vector))
      (setq red-centroid (send *vs-sim* :result))
      (if red-centroid
          (setq red-centroid (cadr (assoc :centroid red-centroid))))
      (format t ";; bmp ~A, psd ~A, red ~A~%" bmp-vec psd-vec red-centroid)
      (cond
       ;;�夬�֤Ĥ��ä���
       ((or (= (aref bmp-vec 2) 1) (= (aref bmp-vec 3) 1))
		(send *rs-sim* :daisya-velocity-vector #f(200 0 90))
		(warn "!!forward~%")
		(unix:usleep (* 300 1000))
	   )
       ;;�����֤Ĥ��ä���
       ((= (aref bmp-vec 0) 1)
	(send *rs-sim* :daisya-velocity-vector #f(-200 0 -90))
	(warn "!!backward (+ front right)~%")
        (unix:usleep (* 300 1000))
	)
       ((= (aref bmp-vec 5) 1)
	(send *rs-sim* :daisya-velocity-vector #f(-200 0 90))
	(warn "!!backward (+ front left)~%")
        (unix:usleep (* 300 1000))
	)
       ;;���˲����֤Ĥ��ä���
       ((= (aref bmp-vec 1) 1)
        (send *rs-sim* :daisya-velocity-vector #f(0 -100 -30))
        (warn "!!move right~%")
        (unix:usleep (* 300 1000))
        )
       ;;�����ɤ���Ť��Ƥ�����
       ((< 10 (aref psd-vec 2) 100)
        (send *rs-sim* :daisya-velocity-vector #f(50 -50 -30))
        (warn "!!move+turn right~%")
        )
       ;;���˲����֤Ĥ��ä���
       ((or (= (aref bmp-vec 4) 1) (< 10 (aref psd-vec 0) 100))
        (send *rs-sim* :daisya-velocity-vector #f(0 100 30))
        (warn "!!move left~%")
        (unix:usleep (* 300 1000))
        )
       ;;�����ɤ���Ť��Ƥ�����
       ((< 10 (aref psd-vec 0) 100)
        (send *rs-sim* :daisya-velocity-vector #f(50 50 30))
        (warn "!!move+turn left~%"))
       ;;
       ;;�����ɤ���Ť��Ƥ�����
       ((< 50 (aref psd-vec 1) 100)
        (send *rs-sim* :daisya-velocity-vector (float-vector 100 (random 20) 0))
        (warn "!!slow down~%")
        )
       ;; red-centroid�˱������ɽ���ư��Ȥ�
       ((and red-centroid (> (aref red-centroid 0) 170))
        (send *rs-sim* :daisya-velocity-vector (float-vector 150 0 -10))
        (if (< 10 (aref psd-vec 1) 50) (move-arm #f(100 100 0) 500))
        (warn "!!turn left VISION~%")
        )
       ((and red-centroid (< (aref red-centroid 0) 150))
        (send *rs-sim* :daisya-velocity-vector (float-vector 150 -0 10))
        (if (< 10 (aref psd-vec 1) 50) (move-arm #f(100 -100 0) 500))
        (warn "!!turn right VISION~%"))
       ;; �ʤˤ�ʤ��Ȥ�
       (t
        (send *rs-sim* :daisya-velocity-vector (float-vector 200 (random 100) (random 30))))
       )
      (unix:usleep (* time 1000))
      )
    ;;��֤�ߤ��
    (send *rs-sim* :wheel-reset)
    ;;�Ӥ˻��������
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;�Ǹ��arm��servo�򤭤�
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    )
  )

(warn "(demo4) ;; using subsumption~%")
(defun demo4
  (&optional (time 200))
  (let ((bmp-vec) (psd-vector)
        (red-centroid) ts)
    ;;robot-server(simulator)�ȤĤʤ�
    (when (not *rs-sim*) (connect-robotsim-server))
    (when (not *vs-sim*) (connect-visionsim-server))
    ;;
    ;; subsumption �����

    (push
     (instance
      subsumption-task :init :name "Back bumper" :priority 100
      :check-func
      #'(lambda (s)
	  (let ((bmp (cdr (assoc :bumper s))))
	    (and bmp (= (length bmp) 6)
		 (or (= (aref bmp 2) 1) (= (aref bmp 3) 1)))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(200 0 90))
          (warn "!!forward~%")
          (unix:usleep (* 500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front psd" :priority 97
      :check-func
      #'(lambda (s)
	  (let ((psd (cdr (assoc :distance s))))
	    (and psd
		 (< 10 (aref psd 0) 120)
		 (< 10 (aref psd 1) 120)
		 (< 10 (aref psd 2) 120))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-200 0 0))
          (unix:usleep (* 500 1000))
          (send *rs-sim* :daisya-velocity-vector #f(0 0 -90))
          (warn "!!backward (from psd)~%")
          (unix:usleep (* 1500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front right bumper" :priority 95
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 0) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-200 0 -90))
          (warn "!!backward (+ front right)~%")
          (unix:usleep (* 500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Front right bumper" :priority 90
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 5) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(-200 0 90))
          (warn "!!backward (+ front left)~%")
          (unix:usleep (* 500 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left bumper" :priority 85
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 1) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(0 -100 -30))
          (warn "!!move right~%")
          (unix:usleep (* 600 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left psd" :priority 75
      :check-func #'(lambda (s)
		      (let ((psd (cdr (assoc :distance  s))))
			(and psd (< 10 (aref psd 2) 100))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(50 -50 -30))
          (warn "!!move+turn right~%")
          (unix:usleep (* 400 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right bumper" :priority 80
      :check-func #'(lambda (s)
		      (let ((bmp (cdr (assoc :bumper s))))
			(and bmp
			     (= (length bmp) 6)
			     (= (aref bmp 1) 1))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(0 100 30))
          (warn "!!move left~%")
          (unix:usleep (* 600 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right psd" :priority 70
      :check-func #'(lambda (s)
		      (let ((psd (cdr (assoc :distance  s))))
			(and psd (< 10 (aref psd 0) 100))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector #f(50 50 30))
          (warn "!!move+turn left~%")
          (unix:usleep (* 400 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Center psd" :priority 55
      :check-func #'(lambda (s)
		      (let ((psd (cdr (assoc :distance  s))))
			(and psd (< 50 (aref psd 1) 100))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector (float-vector 100 (random 20) 0))
          (warn "!!slow down~%")))
     ts)

    (push
     (instance
      subsumption-task :init :name "Left Red object" :priority 35
      :check-func #'(lambda (s)
		      (let ((red (cdr (assoc :red  s))))
			(and red (> (aref red 0) 170))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector (float-vector 150 0 -10))
          (warn "!!turn left VISION~%")
          (unix:usleep (* 1000 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Right Red object" :priority 30
      :check-func #'(lambda (s)
		      (let ((red (cdr (assoc :red  s))))
			(and red (< (aref red 0) 150))))
      :action-func
      #'(lambda ()
          (send *rs-sim* :daisya-velocity-vector (float-vector 150 0 10))
          (warn "!!turn right VISION~%")
          (unix:usleep (* 1000 1000))))
     ts)

    (push
     (instance
      subsumption-task :init :name "Wander" :priority 10
      :check-func #'(lambda (s) t)
      :action-func
      #'(lambda ()
          (print "wander")
          (send *rs-sim* :daisya-velocity-vector (float-vector 200 (- (random 100) 50) (- (random 30) 15)))
          (unix:usleep (* 100 1000))))
     ts)
    ;;
    (setq s (instance subsumption :init ts :debug nil))

    ;;�Ӥ�servo�򤤤��
    (send *rs-sim* :arm-poweron-vector #f(1 1 1 1 1))
    ;;���ᤷ���Ӥ�ư�����Ƥߤ�
    (send *rs-sim* :arm-angle-vector #f(80 20 70 0 0) 1000)
    (unix:usleep (* 1000 1000))
    
    ;;�Ȥꤢ��������Ϥ����
    (send *rs-sim* :daisya-velocity-vector #f(100 0 0))

    ;; subsumption �򥹥�����
    (send s :start-clients)
    ;;key�򲡤����ޤ�³����
    (do-until-key
      (setq bmp-vec (send *rs-sim* :bumper-vector))
      (setq psd-vec (send *rs-sim* :psd-vector))
      (setq red-centroid (send *vs-sim* :result))
      (if red-centroid
          (setq red-centroid (cadr (assoc :centroid red-centroid))))
      (format t ";; bmp ~A, psd ~A, red ~A~%" bmp-vec psd-vec red-centroid)
      (send s :sensor-vector (list (cons :bumper bmp-vec)
				   (cons :distane psd-vec)
				   (cons :red red-centroid)))
      (unix:usleep (* time 1000))
      )
    ;; subsumption ��ߤ��
    (send s :stop-clients)
    
    ;;��֤�ߤ��
    (send *rs-sim* :wheel-reset)
    ;;�Ӥ˻��������
    (send *rs-sim* :arm-angle-vector (float-vector 0 0 0 0 0) 3000)
    (unix:usleep (* 1000 3000))
    ;;�Ǹ��arm��servo�򤭤�
    (send *rs-sim* :arm-poweron-vector #f(0 0 0 0 0))
    )
  )
