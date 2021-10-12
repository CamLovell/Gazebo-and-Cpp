;; Auto-generated. Do not edit!


(when (boundp 'navigation::3dofStates)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow '3dofStates (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::3DOFSTATES")
  (make-package "NAVIGATION::3DOFSTATES"))

(in-package "ROS")
;;//! \htmlinclude 3dofStates.msg.html


(defclass navigation::3dofStates
  :super ros::object
  :slots (_N _E _psi _u _v _r ))

(defmethod navigation::3dofStates
  (:init
   (&key
    ((:N __N) 0.0)
    ((:E __E) 0.0)
    ((:psi __psi) 0.0)
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    ((:r __r) 0.0)
    )
   (send-super :init)
   (setq _N (float __N))
   (setq _E (float __E))
   (setq _psi (float __psi))
   (setq _u (float __u))
   (setq _v (float __v))
   (setq _r (float __r))
   self)
  (:N
   (&optional __N)
   (if __N (setq _N __N)) _N)
  (:E
   (&optional __E)
   (if __E (setq _E __E)) _E)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:serialization-length
   ()
   (+
    ;; float64 _N
    8
    ;; float64 _E
    8
    ;; float64 _psi
    8
    ;; float64 _u
    8
    ;; float64 _v
    8
    ;; float64 _r
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _N
       (sys::poke _N (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _E
       (sys::poke _E (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u
       (sys::poke _u (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _N
     (setq _N (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _E
     (setq _E (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u
     (setq _u (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get navigation::3dofStates :md5sum-) "571db7e1a9de92532ecdf36e8286b584")
(setf (get navigation::3dofStates :datatype-) "navigation/3dofStates")
(setf (get navigation::3dofStates :definition-)
      "float64 N
float64 E
float64 psi
float64 u
float64 v
float64 r

")



(provide :navigation/3dofStates "571db7e1a9de92532ecdf36e8286b584")


