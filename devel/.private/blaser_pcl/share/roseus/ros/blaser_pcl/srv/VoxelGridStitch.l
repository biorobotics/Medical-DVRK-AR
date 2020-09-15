;; Auto-generated. Do not edit!


(when (boundp 'blaser_pcl::VoxelGridStitch)
  (if (not (find-package "BLASER_PCL"))
    (make-package "BLASER_PCL"))
  (shadow 'VoxelGridStitch (find-package "BLASER_PCL")))
(unless (find-package "BLASER_PCL::VOXELGRIDSTITCH")
  (make-package "BLASER_PCL::VOXELGRIDSTITCH"))
(unless (find-package "BLASER_PCL::VOXELGRIDSTITCHREQUEST")
  (make-package "BLASER_PCL::VOXELGRIDSTITCHREQUEST"))
(unless (find-package "BLASER_PCL::VOXELGRIDSTITCHRESPONSE")
  (make-package "BLASER_PCL::VOXELGRIDSTITCHRESPONSE"))

(in-package "ROS")





(defclass blaser_pcl::VoxelGridStitchRequest
  :super ros::object
  :slots (_start _pause _clear _leaf_size ))

(defmethod blaser_pcl::VoxelGridStitchRequest
  (:init
   (&key
    ((:start __start) 0)
    ((:pause __pause) 0)
    ((:clear __clear) 0)
    ((:leaf_size __leaf_size) 0.0)
    )
   (send-super :init)
   (setq _start (round __start))
   (setq _pause (round __pause))
   (setq _clear (round __clear))
   (setq _leaf_size (float __leaf_size))
   self)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:pause
   (&optional __pause)
   (if __pause (setq _pause __pause)) _pause)
  (:clear
   (&optional __clear)
   (if __clear (setq _clear __clear)) _clear)
  (:leaf_size
   (&optional __leaf_size)
   (if __leaf_size (setq _leaf_size __leaf_size)) _leaf_size)
  (:serialization-length
   ()
   (+
    ;; int8 _start
    1
    ;; int8 _pause
    1
    ;; int8 _clear
    1
    ;; float64 _leaf_size
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _start
       (write-byte _start s)
     ;; int8 _pause
       (write-byte _pause s)
     ;; int8 _clear
       (write-byte _clear s)
     ;; float64 _leaf_size
       (sys::poke _leaf_size (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _start
     (setq _start (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _start 127) (setq _start (- _start 256)))
   ;; int8 _pause
     (setq _pause (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _pause 127) (setq _pause (- _pause 256)))
   ;; int8 _clear
     (setq _clear (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _clear 127) (setq _clear (- _clear 256)))
   ;; float64 _leaf_size
     (setq _leaf_size (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass blaser_pcl::VoxelGridStitchResponse
  :super ros::object
  :slots (_status ))

(defmethod blaser_pcl::VoxelGridStitchResponse
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int64 _status
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _status
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _status (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _status) (= (length (_status . bv)) 2)) ;; bignum
              (write-long (ash (elt (_status . bv) 0) 0) s)
              (write-long (ash (elt (_status . bv) 1) -1) s))
             ((and (class _status) (= (length (_status . bv)) 1)) ;; big1
              (write-long (elt (_status . bv) 0) s)
              (write-long (if (>= _status 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _status s)(write-long (if (>= _status 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _status
#+(or :alpha :irix6 :x86_64)
      (setf _status (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _status (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass blaser_pcl::VoxelGridStitch
  :super ros::object
  :slots ())

(setf (get blaser_pcl::VoxelGridStitch :md5sum-) "60c18bc0467086605bc6bd7bde5154dd")
(setf (get blaser_pcl::VoxelGridStitch :datatype-) "blaser_pcl/VoxelGridStitch")
(setf (get blaser_pcl::VoxelGridStitch :request) blaser_pcl::VoxelGridStitchRequest)
(setf (get blaser_pcl::VoxelGridStitch :response) blaser_pcl::VoxelGridStitchResponse)

(defmethod blaser_pcl::VoxelGridStitchRequest
  (:response () (instance blaser_pcl::VoxelGridStitchResponse :init)))

(setf (get blaser_pcl::VoxelGridStitchRequest :md5sum-) "60c18bc0467086605bc6bd7bde5154dd")
(setf (get blaser_pcl::VoxelGridStitchRequest :datatype-) "blaser_pcl/VoxelGridStitchRequest")
(setf (get blaser_pcl::VoxelGridStitchRequest :definition-)
      "int8 start
int8 pause
int8 clear
float64 leaf_size
---
int64 status
")

(setf (get blaser_pcl::VoxelGridStitchResponse :md5sum-) "60c18bc0467086605bc6bd7bde5154dd")
(setf (get blaser_pcl::VoxelGridStitchResponse :datatype-) "blaser_pcl/VoxelGridStitchResponse")
(setf (get blaser_pcl::VoxelGridStitchResponse :definition-)
      "int8 start
int8 pause
int8 clear
float64 leaf_size
---
int64 status
")



(provide :blaser_pcl/VoxelGridStitch "60c18bc0467086605bc6bd7bde5154dd")


