;; Auto-generated. Do not edit!


(when (boundp 'project1::TwoInts)
  (if (not (find-package "PROJECT1"))
    (make-package "PROJECT1"))
  (shadow 'TwoInts (find-package "PROJECT1")))
(unless (find-package "PROJECT1::TWOINTS")
  (make-package "PROJECT1::TWOINTS"))

(in-package "ROS")
;;//! \htmlinclude TwoInts.msg.html


(defclass project1::TwoInts
  :super ros::object
  :slots (_a _b ))

(defmethod project1::TwoInts
  (:init
   (&key
    ((:a __a) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   (setq _b (round __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; int16 _a
    2
    ;; int16 _b
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _a
       (write-word _a s)
     ;; int16 _b
       (write-word _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _a
     (setq _a (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _b
     (setq _b (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get project1::TwoInts :md5sum-) "e01e889cb1a7965611513515df5899bf")
(setf (get project1::TwoInts :datatype-) "project1/TwoInts")
(setf (get project1::TwoInts :definition-)
      "int16 a
int16 b
")



(provide :project1/TwoInts "e01e889cb1a7965611513515df5899bf")


