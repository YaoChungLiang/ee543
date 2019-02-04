;; Auto-generated. Do not edit!


(when (boundp 'my_kuka_robot::robot_IK)
  (if (not (find-package "MY_KUKA_ROBOT"))
    (make-package "MY_KUKA_ROBOT"))
  (shadow 'robot_IK (find-package "MY_KUKA_ROBOT")))
(unless (find-package "MY_KUKA_ROBOT::ROBOT_IK")
  (make-package "MY_KUKA_ROBOT::ROBOT_IK"))
(unless (find-package "MY_KUKA_ROBOT::ROBOT_IKREQUEST")
  (make-package "MY_KUKA_ROBOT::ROBOT_IKREQUEST"))
(unless (find-package "MY_KUKA_ROBOT::ROBOT_IKRESPONSE")
  (make-package "MY_KUKA_ROBOT::ROBOT_IKRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass my_kuka_robot::robot_IKRequest
  :super ros::object
  :slots (_poses ))

(defmethod my_kuka_robot::robot_IKRequest
  (:init
   (&key
    ((:poses __poses) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    )
   (send-super :init)
   (setq _poses __poses)
   self)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass my_kuka_robot::robot_IKResponse
  :super ros::object
  :slots (_points ))

(defmethod my_kuka_robot::robot_IKResponse
  (:init
   (&key
    ((:points __points) (let (r) (dotimes (i 0) (push (instance trajectory_msgs::JointTrajectoryPoint :init) r)) r))
    )
   (send-super :init)
   (setq _points __points)
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectoryPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectoryPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectoryPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance trajectory_msgs::JointTrajectoryPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass my_kuka_robot::robot_IK
  :super ros::object
  :slots ())

(setf (get my_kuka_robot::robot_IK :md5sum-) "e2841ca7335735bd34d77773a974ca4b")
(setf (get my_kuka_robot::robot_IK :datatype-) "my_kuka_robot/robot_IK")
(setf (get my_kuka_robot::robot_IK :request) my_kuka_robot::robot_IKRequest)
(setf (get my_kuka_robot::robot_IK :response) my_kuka_robot::robot_IKResponse)

(defmethod my_kuka_robot::robot_IKRequest
  (:response () (instance my_kuka_robot::robot_IKResponse :init)))

(setf (get my_kuka_robot::robot_IKRequest :md5sum-) "e2841ca7335735bd34d77773a974ca4b")
(setf (get my_kuka_robot::robot_IKRequest :datatype-) "my_kuka_robot/robot_IKRequest")
(setf (get my_kuka_robot::robot_IKRequest :definition-)
      "geometry_msgs/Pose[] poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
trajectory_msgs/JointTrajectoryPoint[] points


================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")

(setf (get my_kuka_robot::robot_IKResponse :md5sum-) "e2841ca7335735bd34d77773a974ca4b")
(setf (get my_kuka_robot::robot_IKResponse :datatype-) "my_kuka_robot/robot_IKResponse")
(setf (get my_kuka_robot::robot_IKResponse :definition-)
      "geometry_msgs/Pose[] poses

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
trajectory_msgs/JointTrajectoryPoint[] points


================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")



(provide :my_kuka_robot/robot_IK "e2841ca7335735bd34d77773a974ca4b")


