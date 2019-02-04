
(cl:in-package :asdf)

(defsystem "my_kuka_robot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "robot_FK" :depends-on ("_package_robot_FK"))
    (:file "_package_robot_FK" :depends-on ("_package"))
    (:file "robot_IK" :depends-on ("_package_robot_IK"))
    (:file "_package_robot_IK" :depends-on ("_package"))
  ))