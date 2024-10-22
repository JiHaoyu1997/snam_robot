
(cl:in-package :asdf)

(defsystem "vpa_robot_decision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RobotInterMng" :depends-on ("_package_RobotInterMng"))
    (:file "_package_RobotInterMng" :depends-on ("_package"))
  ))