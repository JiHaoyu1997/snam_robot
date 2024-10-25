
(cl:in-package :asdf)

(defsystem "vpa_robot_decision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "InterInfo" :depends-on ("_package_InterInfo"))
    (:file "_package_InterInfo" :depends-on ("_package"))
    (:file "RobotInfo" :depends-on ("_package_RobotInfo"))
    (:file "_package_RobotInfo" :depends-on ("_package"))
  ))