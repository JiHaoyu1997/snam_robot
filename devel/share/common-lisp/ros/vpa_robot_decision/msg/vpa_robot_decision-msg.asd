
(cl:in-package :asdf)

(defsystem "vpa_robot_decision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RobotInterInfo" :depends-on ("_package_RobotInterInfo"))
    (:file "_package_RobotInterInfo" :depends-on ("_package"))
  ))