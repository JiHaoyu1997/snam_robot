
(cl:in-package :asdf)

(defsystem "vpa_robot_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LeftWheelT" :depends-on ("_package_LeftWheelT"))
    (:file "_package_LeftWheelT" :depends-on ("_package"))
    (:file "WheelsCmd" :depends-on ("_package_WheelsCmd"))
    (:file "_package_WheelsCmd" :depends-on ("_package"))
    (:file "WheelsEncoder" :depends-on ("_package_WheelsEncoder"))
    (:file "_package_WheelsEncoder" :depends-on ("_package"))
    (:file "WheelsOmegaInfo" :depends-on ("_package_WheelsOmegaInfo"))
    (:file "_package_WheelsOmegaInfo" :depends-on ("_package"))
  ))