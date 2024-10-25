
(cl:in-package :asdf)

(defsystem "vpa_robot_decision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
               :vpa_robot_decision-msg
)
  :components ((:file "_package")
    (:file "InterMng" :depends-on ("_package_InterMng"))
    (:file "_package_InterMng" :depends-on ("_package"))
  ))