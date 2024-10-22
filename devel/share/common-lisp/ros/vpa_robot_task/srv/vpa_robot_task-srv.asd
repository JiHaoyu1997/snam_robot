
(cl:in-package :asdf)

(defsystem "vpa_robot_task-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AssignTask" :depends-on ("_package_AssignTask"))
    (:file "_package_AssignTask" :depends-on ("_package"))
  ))