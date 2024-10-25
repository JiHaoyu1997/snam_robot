
(cl:in-package :asdf)

(defsystem "vpa_robot_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CrossInfo" :depends-on ("_package_CrossInfo"))
    (:file "_package_CrossInfo" :depends-on ("_package"))
  ))