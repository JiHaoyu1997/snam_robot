;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_interface::LeftWheelT)
  (if (not (find-package "VPA_ROBOT_INTERFACE"))
    (make-package "VPA_ROBOT_INTERFACE"))
  (shadow 'LeftWheelT (find-package "VPA_ROBOT_INTERFACE")))
(unless (find-package "VPA_ROBOT_INTERFACE::LEFTWHEELT")
  (make-package "VPA_ROBOT_INTERFACE::LEFTWHEELT"))

(in-package "ROS")
;;//! \htmlinclude LeftWheelT.msg.html


(defclass vpa_robot_interface::LeftWheelT
  :super ros::object
  :slots (_throttle_left ))

(defmethod vpa_robot_interface::LeftWheelT
  (:init
   (&key
    ((:throttle_left __throttle_left) 0.0)
    )
   (send-super :init)
   (setq _throttle_left (float __throttle_left))
   self)
  (:throttle_left
   (&optional __throttle_left)
   (if __throttle_left (setq _throttle_left __throttle_left)) _throttle_left)
  (:serialization-length
   ()
   (+
    ;; float32 _throttle_left
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _throttle_left
       (sys::poke _throttle_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _throttle_left
     (setq _throttle_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vpa_robot_interface::LeftWheelT :md5sum-) "928a92cec7d033a951a07e21e36af4f5")
(setf (get vpa_robot_interface::LeftWheelT :datatype-) "vpa_robot_interface/LeftWheelT")
(setf (get vpa_robot_interface::LeftWheelT :definition-)
      "float32 throttle_left
")



(provide :vpa_robot_interface/LeftWheelT "928a92cec7d033a951a07e21e36af4f5")


