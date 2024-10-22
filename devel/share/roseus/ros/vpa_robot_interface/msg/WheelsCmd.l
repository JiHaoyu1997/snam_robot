;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_interface::WheelsCmd)
  (if (not (find-package "VPA_ROBOT_INTERFACE"))
    (make-package "VPA_ROBOT_INTERFACE"))
  (shadow 'WheelsCmd (find-package "VPA_ROBOT_INTERFACE")))
(unless (find-package "VPA_ROBOT_INTERFACE::WHEELSCMD")
  (make-package "VPA_ROBOT_INTERFACE::WHEELSCMD"))

(in-package "ROS")
;;//! \htmlinclude WheelsCmd.msg.html


(defclass vpa_robot_interface::WheelsCmd
  :super ros::object
  :slots (_vel_left _vel_right _throttle_left _throttle_right ))

(defmethod vpa_robot_interface::WheelsCmd
  (:init
   (&key
    ((:vel_left __vel_left) 0.0)
    ((:vel_right __vel_right) 0.0)
    ((:throttle_left __throttle_left) 0.0)
    ((:throttle_right __throttle_right) 0.0)
    )
   (send-super :init)
   (setq _vel_left (float __vel_left))
   (setq _vel_right (float __vel_right))
   (setq _throttle_left (float __throttle_left))
   (setq _throttle_right (float __throttle_right))
   self)
  (:vel_left
   (&optional __vel_left)
   (if __vel_left (setq _vel_left __vel_left)) _vel_left)
  (:vel_right
   (&optional __vel_right)
   (if __vel_right (setq _vel_right __vel_right)) _vel_right)
  (:throttle_left
   (&optional __throttle_left)
   (if __throttle_left (setq _throttle_left __throttle_left)) _throttle_left)
  (:throttle_right
   (&optional __throttle_right)
   (if __throttle_right (setq _throttle_right __throttle_right)) _throttle_right)
  (:serialization-length
   ()
   (+
    ;; float32 _vel_left
    4
    ;; float32 _vel_right
    4
    ;; float32 _throttle_left
    4
    ;; float32 _throttle_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _vel_left
       (sys::poke _vel_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vel_right
       (sys::poke _vel_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _throttle_left
       (sys::poke _throttle_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _throttle_right
       (sys::poke _throttle_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _vel_left
     (setq _vel_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vel_right
     (setq _vel_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _throttle_left
     (setq _throttle_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _throttle_right
     (setq _throttle_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vpa_robot_interface::WheelsCmd :md5sum-) "55b8120aed29e6f14c585476240d3cac")
(setf (get vpa_robot_interface::WheelsCmd :datatype-) "vpa_robot_interface/WheelsCmd")
(setf (get vpa_robot_interface::WheelsCmd :definition-)
      "float32 vel_left
float32 vel_right
float32 throttle_left
float32 throttle_right
")



(provide :vpa_robot_interface/WheelsCmd "55b8120aed29e6f14c585476240d3cac")


