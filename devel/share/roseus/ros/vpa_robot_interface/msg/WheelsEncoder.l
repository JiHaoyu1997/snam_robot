;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_interface::WheelsEncoder)
  (if (not (find-package "VPA_ROBOT_INTERFACE"))
    (make-package "VPA_ROBOT_INTERFACE"))
  (shadow 'WheelsEncoder (find-package "VPA_ROBOT_INTERFACE")))
(unless (find-package "VPA_ROBOT_INTERFACE::WHEELSENCODER")
  (make-package "VPA_ROBOT_INTERFACE::WHEELSENCODER"))

(in-package "ROS")
;;//! \htmlinclude WheelsEncoder.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass vpa_robot_interface::WheelsEncoder
  :super ros::object
  :slots (_header _omega_left _omega_right _left_ticks _right_ticks ))

(defmethod vpa_robot_interface::WheelsEncoder
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:omega_left __omega_left) 0.0)
    ((:omega_right __omega_right) 0.0)
    ((:left_ticks __left_ticks) 0)
    ((:right_ticks __right_ticks) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _omega_left (float __omega_left))
   (setq _omega_right (float __omega_right))
   (setq _left_ticks (round __left_ticks))
   (setq _right_ticks (round __right_ticks))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:omega_left
   (&optional __omega_left)
   (if __omega_left (setq _omega_left __omega_left)) _omega_left)
  (:omega_right
   (&optional __omega_right)
   (if __omega_right (setq _omega_right __omega_right)) _omega_right)
  (:left_ticks
   (&optional __left_ticks)
   (if __left_ticks (setq _left_ticks __left_ticks)) _left_ticks)
  (:right_ticks
   (&optional __right_ticks)
   (if __right_ticks (setq _right_ticks __right_ticks)) _right_ticks)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _omega_left
    4
    ;; float32 _omega_right
    4
    ;; int32 _left_ticks
    4
    ;; int32 _right_ticks
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _omega_left
       (sys::poke _omega_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _omega_right
       (sys::poke _omega_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _left_ticks
       (write-long _left_ticks s)
     ;; int32 _right_ticks
       (write-long _right_ticks s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _omega_left
     (setq _omega_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _omega_right
     (setq _omega_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _left_ticks
     (setq _left_ticks (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _right_ticks
     (setq _right_ticks (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vpa_robot_interface::WheelsEncoder :md5sum-) "6d20b5ac15310df1d610c29199f12461")
(setf (get vpa_robot_interface::WheelsEncoder :datatype-) "vpa_robot_interface/WheelsEncoder")
(setf (get vpa_robot_interface::WheelsEncoder :definition-)
      "std_msgs/Header header
float32 omega_left
float32 omega_right
int32 left_ticks
int32 right_ticks
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :vpa_robot_interface/WheelsEncoder "6d20b5ac15310df1d610c29199f12461")


