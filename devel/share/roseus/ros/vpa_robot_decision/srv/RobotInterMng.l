;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_decision::RobotInterMng)
  (if (not (find-package "VPA_ROBOT_DECISION"))
    (make-package "VPA_ROBOT_DECISION"))
  (shadow 'RobotInterMng (find-package "VPA_ROBOT_DECISION")))
(unless (find-package "VPA_ROBOT_DECISION::ROBOTINTERMNG")
  (make-package "VPA_ROBOT_DECISION::ROBOTINTERMNG"))
(unless (find-package "VPA_ROBOT_DECISION::ROBOTINTERMNGREQUEST")
  (make-package "VPA_ROBOT_DECISION::ROBOTINTERMNGREQUEST"))
(unless (find-package "VPA_ROBOT_DECISION::ROBOTINTERMNGRESPONSE")
  (make-package "VPA_ROBOT_DECISION::ROBOTINTERMNGRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass vpa_robot_decision::RobotInterMngRequest
  :super ros::object
  :slots (_header _robot_name _from_inter_index _to_inter_index ))

(defmethod vpa_robot_decision::RobotInterMngRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:from_inter_index __from_inter_index) 0)
    ((:to_inter_index __to_inter_index) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _from_inter_index (round __from_inter_index))
   (setq _to_inter_index (round __to_inter_index))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:from_inter_index
   (&optional __from_inter_index)
   (if __from_inter_index (setq _from_inter_index __from_inter_index)) _from_inter_index)
  (:to_inter_index
   (&optional __to_inter_index)
   (if __to_inter_index (setq _to_inter_index __to_inter_index)) _to_inter_index)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; int8 _from_inter_index
    1
    ;; int8 _to_inter_index
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; int8 _from_inter_index
       (write-byte _from_inter_index s)
     ;; int8 _to_inter_index
       (write-byte _to_inter_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _from_inter_index
     (setq _from_inter_index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _from_inter_index 127) (setq _from_inter_index (- _from_inter_index 256)))
   ;; int8 _to_inter_index
     (setq _to_inter_index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _to_inter_index 127) (setq _to_inter_index (- _to_inter_index 256)))
   ;;
   self)
  )

(defclass vpa_robot_decision::RobotInterMngResponse
  :super ros::object
  :slots (_success _message ))

(defmethod vpa_robot_decision::RobotInterMngResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass vpa_robot_decision::RobotInterMng
  :super ros::object
  :slots ())

(setf (get vpa_robot_decision::RobotInterMng :md5sum-) "c0abed4502d76f69bbb34192950a96ca")
(setf (get vpa_robot_decision::RobotInterMng :datatype-) "vpa_robot_decision/RobotInterMng")
(setf (get vpa_robot_decision::RobotInterMng :request) vpa_robot_decision::RobotInterMngRequest)
(setf (get vpa_robot_decision::RobotInterMng :response) vpa_robot_decision::RobotInterMngResponse)

(defmethod vpa_robot_decision::RobotInterMngRequest
  (:response () (instance vpa_robot_decision::RobotInterMngResponse :init)))

(setf (get vpa_robot_decision::RobotInterMngRequest :md5sum-) "c0abed4502d76f69bbb34192950a96ca")
(setf (get vpa_robot_decision::RobotInterMngRequest :datatype-) "vpa_robot_decision/RobotInterMngRequest")
(setf (get vpa_robot_decision::RobotInterMngRequest :definition-)
      "std_msgs/Header header
string robot_name
int8 from_inter_index
int8 to_inter_index

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
---
bool success
string message
")

(setf (get vpa_robot_decision::RobotInterMngResponse :md5sum-) "c0abed4502d76f69bbb34192950a96ca")
(setf (get vpa_robot_decision::RobotInterMngResponse :datatype-) "vpa_robot_decision/RobotInterMngResponse")
(setf (get vpa_robot_decision::RobotInterMngResponse :definition-)
      "std_msgs/Header header
string robot_name
int8 from_inter_index
int8 to_inter_index

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
---
bool success
string message
")



(provide :vpa_robot_decision/RobotInterMng "c0abed4502d76f69bbb34192950a96ca")


