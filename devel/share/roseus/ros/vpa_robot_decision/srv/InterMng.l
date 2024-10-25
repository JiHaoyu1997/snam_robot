;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_decision::InterMng)
  (if (not (find-package "VPA_ROBOT_DECISION"))
    (make-package "VPA_ROBOT_DECISION"))
  (shadow 'InterMng (find-package "VPA_ROBOT_DECISION")))
(unless (find-package "VPA_ROBOT_DECISION::INTERMNG")
  (make-package "VPA_ROBOT_DECISION::INTERMNG"))
(unless (find-package "VPA_ROBOT_DECISION::INTERMNGREQUEST")
  (make-package "VPA_ROBOT_DECISION::INTERMNGREQUEST"))
(unless (find-package "VPA_ROBOT_DECISION::INTERMNGRESPONSE")
  (make-package "VPA_ROBOT_DECISION::INTERMNGRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass vpa_robot_decision::InterMngRequest
  :super ros::object
  :slots (_header _robot_name _last_inter_id _curr_inter_id _robot_info ))

(defmethod vpa_robot_decision::InterMngRequest
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:last_inter_id __last_inter_id) 0)
    ((:curr_inter_id __curr_inter_id) 0)
    ((:robot_info __robot_info) (instance vpa_robot_decision::RobotInfo :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _last_inter_id (round __last_inter_id))
   (setq _curr_inter_id (round __curr_inter_id))
   (setq _robot_info __robot_info)
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
  (:last_inter_id
   (&optional __last_inter_id)
   (if __last_inter_id (setq _last_inter_id __last_inter_id)) _last_inter_id)
  (:curr_inter_id
   (&optional __curr_inter_id)
   (if __curr_inter_id (setq _curr_inter_id __curr_inter_id)) _curr_inter_id)
  (:robot_info
   (&rest __robot_info)
   (if (keywordp (car __robot_info))
       (send* _robot_info __robot_info)
     (progn
       (if __robot_info (setq _robot_info (car __robot_info)))
       _robot_info)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; int8 _last_inter_id
    1
    ;; int8 _curr_inter_id
    1
    ;; vpa_robot_decision/RobotInfo _robot_info
    (send _robot_info :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; int8 _last_inter_id
       (write-byte _last_inter_id s)
     ;; int8 _curr_inter_id
       (write-byte _curr_inter_id s)
     ;; vpa_robot_decision/RobotInfo _robot_info
       (send _robot_info :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _last_inter_id
     (setq _last_inter_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _last_inter_id 127) (setq _last_inter_id (- _last_inter_id 256)))
   ;; int8 _curr_inter_id
     (setq _curr_inter_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _curr_inter_id 127) (setq _curr_inter_id (- _curr_inter_id 256)))
   ;; vpa_robot_decision/RobotInfo _robot_info
     (send _robot_info :deserialize buf ptr-) (incf ptr- (send _robot_info :serialization-length))
   ;;
   self)
  )

(defclass vpa_robot_decision::InterMngResponse
  :super ros::object
  :slots (_success _message ))

(defmethod vpa_robot_decision::InterMngResponse
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

(defclass vpa_robot_decision::InterMng
  :super ros::object
  :slots ())

(setf (get vpa_robot_decision::InterMng :md5sum-) "6a2d2bfc797d5c6e5fd765f81f56acfb")
(setf (get vpa_robot_decision::InterMng :datatype-) "vpa_robot_decision/InterMng")
(setf (get vpa_robot_decision::InterMng :request) vpa_robot_decision::InterMngRequest)
(setf (get vpa_robot_decision::InterMng :response) vpa_robot_decision::InterMngResponse)

(defmethod vpa_robot_decision::InterMngRequest
  (:response () (instance vpa_robot_decision::InterMngResponse :init)))

(setf (get vpa_robot_decision::InterMngRequest :md5sum-) "6a2d2bfc797d5c6e5fd765f81f56acfb")
(setf (get vpa_robot_decision::InterMngRequest :datatype-) "vpa_robot_decision/InterMngRequest")
(setf (get vpa_robot_decision::InterMngRequest :definition-)
      "std_msgs/Header header
string robot_name
int8 last_inter_id
int8 curr_inter_id
RobotInfo robot_info

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

================================================================================
MSG: vpa_robot_decision/RobotInfo
string  robot_name
int8    robot_id
float32 robot_a  # Acceleration
float32 robot_v  # Velocity
float32 robot_p  # Position
float32 robot_enter_time
float32 robot_arrive_cp_time
float32 robot_exit_time
---
bool success
string message
")

(setf (get vpa_robot_decision::InterMngResponse :md5sum-) "6a2d2bfc797d5c6e5fd765f81f56acfb")
(setf (get vpa_robot_decision::InterMngResponse :datatype-) "vpa_robot_decision/InterMngResponse")
(setf (get vpa_robot_decision::InterMngResponse :definition-)
      "std_msgs/Header header
string robot_name
int8 last_inter_id
int8 curr_inter_id
RobotInfo robot_info

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

================================================================================
MSG: vpa_robot_decision/RobotInfo
string  robot_name
int8    robot_id
float32 robot_a  # Acceleration
float32 robot_v  # Velocity
float32 robot_p  # Position
float32 robot_enter_time
float32 robot_arrive_cp_time
float32 robot_exit_time
---
bool success
string message
")



(provide :vpa_robot_decision/InterMng "6a2d2bfc797d5c6e5fd765f81f56acfb")


