; Auto-generated. Do not edit!


(cl:in-package vpa_robot_decision-srv)


;//! \htmlinclude InterMng-request.msg.html

(cl:defclass <InterMng-request> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform "")
   (last_inter_id
    :reader last_inter_id
    :initarg :last_inter_id
    :type cl:fixnum
    :initform 0)
   (curr_inter_id
    :reader curr_inter_id
    :initarg :curr_inter_id
    :type cl:fixnum
    :initform 0)
   (robot_info
    :reader robot_info
    :initarg :robot_info
    :type vpa_robot_decision-msg:RobotInfo
    :initform (cl:make-instance 'vpa_robot_decision-msg:RobotInfo)))
)

(cl:defclass InterMng-request (<InterMng-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterMng-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterMng-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_decision-srv:<InterMng-request> is deprecated: use vpa_robot_decision-srv:InterMng-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <InterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:header-val is deprecated.  Use vpa_robot_decision-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <InterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:robot_name-val is deprecated.  Use vpa_robot_decision-srv:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'last_inter_id-val :lambda-list '(m))
(cl:defmethod last_inter_id-val ((m <InterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:last_inter_id-val is deprecated.  Use vpa_robot_decision-srv:last_inter_id instead.")
  (last_inter_id m))

(cl:ensure-generic-function 'curr_inter_id-val :lambda-list '(m))
(cl:defmethod curr_inter_id-val ((m <InterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:curr_inter_id-val is deprecated.  Use vpa_robot_decision-srv:curr_inter_id instead.")
  (curr_inter_id m))

(cl:ensure-generic-function 'robot_info-val :lambda-list '(m))
(cl:defmethod robot_info-val ((m <InterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:robot_info-val is deprecated.  Use vpa_robot_decision-srv:robot_info instead.")
  (robot_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterMng-request>) ostream)
  "Serializes a message object of type '<InterMng-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:let* ((signed (cl:slot-value msg 'last_inter_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'curr_inter_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_info) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterMng-request>) istream)
  "Deserializes a message object of type '<InterMng-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'last_inter_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'curr_inter_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_info) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterMng-request>)))
  "Returns string type for a service object of type '<InterMng-request>"
  "vpa_robot_decision/InterMngRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterMng-request)))
  "Returns string type for a service object of type 'InterMng-request"
  "vpa_robot_decision/InterMngRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterMng-request>)))
  "Returns md5sum for a message object of type '<InterMng-request>"
  "6a2d2bfc797d5c6e5fd765f81f56acfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterMng-request)))
  "Returns md5sum for a message object of type 'InterMng-request"
  "6a2d2bfc797d5c6e5fd765f81f56acfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterMng-request>)))
  "Returns full string definition for message of type '<InterMng-request>"
  (cl:format cl:nil "std_msgs/Header header~%string robot_name~%int8 last_inter_id~%int8 curr_inter_id~%RobotInfo robot_info~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: vpa_robot_decision/RobotInfo~%string  robot_name~%int8    robot_id~%float32 robot_a  # Acceleration~%float32 robot_v  # Velocity~%float32 robot_p  # Position~%float32 robot_enter_time~%float32 robot_arrive_cp_time~%float32 robot_exit_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterMng-request)))
  "Returns full string definition for message of type 'InterMng-request"
  (cl:format cl:nil "std_msgs/Header header~%string robot_name~%int8 last_inter_id~%int8 curr_inter_id~%RobotInfo robot_info~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: vpa_robot_decision/RobotInfo~%string  robot_name~%int8    robot_id~%float32 robot_a  # Acceleration~%float32 robot_v  # Velocity~%float32 robot_p  # Position~%float32 robot_enter_time~%float32 robot_arrive_cp_time~%float32 robot_exit_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterMng-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_info))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterMng-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InterMng-request
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':last_inter_id (last_inter_id msg))
    (cl:cons ':curr_inter_id (curr_inter_id msg))
    (cl:cons ':robot_info (robot_info msg))
))
;//! \htmlinclude InterMng-response.msg.html

(cl:defclass <InterMng-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass InterMng-response (<InterMng-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterMng-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterMng-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_decision-srv:<InterMng-response> is deprecated: use vpa_robot_decision-srv:InterMng-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InterMng-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:success-val is deprecated.  Use vpa_robot_decision-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <InterMng-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:message-val is deprecated.  Use vpa_robot_decision-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterMng-response>) ostream)
  "Serializes a message object of type '<InterMng-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterMng-response>) istream)
  "Deserializes a message object of type '<InterMng-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterMng-response>)))
  "Returns string type for a service object of type '<InterMng-response>"
  "vpa_robot_decision/InterMngResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterMng-response)))
  "Returns string type for a service object of type 'InterMng-response"
  "vpa_robot_decision/InterMngResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterMng-response>)))
  "Returns md5sum for a message object of type '<InterMng-response>"
  "6a2d2bfc797d5c6e5fd765f81f56acfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterMng-response)))
  "Returns md5sum for a message object of type 'InterMng-response"
  "6a2d2bfc797d5c6e5fd765f81f56acfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterMng-response>)))
  "Returns full string definition for message of type '<InterMng-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterMng-response)))
  "Returns full string definition for message of type 'InterMng-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterMng-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterMng-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InterMng-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InterMng)))
  'InterMng-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InterMng)))
  'InterMng-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterMng)))
  "Returns string type for a service object of type '<InterMng>"
  "vpa_robot_decision/InterMng")