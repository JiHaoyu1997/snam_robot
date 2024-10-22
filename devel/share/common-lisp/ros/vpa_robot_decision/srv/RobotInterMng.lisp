; Auto-generated. Do not edit!


(cl:in-package vpa_robot_decision-srv)


;//! \htmlinclude RobotInterMng-request.msg.html

(cl:defclass <RobotInterMng-request> (roslisp-msg-protocol:ros-message)
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
   (from_inter_index
    :reader from_inter_index
    :initarg :from_inter_index
    :type cl:fixnum
    :initform 0)
   (to_inter_index
    :reader to_inter_index
    :initarg :to_inter_index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotInterMng-request (<RobotInterMng-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotInterMng-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotInterMng-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_decision-srv:<RobotInterMng-request> is deprecated: use vpa_robot_decision-srv:RobotInterMng-request instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotInterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:header-val is deprecated.  Use vpa_robot_decision-srv:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <RobotInterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:robot_name-val is deprecated.  Use vpa_robot_decision-srv:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'from_inter_index-val :lambda-list '(m))
(cl:defmethod from_inter_index-val ((m <RobotInterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:from_inter_index-val is deprecated.  Use vpa_robot_decision-srv:from_inter_index instead.")
  (from_inter_index m))

(cl:ensure-generic-function 'to_inter_index-val :lambda-list '(m))
(cl:defmethod to_inter_index-val ((m <RobotInterMng-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:to_inter_index-val is deprecated.  Use vpa_robot_decision-srv:to_inter_index instead.")
  (to_inter_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotInterMng-request>) ostream)
  "Serializes a message object of type '<RobotInterMng-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:let* ((signed (cl:slot-value msg 'from_inter_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'to_inter_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotInterMng-request>) istream)
  "Deserializes a message object of type '<RobotInterMng-request>"
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
      (cl:setf (cl:slot-value msg 'from_inter_index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'to_inter_index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotInterMng-request>)))
  "Returns string type for a service object of type '<RobotInterMng-request>"
  "vpa_robot_decision/RobotInterMngRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInterMng-request)))
  "Returns string type for a service object of type 'RobotInterMng-request"
  "vpa_robot_decision/RobotInterMngRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotInterMng-request>)))
  "Returns md5sum for a message object of type '<RobotInterMng-request>"
  "c0abed4502d76f69bbb34192950a96ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotInterMng-request)))
  "Returns md5sum for a message object of type 'RobotInterMng-request"
  "c0abed4502d76f69bbb34192950a96ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotInterMng-request>)))
  "Returns full string definition for message of type '<RobotInterMng-request>"
  (cl:format cl:nil "std_msgs/Header header~%string robot_name~%int8 from_inter_index~%int8 to_inter_index~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotInterMng-request)))
  "Returns full string definition for message of type 'RobotInterMng-request"
  (cl:format cl:nil "std_msgs/Header header~%string robot_name~%int8 from_inter_index~%int8 to_inter_index~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotInterMng-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotInterMng-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotInterMng-request
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':from_inter_index (from_inter_index msg))
    (cl:cons ':to_inter_index (to_inter_index msg))
))
;//! \htmlinclude RobotInterMng-response.msg.html

(cl:defclass <RobotInterMng-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass RobotInterMng-response (<RobotInterMng-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotInterMng-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotInterMng-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_decision-srv:<RobotInterMng-response> is deprecated: use vpa_robot_decision-srv:RobotInterMng-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <RobotInterMng-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:success-val is deprecated.  Use vpa_robot_decision-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <RobotInterMng-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-srv:message-val is deprecated.  Use vpa_robot_decision-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotInterMng-response>) ostream)
  "Serializes a message object of type '<RobotInterMng-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotInterMng-response>) istream)
  "Deserializes a message object of type '<RobotInterMng-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotInterMng-response>)))
  "Returns string type for a service object of type '<RobotInterMng-response>"
  "vpa_robot_decision/RobotInterMngResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInterMng-response)))
  "Returns string type for a service object of type 'RobotInterMng-response"
  "vpa_robot_decision/RobotInterMngResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotInterMng-response>)))
  "Returns md5sum for a message object of type '<RobotInterMng-response>"
  "c0abed4502d76f69bbb34192950a96ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotInterMng-response)))
  "Returns md5sum for a message object of type 'RobotInterMng-response"
  "c0abed4502d76f69bbb34192950a96ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotInterMng-response>)))
  "Returns full string definition for message of type '<RobotInterMng-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotInterMng-response)))
  "Returns full string definition for message of type 'RobotInterMng-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotInterMng-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotInterMng-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotInterMng-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotInterMng)))
  'RobotInterMng-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotInterMng)))
  'RobotInterMng-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInterMng)))
  "Returns string type for a service object of type '<RobotInterMng>"
  "vpa_robot_decision/RobotInterMng")