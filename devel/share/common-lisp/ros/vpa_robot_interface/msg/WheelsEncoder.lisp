; Auto-generated. Do not edit!


(cl:in-package vpa_robot_interface-msg)


;//! \htmlinclude WheelsEncoder.msg.html

(cl:defclass <WheelsEncoder> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (omega_left
    :reader omega_left
    :initarg :omega_left
    :type cl:float
    :initform 0.0)
   (omega_right
    :reader omega_right
    :initarg :omega_right
    :type cl:float
    :initform 0.0)
   (left_ticks
    :reader left_ticks
    :initarg :left_ticks
    :type cl:integer
    :initform 0)
   (right_ticks
    :reader right_ticks
    :initarg :right_ticks
    :type cl:integer
    :initform 0))
)

(cl:defclass WheelsEncoder (<WheelsEncoder>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelsEncoder>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelsEncoder)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_interface-msg:<WheelsEncoder> is deprecated: use vpa_robot_interface-msg:WheelsEncoder instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WheelsEncoder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:header-val is deprecated.  Use vpa_robot_interface-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'omega_left-val :lambda-list '(m))
(cl:defmethod omega_left-val ((m <WheelsEncoder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:omega_left-val is deprecated.  Use vpa_robot_interface-msg:omega_left instead.")
  (omega_left m))

(cl:ensure-generic-function 'omega_right-val :lambda-list '(m))
(cl:defmethod omega_right-val ((m <WheelsEncoder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:omega_right-val is deprecated.  Use vpa_robot_interface-msg:omega_right instead.")
  (omega_right m))

(cl:ensure-generic-function 'left_ticks-val :lambda-list '(m))
(cl:defmethod left_ticks-val ((m <WheelsEncoder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:left_ticks-val is deprecated.  Use vpa_robot_interface-msg:left_ticks instead.")
  (left_ticks m))

(cl:ensure-generic-function 'right_ticks-val :lambda-list '(m))
(cl:defmethod right_ticks-val ((m <WheelsEncoder>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:right_ticks-val is deprecated.  Use vpa_robot_interface-msg:right_ticks instead.")
  (right_ticks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelsEncoder>) ostream)
  "Serializes a message object of type '<WheelsEncoder>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'omega_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'omega_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'left_ticks)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_ticks)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelsEncoder>) istream)
  "Deserializes a message object of type '<WheelsEncoder>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'omega_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'omega_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_ticks) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_ticks) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelsEncoder>)))
  "Returns string type for a message object of type '<WheelsEncoder>"
  "vpa_robot_interface/WheelsEncoder")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelsEncoder)))
  "Returns string type for a message object of type 'WheelsEncoder"
  "vpa_robot_interface/WheelsEncoder")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelsEncoder>)))
  "Returns md5sum for a message object of type '<WheelsEncoder>"
  "6d20b5ac15310df1d610c29199f12461")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelsEncoder)))
  "Returns md5sum for a message object of type 'WheelsEncoder"
  "6d20b5ac15310df1d610c29199f12461")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelsEncoder>)))
  "Returns full string definition for message of type '<WheelsEncoder>"
  (cl:format cl:nil "std_msgs/Header header~%float32 omega_left~%float32 omega_right~%int32 left_ticks~%int32 right_ticks~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelsEncoder)))
  "Returns full string definition for message of type 'WheelsEncoder"
  (cl:format cl:nil "std_msgs/Header header~%float32 omega_left~%float32 omega_right~%int32 left_ticks~%int32 right_ticks~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelsEncoder>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelsEncoder>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelsEncoder
    (cl:cons ':header (header msg))
    (cl:cons ':omega_left (omega_left msg))
    (cl:cons ':omega_right (omega_right msg))
    (cl:cons ':left_ticks (left_ticks msg))
    (cl:cons ':right_ticks (right_ticks msg))
))
