; Auto-generated. Do not edit!


(cl:in-package vpa_robot_interface-msg)


;//! \htmlinclude LeftWheelT.msg.html

(cl:defclass <LeftWheelT> (roslisp-msg-protocol:ros-message)
  ((throttle_left
    :reader throttle_left
    :initarg :throttle_left
    :type cl:float
    :initform 0.0))
)

(cl:defclass LeftWheelT (<LeftWheelT>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LeftWheelT>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LeftWheelT)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_interface-msg:<LeftWheelT> is deprecated: use vpa_robot_interface-msg:LeftWheelT instead.")))

(cl:ensure-generic-function 'throttle_left-val :lambda-list '(m))
(cl:defmethod throttle_left-val ((m <LeftWheelT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:throttle_left-val is deprecated.  Use vpa_robot_interface-msg:throttle_left instead.")
  (throttle_left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LeftWheelT>) ostream)
  "Serializes a message object of type '<LeftWheelT>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'throttle_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LeftWheelT>) istream)
  "Deserializes a message object of type '<LeftWheelT>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle_left) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LeftWheelT>)))
  "Returns string type for a message object of type '<LeftWheelT>"
  "vpa_robot_interface/LeftWheelT")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LeftWheelT)))
  "Returns string type for a message object of type 'LeftWheelT"
  "vpa_robot_interface/LeftWheelT")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LeftWheelT>)))
  "Returns md5sum for a message object of type '<LeftWheelT>"
  "928a92cec7d033a951a07e21e36af4f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LeftWheelT)))
  "Returns md5sum for a message object of type 'LeftWheelT"
  "928a92cec7d033a951a07e21e36af4f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LeftWheelT>)))
  "Returns full string definition for message of type '<LeftWheelT>"
  (cl:format cl:nil "float32 throttle_left~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LeftWheelT)))
  "Returns full string definition for message of type 'LeftWheelT"
  (cl:format cl:nil "float32 throttle_left~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LeftWheelT>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LeftWheelT>))
  "Converts a ROS message object to a list"
  (cl:list 'LeftWheelT
    (cl:cons ':throttle_left (throttle_left msg))
))
