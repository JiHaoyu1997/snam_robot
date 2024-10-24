; Auto-generated. Do not edit!


(cl:in-package vpa_robot_interface-msg)


;//! \htmlinclude WheelsCmd.msg.html

(cl:defclass <WheelsCmd> (roslisp-msg-protocol:ros-message)
  ((vel_left
    :reader vel_left
    :initarg :vel_left
    :type cl:float
    :initform 0.0)
   (vel_right
    :reader vel_right
    :initarg :vel_right
    :type cl:float
    :initform 0.0)
   (throttle_left
    :reader throttle_left
    :initarg :throttle_left
    :type cl:float
    :initform 0.0)
   (throttle_right
    :reader throttle_right
    :initarg :throttle_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelsCmd (<WheelsCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelsCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelsCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_interface-msg:<WheelsCmd> is deprecated: use vpa_robot_interface-msg:WheelsCmd instead.")))

(cl:ensure-generic-function 'vel_left-val :lambda-list '(m))
(cl:defmethod vel_left-val ((m <WheelsCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:vel_left-val is deprecated.  Use vpa_robot_interface-msg:vel_left instead.")
  (vel_left m))

(cl:ensure-generic-function 'vel_right-val :lambda-list '(m))
(cl:defmethod vel_right-val ((m <WheelsCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:vel_right-val is deprecated.  Use vpa_robot_interface-msg:vel_right instead.")
  (vel_right m))

(cl:ensure-generic-function 'throttle_left-val :lambda-list '(m))
(cl:defmethod throttle_left-val ((m <WheelsCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:throttle_left-val is deprecated.  Use vpa_robot_interface-msg:throttle_left instead.")
  (throttle_left m))

(cl:ensure-generic-function 'throttle_right-val :lambda-list '(m))
(cl:defmethod throttle_right-val ((m <WheelsCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:throttle_right-val is deprecated.  Use vpa_robot_interface-msg:throttle_right instead.")
  (throttle_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelsCmd>) ostream)
  "Serializes a message object of type '<WheelsCmd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'throttle_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'throttle_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelsCmd>) istream)
  "Deserializes a message object of type '<WheelsCmd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'throttle_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelsCmd>)))
  "Returns string type for a message object of type '<WheelsCmd>"
  "vpa_robot_interface/WheelsCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelsCmd)))
  "Returns string type for a message object of type 'WheelsCmd"
  "vpa_robot_interface/WheelsCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelsCmd>)))
  "Returns md5sum for a message object of type '<WheelsCmd>"
  "55b8120aed29e6f14c585476240d3cac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelsCmd)))
  "Returns md5sum for a message object of type 'WheelsCmd"
  "55b8120aed29e6f14c585476240d3cac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelsCmd>)))
  "Returns full string definition for message of type '<WheelsCmd>"
  (cl:format cl:nil "float32 vel_left~%float32 vel_right~%float32 throttle_left~%float32 throttle_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelsCmd)))
  "Returns full string definition for message of type 'WheelsCmd"
  (cl:format cl:nil "float32 vel_left~%float32 vel_right~%float32 throttle_left~%float32 throttle_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelsCmd>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelsCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelsCmd
    (cl:cons ':vel_left (vel_left msg))
    (cl:cons ':vel_right (vel_right msg))
    (cl:cons ':throttle_left (throttle_left msg))
    (cl:cons ':throttle_right (throttle_right msg))
))
