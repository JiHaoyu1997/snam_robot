; Auto-generated. Do not edit!


(cl:in-package vpa_robot_interface-msg)


;//! \htmlinclude WheelsOmegaInfo.msg.html

(cl:defclass <WheelsOmegaInfo> (roslisp-msg-protocol:ros-message)
  ((omega_left_ref
    :reader omega_left_ref
    :initarg :omega_left_ref
    :type cl:float
    :initform 0.0)
   (omega_left_sig
    :reader omega_left_sig
    :initarg :omega_left_sig
    :type cl:float
    :initform 0.0)
   (omega_right_ref
    :reader omega_right_ref
    :initarg :omega_right_ref
    :type cl:float
    :initform 0.0)
   (omega_right_sig
    :reader omega_right_sig
    :initarg :omega_right_sig
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelsOmegaInfo (<WheelsOmegaInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelsOmegaInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelsOmegaInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_interface-msg:<WheelsOmegaInfo> is deprecated: use vpa_robot_interface-msg:WheelsOmegaInfo instead.")))

(cl:ensure-generic-function 'omega_left_ref-val :lambda-list '(m))
(cl:defmethod omega_left_ref-val ((m <WheelsOmegaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:omega_left_ref-val is deprecated.  Use vpa_robot_interface-msg:omega_left_ref instead.")
  (omega_left_ref m))

(cl:ensure-generic-function 'omega_left_sig-val :lambda-list '(m))
(cl:defmethod omega_left_sig-val ((m <WheelsOmegaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:omega_left_sig-val is deprecated.  Use vpa_robot_interface-msg:omega_left_sig instead.")
  (omega_left_sig m))

(cl:ensure-generic-function 'omega_right_ref-val :lambda-list '(m))
(cl:defmethod omega_right_ref-val ((m <WheelsOmegaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:omega_right_ref-val is deprecated.  Use vpa_robot_interface-msg:omega_right_ref instead.")
  (omega_right_ref m))

(cl:ensure-generic-function 'omega_right_sig-val :lambda-list '(m))
(cl:defmethod omega_right_sig-val ((m <WheelsOmegaInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_interface-msg:omega_right_sig-val is deprecated.  Use vpa_robot_interface-msg:omega_right_sig instead.")
  (omega_right_sig m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelsOmegaInfo>) ostream)
  "Serializes a message object of type '<WheelsOmegaInfo>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'omega_left_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'omega_left_sig))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'omega_right_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'omega_right_sig))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelsOmegaInfo>) istream)
  "Deserializes a message object of type '<WheelsOmegaInfo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'omega_left_ref) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'omega_left_sig) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'omega_right_ref) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'omega_right_sig) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelsOmegaInfo>)))
  "Returns string type for a message object of type '<WheelsOmegaInfo>"
  "vpa_robot_interface/WheelsOmegaInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelsOmegaInfo)))
  "Returns string type for a message object of type 'WheelsOmegaInfo"
  "vpa_robot_interface/WheelsOmegaInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelsOmegaInfo>)))
  "Returns md5sum for a message object of type '<WheelsOmegaInfo>"
  "ba80e47329fa708db7e2fff0c002fd00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelsOmegaInfo)))
  "Returns md5sum for a message object of type 'WheelsOmegaInfo"
  "ba80e47329fa708db7e2fff0c002fd00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelsOmegaInfo>)))
  "Returns full string definition for message of type '<WheelsOmegaInfo>"
  (cl:format cl:nil "float32 omega_left_ref~%float32 omega_left_sig~%float32 omega_right_ref~%float32 omega_right_sig~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelsOmegaInfo)))
  "Returns full string definition for message of type 'WheelsOmegaInfo"
  (cl:format cl:nil "float32 omega_left_ref~%float32 omega_left_sig~%float32 omega_right_ref~%float32 omega_right_sig~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelsOmegaInfo>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelsOmegaInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelsOmegaInfo
    (cl:cons ':omega_left_ref (omega_left_ref msg))
    (cl:cons ':omega_left_sig (omega_left_sig msg))
    (cl:cons ':omega_right_ref (omega_right_ref msg))
    (cl:cons ':omega_right_sig (omega_right_sig msg))
))
