; Auto-generated. Do not edit!


(cl:in-package vpa_robot_vision-msg)


;//! \htmlinclude CrossInfo.msg.html

(cl:defclass <CrossInfo> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform "")
   (cross
    :reader cross
    :initarg :cross
    :type cl:boolean
    :initform cl:nil)
   (last_inter_id
    :reader last_inter_id
    :initarg :last_inter_id
    :type cl:fixnum
    :initform 0)
   (local_inter_id
    :reader local_inter_id
    :initarg :local_inter_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CrossInfo (<CrossInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CrossInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CrossInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_vision-msg:<CrossInfo> is deprecated: use vpa_robot_vision-msg:CrossInfo instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <CrossInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_vision-msg:robot_name-val is deprecated.  Use vpa_robot_vision-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'cross-val :lambda-list '(m))
(cl:defmethod cross-val ((m <CrossInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_vision-msg:cross-val is deprecated.  Use vpa_robot_vision-msg:cross instead.")
  (cross m))

(cl:ensure-generic-function 'last_inter_id-val :lambda-list '(m))
(cl:defmethod last_inter_id-val ((m <CrossInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_vision-msg:last_inter_id-val is deprecated.  Use vpa_robot_vision-msg:last_inter_id instead.")
  (last_inter_id m))

(cl:ensure-generic-function 'local_inter_id-val :lambda-list '(m))
(cl:defmethod local_inter_id-val ((m <CrossInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_vision-msg:local_inter_id-val is deprecated.  Use vpa_robot_vision-msg:local_inter_id instead.")
  (local_inter_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CrossInfo>) ostream)
  "Serializes a message object of type '<CrossInfo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cross) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'last_inter_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'local_inter_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CrossInfo>) istream)
  "Deserializes a message object of type '<CrossInfo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'cross) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'last_inter_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'local_inter_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CrossInfo>)))
  "Returns string type for a message object of type '<CrossInfo>"
  "vpa_robot_vision/CrossInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CrossInfo)))
  "Returns string type for a message object of type 'CrossInfo"
  "vpa_robot_vision/CrossInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CrossInfo>)))
  "Returns md5sum for a message object of type '<CrossInfo>"
  "abc28f0d82de76aab08a8f05250b1372")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CrossInfo)))
  "Returns md5sum for a message object of type 'CrossInfo"
  "abc28f0d82de76aab08a8f05250b1372")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CrossInfo>)))
  "Returns full string definition for message of type '<CrossInfo>"
  (cl:format cl:nil "string robot_name         # Name of the robot~%bool cross                # Indicates if the robot has crossed~%int8 last_inter_id       # ID of the last intersection~%int8 local_inter_id      # ID of the current (local) intersection~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CrossInfo)))
  "Returns full string definition for message of type 'CrossInfo"
  (cl:format cl:nil "string robot_name         # Name of the robot~%bool cross                # Indicates if the robot has crossed~%int8 last_inter_id       # ID of the last intersection~%int8 local_inter_id      # ID of the current (local) intersection~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CrossInfo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robot_name))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CrossInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'CrossInfo
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':cross (cross msg))
    (cl:cons ':last_inter_id (last_inter_id msg))
    (cl:cons ':local_inter_id (local_inter_id msg))
))
