; Auto-generated. Do not edit!


(cl:in-package vpa_robot_decision-msg)


;//! \htmlinclude RobotInterInfo.msg.html

(cl:defclass <RobotInterInfo> (roslisp-msg-protocol:ros-message)
  ((inter_1
    :reader inter_1
    :initarg :inter_1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (inter_2
    :reader inter_2
    :initarg :inter_2
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (inter_3
    :reader inter_3
    :initarg :inter_3
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (inter_4
    :reader inter_4
    :initarg :inter_4
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (inter_5
    :reader inter_5
    :initarg :inter_5
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass RobotInterInfo (<RobotInterInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotInterInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotInterInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_decision-msg:<RobotInterInfo> is deprecated: use vpa_robot_decision-msg:RobotInterInfo instead.")))

(cl:ensure-generic-function 'inter_1-val :lambda-list '(m))
(cl:defmethod inter_1-val ((m <RobotInterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:inter_1-val is deprecated.  Use vpa_robot_decision-msg:inter_1 instead.")
  (inter_1 m))

(cl:ensure-generic-function 'inter_2-val :lambda-list '(m))
(cl:defmethod inter_2-val ((m <RobotInterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:inter_2-val is deprecated.  Use vpa_robot_decision-msg:inter_2 instead.")
  (inter_2 m))

(cl:ensure-generic-function 'inter_3-val :lambda-list '(m))
(cl:defmethod inter_3-val ((m <RobotInterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:inter_3-val is deprecated.  Use vpa_robot_decision-msg:inter_3 instead.")
  (inter_3 m))

(cl:ensure-generic-function 'inter_4-val :lambda-list '(m))
(cl:defmethod inter_4-val ((m <RobotInterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:inter_4-val is deprecated.  Use vpa_robot_decision-msg:inter_4 instead.")
  (inter_4 m))

(cl:ensure-generic-function 'inter_5-val :lambda-list '(m))
(cl:defmethod inter_5-val ((m <RobotInterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:inter_5-val is deprecated.  Use vpa_robot_decision-msg:inter_5 instead.")
  (inter_5 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotInterInfo>) ostream)
  "Serializes a message object of type '<RobotInterInfo>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inter_1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'inter_1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inter_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'inter_2))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inter_3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'inter_3))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inter_4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'inter_4))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'inter_5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'inter_5))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotInterInfo>) istream)
  "Deserializes a message object of type '<RobotInterInfo>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inter_1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inter_1)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inter_2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inter_2)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inter_3) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inter_3)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inter_4) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inter_4)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'inter_5) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'inter_5)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotInterInfo>)))
  "Returns string type for a message object of type '<RobotInterInfo>"
  "vpa_robot_decision/RobotInterInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotInterInfo)))
  "Returns string type for a message object of type 'RobotInterInfo"
  "vpa_robot_decision/RobotInterInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotInterInfo>)))
  "Returns md5sum for a message object of type '<RobotInterInfo>"
  "c7e2780d75dbdc968ce987e67cfdc2df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotInterInfo)))
  "Returns md5sum for a message object of type 'RobotInterInfo"
  "c7e2780d75dbdc968ce987e67cfdc2df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotInterInfo>)))
  "Returns full string definition for message of type '<RobotInterInfo>"
  (cl:format cl:nil "# RobotInterInfo.msg~%~%int8[] inter_1~%int8[] inter_2~%int8[] inter_3~%int8[] inter_4~%int8[] inter_5~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotInterInfo)))
  "Returns full string definition for message of type 'RobotInterInfo"
  (cl:format cl:nil "# RobotInterInfo.msg~%~%int8[] inter_1~%int8[] inter_2~%int8[] inter_3~%int8[] inter_4~%int8[] inter_5~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotInterInfo>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inter_1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inter_2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inter_3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inter_4) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'inter_5) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotInterInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotInterInfo
    (cl:cons ':inter_1 (inter_1 msg))
    (cl:cons ':inter_2 (inter_2 msg))
    (cl:cons ':inter_3 (inter_3 msg))
    (cl:cons ':inter_4 (inter_4 msg))
    (cl:cons ':inter_5 (inter_5 msg))
))
