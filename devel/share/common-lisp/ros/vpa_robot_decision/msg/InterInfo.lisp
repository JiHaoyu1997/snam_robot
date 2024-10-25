; Auto-generated. Do not edit!


(cl:in-package vpa_robot_decision-msg)


;//! \htmlinclude InterInfo.msg.html

(cl:defclass <InterInfo> (roslisp-msg-protocol:ros-message)
  ((inter_id
    :reader inter_id
    :initarg :inter_id
    :type cl:fixnum
    :initform 0)
   (robot_id_list
    :reader robot_id_list
    :initarg :robot_id_list
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (robot_info
    :reader robot_info
    :initarg :robot_info
    :type (cl:vector vpa_robot_decision-msg:RobotInfo)
   :initform (cl:make-array 0 :element-type 'vpa_robot_decision-msg:RobotInfo :initial-element (cl:make-instance 'vpa_robot_decision-msg:RobotInfo))))
)

(cl:defclass InterInfo (<InterInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InterInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InterInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_decision-msg:<InterInfo> is deprecated: use vpa_robot_decision-msg:InterInfo instead.")))

(cl:ensure-generic-function 'inter_id-val :lambda-list '(m))
(cl:defmethod inter_id-val ((m <InterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:inter_id-val is deprecated.  Use vpa_robot_decision-msg:inter_id instead.")
  (inter_id m))

(cl:ensure-generic-function 'robot_id_list-val :lambda-list '(m))
(cl:defmethod robot_id_list-val ((m <InterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:robot_id_list-val is deprecated.  Use vpa_robot_decision-msg:robot_id_list instead.")
  (robot_id_list m))

(cl:ensure-generic-function 'robot_info-val :lambda-list '(m))
(cl:defmethod robot_info-val ((m <InterInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_decision-msg:robot_info-val is deprecated.  Use vpa_robot_decision-msg:robot_info instead.")
  (robot_info m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InterInfo>) ostream)
  "Serializes a message object of type '<InterInfo>"
  (cl:let* ((signed (cl:slot-value msg 'inter_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robot_id_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'robot_id_list))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robot_info))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'robot_info))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InterInfo>) istream)
  "Deserializes a message object of type '<InterInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'inter_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robot_id_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robot_id_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robot_info) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robot_info)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'vpa_robot_decision-msg:RobotInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InterInfo>)))
  "Returns string type for a message object of type '<InterInfo>"
  "vpa_robot_decision/InterInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InterInfo)))
  "Returns string type for a message object of type 'InterInfo"
  "vpa_robot_decision/InterInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InterInfo>)))
  "Returns md5sum for a message object of type '<InterInfo>"
  "a651223f4498c1bc66c36b76f2257d3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InterInfo)))
  "Returns md5sum for a message object of type 'InterInfo"
  "a651223f4498c1bc66c36b76f2257d3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InterInfo>)))
  "Returns full string definition for message of type '<InterInfo>"
  (cl:format cl:nil "int8            inter_id        # Intersection ID~%int8[]          robot_id_list   # List of robot names or IDs~%RobotInfo[]     robot_info      # List of RobotInfo instances~%================================================================================~%MSG: vpa_robot_decision/RobotInfo~%string  robot_name~%int8    robot_id~%float32 robot_a  # Acceleration~%float32 robot_v  # Velocity~%float32 robot_p  # Position~%float32 robot_enter_time~%float32 robot_arrive_cp_time~%float32 robot_exit_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InterInfo)))
  "Returns full string definition for message of type 'InterInfo"
  (cl:format cl:nil "int8            inter_id        # Intersection ID~%int8[]          robot_id_list   # List of robot names or IDs~%RobotInfo[]     robot_info      # List of RobotInfo instances~%================================================================================~%MSG: vpa_robot_decision/RobotInfo~%string  robot_name~%int8    robot_id~%float32 robot_a  # Acceleration~%float32 robot_v  # Velocity~%float32 robot_p  # Position~%float32 robot_enter_time~%float32 robot_arrive_cp_time~%float32 robot_exit_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InterInfo>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_id_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_info) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InterInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'InterInfo
    (cl:cons ':inter_id (inter_id msg))
    (cl:cons ':robot_id_list (robot_id_list msg))
    (cl:cons ':robot_info (robot_info msg))
))
