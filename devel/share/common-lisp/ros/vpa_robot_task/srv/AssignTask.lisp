; Auto-generated. Do not edit!


(cl:in-package vpa_robot_task-srv)


;//! \htmlinclude AssignTask-request.msg.html

(cl:defclass <AssignTask-request> (roslisp-msg-protocol:ros-message)
  ((robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform ""))
)

(cl:defclass AssignTask-request (<AssignTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AssignTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AssignTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_task-srv:<AssignTask-request> is deprecated: use vpa_robot_task-srv:AssignTask-request instead.")))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <AssignTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_task-srv:robot_name-val is deprecated.  Use vpa_robot_task-srv:robot_name instead.")
  (robot_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AssignTask-request>) ostream)
  "Serializes a message object of type '<AssignTask-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AssignTask-request>) istream)
  "Deserializes a message object of type '<AssignTask-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AssignTask-request>)))
  "Returns string type for a service object of type '<AssignTask-request>"
  "vpa_robot_task/AssignTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AssignTask-request)))
  "Returns string type for a service object of type 'AssignTask-request"
  "vpa_robot_task/AssignTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AssignTask-request>)))
  "Returns md5sum for a message object of type '<AssignTask-request>"
  "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AssignTask-request)))
  "Returns md5sum for a message object of type 'AssignTask-request"
  "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AssignTask-request>)))
  "Returns full string definition for message of type '<AssignTask-request>"
  (cl:format cl:nil "string robot_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AssignTask-request)))
  "Returns full string definition for message of type 'AssignTask-request"
  (cl:format cl:nil "string robot_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AssignTask-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robot_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AssignTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AssignTask-request
    (cl:cons ':robot_name (robot_name msg))
))
;//! \htmlinclude AssignTask-response.msg.html

(cl:defclass <AssignTask-response> (roslisp-msg-protocol:ros-message)
  ((task_list
    :reader task_list
    :initarg :task_list
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass AssignTask-response (<AssignTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AssignTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AssignTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vpa_robot_task-srv:<AssignTask-response> is deprecated: use vpa_robot_task-srv:AssignTask-response instead.")))

(cl:ensure-generic-function 'task_list-val :lambda-list '(m))
(cl:defmethod task_list-val ((m <AssignTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vpa_robot_task-srv:task_list-val is deprecated.  Use vpa_robot_task-srv:task_list instead.")
  (task_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AssignTask-response>) ostream)
  "Serializes a message object of type '<AssignTask-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'task_list))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'task_list))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AssignTask-response>) istream)
  "Deserializes a message object of type '<AssignTask-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'task_list) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'task_list)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AssignTask-response>)))
  "Returns string type for a service object of type '<AssignTask-response>"
  "vpa_robot_task/AssignTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AssignTask-response)))
  "Returns string type for a service object of type 'AssignTask-response"
  "vpa_robot_task/AssignTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AssignTask-response>)))
  "Returns md5sum for a message object of type '<AssignTask-response>"
  "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AssignTask-response)))
  "Returns md5sum for a message object of type 'AssignTask-response"
  "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AssignTask-response>)))
  "Returns full string definition for message of type '<AssignTask-response>"
  (cl:format cl:nil "int8[] task_list~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AssignTask-response)))
  "Returns full string definition for message of type 'AssignTask-response"
  (cl:format cl:nil "int8[] task_list~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AssignTask-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'task_list) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AssignTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AssignTask-response
    (cl:cons ':task_list (task_list msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AssignTask)))
  'AssignTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AssignTask)))
  'AssignTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AssignTask)))
  "Returns string type for a service object of type '<AssignTask>"
  "vpa_robot_task/AssignTask")