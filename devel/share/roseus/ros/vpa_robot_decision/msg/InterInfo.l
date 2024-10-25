;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_decision::InterInfo)
  (if (not (find-package "VPA_ROBOT_DECISION"))
    (make-package "VPA_ROBOT_DECISION"))
  (shadow 'InterInfo (find-package "VPA_ROBOT_DECISION")))
(unless (find-package "VPA_ROBOT_DECISION::INTERINFO")
  (make-package "VPA_ROBOT_DECISION::INTERINFO"))

(in-package "ROS")
;;//! \htmlinclude InterInfo.msg.html


(defclass vpa_robot_decision::InterInfo
  :super ros::object
  :slots (_inter_id _robot_id_list _robot_info ))

(defmethod vpa_robot_decision::InterInfo
  (:init
   (&key
    ((:inter_id __inter_id) 0)
    ((:robot_id_list __robot_id_list) (make-array 0 :initial-element 0 :element-type :integer))
    ((:robot_info __robot_info) ())
    )
   (send-super :init)
   (setq _inter_id (round __inter_id))
   (setq _robot_id_list __robot_id_list)
   (setq _robot_info __robot_info)
   self)
  (:inter_id
   (&optional __inter_id)
   (if __inter_id (setq _inter_id __inter_id)) _inter_id)
  (:robot_id_list
   (&optional __robot_id_list)
   (if __robot_id_list (setq _robot_id_list __robot_id_list)) _robot_id_list)
  (:robot_info
   (&rest __robot_info)
   (if (keywordp (car __robot_info))
       (send* _robot_info __robot_info)
     (progn
       (if __robot_info (setq _robot_info (car __robot_info)))
       _robot_info)))
  (:serialization-length
   ()
   (+
    ;; int8 _inter_id
    1
    ;; int8[] _robot_id_list
    (* 1    (length _robot_id_list)) 4
    ;; vpa_robot_decision/RobotInfo[] _robot_info
    (apply #'+ (send-all _robot_info :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _inter_id
       (write-byte _inter_id s)
     ;; int8[] _robot_id_list
     (write-long (length _robot_id_list) s)
     (dotimes (i (length _robot_id_list))
       (write-byte (elt _robot_id_list i) s)
       )
     ;; vpa_robot_decision/RobotInfo[] _robot_info
     (write-long (length _robot_info) s)
     (dolist (elem _robot_info)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _inter_id
     (setq _inter_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _inter_id 127) (setq _inter_id (- _inter_id 256)))
   ;; int8[] _robot_id_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_id_list (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _robot_id_list i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _robot_id_list i) 127) (setf (elt _robot_id_list i) (- (elt _robot_id_list i) 256)))
     ))
   ;; vpa_robot_decision/RobotInfo[] _robot_info
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _robot_info (let (r) (dotimes (i n) (push (instance vpa_robot_decision::RobotInfo :init) r)) r))
     (dolist (elem- _robot_info)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get vpa_robot_decision::InterInfo :md5sum-) "a651223f4498c1bc66c36b76f2257d3e")
(setf (get vpa_robot_decision::InterInfo :datatype-) "vpa_robot_decision/InterInfo")
(setf (get vpa_robot_decision::InterInfo :definition-)
      "int8            inter_id        # Intersection ID
int8[]          robot_id_list   # List of robot names or IDs
RobotInfo[]     robot_info      # List of RobotInfo instances
================================================================================
MSG: vpa_robot_decision/RobotInfo
string  robot_name
int8    robot_id
float32 robot_a  # Acceleration
float32 robot_v  # Velocity
float32 robot_p  # Position
float32 robot_enter_time
float32 robot_arrive_cp_time
float32 robot_exit_time

")



(provide :vpa_robot_decision/InterInfo "a651223f4498c1bc66c36b76f2257d3e")


