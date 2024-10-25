;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_vision::CrossInfo)
  (if (not (find-package "VPA_ROBOT_VISION"))
    (make-package "VPA_ROBOT_VISION"))
  (shadow 'CrossInfo (find-package "VPA_ROBOT_VISION")))
(unless (find-package "VPA_ROBOT_VISION::CROSSINFO")
  (make-package "VPA_ROBOT_VISION::CROSSINFO"))

(in-package "ROS")
;;//! \htmlinclude CrossInfo.msg.html


(defclass vpa_robot_vision::CrossInfo
  :super ros::object
  :slots (_robot_name _cross _last_inter_id _local_inter_id ))

(defmethod vpa_robot_vision::CrossInfo
  (:init
   (&key
    ((:robot_name __robot_name) "")
    ((:cross __cross) nil)
    ((:last_inter_id __last_inter_id) 0)
    ((:local_inter_id __local_inter_id) 0)
    )
   (send-super :init)
   (setq _robot_name (string __robot_name))
   (setq _cross __cross)
   (setq _last_inter_id (round __last_inter_id))
   (setq _local_inter_id (round __local_inter_id))
   self)
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:cross
   (&optional (__cross :null))
   (if (not (eq __cross :null)) (setq _cross __cross)) _cross)
  (:last_inter_id
   (&optional __last_inter_id)
   (if __last_inter_id (setq _last_inter_id __last_inter_id)) _last_inter_id)
  (:local_inter_id
   (&optional __local_inter_id)
   (if __local_inter_id (setq _local_inter_id __local_inter_id)) _local_inter_id)
  (:serialization-length
   ()
   (+
    ;; string _robot_name
    4 (length _robot_name)
    ;; bool _cross
    1
    ;; int8 _last_inter_id
    1
    ;; int8 _local_inter_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; bool _cross
       (if _cross (write-byte -1 s) (write-byte 0 s))
     ;; int8 _last_inter_id
       (write-byte _last_inter_id s)
     ;; int8 _local_inter_id
       (write-byte _local_inter_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _cross
     (setq _cross (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _last_inter_id
     (setq _last_inter_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _last_inter_id 127) (setq _last_inter_id (- _last_inter_id 256)))
   ;; int8 _local_inter_id
     (setq _local_inter_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _local_inter_id 127) (setq _local_inter_id (- _local_inter_id 256)))
   ;;
   self)
  )

(setf (get vpa_robot_vision::CrossInfo :md5sum-) "abc28f0d82de76aab08a8f05250b1372")
(setf (get vpa_robot_vision::CrossInfo :datatype-) "vpa_robot_vision/CrossInfo")
(setf (get vpa_robot_vision::CrossInfo :definition-)
      "string robot_name         # Name of the robot
bool cross                # Indicates if the robot has crossed
int8 last_inter_id       # ID of the last intersection
int8 local_inter_id      # ID of the current (local) intersection

")



(provide :vpa_robot_vision/CrossInfo "abc28f0d82de76aab08a8f05250b1372")


