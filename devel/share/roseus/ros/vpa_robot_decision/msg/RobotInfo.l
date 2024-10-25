;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_decision::RobotInfo)
  (if (not (find-package "VPA_ROBOT_DECISION"))
    (make-package "VPA_ROBOT_DECISION"))
  (shadow 'RobotInfo (find-package "VPA_ROBOT_DECISION")))
(unless (find-package "VPA_ROBOT_DECISION::ROBOTINFO")
  (make-package "VPA_ROBOT_DECISION::ROBOTINFO"))

(in-package "ROS")
;;//! \htmlinclude RobotInfo.msg.html


(defclass vpa_robot_decision::RobotInfo
  :super ros::object
  :slots (_robot_name _robot_id _robot_a _robot_v _robot_p _robot_enter_time _robot_arrive_cp_time _robot_exit_time ))

(defmethod vpa_robot_decision::RobotInfo
  (:init
   (&key
    ((:robot_name __robot_name) "")
    ((:robot_id __robot_id) 0)
    ((:robot_a __robot_a) 0.0)
    ((:robot_v __robot_v) 0.0)
    ((:robot_p __robot_p) 0.0)
    ((:robot_enter_time __robot_enter_time) 0.0)
    ((:robot_arrive_cp_time __robot_arrive_cp_time) 0.0)
    ((:robot_exit_time __robot_exit_time) 0.0)
    )
   (send-super :init)
   (setq _robot_name (string __robot_name))
   (setq _robot_id (round __robot_id))
   (setq _robot_a (float __robot_a))
   (setq _robot_v (float __robot_v))
   (setq _robot_p (float __robot_p))
   (setq _robot_enter_time (float __robot_enter_time))
   (setq _robot_arrive_cp_time (float __robot_arrive_cp_time))
   (setq _robot_exit_time (float __robot_exit_time))
   self)
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:robot_a
   (&optional __robot_a)
   (if __robot_a (setq _robot_a __robot_a)) _robot_a)
  (:robot_v
   (&optional __robot_v)
   (if __robot_v (setq _robot_v __robot_v)) _robot_v)
  (:robot_p
   (&optional __robot_p)
   (if __robot_p (setq _robot_p __robot_p)) _robot_p)
  (:robot_enter_time
   (&optional __robot_enter_time)
   (if __robot_enter_time (setq _robot_enter_time __robot_enter_time)) _robot_enter_time)
  (:robot_arrive_cp_time
   (&optional __robot_arrive_cp_time)
   (if __robot_arrive_cp_time (setq _robot_arrive_cp_time __robot_arrive_cp_time)) _robot_arrive_cp_time)
  (:robot_exit_time
   (&optional __robot_exit_time)
   (if __robot_exit_time (setq _robot_exit_time __robot_exit_time)) _robot_exit_time)
  (:serialization-length
   ()
   (+
    ;; string _robot_name
    4 (length _robot_name)
    ;; int8 _robot_id
    1
    ;; float32 _robot_a
    4
    ;; float32 _robot_v
    4
    ;; float32 _robot_p
    4
    ;; float32 _robot_enter_time
    4
    ;; float32 _robot_arrive_cp_time
    4
    ;; float32 _robot_exit_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; int8 _robot_id
       (write-byte _robot_id s)
     ;; float32 _robot_a
       (sys::poke _robot_a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _robot_v
       (sys::poke _robot_v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _robot_p
       (sys::poke _robot_p (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _robot_enter_time
       (sys::poke _robot_enter_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _robot_arrive_cp_time
       (sys::poke _robot_arrive_cp_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _robot_exit_time
       (sys::poke _robot_exit_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _robot_id
     (setq _robot_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _robot_id 127) (setq _robot_id (- _robot_id 256)))
   ;; float32 _robot_a
     (setq _robot_a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _robot_v
     (setq _robot_v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _robot_p
     (setq _robot_p (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _robot_enter_time
     (setq _robot_enter_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _robot_arrive_cp_time
     (setq _robot_arrive_cp_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _robot_exit_time
     (setq _robot_exit_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vpa_robot_decision::RobotInfo :md5sum-) "11867948fae731e82b48e6596c84aa31")
(setf (get vpa_robot_decision::RobotInfo :datatype-) "vpa_robot_decision/RobotInfo")
(setf (get vpa_robot_decision::RobotInfo :definition-)
      "string  robot_name
int8    robot_id
float32 robot_a  # Acceleration
float32 robot_v  # Velocity
float32 robot_p  # Position
float32 robot_enter_time
float32 robot_arrive_cp_time
float32 robot_exit_time

")



(provide :vpa_robot_decision/RobotInfo "11867948fae731e82b48e6596c84aa31")


