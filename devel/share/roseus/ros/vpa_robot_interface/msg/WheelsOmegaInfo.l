;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_interface::WheelsOmegaInfo)
  (if (not (find-package "VPA_ROBOT_INTERFACE"))
    (make-package "VPA_ROBOT_INTERFACE"))
  (shadow 'WheelsOmegaInfo (find-package "VPA_ROBOT_INTERFACE")))
(unless (find-package "VPA_ROBOT_INTERFACE::WHEELSOMEGAINFO")
  (make-package "VPA_ROBOT_INTERFACE::WHEELSOMEGAINFO"))

(in-package "ROS")
;;//! \htmlinclude WheelsOmegaInfo.msg.html


(defclass vpa_robot_interface::WheelsOmegaInfo
  :super ros::object
  :slots (_omega_left_ref _omega_left_sig _omega_right_ref _omega_right_sig ))

(defmethod vpa_robot_interface::WheelsOmegaInfo
  (:init
   (&key
    ((:omega_left_ref __omega_left_ref) 0.0)
    ((:omega_left_sig __omega_left_sig) 0.0)
    ((:omega_right_ref __omega_right_ref) 0.0)
    ((:omega_right_sig __omega_right_sig) 0.0)
    )
   (send-super :init)
   (setq _omega_left_ref (float __omega_left_ref))
   (setq _omega_left_sig (float __omega_left_sig))
   (setq _omega_right_ref (float __omega_right_ref))
   (setq _omega_right_sig (float __omega_right_sig))
   self)
  (:omega_left_ref
   (&optional __omega_left_ref)
   (if __omega_left_ref (setq _omega_left_ref __omega_left_ref)) _omega_left_ref)
  (:omega_left_sig
   (&optional __omega_left_sig)
   (if __omega_left_sig (setq _omega_left_sig __omega_left_sig)) _omega_left_sig)
  (:omega_right_ref
   (&optional __omega_right_ref)
   (if __omega_right_ref (setq _omega_right_ref __omega_right_ref)) _omega_right_ref)
  (:omega_right_sig
   (&optional __omega_right_sig)
   (if __omega_right_sig (setq _omega_right_sig __omega_right_sig)) _omega_right_sig)
  (:serialization-length
   ()
   (+
    ;; float32 _omega_left_ref
    4
    ;; float32 _omega_left_sig
    4
    ;; float32 _omega_right_ref
    4
    ;; float32 _omega_right_sig
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _omega_left_ref
       (sys::poke _omega_left_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _omega_left_sig
       (sys::poke _omega_left_sig (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _omega_right_ref
       (sys::poke _omega_right_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _omega_right_sig
       (sys::poke _omega_right_sig (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _omega_left_ref
     (setq _omega_left_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _omega_left_sig
     (setq _omega_left_sig (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _omega_right_ref
     (setq _omega_right_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _omega_right_sig
     (setq _omega_right_sig (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get vpa_robot_interface::WheelsOmegaInfo :md5sum-) "ba80e47329fa708db7e2fff0c002fd00")
(setf (get vpa_robot_interface::WheelsOmegaInfo :datatype-) "vpa_robot_interface/WheelsOmegaInfo")
(setf (get vpa_robot_interface::WheelsOmegaInfo :definition-)
      "float32 omega_left_ref
float32 omega_left_sig
float32 omega_right_ref
float32 omega_right_sig
")



(provide :vpa_robot_interface/WheelsOmegaInfo "ba80e47329fa708db7e2fff0c002fd00")


