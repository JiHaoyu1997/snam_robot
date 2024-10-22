;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_decision::RobotInterInfo)
  (if (not (find-package "VPA_ROBOT_DECISION"))
    (make-package "VPA_ROBOT_DECISION"))
  (shadow 'RobotInterInfo (find-package "VPA_ROBOT_DECISION")))
(unless (find-package "VPA_ROBOT_DECISION::ROBOTINTERINFO")
  (make-package "VPA_ROBOT_DECISION::ROBOTINTERINFO"))

(in-package "ROS")
;;//! \htmlinclude RobotInterInfo.msg.html


(defclass vpa_robot_decision::RobotInterInfo
  :super ros::object
  :slots (_inter_1 _inter_2 _inter_3 _inter_4 _inter_5 ))

(defmethod vpa_robot_decision::RobotInterInfo
  (:init
   (&key
    ((:inter_1 __inter_1) (make-array 0 :initial-element 0 :element-type :integer))
    ((:inter_2 __inter_2) (make-array 0 :initial-element 0 :element-type :integer))
    ((:inter_3 __inter_3) (make-array 0 :initial-element 0 :element-type :integer))
    ((:inter_4 __inter_4) (make-array 0 :initial-element 0 :element-type :integer))
    ((:inter_5 __inter_5) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _inter_1 __inter_1)
   (setq _inter_2 __inter_2)
   (setq _inter_3 __inter_3)
   (setq _inter_4 __inter_4)
   (setq _inter_5 __inter_5)
   self)
  (:inter_1
   (&optional __inter_1)
   (if __inter_1 (setq _inter_1 __inter_1)) _inter_1)
  (:inter_2
   (&optional __inter_2)
   (if __inter_2 (setq _inter_2 __inter_2)) _inter_2)
  (:inter_3
   (&optional __inter_3)
   (if __inter_3 (setq _inter_3 __inter_3)) _inter_3)
  (:inter_4
   (&optional __inter_4)
   (if __inter_4 (setq _inter_4 __inter_4)) _inter_4)
  (:inter_5
   (&optional __inter_5)
   (if __inter_5 (setq _inter_5 __inter_5)) _inter_5)
  (:serialization-length
   ()
   (+
    ;; int8[] _inter_1
    (* 1    (length _inter_1)) 4
    ;; int8[] _inter_2
    (* 1    (length _inter_2)) 4
    ;; int8[] _inter_3
    (* 1    (length _inter_3)) 4
    ;; int8[] _inter_4
    (* 1    (length _inter_4)) 4
    ;; int8[] _inter_5
    (* 1    (length _inter_5)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[] _inter_1
     (write-long (length _inter_1) s)
     (dotimes (i (length _inter_1))
       (write-byte (elt _inter_1 i) s)
       )
     ;; int8[] _inter_2
     (write-long (length _inter_2) s)
     (dotimes (i (length _inter_2))
       (write-byte (elt _inter_2 i) s)
       )
     ;; int8[] _inter_3
     (write-long (length _inter_3) s)
     (dotimes (i (length _inter_3))
       (write-byte (elt _inter_3 i) s)
       )
     ;; int8[] _inter_4
     (write-long (length _inter_4) s)
     (dotimes (i (length _inter_4))
       (write-byte (elt _inter_4 i) s)
       )
     ;; int8[] _inter_5
     (write-long (length _inter_5) s)
     (dotimes (i (length _inter_5))
       (write-byte (elt _inter_5 i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[] _inter_1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _inter_1 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _inter_1 i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _inter_1 i) 127) (setf (elt _inter_1 i) (- (elt _inter_1 i) 256)))
     ))
   ;; int8[] _inter_2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _inter_2 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _inter_2 i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _inter_2 i) 127) (setf (elt _inter_2 i) (- (elt _inter_2 i) 256)))
     ))
   ;; int8[] _inter_3
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _inter_3 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _inter_3 i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _inter_3 i) 127) (setf (elt _inter_3 i) (- (elt _inter_3 i) 256)))
     ))
   ;; int8[] _inter_4
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _inter_4 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _inter_4 i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _inter_4 i) 127) (setf (elt _inter_4 i) (- (elt _inter_4 i) 256)))
     ))
   ;; int8[] _inter_5
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _inter_5 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _inter_5 i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _inter_5 i) 127) (setf (elt _inter_5 i) (- (elt _inter_5 i) 256)))
     ))
   ;;
   self)
  )

(setf (get vpa_robot_decision::RobotInterInfo :md5sum-) "c7e2780d75dbdc968ce987e67cfdc2df")
(setf (get vpa_robot_decision::RobotInterInfo :datatype-) "vpa_robot_decision/RobotInterInfo")
(setf (get vpa_robot_decision::RobotInterInfo :definition-)
      "# RobotInterInfo.msg

int8[] inter_1
int8[] inter_2
int8[] inter_3
int8[] inter_4
int8[] inter_5
")



(provide :vpa_robot_decision/RobotInterInfo "c7e2780d75dbdc968ce987e67cfdc2df")


