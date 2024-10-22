;; Auto-generated. Do not edit!


(when (boundp 'vpa_robot_task::AssignTask)
  (if (not (find-package "VPA_ROBOT_TASK"))
    (make-package "VPA_ROBOT_TASK"))
  (shadow 'AssignTask (find-package "VPA_ROBOT_TASK")))
(unless (find-package "VPA_ROBOT_TASK::ASSIGNTASK")
  (make-package "VPA_ROBOT_TASK::ASSIGNTASK"))
(unless (find-package "VPA_ROBOT_TASK::ASSIGNTASKREQUEST")
  (make-package "VPA_ROBOT_TASK::ASSIGNTASKREQUEST"))
(unless (find-package "VPA_ROBOT_TASK::ASSIGNTASKRESPONSE")
  (make-package "VPA_ROBOT_TASK::ASSIGNTASKRESPONSE"))

(in-package "ROS")





(defclass vpa_robot_task::AssignTaskRequest
  :super ros::object
  :slots (_robot_name ))

(defmethod vpa_robot_task::AssignTaskRequest
  (:init
   (&key
    ((:robot_name __robot_name) "")
    )
   (send-super :init)
   (setq _robot_name (string __robot_name))
   self)
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:serialization-length
   ()
   (+
    ;; string _robot_name
    4 (length _robot_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass vpa_robot_task::AssignTaskResponse
  :super ros::object
  :slots (_task_list ))

(defmethod vpa_robot_task::AssignTaskResponse
  (:init
   (&key
    ((:task_list __task_list) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _task_list __task_list)
   self)
  (:task_list
   (&optional __task_list)
   (if __task_list (setq _task_list __task_list)) _task_list)
  (:serialization-length
   ()
   (+
    ;; int8[] _task_list
    (* 1    (length _task_list)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[] _task_list
     (write-long (length _task_list) s)
     (dotimes (i (length _task_list))
       (write-byte (elt _task_list i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[] _task_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _task_list (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _task_list i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _task_list i) 127) (setf (elt _task_list i) (- (elt _task_list i) 256)))
     ))
   ;;
   self)
  )

(defclass vpa_robot_task::AssignTask
  :super ros::object
  :slots ())

(setf (get vpa_robot_task::AssignTask :md5sum-) "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(setf (get vpa_robot_task::AssignTask :datatype-) "vpa_robot_task/AssignTask")
(setf (get vpa_robot_task::AssignTask :request) vpa_robot_task::AssignTaskRequest)
(setf (get vpa_robot_task::AssignTask :response) vpa_robot_task::AssignTaskResponse)

(defmethod vpa_robot_task::AssignTaskRequest
  (:response () (instance vpa_robot_task::AssignTaskResponse :init)))

(setf (get vpa_robot_task::AssignTaskRequest :md5sum-) "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(setf (get vpa_robot_task::AssignTaskRequest :datatype-) "vpa_robot_task/AssignTaskRequest")
(setf (get vpa_robot_task::AssignTaskRequest :definition-)
      "string robot_name
---
int8[] task_list
")

(setf (get vpa_robot_task::AssignTaskResponse :md5sum-) "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")
(setf (get vpa_robot_task::AssignTaskResponse :datatype-) "vpa_robot_task/AssignTaskResponse")
(setf (get vpa_robot_task::AssignTaskResponse :definition-)
      "string robot_name
---
int8[] task_list
")



(provide :vpa_robot_task/AssignTask "cdad0eb519e23dd2b5d3e4c2e2b7ecf6")


