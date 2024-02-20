;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ActuatorUniqueIdReq)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ActuatorUniqueIdReq (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::ACTUATORUNIQUEIDREQ")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::ACTUATORUNIQUEIDREQ"))

(in-package "ROS")
;;//! \htmlinclude ActuatorUniqueIdReq.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ActuatorUniqueIdReq
  :super ros::object
  :slots (_header _confirm ))

(defmethod kartech_linear_actuator_msgs::ActuatorUniqueIdReq
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:confirm
   (&optional (__confirm :null))
   (if (not (eq __confirm :null)) (setq _confirm __confirm)) _confirm)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ActuatorUniqueIdReq :md5sum-) "d73ec12e18c7d20276159c8210d67b94")
(setf (get kartech_linear_actuator_msgs::ActuatorUniqueIdReq :datatype-) "kartech_linear_actuator_msgs/ActuatorUniqueIdReq")
(setf (get kartech_linear_actuator_msgs::ActuatorUniqueIdReq :definition-)
      "std_msgs/Header header
bool confirm

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :kartech_linear_actuator_msgs/ActuatorUniqueIdReq "d73ec12e18c7d20276159c8210d67b94")


