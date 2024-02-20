;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::PositionReachErrorTimeConfigCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'PositionReachErrorTimeConfigCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::POSITIONREACHERRORTIMECONFIGCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::POSITIONREACHERRORTIMECONFIGCMD"))

(in-package "ROS")
;;//! \htmlinclude PositionReachErrorTimeConfigCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::PositionReachErrorTimeConfigCmd
  :super ros::object
  :slots (_header _confirm _position_reach_error_time ))

(defmethod kartech_linear_actuator_msgs::PositionReachErrorTimeConfigCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:position_reach_error_time __position_reach_error_time) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _position_reach_error_time (round __position_reach_error_time))
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
  (:position_reach_error_time
   (&optional __position_reach_error_time)
   (if __position_reach_error_time (setq _position_reach_error_time __position_reach_error_time)) _position_reach_error_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint16 _position_reach_error_time
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _position_reach_error_time
       (write-word _position_reach_error_time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _position_reach_error_time
     (setq _position_reach_error_time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::PositionReachErrorTimeConfigCmd :md5sum-) "2f9859bacc9506f6b0784704786830d8")
(setf (get kartech_linear_actuator_msgs::PositionReachErrorTimeConfigCmd :datatype-) "kartech_linear_actuator_msgs/PositionReachErrorTimeConfigCmd")
(setf (get kartech_linear_actuator_msgs::PositionReachErrorTimeConfigCmd :definition-)
      "std_msgs/Header header
bool confirm
uint16 position_reach_error_time # Time that actuator needs to be stalled before cycling motor output in ms. Default is 3000ms.

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



(provide :kartech_linear_actuator_msgs/PositionReachErrorTimeConfigCmd "2f9859bacc9506f6b0784704786830d8")


