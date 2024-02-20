;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::MotorCurrentRpt)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'MotorCurrentRpt (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::MOTORCURRENTRPT")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::MOTORCURRENTRPT"))

(in-package "ROS")
;;//! \htmlinclude MotorCurrentRpt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::MotorCurrentRpt
  :super ros::object
  :slots (_header _motor_current ))

(defmethod kartech_linear_actuator_msgs::MotorCurrentRpt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:motor_current __motor_current) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _motor_current (round __motor_current))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:motor_current
   (&optional __motor_current)
   (if __motor_current (setq _motor_current __motor_current)) _motor_current)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _motor_current
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _motor_current
       (write-word _motor_current s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _motor_current
     (setq _motor_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::MotorCurrentRpt :md5sum-) "dee1ca7661def0cd0d6c1f63e8b5e45d")
(setf (get kartech_linear_actuator_msgs::MotorCurrentRpt :datatype-) "kartech_linear_actuator_msgs/MotorCurrentRpt")
(setf (get kartech_linear_actuator_msgs::MotorCurrentRpt :definition-)
      "std_msgs/Header header
uint16 motor_current    # The current motor current in mA.

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



(provide :kartech_linear_actuator_msgs/MotorCurrentRpt "dee1ca7661def0cd0d6c1f63e8b5e45d")


