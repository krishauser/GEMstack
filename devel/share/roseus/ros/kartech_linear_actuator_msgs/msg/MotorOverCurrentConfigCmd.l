;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::MotorOverCurrentConfigCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'MotorOverCurrentConfigCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::MOTOROVERCURRENTCONFIGCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::MOTOROVERCURRENTCONFIGCMD"))

(in-package "ROS")
;;//! \htmlinclude MotorOverCurrentConfigCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::MotorOverCurrentConfigCmd
  :super ros::object
  :slots (_header _confirm _over_current ))

(defmethod kartech_linear_actuator_msgs::MotorOverCurrentConfigCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:over_current __over_current) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _over_current (round __over_current))
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
  (:over_current
   (&optional __over_current)
   (if __over_current (setq _over_current __over_current)) _over_current)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint16 _over_current
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
     ;; uint16 _over_current
       (write-word _over_current s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _over_current
     (setq _over_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::MotorOverCurrentConfigCmd :md5sum-) "65da7043e246e0aea73fa57120f9071a")
(setf (get kartech_linear_actuator_msgs::MotorOverCurrentConfigCmd :datatype-) "kartech_linear_actuator_msgs/MotorOverCurrentConfigCmd")
(setf (get kartech_linear_actuator_msgs::MotorOverCurrentConfigCmd :definition-)
      "std_msgs/Header header
bool confirm
uint16 over_current # The over-current value in mA. Default is 6500mA.

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



(provide :kartech_linear_actuator_msgs/MotorOverCurrentConfigCmd "65da7043e246e0aea73fa57120f9071a")


