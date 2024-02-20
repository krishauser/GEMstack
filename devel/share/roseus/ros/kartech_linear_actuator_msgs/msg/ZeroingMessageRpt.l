;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ZeroingMessageRpt)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ZeroingMessageRpt (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::ZEROINGMESSAGERPT")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::ZEROINGMESSAGERPT"))

(in-package "ROS")
;;//! \htmlinclude ZeroingMessageRpt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ZeroingMessageRpt
  :super ros::object
  :slots (_header _chip_1_voltage _chip_2_voltage _chip_error_1 _chip_error_2 ))

(defmethod kartech_linear_actuator_msgs::ZeroingMessageRpt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:chip_1_voltage __chip_1_voltage) 0)
    ((:chip_2_voltage __chip_2_voltage) 0)
    ((:chip_error_1 __chip_error_1) 0)
    ((:chip_error_2 __chip_error_2) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _chip_1_voltage (round __chip_1_voltage))
   (setq _chip_2_voltage (round __chip_2_voltage))
   (setq _chip_error_1 (round __chip_error_1))
   (setq _chip_error_2 (round __chip_error_2))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:chip_1_voltage
   (&optional __chip_1_voltage)
   (if __chip_1_voltage (setq _chip_1_voltage __chip_1_voltage)) _chip_1_voltage)
  (:chip_2_voltage
   (&optional __chip_2_voltage)
   (if __chip_2_voltage (setq _chip_2_voltage __chip_2_voltage)) _chip_2_voltage)
  (:chip_error_1
   (&optional __chip_error_1)
   (if __chip_error_1 (setq _chip_error_1 __chip_error_1)) _chip_error_1)
  (:chip_error_2
   (&optional __chip_error_2)
   (if __chip_error_2 (setq _chip_error_2 __chip_error_2)) _chip_error_2)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _chip_1_voltage
    2
    ;; uint16 _chip_2_voltage
    2
    ;; uint8 _chip_error_1
    1
    ;; uint8 _chip_error_2
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _chip_1_voltage
       (write-word _chip_1_voltage s)
     ;; uint16 _chip_2_voltage
       (write-word _chip_2_voltage s)
     ;; uint8 _chip_error_1
       (write-byte _chip_error_1 s)
     ;; uint8 _chip_error_2
       (write-byte _chip_error_2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _chip_1_voltage
     (setq _chip_1_voltage (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _chip_2_voltage
     (setq _chip_2_voltage (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _chip_error_1
     (setq _chip_error_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _chip_error_2
     (setq _chip_error_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ZeroingMessageRpt :md5sum-) "1be34276909afaf5d9fd5f38a98c32a7")
(setf (get kartech_linear_actuator_msgs::ZeroingMessageRpt :datatype-) "kartech_linear_actuator_msgs/ZeroingMessageRpt")
(setf (get kartech_linear_actuator_msgs::ZeroingMessageRpt :definition-)
      "std_msgs/Header header
uint16 chip_1_voltage
uint16 chip_2_voltage
uint8 chip_error_1
uint8 chip_error_2

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



(provide :kartech_linear_actuator_msgs/ZeroingMessageRpt "1be34276909afaf5d9fd5f38a98c32a7")


