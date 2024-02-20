;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::SoftwareRevisionRpt)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'SoftwareRevisionRpt (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::SOFTWAREREVISIONRPT")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::SOFTWAREREVISIONRPT"))

(in-package "ROS")
;;//! \htmlinclude SoftwareRevisionRpt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::SoftwareRevisionRpt
  :super ros::object
  :slots (_header _software_version_0 _software_version_1 _software_version_2 _software_day _software_month_year ))

(defmethod kartech_linear_actuator_msgs::SoftwareRevisionRpt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:software_version_0 __software_version_0) 0)
    ((:software_version_1 __software_version_1) 0)
    ((:software_version_2 __software_version_2) 0)
    ((:software_day __software_day) 0)
    ((:software_month_year __software_month_year) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _software_version_0 (round __software_version_0))
   (setq _software_version_1 (round __software_version_1))
   (setq _software_version_2 (round __software_version_2))
   (setq _software_day (round __software_day))
   (setq _software_month_year (round __software_month_year))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:software_version_0
   (&optional __software_version_0)
   (if __software_version_0 (setq _software_version_0 __software_version_0)) _software_version_0)
  (:software_version_1
   (&optional __software_version_1)
   (if __software_version_1 (setq _software_version_1 __software_version_1)) _software_version_1)
  (:software_version_2
   (&optional __software_version_2)
   (if __software_version_2 (setq _software_version_2 __software_version_2)) _software_version_2)
  (:software_day
   (&optional __software_day)
   (if __software_day (setq _software_day __software_day)) _software_day)
  (:software_month_year
   (&optional __software_month_year)
   (if __software_month_year (setq _software_month_year __software_month_year)) _software_month_year)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _software_version_0
    1
    ;; uint8 _software_version_1
    1
    ;; uint8 _software_version_2
    1
    ;; uint8 _software_day
    1
    ;; uint16 _software_month_year
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _software_version_0
       (write-byte _software_version_0 s)
     ;; uint8 _software_version_1
       (write-byte _software_version_1 s)
     ;; uint8 _software_version_2
       (write-byte _software_version_2 s)
     ;; uint8 _software_day
       (write-byte _software_day s)
     ;; uint16 _software_month_year
       (write-word _software_month_year s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _software_version_0
     (setq _software_version_0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _software_version_1
     (setq _software_version_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _software_version_2
     (setq _software_version_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _software_day
     (setq _software_day (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _software_month_year
     (setq _software_month_year (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::SoftwareRevisionRpt :md5sum-) "5b4e8937bac2714ef707d040a16f320a")
(setf (get kartech_linear_actuator_msgs::SoftwareRevisionRpt :datatype-) "kartech_linear_actuator_msgs/SoftwareRevisionRpt")
(setf (get kartech_linear_actuator_msgs::SoftwareRevisionRpt :definition-)
      "std_msgs/Header header
uint8 software_version_0
uint8 software_version_1
uint8 software_version_2
uint8 software_day
uint16 software_month_year

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



(provide :kartech_linear_actuator_msgs/SoftwareRevisionRpt "5b4e8937bac2714ef707d040a16f320a")


