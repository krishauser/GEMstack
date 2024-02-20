;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::Sensors)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'Sensors (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::SENSORS")
  (make-package "NEOBOTIX_USBOARD_MSGS::SENSORS"))

(in-package "ROS")
;;//! \htmlinclude Sensors.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neobotix_usboard_msgs::Sensors
  :super ros::object
  :slots (_header _sensors ))

(defmethod neobotix_usboard_msgs::Sensors
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sensors __sensors) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _sensors __sensors)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sensors
   (&rest __sensors)
   (if (keywordp (car __sensors))
       (send* _sensors __sensors)
     (progn
       (if __sensors (setq _sensors (car __sensors)))
       _sensors)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; neobotix_usboard_msgs/SensorData[] _sensors
    (apply #'+ (send-all _sensors :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; neobotix_usboard_msgs/SensorData[] _sensors
     (write-long (length _sensors) s)
     (dolist (elem _sensors)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; neobotix_usboard_msgs/SensorData[] _sensors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensors (let (r) (dotimes (i n) (push (instance neobotix_usboard_msgs::SensorData :init) r)) r))
     (dolist (elem- _sensors)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::Sensors :md5sum-) "249620a0b67e8ec68e57bd709f06f5af")
(setf (get neobotix_usboard_msgs::Sensors :datatype-) "neobotix_usboard_msgs/Sensors")
(setf (get neobotix_usboard_msgs::Sensors :definition-)
      "# Message file for SensorData

std_msgs/Header header

neobotix_usboard_msgs/SensorData[] sensors

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

================================================================================
MSG: neobotix_usboard_msgs/SensorData
# Message file for SensorData

uint8   distance # cm
bool    warn
bool    alarm
bool    active

")



(provide :neobotix_usboard_msgs/Sensors "249620a0b67e8ec68e57bd709f06f5af")


