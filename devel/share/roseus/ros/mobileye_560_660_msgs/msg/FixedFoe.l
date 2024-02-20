;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::FixedFoe)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'FixedFoe (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::FIXEDFOE")
  (make-package "MOBILEYE_560_660_MSGS::FIXEDFOE"))

(in-package "ROS")
;;//! \htmlinclude FixedFoe.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mobileye_560_660_msgs::FixedFoe
  :super ros::object
  :slots (_header _fixed_yaw _fixed_horizon ))

(defmethod mobileye_560_660_msgs::FixedFoe
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:fixed_yaw __fixed_yaw) 0.0)
    ((:fixed_horizon __fixed_horizon) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _fixed_yaw (float __fixed_yaw))
   (setq _fixed_horizon (float __fixed_horizon))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:fixed_yaw
   (&optional __fixed_yaw)
   (if __fixed_yaw (setq _fixed_yaw __fixed_yaw)) _fixed_yaw)
  (:fixed_horizon
   (&optional __fixed_horizon)
   (if __fixed_horizon (setq _fixed_horizon __fixed_horizon)) _fixed_horizon)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _fixed_yaw
    8
    ;; float64 _fixed_horizon
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _fixed_yaw
       (sys::poke _fixed_yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _fixed_horizon
       (sys::poke _fixed_horizon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _fixed_yaw
     (setq _fixed_yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _fixed_horizon
     (setq _fixed_horizon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::FixedFoe :md5sum-) "b4f93d021949d9d8c671473a5bedf703")
(setf (get mobileye_560_660_msgs::FixedFoe :datatype-) "mobileye_560_660_msgs/FixedFoe")
(setf (get mobileye_560_660_msgs::FixedFoe :definition-)
      "std_msgs/Header header

float64 fixed_yaw
float64 fixed_horizon

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



(provide :mobileye_560_660_msgs/FixedFoe "b4f93d021949d9d8c671473a5bedf703")


