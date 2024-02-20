;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::PositionRpt)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'PositionRpt (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::POSITIONRPT")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::POSITIONRPT"))

(in-package "ROS")
;;//! \htmlinclude PositionRpt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::PositionRpt
  :super ros::object
  :slots (_header _shaft_extension ))

(defmethod kartech_linear_actuator_msgs::PositionRpt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:shaft_extension __shaft_extension) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _shaft_extension (float __shaft_extension))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:shaft_extension
   (&optional __shaft_extension)
   (if __shaft_extension (setq _shaft_extension __shaft_extension)) _shaft_extension)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _shaft_extension
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _shaft_extension
       (sys::poke _shaft_extension (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _shaft_extension
     (setq _shaft_extension (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::PositionRpt :md5sum-) "bc025045a75f313163c8d3ca0cedadf9")
(setf (get kartech_linear_actuator_msgs::PositionRpt :datatype-) "kartech_linear_actuator_msgs/PositionRpt")
(setf (get kartech_linear_actuator_msgs::PositionRpt :definition-)
      "std_msgs/Header header
float64 shaft_extension     # The current shaft position in 0.001\" increments.

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



(provide :kartech_linear_actuator_msgs/PositionRpt "bc025045a75f313163c8d3ca0cedadf9")


