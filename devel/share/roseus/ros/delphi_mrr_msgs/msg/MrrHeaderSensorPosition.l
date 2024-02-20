;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrHeaderSensorPosition)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrHeaderSensorPosition (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRHEADERSENSORPOSITION")
  (make-package "DELPHI_MRR_MSGS::MRRHEADERSENSORPOSITION"))

(in-package "ROS")
;;//! \htmlinclude MrrHeaderSensorPosition.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrHeaderSensorPosition
  :super ros::object
  :slots (_header _can_sensor_polarity _can_sensor_lat_offset _can_sensor_long_offset _can_sensor_hangle_offset ))

(defmethod delphi_mrr_msgs::MrrHeaderSensorPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_sensor_polarity __can_sensor_polarity) nil)
    ((:can_sensor_lat_offset __can_sensor_lat_offset) 0.0)
    ((:can_sensor_long_offset __can_sensor_long_offset) 0.0)
    ((:can_sensor_hangle_offset __can_sensor_hangle_offset) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_sensor_polarity __can_sensor_polarity)
   (setq _can_sensor_lat_offset (float __can_sensor_lat_offset))
   (setq _can_sensor_long_offset (float __can_sensor_long_offset))
   (setq _can_sensor_hangle_offset (float __can_sensor_hangle_offset))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_sensor_polarity
   (&optional (__can_sensor_polarity :null))
   (if (not (eq __can_sensor_polarity :null)) (setq _can_sensor_polarity __can_sensor_polarity)) _can_sensor_polarity)
  (:can_sensor_lat_offset
   (&optional __can_sensor_lat_offset)
   (if __can_sensor_lat_offset (setq _can_sensor_lat_offset __can_sensor_lat_offset)) _can_sensor_lat_offset)
  (:can_sensor_long_offset
   (&optional __can_sensor_long_offset)
   (if __can_sensor_long_offset (setq _can_sensor_long_offset __can_sensor_long_offset)) _can_sensor_long_offset)
  (:can_sensor_hangle_offset
   (&optional __can_sensor_hangle_offset)
   (if __can_sensor_hangle_offset (setq _can_sensor_hangle_offset __can_sensor_hangle_offset)) _can_sensor_hangle_offset)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _can_sensor_polarity
    1
    ;; float32 _can_sensor_lat_offset
    4
    ;; float32 _can_sensor_long_offset
    4
    ;; float32 _can_sensor_hangle_offset
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _can_sensor_polarity
       (if _can_sensor_polarity (write-byte -1 s) (write-byte 0 s))
     ;; float32 _can_sensor_lat_offset
       (sys::poke _can_sensor_lat_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_sensor_long_offset
       (sys::poke _can_sensor_long_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_sensor_hangle_offset
       (sys::poke _can_sensor_hangle_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _can_sensor_polarity
     (setq _can_sensor_polarity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _can_sensor_lat_offset
     (setq _can_sensor_lat_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_sensor_long_offset
     (setq _can_sensor_long_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_sensor_hangle_offset
     (setq _can_sensor_hangle_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrHeaderSensorPosition :md5sum-) "1f8dc3bc1765d0520979335ee0f8d03a")
(setf (get delphi_mrr_msgs::MrrHeaderSensorPosition :datatype-) "delphi_mrr_msgs/MrrHeaderSensorPosition")
(setf (get delphi_mrr_msgs::MrrHeaderSensorPosition :definition-)
      "std_msgs/Header header

bool    can_sensor_polarity
float32 can_sensor_lat_offset
float32 can_sensor_long_offset
float32 can_sensor_hangle_offset

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



(provide :delphi_mrr_msgs/MrrHeaderSensorPosition "1f8dc3bc1765d0520979335ee0f8d03a")


