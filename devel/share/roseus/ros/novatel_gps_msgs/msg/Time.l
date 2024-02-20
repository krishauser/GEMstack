;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Time)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Time (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::TIME")
  (make-package "NOVATEL_GPS_MSGS::TIME"))

(in-package "ROS")
;;//! \htmlinclude Time.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Time
  :super ros::object
  :slots (_header _clock_status _offset _offset_std _utc_offset _utc_year _utc_month _utc_day _utc_hour _utc_minute _utc_millisecond _utc_status ))

(defmethod novatel_gps_msgs::Time
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:clock_status __clock_status) "")
    ((:offset __offset) 0.0)
    ((:offset_std __offset_std) 0.0)
    ((:utc_offset __utc_offset) 0.0)
    ((:utc_year __utc_year) 0)
    ((:utc_month __utc_month) 0)
    ((:utc_day __utc_day) 0)
    ((:utc_hour __utc_hour) 0)
    ((:utc_minute __utc_minute) 0)
    ((:utc_millisecond __utc_millisecond) 0)
    ((:utc_status __utc_status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _clock_status (string __clock_status))
   (setq _offset (float __offset))
   (setq _offset_std (float __offset_std))
   (setq _utc_offset (float __utc_offset))
   (setq _utc_year (round __utc_year))
   (setq _utc_month (round __utc_month))
   (setq _utc_day (round __utc_day))
   (setq _utc_hour (round __utc_hour))
   (setq _utc_minute (round __utc_minute))
   (setq _utc_millisecond (round __utc_millisecond))
   (setq _utc_status (string __utc_status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:clock_status
   (&optional __clock_status)
   (if __clock_status (setq _clock_status __clock_status)) _clock_status)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:offset_std
   (&optional __offset_std)
   (if __offset_std (setq _offset_std __offset_std)) _offset_std)
  (:utc_offset
   (&optional __utc_offset)
   (if __utc_offset (setq _utc_offset __utc_offset)) _utc_offset)
  (:utc_year
   (&optional __utc_year)
   (if __utc_year (setq _utc_year __utc_year)) _utc_year)
  (:utc_month
   (&optional __utc_month)
   (if __utc_month (setq _utc_month __utc_month)) _utc_month)
  (:utc_day
   (&optional __utc_day)
   (if __utc_day (setq _utc_day __utc_day)) _utc_day)
  (:utc_hour
   (&optional __utc_hour)
   (if __utc_hour (setq _utc_hour __utc_hour)) _utc_hour)
  (:utc_minute
   (&optional __utc_minute)
   (if __utc_minute (setq _utc_minute __utc_minute)) _utc_minute)
  (:utc_millisecond
   (&optional __utc_millisecond)
   (if __utc_millisecond (setq _utc_millisecond __utc_millisecond)) _utc_millisecond)
  (:utc_status
   (&optional __utc_status)
   (if __utc_status (setq _utc_status __utc_status)) _utc_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _clock_status
    4 (length _clock_status)
    ;; float64 _offset
    8
    ;; float64 _offset_std
    8
    ;; float64 _utc_offset
    8
    ;; uint32 _utc_year
    4
    ;; uint8 _utc_month
    1
    ;; uint8 _utc_day
    1
    ;; uint8 _utc_hour
    1
    ;; uint8 _utc_minute
    1
    ;; uint32 _utc_millisecond
    4
    ;; string _utc_status
    4 (length _utc_status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _clock_status
       (write-long (length _clock_status) s) (princ _clock_status s)
     ;; float64 _offset
       (sys::poke _offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _offset_std
       (sys::poke _offset_std (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utc_offset
       (sys::poke _utc_offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _utc_year
       (write-long _utc_year s)
     ;; uint8 _utc_month
       (write-byte _utc_month s)
     ;; uint8 _utc_day
       (write-byte _utc_day s)
     ;; uint8 _utc_hour
       (write-byte _utc_hour s)
     ;; uint8 _utc_minute
       (write-byte _utc_minute s)
     ;; uint32 _utc_millisecond
       (write-long _utc_millisecond s)
     ;; string _utc_status
       (write-long (length _utc_status) s) (princ _utc_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _clock_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _clock_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _offset
     (setq _offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _offset_std
     (setq _offset_std (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utc_offset
     (setq _utc_offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _utc_year
     (setq _utc_year (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _utc_month
     (setq _utc_month (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _utc_day
     (setq _utc_day (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _utc_hour
     (setq _utc_hour (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _utc_minute
     (setq _utc_minute (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _utc_millisecond
     (setq _utc_millisecond (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _utc_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _utc_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Time :md5sum-) "65d339585d71de8242304ff93e8a4f1a")
(setf (get novatel_gps_msgs::Time :datatype-) "novatel_gps_msgs/Time")
(setf (get novatel_gps_msgs::Time :definition-)
      "# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver
Header header

# Clock model status
# see Table 65 on pg 322 of the OEM6 Family Firmware Reference Manual, Rev3
string clock_status

# Receiver clock offset, in seconds, from GPS reference time. A positive offset
# implies that the receiver clock is ahead of GPS reference time. To derive
# GPS reference time, use the following formula:
# GPS reference time = receiver time - offset
float64 offset

# Standard deviation of the offset
float64 offset_std

# The offset of the GPS reference time from UTC time, computed using almanac
# parameters. UTC time is GPS reference time plus the current UTC offset plus
# the receiver clock offset:
# UTC time = GPS reference time + offset + UTC offset
float64 utc_offset

uint32 utc_year
uint8 utc_month
uint8 utc_day
uint8 utc_hour
uint8 utc_minute
uint32 utc_millisecond

string utc_status
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



(provide :novatel_gps_msgs/Time "65d339585d71de8242304ff93e8a4f1a")


