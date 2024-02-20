;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Gprmc)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Gprmc (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::GPRMC")
  (make-package "NOVATEL_GPS_MSGS::GPRMC"))

(in-package "ROS")
;;//! \htmlinclude Gprmc.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Gprmc
  :super ros::object
  :slots (_header _message_id _utc_seconds _position_status _lat _lon _lat_dir _lon_dir _speed _track _date _mag_var _mag_var_direction _mode_indicator ))

(defmethod novatel_gps_msgs::Gprmc
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) "")
    ((:utc_seconds __utc_seconds) 0.0)
    ((:position_status __position_status) "")
    ((:lat __lat) 0.0)
    ((:lon __lon) 0.0)
    ((:lat_dir __lat_dir) "")
    ((:lon_dir __lon_dir) "")
    ((:speed __speed) 0.0)
    ((:track __track) 0.0)
    ((:date __date) "")
    ((:mag_var __mag_var) 0.0)
    ((:mag_var_direction __mag_var_direction) "")
    ((:mode_indicator __mode_indicator) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (string __message_id))
   (setq _utc_seconds (float __utc_seconds))
   (setq _position_status (string __position_status))
   (setq _lat (float __lat))
   (setq _lon (float __lon))
   (setq _lat_dir (string __lat_dir))
   (setq _lon_dir (string __lon_dir))
   (setq _speed (float __speed))
   (setq _track (float __track))
   (setq _date (string __date))
   (setq _mag_var (float __mag_var))
   (setq _mag_var_direction (string __mag_var_direction))
   (setq _mode_indicator (string __mode_indicator))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:message_id
   (&optional __message_id)
   (if __message_id (setq _message_id __message_id)) _message_id)
  (:utc_seconds
   (&optional __utc_seconds)
   (if __utc_seconds (setq _utc_seconds __utc_seconds)) _utc_seconds)
  (:position_status
   (&optional __position_status)
   (if __position_status (setq _position_status __position_status)) _position_status)
  (:lat
   (&optional __lat)
   (if __lat (setq _lat __lat)) _lat)
  (:lon
   (&optional __lon)
   (if __lon (setq _lon __lon)) _lon)
  (:lat_dir
   (&optional __lat_dir)
   (if __lat_dir (setq _lat_dir __lat_dir)) _lat_dir)
  (:lon_dir
   (&optional __lon_dir)
   (if __lon_dir (setq _lon_dir __lon_dir)) _lon_dir)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:track
   (&optional __track)
   (if __track (setq _track __track)) _track)
  (:date
   (&optional __date)
   (if __date (setq _date __date)) _date)
  (:mag_var
   (&optional __mag_var)
   (if __mag_var (setq _mag_var __mag_var)) _mag_var)
  (:mag_var_direction
   (&optional __mag_var_direction)
   (if __mag_var_direction (setq _mag_var_direction __mag_var_direction)) _mag_var_direction)
  (:mode_indicator
   (&optional __mode_indicator)
   (if __mode_indicator (setq _mode_indicator __mode_indicator)) _mode_indicator)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _message_id
    4 (length _message_id)
    ;; float64 _utc_seconds
    8
    ;; string _position_status
    4 (length _position_status)
    ;; float64 _lat
    8
    ;; float64 _lon
    8
    ;; string _lat_dir
    4 (length _lat_dir)
    ;; string _lon_dir
    4 (length _lon_dir)
    ;; float32 _speed
    4
    ;; float32 _track
    4
    ;; string _date
    4 (length _date)
    ;; float32 _mag_var
    4
    ;; string _mag_var_direction
    4 (length _mag_var_direction)
    ;; string _mode_indicator
    4 (length _mode_indicator)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _message_id
       (write-long (length _message_id) s) (princ _message_id s)
     ;; float64 _utc_seconds
       (sys::poke _utc_seconds (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _position_status
       (write-long (length _position_status) s) (princ _position_status s)
     ;; float64 _lat
       (sys::poke _lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lon
       (sys::poke _lon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _lat_dir
       (write-long (length _lat_dir) s) (princ _lat_dir s)
     ;; string _lon_dir
       (write-long (length _lon_dir) s) (princ _lon_dir s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _track
       (sys::poke _track (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _date
       (write-long (length _date) s) (princ _date s)
     ;; float32 _mag_var
       (sys::poke _mag_var (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _mag_var_direction
       (write-long (length _mag_var_direction) s) (princ _mag_var_direction s)
     ;; string _mode_indicator
       (write-long (length _mode_indicator) s) (princ _mode_indicator s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _message_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _utc_seconds
     (setq _utc_seconds (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _position_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _position_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _lat
     (setq _lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lon
     (setq _lon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _lat_dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lat_dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lon_dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lon_dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _track
     (setq _track (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _date
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _date (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _mag_var
     (setq _mag_var (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _mag_var_direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mag_var_direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _mode_indicator
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode_indicator (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Gprmc :md5sum-) "02533bac67f17457b2e3538525ba1aae")
(setf (get novatel_gps_msgs::Gprmc :datatype-) "novatel_gps_msgs/Gprmc")
(setf (get novatel_gps_msgs::Gprmc :definition-)
      "# Message from GPRMC NMEA String
Header header

string message_id

float64 utc_seconds
string position_status

float64 lat
float64 lon

string lat_dir
string lon_dir

float32 speed
float32 track
string date
float32 mag_var
string mag_var_direction
string mode_indicator
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



(provide :novatel_gps_msgs/Gprmc "02533bac67f17457b2e3538525ba1aae")


