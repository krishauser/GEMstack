;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Gpgga)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Gpgga (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::GPGGA")
  (make-package "NOVATEL_GPS_MSGS::GPGGA"))

(in-package "ROS")
;;//! \htmlinclude Gpgga.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GPS_QUAL_INVALID*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_INVALID* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_INVALID* 0)
(intern "*GPS_QUAL_SINGLE_POINT*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_SINGLE_POINT* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_SINGLE_POINT* 1)
(intern "*GPS_QUAL_PSEUDORANGE_DIFFERENTIAL*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_PSEUDORANGE_DIFFERENTIAL* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_PSEUDORANGE_DIFFERENTIAL* 2)
(intern "*GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION* 4)
(intern "*GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION* 5)
(intern "*GPS_QUAL_DEAD_RECKONING_MODE*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_DEAD_RECKONING_MODE* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_DEAD_RECKONING_MODE* 6)
(intern "*GPS_QUAL_MANUAL_INPUT_MODE*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_MANUAL_INPUT_MODE* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_MANUAL_INPUT_MODE* 7)
(intern "*GPS_QUAL_SIMULATION_MODE*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_SIMULATION_MODE* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_SIMULATION_MODE* 8)
(intern "*GPS_QUAL_WASS*" (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(shadow '*GPS_QUAL_WASS* (find-package "NOVATEL_GPS_MSGS::GPGGA"))
(defconstant novatel_gps_msgs::Gpgga::*GPS_QUAL_WASS* 9)

(defun novatel_gps_msgs::Gpgga-to-symbol (const)
  (cond
        ((= const 0) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_INVALID*)
        ((= const 1) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_SINGLE_POINT*)
        ((= const 2) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_PSEUDORANGE_DIFFERENTIAL*)
        ((= const 4) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION*)
        ((= const 5) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION*)
        ((= const 6) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_DEAD_RECKONING_MODE*)
        ((= const 7) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_MANUAL_INPUT_MODE*)
        ((= const 8) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_SIMULATION_MODE*)
        ((= const 9) 'novatel_gps_msgs::Gpgga::*GPS_QUAL_WASS*)
        (t nil)))

(defclass novatel_gps_msgs::Gpgga
  :super ros::object
  :slots (_header _message_id _utc_seconds _lat _lon _lat_dir _lon_dir _gps_qual _num_sats _hdop _alt _altitude_units _undulation _undulation_units _diff_age _station_id ))

(defmethod novatel_gps_msgs::Gpgga
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) "")
    ((:utc_seconds __utc_seconds) 0.0)
    ((:lat __lat) 0.0)
    ((:lon __lon) 0.0)
    ((:lat_dir __lat_dir) "")
    ((:lon_dir __lon_dir) "")
    ((:gps_qual __gps_qual) 0)
    ((:num_sats __num_sats) 0)
    ((:hdop __hdop) 0.0)
    ((:alt __alt) 0.0)
    ((:altitude_units __altitude_units) "")
    ((:undulation __undulation) 0.0)
    ((:undulation_units __undulation_units) "")
    ((:diff_age __diff_age) 0)
    ((:station_id __station_id) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (string __message_id))
   (setq _utc_seconds (float __utc_seconds))
   (setq _lat (float __lat))
   (setq _lon (float __lon))
   (setq _lat_dir (string __lat_dir))
   (setq _lon_dir (string __lon_dir))
   (setq _gps_qual (round __gps_qual))
   (setq _num_sats (round __num_sats))
   (setq _hdop (float __hdop))
   (setq _alt (float __alt))
   (setq _altitude_units (string __altitude_units))
   (setq _undulation (float __undulation))
   (setq _undulation_units (string __undulation_units))
   (setq _diff_age (round __diff_age))
   (setq _station_id (string __station_id))
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
  (:gps_qual
   (&optional __gps_qual)
   (if __gps_qual (setq _gps_qual __gps_qual)) _gps_qual)
  (:num_sats
   (&optional __num_sats)
   (if __num_sats (setq _num_sats __num_sats)) _num_sats)
  (:hdop
   (&optional __hdop)
   (if __hdop (setq _hdop __hdop)) _hdop)
  (:alt
   (&optional __alt)
   (if __alt (setq _alt __alt)) _alt)
  (:altitude_units
   (&optional __altitude_units)
   (if __altitude_units (setq _altitude_units __altitude_units)) _altitude_units)
  (:undulation
   (&optional __undulation)
   (if __undulation (setq _undulation __undulation)) _undulation)
  (:undulation_units
   (&optional __undulation_units)
   (if __undulation_units (setq _undulation_units __undulation_units)) _undulation_units)
  (:diff_age
   (&optional __diff_age)
   (if __diff_age (setq _diff_age __diff_age)) _diff_age)
  (:station_id
   (&optional __station_id)
   (if __station_id (setq _station_id __station_id)) _station_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _message_id
    4 (length _message_id)
    ;; float64 _utc_seconds
    8
    ;; float64 _lat
    8
    ;; float64 _lon
    8
    ;; string _lat_dir
    4 (length _lat_dir)
    ;; string _lon_dir
    4 (length _lon_dir)
    ;; uint32 _gps_qual
    4
    ;; uint32 _num_sats
    4
    ;; float32 _hdop
    4
    ;; float32 _alt
    4
    ;; string _altitude_units
    4 (length _altitude_units)
    ;; float32 _undulation
    4
    ;; string _undulation_units
    4 (length _undulation_units)
    ;; uint32 _diff_age
    4
    ;; string _station_id
    4 (length _station_id)
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
     ;; float64 _lat
       (sys::poke _lat (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lon
       (sys::poke _lon (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _lat_dir
       (write-long (length _lat_dir) s) (princ _lat_dir s)
     ;; string _lon_dir
       (write-long (length _lon_dir) s) (princ _lon_dir s)
     ;; uint32 _gps_qual
       (write-long _gps_qual s)
     ;; uint32 _num_sats
       (write-long _num_sats s)
     ;; float32 _hdop
       (sys::poke _hdop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _alt
       (sys::poke _alt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _altitude_units
       (write-long (length _altitude_units) s) (princ _altitude_units s)
     ;; float32 _undulation
       (sys::poke _undulation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _undulation_units
       (write-long (length _undulation_units) s) (princ _undulation_units s)
     ;; uint32 _diff_age
       (write-long _diff_age s)
     ;; string _station_id
       (write-long (length _station_id) s) (princ _station_id s)
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
   ;; float64 _lat
     (setq _lat (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lon
     (setq _lon (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _lat_dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lat_dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lon_dir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lon_dir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _gps_qual
     (setq _gps_qual (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _num_sats
     (setq _num_sats (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _hdop
     (setq _hdop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _alt
     (setq _alt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _altitude_units
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _altitude_units (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _undulation
     (setq _undulation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _undulation_units
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _undulation_units (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _diff_age
     (setq _diff_age (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _station_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _station_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Gpgga :md5sum-) "bc1ad0f59948d0d18a275b656db48121")
(setf (get novatel_gps_msgs::Gpgga :datatype-) "novatel_gps_msgs/Gpgga")
(setf (get novatel_gps_msgs::Gpgga :definition-)
      "# Message from GPGGA NMEA String
# Based on https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds

float64 lat
float64 lon

string lat_dir
string lon_dir

uint32 GPS_QUAL_INVALID=0
uint32 GPS_QUAL_SINGLE_POINT=1
uint32 GPS_QUAL_PSEUDORANGE_DIFFERENTIAL=2
uint32 GPS_QUAL_RTK_FIXED_AMBIGUITY_SOLUTION=4
uint32 GPS_QUAL_RTK_FLOATING_AMBIGUITY_SOLUTION=5
uint32 GPS_QUAL_DEAD_RECKONING_MODE=6
uint32 GPS_QUAL_MANUAL_INPUT_MODE=7
uint32 GPS_QUAL_SIMULATION_MODE=8
uint32 GPS_QUAL_WASS=9
uint32 gps_qual

uint32 num_sats
float32 hdop 
float32 alt
string altitude_units

float32 undulation
string undulation_units
uint32 diff_age
string station_id


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



(provide :novatel_gps_msgs/Gpgga "bc1ad0f59948d0d18a275b656db48121")


