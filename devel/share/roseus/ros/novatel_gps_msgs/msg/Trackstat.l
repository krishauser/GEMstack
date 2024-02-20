;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Trackstat)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Trackstat (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::TRACKSTAT")
  (make-package "NOVATEL_GPS_MSGS::TRACKSTAT"))

(in-package "ROS")
;;//! \htmlinclude Trackstat.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Trackstat
  :super ros::object
  :slots (_header _solution_status _position_type _cutoff _channels ))

(defmethod novatel_gps_msgs::Trackstat
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:solution_status __solution_status) "")
    ((:position_type __position_type) "")
    ((:cutoff __cutoff) 0.0)
    ((:channels __channels) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _solution_status (string __solution_status))
   (setq _position_type (string __position_type))
   (setq _cutoff (float __cutoff))
   (setq _channels __channels)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:solution_status
   (&optional __solution_status)
   (if __solution_status (setq _solution_status __solution_status)) _solution_status)
  (:position_type
   (&optional __position_type)
   (if __position_type (setq _position_type __position_type)) _position_type)
  (:cutoff
   (&optional __cutoff)
   (if __cutoff (setq _cutoff __cutoff)) _cutoff)
  (:channels
   (&rest __channels)
   (if (keywordp (car __channels))
       (send* _channels __channels)
     (progn
       (if __channels (setq _channels (car __channels)))
       _channels)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _solution_status
    4 (length _solution_status)
    ;; string _position_type
    4 (length _position_type)
    ;; float32 _cutoff
    4
    ;; novatel_gps_msgs/TrackstatChannel[] _channels
    (apply #'+ (send-all _channels :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _solution_status
       (write-long (length _solution_status) s) (princ _solution_status s)
     ;; string _position_type
       (write-long (length _position_type) s) (princ _position_type s)
     ;; float32 _cutoff
       (sys::poke _cutoff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; novatel_gps_msgs/TrackstatChannel[] _channels
     (write-long (length _channels) s)
     (dolist (elem _channels)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _solution_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _solution_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _position_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _position_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _cutoff
     (setq _cutoff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; novatel_gps_msgs/TrackstatChannel[] _channels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _channels (let (r) (dotimes (i n) (push (instance novatel_gps_msgs::TrackstatChannel :init) r)) r))
     (dolist (elem- _channels)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Trackstat :md5sum-) "10e52c1ea54daca4de3c8cdda3a79817")
(setf (get novatel_gps_msgs::Trackstat :datatype-) "novatel_gps_msgs/Trackstat")
(setf (get novatel_gps_msgs::Trackstat :definition-)
      "# Channel tracking status information for each of the receiver parallel channels

Header header

string solution_status
string position_type

# Tracking elevation cutff-off angle
float32 cutoff

TrackstatChannel[] channels
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
MSG: novatel_gps_msgs/TrackstatChannel
# A submessage of Trackstat that contains all of the data about a single hardware channel

# Satellite PRN number
int16 prn

# GLONASS Frequency +7
int16 glofreq

# Channel tracking status
uint32 ch_tr_status

# Pseudorange (m)
float64 psr

# Doppler frequency (Hz)
float32 doppler

# Carrier to noise density ratio (dB-Hz)
float32 c_no

# Number of seconds of continuous tracking (no cycle slips)
float32 locktime

# Pseudorange residual from pseudorange filter (m)
float32 psr_res

# Range reject code from pseudorange filter
string reject

# Pseudorange filter weighting
float32 psr_weight
")



(provide :novatel_gps_msgs/Trackstat "10e52c1ea54daca4de3c8cdda3a79817")


