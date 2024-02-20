;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Gpgsv)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Gpgsv (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::GPGSV")
  (make-package "NOVATEL_GPS_MSGS::GPGSV"))

(in-package "ROS")
;;//! \htmlinclude Gpgsv.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Gpgsv
  :super ros::object
  :slots (_header _message_id _n_msgs _msg_number _n_satellites _satellites ))

(defmethod novatel_gps_msgs::Gpgsv
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) "")
    ((:n_msgs __n_msgs) 0)
    ((:msg_number __msg_number) 0)
    ((:n_satellites __n_satellites) 0)
    ((:satellites __satellites) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (string __message_id))
   (setq _n_msgs (round __n_msgs))
   (setq _msg_number (round __msg_number))
   (setq _n_satellites (round __n_satellites))
   (setq _satellites __satellites)
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
  (:n_msgs
   (&optional __n_msgs)
   (if __n_msgs (setq _n_msgs __n_msgs)) _n_msgs)
  (:msg_number
   (&optional __msg_number)
   (if __msg_number (setq _msg_number __msg_number)) _msg_number)
  (:n_satellites
   (&optional __n_satellites)
   (if __n_satellites (setq _n_satellites __n_satellites)) _n_satellites)
  (:satellites
   (&rest __satellites)
   (if (keywordp (car __satellites))
       (send* _satellites __satellites)
     (progn
       (if __satellites (setq _satellites (car __satellites)))
       _satellites)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _message_id
    4 (length _message_id)
    ;; uint8 _n_msgs
    1
    ;; uint8 _msg_number
    1
    ;; uint8 _n_satellites
    1
    ;; novatel_gps_msgs/Satellite[] _satellites
    (apply #'+ (send-all _satellites :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _message_id
       (write-long (length _message_id) s) (princ _message_id s)
     ;; uint8 _n_msgs
       (write-byte _n_msgs s)
     ;; uint8 _msg_number
       (write-byte _msg_number s)
     ;; uint8 _n_satellites
       (write-byte _n_satellites s)
     ;; novatel_gps_msgs/Satellite[] _satellites
     (write-long (length _satellites) s)
     (dolist (elem _satellites)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _message_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _n_msgs
     (setq _n_msgs (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _msg_number
     (setq _msg_number (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _n_satellites
     (setq _n_satellites (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; novatel_gps_msgs/Satellite[] _satellites
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _satellites (let (r) (dotimes (i n) (push (instance novatel_gps_msgs::Satellite :init) r)) r))
     (dolist (elem- _satellites)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Gpgsv :md5sum-) "6f34bebc32fe085313c942a96fd39c77")
(setf (get novatel_gps_msgs::Gpgsv :datatype-) "novatel_gps_msgs/Gpgsv")
(setf (get novatel_gps_msgs::Gpgsv :definition-)
      "# Total number of satellites in view and data about satellites
# Because the NMEA sentence is limited to 4 satellites per message, several
# of these messages may need to be synthesized to get data about all visible
# satellites.

Header header

string message_id

# Number of messages in this sequence
uint8 n_msgs
# This messages number in its sequence. The first message is number 1.
uint8 msg_number

# Number of satellites currently visible
uint8 n_satellites

# Up to 4 satellites are described in each message
Satellite[] satellites
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
MSG: novatel_gps_msgs/Satellite
# Satellite data structure used in GPGSV messages

# PRN number of the satellite
# GPS = 1..32
# SBAS = 33..64
# GLO = 65..96
uint8 prn

# Elevation, degrees. Maximum 90
uint8 elevation

# Azimuth, True North degrees. [0, 359]
uint16 azimuth

# Signal to noise ratio, 0-99 dB. -1 when null in NMEA sentence (not tracking)
int8 snr
")



(provide :novatel_gps_msgs/Gpgsv "6f34bebc32fe085313c942a96fd39c77")


