;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Gpgsa)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Gpgsa (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::GPGSA")
  (make-package "NOVATEL_GPS_MSGS::GPGSA"))

(in-package "ROS")
;;//! \htmlinclude Gpgsa.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Gpgsa
  :super ros::object
  :slots (_header _message_id _auto_manual_mode _fix_mode _sv_ids _pdop _hdop _vdop ))

(defmethod novatel_gps_msgs::Gpgsa
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) "")
    ((:auto_manual_mode __auto_manual_mode) "")
    ((:fix_mode __fix_mode) 0)
    ((:sv_ids __sv_ids) (make-array 0 :initial-element 0 :element-type :char))
    ((:pdop __pdop) 0.0)
    ((:hdop __hdop) 0.0)
    ((:vdop __vdop) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (string __message_id))
   (setq _auto_manual_mode (string __auto_manual_mode))
   (setq _fix_mode (round __fix_mode))
   (setq _sv_ids __sv_ids)
   (setq _pdop (float __pdop))
   (setq _hdop (float __hdop))
   (setq _vdop (float __vdop))
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
  (:auto_manual_mode
   (&optional __auto_manual_mode)
   (if __auto_manual_mode (setq _auto_manual_mode __auto_manual_mode)) _auto_manual_mode)
  (:fix_mode
   (&optional __fix_mode)
   (if __fix_mode (setq _fix_mode __fix_mode)) _fix_mode)
  (:sv_ids
   (&optional __sv_ids)
   (if __sv_ids (setq _sv_ids __sv_ids)) _sv_ids)
  (:pdop
   (&optional __pdop)
   (if __pdop (setq _pdop __pdop)) _pdop)
  (:hdop
   (&optional __hdop)
   (if __hdop (setq _hdop __hdop)) _hdop)
  (:vdop
   (&optional __vdop)
   (if __vdop (setq _vdop __vdop)) _vdop)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _message_id
    4 (length _message_id)
    ;; string _auto_manual_mode
    4 (length _auto_manual_mode)
    ;; uint8 _fix_mode
    1
    ;; uint8[] _sv_ids
    (* 1    (length _sv_ids)) 4
    ;; float32 _pdop
    4
    ;; float32 _hdop
    4
    ;; float32 _vdop
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _message_id
       (write-long (length _message_id) s) (princ _message_id s)
     ;; string _auto_manual_mode
       (write-long (length _auto_manual_mode) s) (princ _auto_manual_mode s)
     ;; uint8 _fix_mode
       (write-byte _fix_mode s)
     ;; uint8[] _sv_ids
     (write-long (length _sv_ids) s)
     (princ _sv_ids s)
     ;; float32 _pdop
       (sys::poke _pdop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hdop
       (sys::poke _hdop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vdop
       (sys::poke _vdop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _message_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _auto_manual_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _auto_manual_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _fix_mode
     (setq _fix_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _sv_ids
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _sv_ids (make-array n :element-type :char))
     (replace _sv_ids buf :start2 ptr-) (incf ptr- n))
   ;; float32 _pdop
     (setq _pdop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hdop
     (setq _hdop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vdop
     (setq _vdop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Gpgsa :md5sum-) "94a6ef4a36d322374b16097a5d03433e")
(setf (get novatel_gps_msgs::Gpgsa :datatype-) "novatel_gps_msgs/Gpgsa")
(setf (get novatel_gps_msgs::Gpgsa :definition-)
      "# Message from GPGSA NMEA String
Header header

string message_id

string auto_manual_mode
uint8 fix_mode

uint8[] sv_ids

float32 pdop
float32 hdop
float32 vdop
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



(provide :novatel_gps_msgs/Gpgsa "94a6ef4a36d322374b16097a5d03433e")


