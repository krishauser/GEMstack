;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Gphdt)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Gphdt (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::GPHDT")
  (make-package "NOVATEL_GPS_MSGS::GPHDT"))

(in-package "ROS")
;;//! \htmlinclude Gphdt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Gphdt
  :super ros::object
  :slots (_header _message_id _heading _t ))

(defmethod novatel_gps_msgs::Gphdt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:message_id __message_id) "")
    ((:heading __heading) 0.0)
    ((:t __t) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _message_id (string __message_id))
   (setq _heading (float __heading))
   (setq _t (string __t))
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
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _message_id
    4 (length _message_id)
    ;; float64 _heading
    8
    ;; string _t
    4 (length _t)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _message_id
       (write-long (length _message_id) s) (princ _message_id s)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _t
       (write-long (length _t) s) (princ _t s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _message_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _t
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _t (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Gphdt :md5sum-) "ddf912a83c312999924f468d3d13a183")
(setf (get novatel_gps_msgs::Gphdt :datatype-) "novatel_gps_msgs/Gphdt")
(setf (get novatel_gps_msgs::Gphdt :definition-)
      "# Message from GPHDT NMEA String
Header header

string message_id

# Heading in degrees (clockwise)
float64 heading

# T: Indicates heading relative to True North
string t
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



(provide :novatel_gps_msgs/Gphdt "ddf912a83c312999924f468d3d13a183")


