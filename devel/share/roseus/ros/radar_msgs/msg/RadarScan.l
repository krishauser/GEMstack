;; Auto-generated. Do not edit!


(when (boundp 'radar_msgs::RadarScan)
  (if (not (find-package "RADAR_MSGS"))
    (make-package "RADAR_MSGS"))
  (shadow 'RadarScan (find-package "RADAR_MSGS")))
(unless (find-package "RADAR_MSGS::RADARSCAN")
  (make-package "RADAR_MSGS::RADARSCAN"))

(in-package "ROS")
;;//! \htmlinclude RadarScan.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass radar_msgs::RadarScan
  :super ros::object
  :slots (_header _returns ))

(defmethod radar_msgs::RadarScan
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:returns __returns) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _returns __returns)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:returns
   (&rest __returns)
   (if (keywordp (car __returns))
       (send* _returns __returns)
     (progn
       (if __returns (setq _returns (car __returns)))
       _returns)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; radar_msgs/RadarReturn[] _returns
    (apply #'+ (send-all _returns :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; radar_msgs/RadarReturn[] _returns
     (write-long (length _returns) s)
     (dolist (elem _returns)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; radar_msgs/RadarReturn[] _returns
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _returns (let (r) (dotimes (i n) (push (instance radar_msgs::RadarReturn :init) r)) r))
     (dolist (elem- _returns)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get radar_msgs::RadarScan :md5sum-) "6dfacef1e665538dbd8e159d5ce7a97a")
(setf (get radar_msgs::RadarScan :datatype-) "radar_msgs/RadarScan")
(setf (get radar_msgs::RadarScan :definition-)
      "std_msgs/Header header

radar_msgs/RadarReturn[] returns

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
MSG: radar_msgs/RadarReturn
# All variables below are relative to the radar's frame of reference.
# This message is not meant to be used alone but as part of a stamped or array message.

float32 range                            # Distance (m) from the sensor to the detected return.
float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.
                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.
float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.
                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.
float32 doppler_velocity                 # The doppler speeds (m/s) of the return.
float32 amplitude                        # The amplitude of the of the return (dB)

")



(provide :radar_msgs/RadarScan "6dfacef1e665538dbd8e159d5ce7a97a")


