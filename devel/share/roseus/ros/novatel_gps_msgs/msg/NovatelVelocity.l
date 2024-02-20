;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelVelocity)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelVelocity (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELVELOCITY")
  (make-package "NOVATEL_GPS_MSGS::NOVATELVELOCITY"))

(in-package "ROS")
;;//! \htmlinclude NovatelVelocity.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::NovatelVelocity
  :super ros::object
  :slots (_header _novatel_msg_header _solution_status _velocity_type _latency _age _horizontal_speed _track_ground _vertical_speed ))

(defmethod novatel_gps_msgs::NovatelVelocity
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:novatel_msg_header __novatel_msg_header) (instance novatel_gps_msgs::NovatelMessageHeader :init))
    ((:solution_status __solution_status) "")
    ((:velocity_type __velocity_type) "")
    ((:latency __latency) 0.0)
    ((:age __age) 0.0)
    ((:horizontal_speed __horizontal_speed) 0.0)
    ((:track_ground __track_ground) 0.0)
    ((:vertical_speed __vertical_speed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _novatel_msg_header __novatel_msg_header)
   (setq _solution_status (string __solution_status))
   (setq _velocity_type (string __velocity_type))
   (setq _latency (float __latency))
   (setq _age (float __age))
   (setq _horizontal_speed (float __horizontal_speed))
   (setq _track_ground (float __track_ground))
   (setq _vertical_speed (float __vertical_speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:novatel_msg_header
   (&rest __novatel_msg_header)
   (if (keywordp (car __novatel_msg_header))
       (send* _novatel_msg_header __novatel_msg_header)
     (progn
       (if __novatel_msg_header (setq _novatel_msg_header (car __novatel_msg_header)))
       _novatel_msg_header)))
  (:solution_status
   (&optional __solution_status)
   (if __solution_status (setq _solution_status __solution_status)) _solution_status)
  (:velocity_type
   (&optional __velocity_type)
   (if __velocity_type (setq _velocity_type __velocity_type)) _velocity_type)
  (:latency
   (&optional __latency)
   (if __latency (setq _latency __latency)) _latency)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:horizontal_speed
   (&optional __horizontal_speed)
   (if __horizontal_speed (setq _horizontal_speed __horizontal_speed)) _horizontal_speed)
  (:track_ground
   (&optional __track_ground)
   (if __track_ground (setq _track_ground __track_ground)) _track_ground)
  (:vertical_speed
   (&optional __vertical_speed)
   (if __vertical_speed (setq _vertical_speed __vertical_speed)) _vertical_speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
    (send _novatel_msg_header :serialization-length)
    ;; string _solution_status
    4 (length _solution_status)
    ;; string _velocity_type
    4 (length _velocity_type)
    ;; float32 _latency
    4
    ;; float32 _age
    4
    ;; float64 _horizontal_speed
    8
    ;; float64 _track_ground
    8
    ;; float64 _vertical_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
       (send _novatel_msg_header :serialize s)
     ;; string _solution_status
       (write-long (length _solution_status) s) (princ _solution_status s)
     ;; string _velocity_type
       (write-long (length _velocity_type) s) (princ _velocity_type s)
     ;; float32 _latency
       (sys::poke _latency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _age
       (sys::poke _age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _horizontal_speed
       (sys::poke _horizontal_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _track_ground
       (sys::poke _track_ground (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_speed
       (sys::poke _vertical_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
     (send _novatel_msg_header :deserialize buf ptr-) (incf ptr- (send _novatel_msg_header :serialization-length))
   ;; string _solution_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _solution_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _velocity_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _velocity_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _latency
     (setq _latency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _age
     (setq _age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _horizontal_speed
     (setq _horizontal_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _track_ground
     (setq _track_ground (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_speed
     (setq _vertical_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelVelocity :md5sum-) "a8fb7d9232aaf68f98caa2b4cda2597b")
(setf (get novatel_gps_msgs::NovatelVelocity :datatype-) "novatel_gps_msgs/NovatelVelocity")
(setf (get novatel_gps_msgs::NovatelVelocity :definition-)
      "# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver
Header header

NovatelMessageHeader novatel_msg_header

string solution_status
string velocity_type

# A measure of the latency in the velocity time tag in seconds.
float32 latency

# Differential age in seconds
float32 age

# Horizontal speed over ground, meters/second
float64 horizontal_speed

# Direction of motion over ground with respect to true North, degrees
float64 track_ground

# Vertical speed, where positive values correspond to increasing altitude, meters/second
float64 vertical_speed
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
MSG: novatel_gps_msgs/NovatelMessageHeader
# Novatel Message Header

string message_name
string port
uint32 sequence_num
float32 percent_idle_time
string gps_time_status
uint32 gps_week_num
float64 gps_seconds

# Bit       Mask      Description
#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)
#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)
#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)
#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)
#  4     0x00000010   <Reserved>
#  5     0x00000020   Antenna open flag (0: OK, 1: Open)
#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)
#  7     0x00000080   CPU overload flag
#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)
#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)
#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)
#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)
#  12    0x00001000   <Reserved>
#  13    0x00002000   <Reserved>
#  14    0x00004000   <Reserved>
#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)
#  16    0x00010000   <Reserverd>
#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)
#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)
#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)
#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)
#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)
#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)
#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)
#  24    0x01000000   Software resource (0: OK, 1: Warning)
#  25    0x02000000   <Reserved>
#  26    0x04000000   <Reserved>
#  27    0x08000000   <Reserved>
#  28    0x10000000   <Reserved>
#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)
#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)
#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)
NovatelReceiverStatus receiver_status

# Receiver build number (0-65535)
uint32 receiver_software_version
================================================================================
MSG: novatel_gps_msgs/NovatelReceiverStatus
# From the original Novatel receiver status message bitfield
#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)
#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)
#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)
#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)
#  4     0x00000010   <Reserved>
#  5     0x00000020   Antenna open flag (0: OK, 1: Open)
#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)
#  7     0x00000080   CPU overload flag
#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)
#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)
#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)
#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)
#  12    0x00001000   <Reserved>
#  13    0x00002000   <Reserved>
#  14    0x00004000   <Reserved>
#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)
#  16    0x00010000   <Reserverd>
#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)
#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)
#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)
#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)
#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)
#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)
#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)
#  24    0x01000000   Software resource (0: OK, 1: Warning)
#  25    0x02000000   <Reserved>
#  26    0x04000000   <Reserved>
#  27    0x08000000   <Reserved>
#  28    0x10000000   <Reserved>
#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)
#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)
#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)
uint32 original_status_code
bool error_flag
bool temperature_flag
bool voltage_supply_flag
bool antenna_powered
bool antenna_is_open
bool antenna_is_shorted
bool cpu_overload_flag
bool com1_buffer_overrun
bool com2_buffer_overrun
bool com3_buffer_overrun
bool usb_buffer_overrun
bool rf1_agc_flag
bool rf2_agc_flag
bool almanac_flag
bool position_solution_flag
bool position_fixed_flag
bool clock_steering_status_enabled
bool clock_model_flag
bool oemv_external_oscillator_flag
bool software_resource_flag
bool aux1_status_event_flag
bool aux2_status_event_flag
bool aux3_status_event_flag


")



(provide :novatel_gps_msgs/NovatelVelocity "a8fb7d9232aaf68f98caa2b4cda2597b")


