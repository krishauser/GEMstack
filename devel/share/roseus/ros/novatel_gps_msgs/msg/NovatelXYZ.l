;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelXYZ)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelXYZ (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELXYZ")
  (make-package "NOVATEL_GPS_MSGS::NOVATELXYZ"))

(in-package "ROS")
;;//! \htmlinclude NovatelXYZ.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::NovatelXYZ
  :super ros::object
  :slots (_header _novatel_msg_header _solution_status _position_type _x _y _z _x_sigma _y_sigma _z_sigma _velocity_solution_status _velocity_type _x_vel _y_vel _z_vel _x_vel_sigma _y_vel_sigma _z_vel_sigma _base_station_id _velocity_latency _diff_age _solution_age _num_satellites_tracked _num_satellites_used_in_solution _num_gps_and_glonass_l1_used_in_solution _num_gps_and_glonass_l1_and_l2_used_in_solution _extended_solution_status _signal_mask ))

(defmethod novatel_gps_msgs::NovatelXYZ
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:novatel_msg_header __novatel_msg_header) (instance novatel_gps_msgs::NovatelMessageHeader :init))
    ((:solution_status __solution_status) "")
    ((:position_type __position_type) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:x_sigma __x_sigma) 0.0)
    ((:y_sigma __y_sigma) 0.0)
    ((:z_sigma __z_sigma) 0.0)
    ((:velocity_solution_status __velocity_solution_status) "")
    ((:velocity_type __velocity_type) "")
    ((:x_vel __x_vel) 0.0)
    ((:y_vel __y_vel) 0.0)
    ((:z_vel __z_vel) 0.0)
    ((:x_vel_sigma __x_vel_sigma) 0.0)
    ((:y_vel_sigma __y_vel_sigma) 0.0)
    ((:z_vel_sigma __z_vel_sigma) 0.0)
    ((:base_station_id __base_station_id) "")
    ((:velocity_latency __velocity_latency) 0.0)
    ((:diff_age __diff_age) 0.0)
    ((:solution_age __solution_age) 0.0)
    ((:num_satellites_tracked __num_satellites_tracked) 0)
    ((:num_satellites_used_in_solution __num_satellites_used_in_solution) 0)
    ((:num_gps_and_glonass_l1_used_in_solution __num_gps_and_glonass_l1_used_in_solution) 0)
    ((:num_gps_and_glonass_l1_and_l2_used_in_solution __num_gps_and_glonass_l1_and_l2_used_in_solution) 0)
    ((:extended_solution_status __extended_solution_status) (instance novatel_gps_msgs::NovatelExtendedSolutionStatus :init))
    ((:signal_mask __signal_mask) (instance novatel_gps_msgs::NovatelSignalMask :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _novatel_msg_header __novatel_msg_header)
   (setq _solution_status (string __solution_status))
   (setq _position_type (string __position_type))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _x_sigma (float __x_sigma))
   (setq _y_sigma (float __y_sigma))
   (setq _z_sigma (float __z_sigma))
   (setq _velocity_solution_status (string __velocity_solution_status))
   (setq _velocity_type (string __velocity_type))
   (setq _x_vel (float __x_vel))
   (setq _y_vel (float __y_vel))
   (setq _z_vel (float __z_vel))
   (setq _x_vel_sigma (float __x_vel_sigma))
   (setq _y_vel_sigma (float __y_vel_sigma))
   (setq _z_vel_sigma (float __z_vel_sigma))
   (setq _base_station_id (string __base_station_id))
   (setq _velocity_latency (float __velocity_latency))
   (setq _diff_age (float __diff_age))
   (setq _solution_age (float __solution_age))
   (setq _num_satellites_tracked (round __num_satellites_tracked))
   (setq _num_satellites_used_in_solution (round __num_satellites_used_in_solution))
   (setq _num_gps_and_glonass_l1_used_in_solution (round __num_gps_and_glonass_l1_used_in_solution))
   (setq _num_gps_and_glonass_l1_and_l2_used_in_solution (round __num_gps_and_glonass_l1_and_l2_used_in_solution))
   (setq _extended_solution_status __extended_solution_status)
   (setq _signal_mask __signal_mask)
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
  (:position_type
   (&optional __position_type)
   (if __position_type (setq _position_type __position_type)) _position_type)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:x_sigma
   (&optional __x_sigma)
   (if __x_sigma (setq _x_sigma __x_sigma)) _x_sigma)
  (:y_sigma
   (&optional __y_sigma)
   (if __y_sigma (setq _y_sigma __y_sigma)) _y_sigma)
  (:z_sigma
   (&optional __z_sigma)
   (if __z_sigma (setq _z_sigma __z_sigma)) _z_sigma)
  (:velocity_solution_status
   (&optional __velocity_solution_status)
   (if __velocity_solution_status (setq _velocity_solution_status __velocity_solution_status)) _velocity_solution_status)
  (:velocity_type
   (&optional __velocity_type)
   (if __velocity_type (setq _velocity_type __velocity_type)) _velocity_type)
  (:x_vel
   (&optional __x_vel)
   (if __x_vel (setq _x_vel __x_vel)) _x_vel)
  (:y_vel
   (&optional __y_vel)
   (if __y_vel (setq _y_vel __y_vel)) _y_vel)
  (:z_vel
   (&optional __z_vel)
   (if __z_vel (setq _z_vel __z_vel)) _z_vel)
  (:x_vel_sigma
   (&optional __x_vel_sigma)
   (if __x_vel_sigma (setq _x_vel_sigma __x_vel_sigma)) _x_vel_sigma)
  (:y_vel_sigma
   (&optional __y_vel_sigma)
   (if __y_vel_sigma (setq _y_vel_sigma __y_vel_sigma)) _y_vel_sigma)
  (:z_vel_sigma
   (&optional __z_vel_sigma)
   (if __z_vel_sigma (setq _z_vel_sigma __z_vel_sigma)) _z_vel_sigma)
  (:base_station_id
   (&optional __base_station_id)
   (if __base_station_id (setq _base_station_id __base_station_id)) _base_station_id)
  (:velocity_latency
   (&optional __velocity_latency)
   (if __velocity_latency (setq _velocity_latency __velocity_latency)) _velocity_latency)
  (:diff_age
   (&optional __diff_age)
   (if __diff_age (setq _diff_age __diff_age)) _diff_age)
  (:solution_age
   (&optional __solution_age)
   (if __solution_age (setq _solution_age __solution_age)) _solution_age)
  (:num_satellites_tracked
   (&optional __num_satellites_tracked)
   (if __num_satellites_tracked (setq _num_satellites_tracked __num_satellites_tracked)) _num_satellites_tracked)
  (:num_satellites_used_in_solution
   (&optional __num_satellites_used_in_solution)
   (if __num_satellites_used_in_solution (setq _num_satellites_used_in_solution __num_satellites_used_in_solution)) _num_satellites_used_in_solution)
  (:num_gps_and_glonass_l1_used_in_solution
   (&optional __num_gps_and_glonass_l1_used_in_solution)
   (if __num_gps_and_glonass_l1_used_in_solution (setq _num_gps_and_glonass_l1_used_in_solution __num_gps_and_glonass_l1_used_in_solution)) _num_gps_and_glonass_l1_used_in_solution)
  (:num_gps_and_glonass_l1_and_l2_used_in_solution
   (&optional __num_gps_and_glonass_l1_and_l2_used_in_solution)
   (if __num_gps_and_glonass_l1_and_l2_used_in_solution (setq _num_gps_and_glonass_l1_and_l2_used_in_solution __num_gps_and_glonass_l1_and_l2_used_in_solution)) _num_gps_and_glonass_l1_and_l2_used_in_solution)
  (:extended_solution_status
   (&rest __extended_solution_status)
   (if (keywordp (car __extended_solution_status))
       (send* _extended_solution_status __extended_solution_status)
     (progn
       (if __extended_solution_status (setq _extended_solution_status (car __extended_solution_status)))
       _extended_solution_status)))
  (:signal_mask
   (&rest __signal_mask)
   (if (keywordp (car __signal_mask))
       (send* _signal_mask __signal_mask)
     (progn
       (if __signal_mask (setq _signal_mask (car __signal_mask)))
       _signal_mask)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
    (send _novatel_msg_header :serialization-length)
    ;; string _solution_status
    4 (length _solution_status)
    ;; string _position_type
    4 (length _position_type)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float32 _x_sigma
    4
    ;; float32 _y_sigma
    4
    ;; float32 _z_sigma
    4
    ;; string _velocity_solution_status
    4 (length _velocity_solution_status)
    ;; string _velocity_type
    4 (length _velocity_type)
    ;; float64 _x_vel
    8
    ;; float64 _y_vel
    8
    ;; float64 _z_vel
    8
    ;; float32 _x_vel_sigma
    4
    ;; float32 _y_vel_sigma
    4
    ;; float32 _z_vel_sigma
    4
    ;; string _base_station_id
    4 (length _base_station_id)
    ;; float32 _velocity_latency
    4
    ;; float32 _diff_age
    4
    ;; float32 _solution_age
    4
    ;; uint8 _num_satellites_tracked
    1
    ;; uint8 _num_satellites_used_in_solution
    1
    ;; uint8 _num_gps_and_glonass_l1_used_in_solution
    1
    ;; uint8 _num_gps_and_glonass_l1_and_l2_used_in_solution
    1
    ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
    (send _extended_solution_status :serialization-length)
    ;; novatel_gps_msgs/NovatelSignalMask _signal_mask
    (send _signal_mask :serialization-length)
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
     ;; string _position_type
       (write-long (length _position_type) s) (princ _position_type s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _x_sigma
       (sys::poke _x_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_sigma
       (sys::poke _y_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_sigma
       (sys::poke _z_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _velocity_solution_status
       (write-long (length _velocity_solution_status) s) (princ _velocity_solution_status s)
     ;; string _velocity_type
       (write-long (length _velocity_type) s) (princ _velocity_type s)
     ;; float64 _x_vel
       (sys::poke _x_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_vel
       (sys::poke _y_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_vel
       (sys::poke _z_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _x_vel_sigma
       (sys::poke _x_vel_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y_vel_sigma
       (sys::poke _y_vel_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_vel_sigma
       (sys::poke _z_vel_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _base_station_id
       (write-long (length _base_station_id) s) (princ _base_station_id s)
     ;; float32 _velocity_latency
       (sys::poke _velocity_latency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _diff_age
       (sys::poke _diff_age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _solution_age
       (sys::poke _solution_age (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _num_satellites_tracked
       (write-byte _num_satellites_tracked s)
     ;; uint8 _num_satellites_used_in_solution
       (write-byte _num_satellites_used_in_solution s)
     ;; uint8 _num_gps_and_glonass_l1_used_in_solution
       (write-byte _num_gps_and_glonass_l1_used_in_solution s)
     ;; uint8 _num_gps_and_glonass_l1_and_l2_used_in_solution
       (write-byte _num_gps_and_glonass_l1_and_l2_used_in_solution s)
     ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
       (send _extended_solution_status :serialize s)
     ;; novatel_gps_msgs/NovatelSignalMask _signal_mask
       (send _signal_mask :serialize s)
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
   ;; string _position_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _position_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _x_sigma
     (setq _x_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_sigma
     (setq _y_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_sigma
     (setq _z_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _velocity_solution_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _velocity_solution_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _velocity_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _velocity_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x_vel
     (setq _x_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_vel
     (setq _y_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_vel
     (setq _z_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _x_vel_sigma
     (setq _x_vel_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y_vel_sigma
     (setq _y_vel_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_vel_sigma
     (setq _z_vel_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _base_station_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _base_station_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _velocity_latency
     (setq _velocity_latency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _diff_age
     (setq _diff_age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _solution_age
     (setq _solution_age (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _num_satellites_tracked
     (setq _num_satellites_tracked (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _num_satellites_used_in_solution
     (setq _num_satellites_used_in_solution (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _num_gps_and_glonass_l1_used_in_solution
     (setq _num_gps_and_glonass_l1_used_in_solution (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _num_gps_and_glonass_l1_and_l2_used_in_solution
     (setq _num_gps_and_glonass_l1_and_l2_used_in_solution (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
     (send _extended_solution_status :deserialize buf ptr-) (incf ptr- (send _extended_solution_status :serialization-length))
   ;; novatel_gps_msgs/NovatelSignalMask _signal_mask
     (send _signal_mask :deserialize buf ptr-) (incf ptr- (send _signal_mask :serialization-length))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelXYZ :md5sum-) "d1ffc3181aa742b6133febb9d8f77d12")
(setf (get novatel_gps_msgs::NovatelXYZ :datatype-) "novatel_gps_msgs/NovatelXYZ")
(setf (get novatel_gps_msgs::NovatelXYZ :definition-)
      "# Parsed Best Position in the WGS-84 ECEF frame from Novatel receiver
Header header

NovatelMessageHeader novatel_msg_header

# Position Data (m)
string solution_status
string position_type

float64 x
float64 y
float64 z

# Position Standard Deviation (m)
float32 x_sigma
float32 y_sigma
float32 z_sigma

# Velocity Data
string velocity_solution_status
string velocity_type

float64 x_vel
float64 y_vel
float64 z_vel

# Velocity Standard Deviation (m/s)
float32 x_vel_sigma
float32 y_vel_sigma
float32 z_vel_sigma

string base_station_id
float32 velocity_latency

# Data Ages (sec)
float32 diff_age
float32 solution_age

# Satellite Usage
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

# Extended Solution Status
NovatelExtendedSolutionStatus extended_solution_status

# Signal Masks
NovatelSignalMask signal_mask
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


================================================================================
MSG: novatel_gps_msgs/NovatelExtendedSolutionStatus
# Bit    Mask      Description
#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)
# 1-3    0x0E      Pseudorange Ionosphere Correction
#                    0 = unknown
#                    1 = Klobuchar Broadcast
#                    2 = SBAS Broadcast
#                    3 = Multi-frequency Computed
#                    4 = PSRDiff Correction
#                    5 = NovaTel Blended Ionosphere Value
# 4-7  0xF0        <Reserved>
uint32 original_mask
bool advance_rtk_verified
string psuedorange_iono_correction

================================================================================
MSG: novatel_gps_msgs/NovatelSignalMask
# Bit    Mask      Description
#  0     0x01      GPS L1 used in Solution
#  1     0x02      GPS L2 used in Solution
#  2     0x04      GPS L5 used in Solution
#  3     0x08      <Reserved>
#  4     0x10      GLONASS L1 used in Solution
#  5     0x20      GLONASS L2 used in Solution
# 6-7  0x40-0x80   <Reserved>
uint32 original_mask
bool gps_L1_used_in_solution
bool gps_L2_used_in_solution
bool gps_L3_used_in_solution
bool glonass_L1_used_in_solution
bool glonass_L2_used_in_solution
")



(provide :novatel_gps_msgs/NovatelXYZ "d1ffc3181aa742b6133febb9d8f77d12")


