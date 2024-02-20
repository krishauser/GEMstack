;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelDualAntennaHeading)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelDualAntennaHeading (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELDUALANTENNAHEADING")
  (make-package "NOVATEL_GPS_MSGS::NOVATELDUALANTENNAHEADING"))

(in-package "ROS")
;;//! \htmlinclude NovatelDualAntennaHeading.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOURCE_PRIMARY_ANTENNA*" (find-package "NOVATEL_GPS_MSGS::NOVATELDUALANTENNAHEADING"))
(shadow '*SOURCE_PRIMARY_ANTENNA* (find-package "NOVATEL_GPS_MSGS::NOVATELDUALANTENNAHEADING"))
(defconstant novatel_gps_msgs::NovatelDualAntennaHeading::*SOURCE_PRIMARY_ANTENNA* 0)
(intern "*SOURCE_SECONDARY_ANTENNA*" (find-package "NOVATEL_GPS_MSGS::NOVATELDUALANTENNAHEADING"))
(shadow '*SOURCE_SECONDARY_ANTENNA* (find-package "NOVATEL_GPS_MSGS::NOVATELDUALANTENNAHEADING"))
(defconstant novatel_gps_msgs::NovatelDualAntennaHeading::*SOURCE_SECONDARY_ANTENNA* 1)

(defun novatel_gps_msgs::NovatelDualAntennaHeading-to-symbol (const)
  (cond
        ((= const 0) 'novatel_gps_msgs::NovatelDualAntennaHeading::*SOURCE_PRIMARY_ANTENNA*)
        ((= const 1) 'novatel_gps_msgs::NovatelDualAntennaHeading::*SOURCE_SECONDARY_ANTENNA*)
        (t nil)))

(defclass novatel_gps_msgs::NovatelDualAntennaHeading
  :super ros::object
  :slots (_header _novatel_msg_header _solution_status _position_type _baseline_length _heading _pitch _heading_sigma _pitch_sigma _station_id _num_satellites_tracked _num_satellites_used_in_solution _num_satellites_above_elevation_mask_angle _num_satellites_above_elevation_mask_angle_l2 _solution_source _extended_solution_status _signal_mask ))

(defmethod novatel_gps_msgs::NovatelDualAntennaHeading
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:novatel_msg_header __novatel_msg_header) (instance novatel_gps_msgs::NovatelMessageHeader :init))
    ((:solution_status __solution_status) "")
    ((:position_type __position_type) "")
    ((:baseline_length __baseline_length) 0.0)
    ((:heading __heading) 0.0)
    ((:pitch __pitch) 0.0)
    ((:heading_sigma __heading_sigma) 0.0)
    ((:pitch_sigma __pitch_sigma) 0.0)
    ((:station_id __station_id) "")
    ((:num_satellites_tracked __num_satellites_tracked) 0)
    ((:num_satellites_used_in_solution __num_satellites_used_in_solution) 0)
    ((:num_satellites_above_elevation_mask_angle __num_satellites_above_elevation_mask_angle) 0)
    ((:num_satellites_above_elevation_mask_angle_l2 __num_satellites_above_elevation_mask_angle_l2) 0)
    ((:solution_source __solution_source) 0)
    ((:extended_solution_status __extended_solution_status) (instance novatel_gps_msgs::NovatelExtendedSolutionStatus :init))
    ((:signal_mask __signal_mask) (instance novatel_gps_msgs::NovatelSignalMask :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _novatel_msg_header __novatel_msg_header)
   (setq _solution_status (string __solution_status))
   (setq _position_type (string __position_type))
   (setq _baseline_length (float __baseline_length))
   (setq _heading (float __heading))
   (setq _pitch (float __pitch))
   (setq _heading_sigma (float __heading_sigma))
   (setq _pitch_sigma (float __pitch_sigma))
   (setq _station_id (string __station_id))
   (setq _num_satellites_tracked (round __num_satellites_tracked))
   (setq _num_satellites_used_in_solution (round __num_satellites_used_in_solution))
   (setq _num_satellites_above_elevation_mask_angle (round __num_satellites_above_elevation_mask_angle))
   (setq _num_satellites_above_elevation_mask_angle_l2 (round __num_satellites_above_elevation_mask_angle_l2))
   (setq _solution_source (round __solution_source))
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
  (:baseline_length
   (&optional __baseline_length)
   (if __baseline_length (setq _baseline_length __baseline_length)) _baseline_length)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:heading_sigma
   (&optional __heading_sigma)
   (if __heading_sigma (setq _heading_sigma __heading_sigma)) _heading_sigma)
  (:pitch_sigma
   (&optional __pitch_sigma)
   (if __pitch_sigma (setq _pitch_sigma __pitch_sigma)) _pitch_sigma)
  (:station_id
   (&optional __station_id)
   (if __station_id (setq _station_id __station_id)) _station_id)
  (:num_satellites_tracked
   (&optional __num_satellites_tracked)
   (if __num_satellites_tracked (setq _num_satellites_tracked __num_satellites_tracked)) _num_satellites_tracked)
  (:num_satellites_used_in_solution
   (&optional __num_satellites_used_in_solution)
   (if __num_satellites_used_in_solution (setq _num_satellites_used_in_solution __num_satellites_used_in_solution)) _num_satellites_used_in_solution)
  (:num_satellites_above_elevation_mask_angle
   (&optional __num_satellites_above_elevation_mask_angle)
   (if __num_satellites_above_elevation_mask_angle (setq _num_satellites_above_elevation_mask_angle __num_satellites_above_elevation_mask_angle)) _num_satellites_above_elevation_mask_angle)
  (:num_satellites_above_elevation_mask_angle_l2
   (&optional __num_satellites_above_elevation_mask_angle_l2)
   (if __num_satellites_above_elevation_mask_angle_l2 (setq _num_satellites_above_elevation_mask_angle_l2 __num_satellites_above_elevation_mask_angle_l2)) _num_satellites_above_elevation_mask_angle_l2)
  (:solution_source
   (&optional __solution_source)
   (if __solution_source (setq _solution_source __solution_source)) _solution_source)
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
    ;; float32 _baseline_length
    4
    ;; float32 _heading
    4
    ;; float32 _pitch
    4
    ;; float32 _heading_sigma
    4
    ;; float32 _pitch_sigma
    4
    ;; string _station_id
    4 (length _station_id)
    ;; uint8 _num_satellites_tracked
    1
    ;; uint8 _num_satellites_used_in_solution
    1
    ;; uint8 _num_satellites_above_elevation_mask_angle
    1
    ;; uint8 _num_satellites_above_elevation_mask_angle_l2
    1
    ;; uint8 _solution_source
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
     ;; float32 _baseline_length
       (sys::poke _baseline_length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading_sigma
       (sys::poke _heading_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_sigma
       (sys::poke _pitch_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _station_id
       (write-long (length _station_id) s) (princ _station_id s)
     ;; uint8 _num_satellites_tracked
       (write-byte _num_satellites_tracked s)
     ;; uint8 _num_satellites_used_in_solution
       (write-byte _num_satellites_used_in_solution s)
     ;; uint8 _num_satellites_above_elevation_mask_angle
       (write-byte _num_satellites_above_elevation_mask_angle s)
     ;; uint8 _num_satellites_above_elevation_mask_angle_l2
       (write-byte _num_satellites_above_elevation_mask_angle_l2 s)
     ;; uint8 _solution_source
       (write-byte _solution_source s)
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
   ;; float32 _baseline_length
     (setq _baseline_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading_sigma
     (setq _heading_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_sigma
     (setq _pitch_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _station_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _station_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _num_satellites_tracked
     (setq _num_satellites_tracked (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _num_satellites_used_in_solution
     (setq _num_satellites_used_in_solution (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _num_satellites_above_elevation_mask_angle
     (setq _num_satellites_above_elevation_mask_angle (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _num_satellites_above_elevation_mask_angle_l2
     (setq _num_satellites_above_elevation_mask_angle_l2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _solution_source
     (setq _solution_source (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
     (send _extended_solution_status :deserialize buf ptr-) (incf ptr- (send _extended_solution_status :serialization-length))
   ;; novatel_gps_msgs/NovatelSignalMask _signal_mask
     (send _signal_mask :deserialize buf ptr-) (incf ptr- (send _signal_mask :serialization-length))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelDualAntennaHeading :md5sum-) "9e80eaa83bf25f057aea59ce70d859e5")
(setf (get novatel_gps_msgs::NovatelDualAntennaHeading :datatype-) "novatel_gps_msgs/NovatelDualAntennaHeading")
(setf (get novatel_gps_msgs::NovatelDualAntennaHeading :definition-)
      "# Parsed Heading East of North from Novatel receiver. Only ALIGN capable recievers can publish this
Header header

NovatelMessageHeader novatel_msg_header

# Solution Status
string solution_status
string position_type

# Baseline length (m)
float32 baseline_length

# Heading in degrees [0,360)
float32 heading

# Pitch in degrees +- 90
float32 pitch

# Orientation Standard Deviations (deg)
float32 heading_sigma
float32 pitch_sigma

# Station ids
string station_id

# Satellite Usage
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_satellites_above_elevation_mask_angle
uint8 num_satellites_above_elevation_mask_angle_l2

# Enum for solution source
uint8 solution_source

uint8 SOURCE_PRIMARY_ANTENNA=0
uint8 SOURCE_SECONDARY_ANTENNA=1

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



(provide :novatel_gps_msgs/NovatelDualAntennaHeading "9e80eaa83bf25f057aea59ce70d859e5")


