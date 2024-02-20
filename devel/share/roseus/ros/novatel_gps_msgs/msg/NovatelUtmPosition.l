;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelUtmPosition)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelUtmPosition (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELUTMPOSITION")
  (make-package "NOVATEL_GPS_MSGS::NOVATELUTMPOSITION"))

(in-package "ROS")
;;//! \htmlinclude NovatelUtmPosition.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::NovatelUtmPosition
  :super ros::object
  :slots (_header _novatel_msg_header _solution_status _position_type _lon_zone_number _lat_zone_letter _northing _easting _height _undulation _datum_id _northing_sigma _easting_sigma _height_sigma _base_station_id _diff_age _solution_age _num_satellites_tracked _num_satellites_used_in_solution _num_gps_and_glonass_l1_used_in_solution _num_gps_and_glonass_l1_and_l2_used_in_solution _extended_solution_status _signal_mask ))

(defmethod novatel_gps_msgs::NovatelUtmPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:novatel_msg_header __novatel_msg_header) (instance novatel_gps_msgs::NovatelMessageHeader :init))
    ((:solution_status __solution_status) "")
    ((:position_type __position_type) "")
    ((:lon_zone_number __lon_zone_number) 0)
    ((:lat_zone_letter __lat_zone_letter) "")
    ((:northing __northing) 0.0)
    ((:easting __easting) 0.0)
    ((:height __height) 0.0)
    ((:undulation __undulation) 0.0)
    ((:datum_id __datum_id) "")
    ((:northing_sigma __northing_sigma) 0.0)
    ((:easting_sigma __easting_sigma) 0.0)
    ((:height_sigma __height_sigma) 0.0)
    ((:base_station_id __base_station_id) "")
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
   (setq _lon_zone_number (round __lon_zone_number))
   (setq _lat_zone_letter (string __lat_zone_letter))
   (setq _northing (float __northing))
   (setq _easting (float __easting))
   (setq _height (float __height))
   (setq _undulation (float __undulation))
   (setq _datum_id (string __datum_id))
   (setq _northing_sigma (float __northing_sigma))
   (setq _easting_sigma (float __easting_sigma))
   (setq _height_sigma (float __height_sigma))
   (setq _base_station_id (string __base_station_id))
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
  (:lon_zone_number
   (&optional __lon_zone_number)
   (if __lon_zone_number (setq _lon_zone_number __lon_zone_number)) _lon_zone_number)
  (:lat_zone_letter
   (&optional __lat_zone_letter)
   (if __lat_zone_letter (setq _lat_zone_letter __lat_zone_letter)) _lat_zone_letter)
  (:northing
   (&optional __northing)
   (if __northing (setq _northing __northing)) _northing)
  (:easting
   (&optional __easting)
   (if __easting (setq _easting __easting)) _easting)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:undulation
   (&optional __undulation)
   (if __undulation (setq _undulation __undulation)) _undulation)
  (:datum_id
   (&optional __datum_id)
   (if __datum_id (setq _datum_id __datum_id)) _datum_id)
  (:northing_sigma
   (&optional __northing_sigma)
   (if __northing_sigma (setq _northing_sigma __northing_sigma)) _northing_sigma)
  (:easting_sigma
   (&optional __easting_sigma)
   (if __easting_sigma (setq _easting_sigma __easting_sigma)) _easting_sigma)
  (:height_sigma
   (&optional __height_sigma)
   (if __height_sigma (setq _height_sigma __height_sigma)) _height_sigma)
  (:base_station_id
   (&optional __base_station_id)
   (if __base_station_id (setq _base_station_id __base_station_id)) _base_station_id)
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
    ;; uint32 _lon_zone_number
    4
    ;; string _lat_zone_letter
    4 (length _lat_zone_letter)
    ;; float64 _northing
    8
    ;; float64 _easting
    8
    ;; float64 _height
    8
    ;; float32 _undulation
    4
    ;; string _datum_id
    4 (length _datum_id)
    ;; float32 _northing_sigma
    4
    ;; float32 _easting_sigma
    4
    ;; float32 _height_sigma
    4
    ;; string _base_station_id
    4 (length _base_station_id)
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
     ;; uint32 _lon_zone_number
       (write-long _lon_zone_number s)
     ;; string _lat_zone_letter
       (write-long (length _lat_zone_letter) s) (princ _lat_zone_letter s)
     ;; float64 _northing
       (sys::poke _northing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _easting
       (sys::poke _easting (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _undulation
       (sys::poke _undulation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _datum_id
       (write-long (length _datum_id) s) (princ _datum_id s)
     ;; float32 _northing_sigma
       (sys::poke _northing_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _easting_sigma
       (sys::poke _easting_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height_sigma
       (sys::poke _height_sigma (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _base_station_id
       (write-long (length _base_station_id) s) (princ _base_station_id s)
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
   ;; uint32 _lon_zone_number
     (setq _lon_zone_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _lat_zone_letter
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lat_zone_letter (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _northing
     (setq _northing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _easting
     (setq _easting (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _undulation
     (setq _undulation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _datum_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _datum_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _northing_sigma
     (setq _northing_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _easting_sigma
     (setq _easting_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height_sigma
     (setq _height_sigma (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _base_station_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _base_station_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
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

(setf (get novatel_gps_msgs::NovatelUtmPosition :md5sum-) "401ec19560e710c1e0aab9f0eec02ece")
(setf (get novatel_gps_msgs::NovatelUtmPosition :datatype-) "novatel_gps_msgs/NovatelUtmPosition")
(setf (get novatel_gps_msgs::NovatelUtmPosition :definition-)
      "# Parsed Best UTM data from Novatel OEM6 receiver
Header header

NovatelMessageHeader novatel_msg_header

string solution_status
string position_type

# Position Data
uint32 lon_zone_number
string lat_zone_letter
float64 northing
float64 easting
float64 height

float32 undulation
string datum_id

# Accuracy Statistics (units?)
float32 northing_sigma
float32 easting_sigma
float32 height_sigma
string base_station_id
float32 diff_age
float32 solution_age
uint8 num_satellites_tracked
uint8 num_satellites_used_in_solution
uint8 num_gps_and_glonass_l1_used_in_solution
uint8 num_gps_and_glonass_l1_and_l2_used_in_solution

NovatelExtendedSolutionStatus extended_solution_status

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



(provide :novatel_gps_msgs/NovatelUtmPosition "401ec19560e710c1e0aab9f0eec02ece")


