;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::Insstdev)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'Insstdev (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::INSSTDEV")
  (make-package "NOVATEL_GPS_MSGS::INSSTDEV"))

(in-package "ROS")
;;//! \htmlinclude Insstdev.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass novatel_gps_msgs::Insstdev
  :super ros::object
  :slots (_header _novatel_msg_header _latitude_dev _longitude_dev _height_dev _north_velocity_dev _east_velocity_dev _up_velocity_dev _roll_dev _pitch_dev _azimuth_dev _extended_solution_status _time_since_update ))

(defmethod novatel_gps_msgs::Insstdev
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:novatel_msg_header __novatel_msg_header) (instance novatel_gps_msgs::NovatelMessageHeader :init))
    ((:latitude_dev __latitude_dev) 0.0)
    ((:longitude_dev __longitude_dev) 0.0)
    ((:height_dev __height_dev) 0.0)
    ((:north_velocity_dev __north_velocity_dev) 0.0)
    ((:east_velocity_dev __east_velocity_dev) 0.0)
    ((:up_velocity_dev __up_velocity_dev) 0.0)
    ((:roll_dev __roll_dev) 0.0)
    ((:pitch_dev __pitch_dev) 0.0)
    ((:azimuth_dev __azimuth_dev) 0.0)
    ((:extended_solution_status __extended_solution_status) (instance novatel_gps_msgs::NovatelExtendedSolutionStatus :init))
    ((:time_since_update __time_since_update) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _novatel_msg_header __novatel_msg_header)
   (setq _latitude_dev (float __latitude_dev))
   (setq _longitude_dev (float __longitude_dev))
   (setq _height_dev (float __height_dev))
   (setq _north_velocity_dev (float __north_velocity_dev))
   (setq _east_velocity_dev (float __east_velocity_dev))
   (setq _up_velocity_dev (float __up_velocity_dev))
   (setq _roll_dev (float __roll_dev))
   (setq _pitch_dev (float __pitch_dev))
   (setq _azimuth_dev (float __azimuth_dev))
   (setq _extended_solution_status __extended_solution_status)
   (setq _time_since_update (round __time_since_update))
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
  (:latitude_dev
   (&optional __latitude_dev)
   (if __latitude_dev (setq _latitude_dev __latitude_dev)) _latitude_dev)
  (:longitude_dev
   (&optional __longitude_dev)
   (if __longitude_dev (setq _longitude_dev __longitude_dev)) _longitude_dev)
  (:height_dev
   (&optional __height_dev)
   (if __height_dev (setq _height_dev __height_dev)) _height_dev)
  (:north_velocity_dev
   (&optional __north_velocity_dev)
   (if __north_velocity_dev (setq _north_velocity_dev __north_velocity_dev)) _north_velocity_dev)
  (:east_velocity_dev
   (&optional __east_velocity_dev)
   (if __east_velocity_dev (setq _east_velocity_dev __east_velocity_dev)) _east_velocity_dev)
  (:up_velocity_dev
   (&optional __up_velocity_dev)
   (if __up_velocity_dev (setq _up_velocity_dev __up_velocity_dev)) _up_velocity_dev)
  (:roll_dev
   (&optional __roll_dev)
   (if __roll_dev (setq _roll_dev __roll_dev)) _roll_dev)
  (:pitch_dev
   (&optional __pitch_dev)
   (if __pitch_dev (setq _pitch_dev __pitch_dev)) _pitch_dev)
  (:azimuth_dev
   (&optional __azimuth_dev)
   (if __azimuth_dev (setq _azimuth_dev __azimuth_dev)) _azimuth_dev)
  (:extended_solution_status
   (&rest __extended_solution_status)
   (if (keywordp (car __extended_solution_status))
       (send* _extended_solution_status __extended_solution_status)
     (progn
       (if __extended_solution_status (setq _extended_solution_status (car __extended_solution_status)))
       _extended_solution_status)))
  (:time_since_update
   (&optional __time_since_update)
   (if __time_since_update (setq _time_since_update __time_since_update)) _time_since_update)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
    (send _novatel_msg_header :serialization-length)
    ;; float32 _latitude_dev
    4
    ;; float32 _longitude_dev
    4
    ;; float32 _height_dev
    4
    ;; float32 _north_velocity_dev
    4
    ;; float32 _east_velocity_dev
    4
    ;; float32 _up_velocity_dev
    4
    ;; float32 _roll_dev
    4
    ;; float32 _pitch_dev
    4
    ;; float32 _azimuth_dev
    4
    ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
    (send _extended_solution_status :serialization-length)
    ;; uint16 _time_since_update
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
       (send _novatel_msg_header :serialize s)
     ;; float32 _latitude_dev
       (sys::poke _latitude_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longitude_dev
       (sys::poke _longitude_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height_dev
       (sys::poke _height_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _north_velocity_dev
       (sys::poke _north_velocity_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _east_velocity_dev
       (sys::poke _east_velocity_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _up_velocity_dev
       (sys::poke _up_velocity_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_dev
       (sys::poke _roll_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_dev
       (sys::poke _pitch_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _azimuth_dev
       (sys::poke _azimuth_dev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
       (send _extended_solution_status :serialize s)
     ;; uint16 _time_since_update
       (write-word _time_since_update s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; novatel_gps_msgs/NovatelMessageHeader _novatel_msg_header
     (send _novatel_msg_header :deserialize buf ptr-) (incf ptr- (send _novatel_msg_header :serialization-length))
   ;; float32 _latitude_dev
     (setq _latitude_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longitude_dev
     (setq _longitude_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height_dev
     (setq _height_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _north_velocity_dev
     (setq _north_velocity_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _east_velocity_dev
     (setq _east_velocity_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _up_velocity_dev
     (setq _up_velocity_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_dev
     (setq _roll_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_dev
     (setq _pitch_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _azimuth_dev
     (setq _azimuth_dev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; novatel_gps_msgs/NovatelExtendedSolutionStatus _extended_solution_status
     (send _extended_solution_status :deserialize buf ptr-) (incf ptr- (send _extended_solution_status :serialization-length))
   ;; uint16 _time_since_update
     (setq _time_since_update (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::Insstdev :md5sum-) "5a3ffc9969b49cd107b55c9843133d1c")
(setf (get novatel_gps_msgs::Insstdev :datatype-) "novatel_gps_msgs/Insstdev")
(setf (get novatel_gps_msgs::Insstdev :definition-)
      "# INS PVA standard deviations

Header header

NovatelMessageHeader novatel_msg_header

float32 latitude_dev
float32 longitude_dev
float32 height_dev
float32 north_velocity_dev
float32 east_velocity_dev
float32 up_velocity_dev
float32 roll_dev
float32 pitch_dev
float32 azimuth_dev
NovatelExtendedSolutionStatus extended_solution_status
uint16 time_since_update


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

")



(provide :novatel_gps_msgs/Insstdev "5a3ffc9969b49cd107b55c9843133d1c")


