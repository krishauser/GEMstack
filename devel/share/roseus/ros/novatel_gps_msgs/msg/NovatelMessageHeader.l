;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelMessageHeader)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelMessageHeader (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELMESSAGEHEADER")
  (make-package "NOVATEL_GPS_MSGS::NOVATELMESSAGEHEADER"))

(in-package "ROS")
;;//! \htmlinclude NovatelMessageHeader.msg.html


(defclass novatel_gps_msgs::NovatelMessageHeader
  :super ros::object
  :slots (_message_name _port _sequence_num _percent_idle_time _gps_time_status _gps_week_num _gps_seconds _receiver_status _receiver_software_version ))

(defmethod novatel_gps_msgs::NovatelMessageHeader
  (:init
   (&key
    ((:message_name __message_name) "")
    ((:port __port) "")
    ((:sequence_num __sequence_num) 0)
    ((:percent_idle_time __percent_idle_time) 0.0)
    ((:gps_time_status __gps_time_status) "")
    ((:gps_week_num __gps_week_num) 0)
    ((:gps_seconds __gps_seconds) 0.0)
    ((:receiver_status __receiver_status) (instance novatel_gps_msgs::NovatelReceiverStatus :init))
    ((:receiver_software_version __receiver_software_version) 0)
    )
   (send-super :init)
   (setq _message_name (string __message_name))
   (setq _port (string __port))
   (setq _sequence_num (round __sequence_num))
   (setq _percent_idle_time (float __percent_idle_time))
   (setq _gps_time_status (string __gps_time_status))
   (setq _gps_week_num (round __gps_week_num))
   (setq _gps_seconds (float __gps_seconds))
   (setq _receiver_status __receiver_status)
   (setq _receiver_software_version (round __receiver_software_version))
   self)
  (:message_name
   (&optional __message_name)
   (if __message_name (setq _message_name __message_name)) _message_name)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:sequence_num
   (&optional __sequence_num)
   (if __sequence_num (setq _sequence_num __sequence_num)) _sequence_num)
  (:percent_idle_time
   (&optional __percent_idle_time)
   (if __percent_idle_time (setq _percent_idle_time __percent_idle_time)) _percent_idle_time)
  (:gps_time_status
   (&optional __gps_time_status)
   (if __gps_time_status (setq _gps_time_status __gps_time_status)) _gps_time_status)
  (:gps_week_num
   (&optional __gps_week_num)
   (if __gps_week_num (setq _gps_week_num __gps_week_num)) _gps_week_num)
  (:gps_seconds
   (&optional __gps_seconds)
   (if __gps_seconds (setq _gps_seconds __gps_seconds)) _gps_seconds)
  (:receiver_status
   (&rest __receiver_status)
   (if (keywordp (car __receiver_status))
       (send* _receiver_status __receiver_status)
     (progn
       (if __receiver_status (setq _receiver_status (car __receiver_status)))
       _receiver_status)))
  (:receiver_software_version
   (&optional __receiver_software_version)
   (if __receiver_software_version (setq _receiver_software_version __receiver_software_version)) _receiver_software_version)
  (:serialization-length
   ()
   (+
    ;; string _message_name
    4 (length _message_name)
    ;; string _port
    4 (length _port)
    ;; uint32 _sequence_num
    4
    ;; float32 _percent_idle_time
    4
    ;; string _gps_time_status
    4 (length _gps_time_status)
    ;; uint32 _gps_week_num
    4
    ;; float64 _gps_seconds
    8
    ;; novatel_gps_msgs/NovatelReceiverStatus _receiver_status
    (send _receiver_status :serialization-length)
    ;; uint32 _receiver_software_version
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message_name
       (write-long (length _message_name) s) (princ _message_name s)
     ;; string _port
       (write-long (length _port) s) (princ _port s)
     ;; uint32 _sequence_num
       (write-long _sequence_num s)
     ;; float32 _percent_idle_time
       (sys::poke _percent_idle_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _gps_time_status
       (write-long (length _gps_time_status) s) (princ _gps_time_status s)
     ;; uint32 _gps_week_num
       (write-long _gps_week_num s)
     ;; float64 _gps_seconds
       (sys::poke _gps_seconds (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; novatel_gps_msgs/NovatelReceiverStatus _receiver_status
       (send _receiver_status :serialize s)
     ;; uint32 _receiver_software_version
       (write-long _receiver_software_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _port
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _port (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _sequence_num
     (setq _sequence_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _percent_idle_time
     (setq _percent_idle_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _gps_time_status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gps_time_status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _gps_week_num
     (setq _gps_week_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _gps_seconds
     (setq _gps_seconds (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; novatel_gps_msgs/NovatelReceiverStatus _receiver_status
     (send _receiver_status :deserialize buf ptr-) (incf ptr- (send _receiver_status :serialization-length))
   ;; uint32 _receiver_software_version
     (setq _receiver_software_version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelMessageHeader :md5sum-) "43b536606c527a56309297282bb7adef")
(setf (get novatel_gps_msgs::NovatelMessageHeader :datatype-) "novatel_gps_msgs/NovatelMessageHeader")
(setf (get novatel_gps_msgs::NovatelMessageHeader :definition-)
      "# Novatel Message Header

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



(provide :novatel_gps_msgs/NovatelMessageHeader "43b536606c527a56309297282bb7adef")


