;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelReceiverStatus)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelReceiverStatus (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELRECEIVERSTATUS")
  (make-package "NOVATEL_GPS_MSGS::NOVATELRECEIVERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude NovatelReceiverStatus.msg.html


(defclass novatel_gps_msgs::NovatelReceiverStatus
  :super ros::object
  :slots (_original_status_code _error_flag _temperature_flag _voltage_supply_flag _antenna_powered _antenna_is_open _antenna_is_shorted _cpu_overload_flag _com1_buffer_overrun _com2_buffer_overrun _com3_buffer_overrun _usb_buffer_overrun _rf1_agc_flag _rf2_agc_flag _almanac_flag _position_solution_flag _position_fixed_flag _clock_steering_status_enabled _clock_model_flag _oemv_external_oscillator_flag _software_resource_flag _aux1_status_event_flag _aux2_status_event_flag _aux3_status_event_flag ))

(defmethod novatel_gps_msgs::NovatelReceiverStatus
  (:init
   (&key
    ((:original_status_code __original_status_code) 0)
    ((:error_flag __error_flag) nil)
    ((:temperature_flag __temperature_flag) nil)
    ((:voltage_supply_flag __voltage_supply_flag) nil)
    ((:antenna_powered __antenna_powered) nil)
    ((:antenna_is_open __antenna_is_open) nil)
    ((:antenna_is_shorted __antenna_is_shorted) nil)
    ((:cpu_overload_flag __cpu_overload_flag) nil)
    ((:com1_buffer_overrun __com1_buffer_overrun) nil)
    ((:com2_buffer_overrun __com2_buffer_overrun) nil)
    ((:com3_buffer_overrun __com3_buffer_overrun) nil)
    ((:usb_buffer_overrun __usb_buffer_overrun) nil)
    ((:rf1_agc_flag __rf1_agc_flag) nil)
    ((:rf2_agc_flag __rf2_agc_flag) nil)
    ((:almanac_flag __almanac_flag) nil)
    ((:position_solution_flag __position_solution_flag) nil)
    ((:position_fixed_flag __position_fixed_flag) nil)
    ((:clock_steering_status_enabled __clock_steering_status_enabled) nil)
    ((:clock_model_flag __clock_model_flag) nil)
    ((:oemv_external_oscillator_flag __oemv_external_oscillator_flag) nil)
    ((:software_resource_flag __software_resource_flag) nil)
    ((:aux1_status_event_flag __aux1_status_event_flag) nil)
    ((:aux2_status_event_flag __aux2_status_event_flag) nil)
    ((:aux3_status_event_flag __aux3_status_event_flag) nil)
    )
   (send-super :init)
   (setq _original_status_code (round __original_status_code))
   (setq _error_flag __error_flag)
   (setq _temperature_flag __temperature_flag)
   (setq _voltage_supply_flag __voltage_supply_flag)
   (setq _antenna_powered __antenna_powered)
   (setq _antenna_is_open __antenna_is_open)
   (setq _antenna_is_shorted __antenna_is_shorted)
   (setq _cpu_overload_flag __cpu_overload_flag)
   (setq _com1_buffer_overrun __com1_buffer_overrun)
   (setq _com2_buffer_overrun __com2_buffer_overrun)
   (setq _com3_buffer_overrun __com3_buffer_overrun)
   (setq _usb_buffer_overrun __usb_buffer_overrun)
   (setq _rf1_agc_flag __rf1_agc_flag)
   (setq _rf2_agc_flag __rf2_agc_flag)
   (setq _almanac_flag __almanac_flag)
   (setq _position_solution_flag __position_solution_flag)
   (setq _position_fixed_flag __position_fixed_flag)
   (setq _clock_steering_status_enabled __clock_steering_status_enabled)
   (setq _clock_model_flag __clock_model_flag)
   (setq _oemv_external_oscillator_flag __oemv_external_oscillator_flag)
   (setq _software_resource_flag __software_resource_flag)
   (setq _aux1_status_event_flag __aux1_status_event_flag)
   (setq _aux2_status_event_flag __aux2_status_event_flag)
   (setq _aux3_status_event_flag __aux3_status_event_flag)
   self)
  (:original_status_code
   (&optional __original_status_code)
   (if __original_status_code (setq _original_status_code __original_status_code)) _original_status_code)
  (:error_flag
   (&optional (__error_flag :null))
   (if (not (eq __error_flag :null)) (setq _error_flag __error_flag)) _error_flag)
  (:temperature_flag
   (&optional (__temperature_flag :null))
   (if (not (eq __temperature_flag :null)) (setq _temperature_flag __temperature_flag)) _temperature_flag)
  (:voltage_supply_flag
   (&optional (__voltage_supply_flag :null))
   (if (not (eq __voltage_supply_flag :null)) (setq _voltage_supply_flag __voltage_supply_flag)) _voltage_supply_flag)
  (:antenna_powered
   (&optional (__antenna_powered :null))
   (if (not (eq __antenna_powered :null)) (setq _antenna_powered __antenna_powered)) _antenna_powered)
  (:antenna_is_open
   (&optional (__antenna_is_open :null))
   (if (not (eq __antenna_is_open :null)) (setq _antenna_is_open __antenna_is_open)) _antenna_is_open)
  (:antenna_is_shorted
   (&optional (__antenna_is_shorted :null))
   (if (not (eq __antenna_is_shorted :null)) (setq _antenna_is_shorted __antenna_is_shorted)) _antenna_is_shorted)
  (:cpu_overload_flag
   (&optional (__cpu_overload_flag :null))
   (if (not (eq __cpu_overload_flag :null)) (setq _cpu_overload_flag __cpu_overload_flag)) _cpu_overload_flag)
  (:com1_buffer_overrun
   (&optional (__com1_buffer_overrun :null))
   (if (not (eq __com1_buffer_overrun :null)) (setq _com1_buffer_overrun __com1_buffer_overrun)) _com1_buffer_overrun)
  (:com2_buffer_overrun
   (&optional (__com2_buffer_overrun :null))
   (if (not (eq __com2_buffer_overrun :null)) (setq _com2_buffer_overrun __com2_buffer_overrun)) _com2_buffer_overrun)
  (:com3_buffer_overrun
   (&optional (__com3_buffer_overrun :null))
   (if (not (eq __com3_buffer_overrun :null)) (setq _com3_buffer_overrun __com3_buffer_overrun)) _com3_buffer_overrun)
  (:usb_buffer_overrun
   (&optional (__usb_buffer_overrun :null))
   (if (not (eq __usb_buffer_overrun :null)) (setq _usb_buffer_overrun __usb_buffer_overrun)) _usb_buffer_overrun)
  (:rf1_agc_flag
   (&optional (__rf1_agc_flag :null))
   (if (not (eq __rf1_agc_flag :null)) (setq _rf1_agc_flag __rf1_agc_flag)) _rf1_agc_flag)
  (:rf2_agc_flag
   (&optional (__rf2_agc_flag :null))
   (if (not (eq __rf2_agc_flag :null)) (setq _rf2_agc_flag __rf2_agc_flag)) _rf2_agc_flag)
  (:almanac_flag
   (&optional (__almanac_flag :null))
   (if (not (eq __almanac_flag :null)) (setq _almanac_flag __almanac_flag)) _almanac_flag)
  (:position_solution_flag
   (&optional (__position_solution_flag :null))
   (if (not (eq __position_solution_flag :null)) (setq _position_solution_flag __position_solution_flag)) _position_solution_flag)
  (:position_fixed_flag
   (&optional (__position_fixed_flag :null))
   (if (not (eq __position_fixed_flag :null)) (setq _position_fixed_flag __position_fixed_flag)) _position_fixed_flag)
  (:clock_steering_status_enabled
   (&optional (__clock_steering_status_enabled :null))
   (if (not (eq __clock_steering_status_enabled :null)) (setq _clock_steering_status_enabled __clock_steering_status_enabled)) _clock_steering_status_enabled)
  (:clock_model_flag
   (&optional (__clock_model_flag :null))
   (if (not (eq __clock_model_flag :null)) (setq _clock_model_flag __clock_model_flag)) _clock_model_flag)
  (:oemv_external_oscillator_flag
   (&optional (__oemv_external_oscillator_flag :null))
   (if (not (eq __oemv_external_oscillator_flag :null)) (setq _oemv_external_oscillator_flag __oemv_external_oscillator_flag)) _oemv_external_oscillator_flag)
  (:software_resource_flag
   (&optional (__software_resource_flag :null))
   (if (not (eq __software_resource_flag :null)) (setq _software_resource_flag __software_resource_flag)) _software_resource_flag)
  (:aux1_status_event_flag
   (&optional (__aux1_status_event_flag :null))
   (if (not (eq __aux1_status_event_flag :null)) (setq _aux1_status_event_flag __aux1_status_event_flag)) _aux1_status_event_flag)
  (:aux2_status_event_flag
   (&optional (__aux2_status_event_flag :null))
   (if (not (eq __aux2_status_event_flag :null)) (setq _aux2_status_event_flag __aux2_status_event_flag)) _aux2_status_event_flag)
  (:aux3_status_event_flag
   (&optional (__aux3_status_event_flag :null))
   (if (not (eq __aux3_status_event_flag :null)) (setq _aux3_status_event_flag __aux3_status_event_flag)) _aux3_status_event_flag)
  (:serialization-length
   ()
   (+
    ;; uint32 _original_status_code
    4
    ;; bool _error_flag
    1
    ;; bool _temperature_flag
    1
    ;; bool _voltage_supply_flag
    1
    ;; bool _antenna_powered
    1
    ;; bool _antenna_is_open
    1
    ;; bool _antenna_is_shorted
    1
    ;; bool _cpu_overload_flag
    1
    ;; bool _com1_buffer_overrun
    1
    ;; bool _com2_buffer_overrun
    1
    ;; bool _com3_buffer_overrun
    1
    ;; bool _usb_buffer_overrun
    1
    ;; bool _rf1_agc_flag
    1
    ;; bool _rf2_agc_flag
    1
    ;; bool _almanac_flag
    1
    ;; bool _position_solution_flag
    1
    ;; bool _position_fixed_flag
    1
    ;; bool _clock_steering_status_enabled
    1
    ;; bool _clock_model_flag
    1
    ;; bool _oemv_external_oscillator_flag
    1
    ;; bool _software_resource_flag
    1
    ;; bool _aux1_status_event_flag
    1
    ;; bool _aux2_status_event_flag
    1
    ;; bool _aux3_status_event_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _original_status_code
       (write-long _original_status_code s)
     ;; bool _error_flag
       (if _error_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _temperature_flag
       (if _temperature_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _voltage_supply_flag
       (if _voltage_supply_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _antenna_powered
       (if _antenna_powered (write-byte -1 s) (write-byte 0 s))
     ;; bool _antenna_is_open
       (if _antenna_is_open (write-byte -1 s) (write-byte 0 s))
     ;; bool _antenna_is_shorted
       (if _antenna_is_shorted (write-byte -1 s) (write-byte 0 s))
     ;; bool _cpu_overload_flag
       (if _cpu_overload_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _com1_buffer_overrun
       (if _com1_buffer_overrun (write-byte -1 s) (write-byte 0 s))
     ;; bool _com2_buffer_overrun
       (if _com2_buffer_overrun (write-byte -1 s) (write-byte 0 s))
     ;; bool _com3_buffer_overrun
       (if _com3_buffer_overrun (write-byte -1 s) (write-byte 0 s))
     ;; bool _usb_buffer_overrun
       (if _usb_buffer_overrun (write-byte -1 s) (write-byte 0 s))
     ;; bool _rf1_agc_flag
       (if _rf1_agc_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _rf2_agc_flag
       (if _rf2_agc_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _almanac_flag
       (if _almanac_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _position_solution_flag
       (if _position_solution_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _position_fixed_flag
       (if _position_fixed_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _clock_steering_status_enabled
       (if _clock_steering_status_enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _clock_model_flag
       (if _clock_model_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _oemv_external_oscillator_flag
       (if _oemv_external_oscillator_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _software_resource_flag
       (if _software_resource_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _aux1_status_event_flag
       (if _aux1_status_event_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _aux2_status_event_flag
       (if _aux2_status_event_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _aux3_status_event_flag
       (if _aux3_status_event_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _original_status_code
     (setq _original_status_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _error_flag
     (setq _error_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _temperature_flag
     (setq _temperature_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _voltage_supply_flag
     (setq _voltage_supply_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _antenna_powered
     (setq _antenna_powered (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _antenna_is_open
     (setq _antenna_is_open (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _antenna_is_shorted
     (setq _antenna_is_shorted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cpu_overload_flag
     (setq _cpu_overload_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _com1_buffer_overrun
     (setq _com1_buffer_overrun (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _com2_buffer_overrun
     (setq _com2_buffer_overrun (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _com3_buffer_overrun
     (setq _com3_buffer_overrun (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _usb_buffer_overrun
     (setq _usb_buffer_overrun (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rf1_agc_flag
     (setq _rf1_agc_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rf2_agc_flag
     (setq _rf2_agc_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _almanac_flag
     (setq _almanac_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _position_solution_flag
     (setq _position_solution_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _position_fixed_flag
     (setq _position_fixed_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clock_steering_status_enabled
     (setq _clock_steering_status_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clock_model_flag
     (setq _clock_model_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _oemv_external_oscillator_flag
     (setq _oemv_external_oscillator_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _software_resource_flag
     (setq _software_resource_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _aux1_status_event_flag
     (setq _aux1_status_event_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _aux2_status_event_flag
     (setq _aux2_status_event_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _aux3_status_event_flag
     (setq _aux3_status_event_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelReceiverStatus :md5sum-) "cf2774401808a6dde392e2ebdb09ca15")
(setf (get novatel_gps_msgs::NovatelReceiverStatus :datatype-) "novatel_gps_msgs/NovatelReceiverStatus")
(setf (get novatel_gps_msgs::NovatelReceiverStatus :definition-)
      "# From the original Novatel receiver status message bitfield
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



(provide :novatel_gps_msgs/NovatelReceiverStatus "cf2774401808a6dde392e2ebdb09ca15")


