;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::ClockSteering)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'ClockSteering (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING")
  (make-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))

(in-package "ROS")
;;//! \htmlinclude ClockSteering.msg.html


(intern "*INTERNAL_SOURCE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*INTERNAL_SOURCE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*INTERNAL_SOURCE* 0)
(intern "*EXTERNAL_SOURCE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*EXTERNAL_SOURCE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*EXTERNAL_SOURCE* 1)
(intern "*FIRST_ORDER_STEERING_STATE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*FIRST_ORDER_STEERING_STATE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*FIRST_ORDER_STEERING_STATE* 0)
(intern "*SECOND_ORDER_STEERING_STATE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*SECOND_ORDER_STEERING_STATE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*SECOND_ORDER_STEERING_STATE* 1)
(intern "*CALIBRATE_HIGH_STEERING_STATE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*CALIBRATE_HIGH_STEERING_STATE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*CALIBRATE_HIGH_STEERING_STATE* 2)
(intern "*CALIBRATE_LOW_STEERING_STATE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*CALIBRATE_LOW_STEERING_STATE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*CALIBRATE_LOW_STEERING_STATE* 3)
(intern "*CALIBRATE_CENTER_STEERING_STATE*" (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(shadow '*CALIBRATE_CENTER_STEERING_STATE* (find-package "NOVATEL_GPS_MSGS::CLOCKSTEERING"))
(defconstant novatel_gps_msgs::ClockSteering::*CALIBRATE_CENTER_STEERING_STATE* 4)

(defun novatel_gps_msgs::ClockSteering-to-symbol (const)
  (cond
        ((= const 0) 'novatel_gps_msgs::ClockSteering::*INTERNAL_SOURCE*)
        ((= const 1) 'novatel_gps_msgs::ClockSteering::*EXTERNAL_SOURCE*)
        ((= const 0) 'novatel_gps_msgs::ClockSteering::*FIRST_ORDER_STEERING_STATE*)
        ((= const 1) 'novatel_gps_msgs::ClockSteering::*SECOND_ORDER_STEERING_STATE*)
        ((= const 2) 'novatel_gps_msgs::ClockSteering::*CALIBRATE_HIGH_STEERING_STATE*)
        ((= const 3) 'novatel_gps_msgs::ClockSteering::*CALIBRATE_LOW_STEERING_STATE*)
        ((= const 4) 'novatel_gps_msgs::ClockSteering::*CALIBRATE_CENTER_STEERING_STATE*)
        (t nil)))

(defclass novatel_gps_msgs::ClockSteering
  :super ros::object
  :slots (_source _steering_state _period _pulse_width _bandwidth _slope _offset _drift_rate ))

(defmethod novatel_gps_msgs::ClockSteering
  (:init
   (&key
    ((:source __source) "")
    ((:steering_state __steering_state) "")
    ((:period __period) 0)
    ((:pulse_width __pulse_width) 0.0)
    ((:bandwidth __bandwidth) 0.0)
    ((:slope __slope) 0.0)
    ((:offset __offset) 0.0)
    ((:drift_rate __drift_rate) 0.0)
    )
   (send-super :init)
   (setq _source (string __source))
   (setq _steering_state (string __steering_state))
   (setq _period (round __period))
   (setq _pulse_width (float __pulse_width))
   (setq _bandwidth (float __bandwidth))
   (setq _slope (float __slope))
   (setq _offset (float __offset))
   (setq _drift_rate (float __drift_rate))
   self)
  (:source
   (&optional __source)
   (if __source (setq _source __source)) _source)
  (:steering_state
   (&optional __steering_state)
   (if __steering_state (setq _steering_state __steering_state)) _steering_state)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:pulse_width
   (&optional __pulse_width)
   (if __pulse_width (setq _pulse_width __pulse_width)) _pulse_width)
  (:bandwidth
   (&optional __bandwidth)
   (if __bandwidth (setq _bandwidth __bandwidth)) _bandwidth)
  (:slope
   (&optional __slope)
   (if __slope (setq _slope __slope)) _slope)
  (:offset
   (&optional __offset)
   (if __offset (setq _offset __offset)) _offset)
  (:drift_rate
   (&optional __drift_rate)
   (if __drift_rate (setq _drift_rate __drift_rate)) _drift_rate)
  (:serialization-length
   ()
   (+
    ;; string _source
    4 (length _source)
    ;; string _steering_state
    4 (length _steering_state)
    ;; uint32 _period
    4
    ;; float64 _pulse_width
    8
    ;; float64 _bandwidth
    8
    ;; float32 _slope
    4
    ;; float64 _offset
    8
    ;; float64 _drift_rate
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _source
       (write-long (length _source) s) (princ _source s)
     ;; string _steering_state
       (write-long (length _steering_state) s) (princ _steering_state s)
     ;; uint32 _period
       (write-long _period s)
     ;; float64 _pulse_width
       (sys::poke _pulse_width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _bandwidth
       (sys::poke _bandwidth (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _slope
       (sys::poke _slope (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _offset
       (sys::poke _offset (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _drift_rate
       (sys::poke _drift_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _source
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _steering_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _steering_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _period
     (setq _period (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _pulse_width
     (setq _pulse_width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _bandwidth
     (setq _bandwidth (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _slope
     (setq _slope (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _offset
     (setq _offset (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _drift_rate
     (setq _drift_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::ClockSteering :md5sum-) "03024ea60365b742dd5e56411830735e")
(setf (get novatel_gps_msgs::ClockSteering :datatype-) "novatel_gps_msgs/ClockSteering")
(setf (get novatel_gps_msgs::ClockSteering :definition-)
      "# The CLOCKSTEERING log is used to monitor the current state of the clock steering process.

int8 INTERNAL_SOURCE=0
int8 EXTERNAL_SOURCE=1

int8 FIRST_ORDER_STEERING_STATE=0
int8 SECOND_ORDER_STEERING_STATE=1
int8 CALIBRATE_HIGH_STEERING_STATE=2
int8 CALIBRATE_LOW_STEERING_STATE=3
int8 CALIBRATE_CENTER_STEERING_STATE=4

# Clock source
string source

# Steering state
string steering_state

# Period of the FREQUENCYOUT signal used to control the oscillator
uint32 period

# Current pulse width of the FREQUENCYOUT signal. 
float64 pulse_width

# The current band width of the clock steering tracking loop in Hz.
float64 bandwidth

# The current clock drift change in m/s/bit for a 1 LSB pulse width. 
float32 slope

# The last valid receiver clock offset computed (m).
float64 offset

# The last valid receiver clock drift rate received (m/s).
float64 drift_rate

")



(provide :novatel_gps_msgs/ClockSteering "03024ea60365b742dd5e56411830735e")


