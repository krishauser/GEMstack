;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::RangeInformation)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'RangeInformation (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::RANGEINFORMATION")
  (make-package "NOVATEL_GPS_MSGS::RANGEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude RangeInformation.msg.html


(defclass novatel_gps_msgs::RangeInformation
  :super ros::object
  :slots (_prn_number _glofreq _psr _psr_std _adr _adr_std _dopp _noise_density_ratio _locktime _tracking_status ))

(defmethod novatel_gps_msgs::RangeInformation
  (:init
   (&key
    ((:prn_number __prn_number) 0)
    ((:glofreq __glofreq) 0)
    ((:psr __psr) 0.0)
    ((:psr_std __psr_std) 0.0)
    ((:adr __adr) 0.0)
    ((:adr_std __adr_std) 0.0)
    ((:dopp __dopp) 0.0)
    ((:noise_density_ratio __noise_density_ratio) 0.0)
    ((:locktime __locktime) 0.0)
    ((:tracking_status __tracking_status) 0)
    )
   (send-super :init)
   (setq _prn_number (round __prn_number))
   (setq _glofreq (round __glofreq))
   (setq _psr (float __psr))
   (setq _psr_std (float __psr_std))
   (setq _adr (float __adr))
   (setq _adr_std (float __adr_std))
   (setq _dopp (float __dopp))
   (setq _noise_density_ratio (float __noise_density_ratio))
   (setq _locktime (float __locktime))
   (setq _tracking_status (round __tracking_status))
   self)
  (:prn_number
   (&optional __prn_number)
   (if __prn_number (setq _prn_number __prn_number)) _prn_number)
  (:glofreq
   (&optional __glofreq)
   (if __glofreq (setq _glofreq __glofreq)) _glofreq)
  (:psr
   (&optional __psr)
   (if __psr (setq _psr __psr)) _psr)
  (:psr_std
   (&optional __psr_std)
   (if __psr_std (setq _psr_std __psr_std)) _psr_std)
  (:adr
   (&optional __adr)
   (if __adr (setq _adr __adr)) _adr)
  (:adr_std
   (&optional __adr_std)
   (if __adr_std (setq _adr_std __adr_std)) _adr_std)
  (:dopp
   (&optional __dopp)
   (if __dopp (setq _dopp __dopp)) _dopp)
  (:noise_density_ratio
   (&optional __noise_density_ratio)
   (if __noise_density_ratio (setq _noise_density_ratio __noise_density_ratio)) _noise_density_ratio)
  (:locktime
   (&optional __locktime)
   (if __locktime (setq _locktime __locktime)) _locktime)
  (:tracking_status
   (&optional __tracking_status)
   (if __tracking_status (setq _tracking_status __tracking_status)) _tracking_status)
  (:serialization-length
   ()
   (+
    ;; uint16 _prn_number
    2
    ;; uint16 _glofreq
    2
    ;; float64 _psr
    8
    ;; float32 _psr_std
    4
    ;; float64 _adr
    8
    ;; float32 _adr_std
    4
    ;; float32 _dopp
    4
    ;; float32 _noise_density_ratio
    4
    ;; float32 _locktime
    4
    ;; uint32 _tracking_status
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _prn_number
       (write-word _prn_number s)
     ;; uint16 _glofreq
       (write-word _glofreq s)
     ;; float64 _psr
       (sys::poke _psr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _psr_std
       (sys::poke _psr_std (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _adr
       (sys::poke _adr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _adr_std
       (sys::poke _adr_std (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dopp
       (sys::poke _dopp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _noise_density_ratio
       (sys::poke _noise_density_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _locktime
       (sys::poke _locktime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _tracking_status
       (write-long _tracking_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _prn_number
     (setq _prn_number (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _glofreq
     (setq _glofreq (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _psr
     (setq _psr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _psr_std
     (setq _psr_std (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _adr
     (setq _adr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _adr_std
     (setq _adr_std (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dopp
     (setq _dopp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _noise_density_ratio
     (setq _noise_density_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _locktime
     (setq _locktime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _tracking_status
     (setq _tracking_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::RangeInformation :md5sum-) "2c29299d245fc707e8f7544af871f110")
(setf (get novatel_gps_msgs::RangeInformation :datatype-) "novatel_gps_msgs/RangeInformation")
(setf (get novatel_gps_msgs::RangeInformation :definition-)
      "#Satellite Range information structure used in range messages

#Satelite PRN number of range measurement
uint16 prn_number

#GLONASS Frequency
uint16 glofreq

#Pseudorange measurement(m)
float64 psr

#Pseudorange measurement standard deviation(m)
float32 psr_std

#Carrier phase, in cycles
float64 adr

#Estimated carrier phase standard deviation(cycles)
float32 adr_std

#Instantaneous carrier Doppler frequency(Hz)
float32 dopp

#Carrier to noise density ratio
float32 noise_density_ratio

## of seconds of continous tracking
float32 locktime

#Tracking status
uint32 tracking_status

")



(provide :novatel_gps_msgs/RangeInformation "2c29299d245fc707e8f7544af871f110")


