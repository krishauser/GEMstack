;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::TrackstatChannel)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'TrackstatChannel (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::TRACKSTATCHANNEL")
  (make-package "NOVATEL_GPS_MSGS::TRACKSTATCHANNEL"))

(in-package "ROS")
;;//! \htmlinclude TrackstatChannel.msg.html


(defclass novatel_gps_msgs::TrackstatChannel
  :super ros::object
  :slots (_prn _glofreq _ch_tr_status _psr _doppler _c_no _locktime _psr_res _reject _psr_weight ))

(defmethod novatel_gps_msgs::TrackstatChannel
  (:init
   (&key
    ((:prn __prn) 0)
    ((:glofreq __glofreq) 0)
    ((:ch_tr_status __ch_tr_status) 0)
    ((:psr __psr) 0.0)
    ((:doppler __doppler) 0.0)
    ((:c_no __c_no) 0.0)
    ((:locktime __locktime) 0.0)
    ((:psr_res __psr_res) 0.0)
    ((:reject __reject) "")
    ((:psr_weight __psr_weight) 0.0)
    )
   (send-super :init)
   (setq _prn (round __prn))
   (setq _glofreq (round __glofreq))
   (setq _ch_tr_status (round __ch_tr_status))
   (setq _psr (float __psr))
   (setq _doppler (float __doppler))
   (setq _c_no (float __c_no))
   (setq _locktime (float __locktime))
   (setq _psr_res (float __psr_res))
   (setq _reject (string __reject))
   (setq _psr_weight (float __psr_weight))
   self)
  (:prn
   (&optional __prn)
   (if __prn (setq _prn __prn)) _prn)
  (:glofreq
   (&optional __glofreq)
   (if __glofreq (setq _glofreq __glofreq)) _glofreq)
  (:ch_tr_status
   (&optional __ch_tr_status)
   (if __ch_tr_status (setq _ch_tr_status __ch_tr_status)) _ch_tr_status)
  (:psr
   (&optional __psr)
   (if __psr (setq _psr __psr)) _psr)
  (:doppler
   (&optional __doppler)
   (if __doppler (setq _doppler __doppler)) _doppler)
  (:c_no
   (&optional __c_no)
   (if __c_no (setq _c_no __c_no)) _c_no)
  (:locktime
   (&optional __locktime)
   (if __locktime (setq _locktime __locktime)) _locktime)
  (:psr_res
   (&optional __psr_res)
   (if __psr_res (setq _psr_res __psr_res)) _psr_res)
  (:reject
   (&optional __reject)
   (if __reject (setq _reject __reject)) _reject)
  (:psr_weight
   (&optional __psr_weight)
   (if __psr_weight (setq _psr_weight __psr_weight)) _psr_weight)
  (:serialization-length
   ()
   (+
    ;; int16 _prn
    2
    ;; int16 _glofreq
    2
    ;; uint32 _ch_tr_status
    4
    ;; float64 _psr
    8
    ;; float32 _doppler
    4
    ;; float32 _c_no
    4
    ;; float32 _locktime
    4
    ;; float32 _psr_res
    4
    ;; string _reject
    4 (length _reject)
    ;; float32 _psr_weight
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _prn
       (write-word _prn s)
     ;; int16 _glofreq
       (write-word _glofreq s)
     ;; uint32 _ch_tr_status
       (write-long _ch_tr_status s)
     ;; float64 _psr
       (sys::poke _psr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _doppler
       (sys::poke _doppler (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _c_no
       (sys::poke _c_no (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _locktime
       (sys::poke _locktime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psr_res
       (sys::poke _psr_res (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _reject
       (write-long (length _reject) s) (princ _reject s)
     ;; float32 _psr_weight
       (sys::poke _psr_weight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _prn
     (setq _prn (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _glofreq
     (setq _glofreq (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _ch_tr_status
     (setq _ch_tr_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _psr
     (setq _psr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _doppler
     (setq _doppler (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _c_no
     (setq _c_no (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _locktime
     (setq _locktime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psr_res
     (setq _psr_res (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _reject
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reject (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _psr_weight
     (setq _psr_weight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::TrackstatChannel :md5sum-) "295831118c5ddfb83ac5b655586ae7ef")
(setf (get novatel_gps_msgs::TrackstatChannel :datatype-) "novatel_gps_msgs/TrackstatChannel")
(setf (get novatel_gps_msgs::TrackstatChannel :definition-)
      "# A submessage of Trackstat that contains all of the data about a single hardware channel

# Satellite PRN number
int16 prn

# GLONASS Frequency +7
int16 glofreq

# Channel tracking status
uint32 ch_tr_status

# Pseudorange (m)
float64 psr

# Doppler frequency (Hz)
float32 doppler

# Carrier to noise density ratio (dB-Hz)
float32 c_no

# Number of seconds of continuous tracking (no cycle slips)
float32 locktime

# Pseudorange residual from pseudorange filter (m)
float32 psr_res

# Range reject code from pseudorange filter
string reject

# Pseudorange filter weighting
float32 psr_weight
")



(provide :novatel_gps_msgs/TrackstatChannel "295831118c5ddfb83ac5b655586ae7ef")


