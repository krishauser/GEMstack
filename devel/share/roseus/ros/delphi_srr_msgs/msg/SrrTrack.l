;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrTrack)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrTrack (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRTRACK")
  (make-package "DELPHI_SRR_MSGS::SRRTRACK"))

(in-package "ROS")
;;//! \htmlinclude SrrTrack.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION* 0)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_1*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_1* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_1* 1)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_2*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_2* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_2* 2)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_3*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_3* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_3* 3)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_4*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_4* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_4* 4)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_5*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_5* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_5* 5)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_6*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_6* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_6* 6)
(intern "*CAN_TX_DETECT_VALID_LEVEL_LEVEL_7*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_VALID_LEVEL_LEVEL_7* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_7* 7)
(intern "*CAN_TX_DETECT_STATUS_NO_DATA*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_STATUS_NO_DATA* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_STATUS_NO_DATA* nil)
(intern "*CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT*" (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(shadow '*CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT* (find-package "DELPHI_SRR_MSGS::SRRTRACK"))
(defconstant delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT* t)

(defun delphi_srr_msgs::SrrTrack-to-symbol (const)
  (cond
        ((= const 0) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION*)
        ((= const 1) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_1*)
        ((= const 2) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_2*)
        ((= const 3) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_3*)
        ((= const 4) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_4*)
        ((= const 5) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_5*)
        ((= const 6) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_6*)
        ((= const 7) 'delphi_srr_msgs::SrrTrack::*CAN_TX_DETECT_VALID_LEVEL_LEVEL_7*)
        (t nil)))

(defclass delphi_srr_msgs::SrrTrack
  :super ros::object
  :slots (_header _can_tx_detect_valid_level _can_tx_detect_status _can_tx_detect_range_rate _can_tx_detect_range _can_tx_detect_angle _can_tx_detect_amplitude ))

(defmethod delphi_srr_msgs::SrrTrack
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_tx_detect_valid_level __can_tx_detect_valid_level) 0)
    ((:can_tx_detect_status __can_tx_detect_status) nil)
    ((:can_tx_detect_range_rate __can_tx_detect_range_rate) 0.0)
    ((:can_tx_detect_range __can_tx_detect_range) 0.0)
    ((:can_tx_detect_angle __can_tx_detect_angle) 0.0)
    ((:can_tx_detect_amplitude __can_tx_detect_amplitude) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_tx_detect_valid_level (round __can_tx_detect_valid_level))
   (setq _can_tx_detect_status __can_tx_detect_status)
   (setq _can_tx_detect_range_rate (float __can_tx_detect_range_rate))
   (setq _can_tx_detect_range (float __can_tx_detect_range))
   (setq _can_tx_detect_angle (float __can_tx_detect_angle))
   (setq _can_tx_detect_amplitude (float __can_tx_detect_amplitude))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_tx_detect_valid_level
   (&optional __can_tx_detect_valid_level)
   (if __can_tx_detect_valid_level (setq _can_tx_detect_valid_level __can_tx_detect_valid_level)) _can_tx_detect_valid_level)
  (:can_tx_detect_status
   (&optional (__can_tx_detect_status :null))
   (if (not (eq __can_tx_detect_status :null)) (setq _can_tx_detect_status __can_tx_detect_status)) _can_tx_detect_status)
  (:can_tx_detect_range_rate
   (&optional __can_tx_detect_range_rate)
   (if __can_tx_detect_range_rate (setq _can_tx_detect_range_rate __can_tx_detect_range_rate)) _can_tx_detect_range_rate)
  (:can_tx_detect_range
   (&optional __can_tx_detect_range)
   (if __can_tx_detect_range (setq _can_tx_detect_range __can_tx_detect_range)) _can_tx_detect_range)
  (:can_tx_detect_angle
   (&optional __can_tx_detect_angle)
   (if __can_tx_detect_angle (setq _can_tx_detect_angle __can_tx_detect_angle)) _can_tx_detect_angle)
  (:can_tx_detect_amplitude
   (&optional __can_tx_detect_amplitude)
   (if __can_tx_detect_amplitude (setq _can_tx_detect_amplitude __can_tx_detect_amplitude)) _can_tx_detect_amplitude)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_tx_detect_valid_level
    1
    ;; bool _can_tx_detect_status
    1
    ;; float32 _can_tx_detect_range_rate
    4
    ;; float32 _can_tx_detect_range
    4
    ;; float32 _can_tx_detect_angle
    4
    ;; float32 _can_tx_detect_amplitude
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_tx_detect_valid_level
       (write-byte _can_tx_detect_valid_level s)
     ;; bool _can_tx_detect_status
       (if _can_tx_detect_status (write-byte -1 s) (write-byte 0 s))
     ;; float32 _can_tx_detect_range_rate
       (sys::poke _can_tx_detect_range_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_detect_range
       (sys::poke _can_tx_detect_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_detect_angle
       (sys::poke _can_tx_detect_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_detect_amplitude
       (sys::poke _can_tx_detect_amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_tx_detect_valid_level
     (setq _can_tx_detect_valid_level (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_tx_detect_status
     (setq _can_tx_detect_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _can_tx_detect_range_rate
     (setq _can_tx_detect_range_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_detect_range
     (setq _can_tx_detect_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_detect_angle
     (setq _can_tx_detect_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_detect_amplitude
     (setq _can_tx_detect_amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrTrack :md5sum-) "a689930ba3ce2066d655a7425f6fdbde")
(setf (get delphi_srr_msgs::SrrTrack :datatype-) "delphi_srr_msgs/SrrTrack")
(setf (get delphi_srr_msgs::SrrTrack :definition-)
      "# Message file for srr_track

std_msgs/Header header

uint8     can_tx_detect_valid_level
uint8     CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION=0
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_1=1
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_2=2
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_3=3
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_4=4
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_5=5
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_6=6
uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_7=7

bool      can_tx_detect_status
bool      CAN_TX_DETECT_STATUS_NO_DATA=0
bool      CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT=1

float32   can_tx_detect_range_rate                 # m/s
float32   can_tx_detect_range                      # m
float32   can_tx_detect_angle                      # deg
float32   can_tx_detect_amplitude                  # dbsm

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

")



(provide :delphi_srr_msgs/SrrTrack "a689930ba3ce2066d655a7425f6fdbde")


