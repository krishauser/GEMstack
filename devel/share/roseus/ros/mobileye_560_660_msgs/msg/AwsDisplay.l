;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::AwsDisplay)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'AwsDisplay (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY")
  (make-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))

(in-package "ROS")
;;//! \htmlinclude AwsDisplay.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SOUND_SILENT*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_SILENT* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_SILENT* 0)
(intern "*SOUND_LDWL*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_LDWL* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_LDWL* 1)
(intern "*SOUND_LDWR*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_LDWR* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_LDWR* 2)
(intern "*SOUND_FAR_HW*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_FAR_HW* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_FAR_HW* 3)
(intern "*SOUND_NEAR_HW*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_NEAR_HW* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_NEAR_HW* 4)
(intern "*SOUND_SOFT_FCW*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_SOFT_FCW* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_SOFT_FCW* 5)
(intern "*SOUND_HARD_FCW*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_HARD_FCW* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_HARD_FCW* 6)
(intern "*SOUND_RESERVED*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*SOUND_RESERVED* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*SOUND_RESERVED* 7)
(intern "*HEADWAY_LEVEL_OFF*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*HEADWAY_LEVEL_OFF* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_OFF* 0)
(intern "*HEADWAY_LEVEL_GREEN*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*HEADWAY_LEVEL_GREEN* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_GREEN* 1)
(intern "*HEADWAY_LEVEL_ORANGE*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*HEADWAY_LEVEL_ORANGE* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_ORANGE* 2)
(intern "*HEADWAY_LEVEL_RED*" (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(shadow '*HEADWAY_LEVEL_RED* (find-package "MOBILEYE_560_660_MSGS::AWSDISPLAY"))
(defconstant mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_RED* 3)

(defun mobileye_560_660_msgs::AwsDisplay-to-symbol (const)
  (cond
        ((= const 0) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_SILENT*)
        ((= const 1) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_LDWL*)
        ((= const 2) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_LDWR*)
        ((= const 3) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_FAR_HW*)
        ((= const 4) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_NEAR_HW*)
        ((= const 5) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_SOFT_FCW*)
        ((= const 6) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_HARD_FCW*)
        ((= const 7) 'mobileye_560_660_msgs::AwsDisplay::*SOUND_RESERVED*)
        ((= const 0) 'mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_OFF*)
        ((= const 1) 'mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_GREEN*)
        ((= const 2) 'mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_ORANGE*)
        ((= const 3) 'mobileye_560_660_msgs::AwsDisplay::*HEADWAY_LEVEL_RED*)
        (t nil)))

(defclass mobileye_560_660_msgs::AwsDisplay
  :super ros::object
  :slots (_header _suppress_sound _night_time _dusk_time _sound_type _headway_valid _headway_measurement _lanes_on _left_ldw_on _right_ldw_on _fcw_on _left_crossing _right_crossing _maintenance _failsafe _ped_fcw _ped_in_dz _headway_warning_level ))

(defmethod mobileye_560_660_msgs::AwsDisplay
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:suppress_sound __suppress_sound) nil)
    ((:night_time __night_time) nil)
    ((:dusk_time __dusk_time) nil)
    ((:sound_type __sound_type) 0)
    ((:headway_valid __headway_valid) nil)
    ((:headway_measurement __headway_measurement) 0.0)
    ((:lanes_on __lanes_on) nil)
    ((:left_ldw_on __left_ldw_on) nil)
    ((:right_ldw_on __right_ldw_on) nil)
    ((:fcw_on __fcw_on) nil)
    ((:left_crossing __left_crossing) nil)
    ((:right_crossing __right_crossing) nil)
    ((:maintenance __maintenance) nil)
    ((:failsafe __failsafe) nil)
    ((:ped_fcw __ped_fcw) nil)
    ((:ped_in_dz __ped_in_dz) nil)
    ((:headway_warning_level __headway_warning_level) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _suppress_sound __suppress_sound)
   (setq _night_time __night_time)
   (setq _dusk_time __dusk_time)
   (setq _sound_type (round __sound_type))
   (setq _headway_valid __headway_valid)
   (setq _headway_measurement (float __headway_measurement))
   (setq _lanes_on __lanes_on)
   (setq _left_ldw_on __left_ldw_on)
   (setq _right_ldw_on __right_ldw_on)
   (setq _fcw_on __fcw_on)
   (setq _left_crossing __left_crossing)
   (setq _right_crossing __right_crossing)
   (setq _maintenance __maintenance)
   (setq _failsafe __failsafe)
   (setq _ped_fcw __ped_fcw)
   (setq _ped_in_dz __ped_in_dz)
   (setq _headway_warning_level (round __headway_warning_level))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:suppress_sound
   (&optional (__suppress_sound :null))
   (if (not (eq __suppress_sound :null)) (setq _suppress_sound __suppress_sound)) _suppress_sound)
  (:night_time
   (&optional (__night_time :null))
   (if (not (eq __night_time :null)) (setq _night_time __night_time)) _night_time)
  (:dusk_time
   (&optional (__dusk_time :null))
   (if (not (eq __dusk_time :null)) (setq _dusk_time __dusk_time)) _dusk_time)
  (:sound_type
   (&optional __sound_type)
   (if __sound_type (setq _sound_type __sound_type)) _sound_type)
  (:headway_valid
   (&optional (__headway_valid :null))
   (if (not (eq __headway_valid :null)) (setq _headway_valid __headway_valid)) _headway_valid)
  (:headway_measurement
   (&optional __headway_measurement)
   (if __headway_measurement (setq _headway_measurement __headway_measurement)) _headway_measurement)
  (:lanes_on
   (&optional (__lanes_on :null))
   (if (not (eq __lanes_on :null)) (setq _lanes_on __lanes_on)) _lanes_on)
  (:left_ldw_on
   (&optional (__left_ldw_on :null))
   (if (not (eq __left_ldw_on :null)) (setq _left_ldw_on __left_ldw_on)) _left_ldw_on)
  (:right_ldw_on
   (&optional (__right_ldw_on :null))
   (if (not (eq __right_ldw_on :null)) (setq _right_ldw_on __right_ldw_on)) _right_ldw_on)
  (:fcw_on
   (&optional (__fcw_on :null))
   (if (not (eq __fcw_on :null)) (setq _fcw_on __fcw_on)) _fcw_on)
  (:left_crossing
   (&optional (__left_crossing :null))
   (if (not (eq __left_crossing :null)) (setq _left_crossing __left_crossing)) _left_crossing)
  (:right_crossing
   (&optional (__right_crossing :null))
   (if (not (eq __right_crossing :null)) (setq _right_crossing __right_crossing)) _right_crossing)
  (:maintenance
   (&optional (__maintenance :null))
   (if (not (eq __maintenance :null)) (setq _maintenance __maintenance)) _maintenance)
  (:failsafe
   (&optional (__failsafe :null))
   (if (not (eq __failsafe :null)) (setq _failsafe __failsafe)) _failsafe)
  (:ped_fcw
   (&optional (__ped_fcw :null))
   (if (not (eq __ped_fcw :null)) (setq _ped_fcw __ped_fcw)) _ped_fcw)
  (:ped_in_dz
   (&optional (__ped_in_dz :null))
   (if (not (eq __ped_in_dz :null)) (setq _ped_in_dz __ped_in_dz)) _ped_in_dz)
  (:headway_warning_level
   (&optional __headway_warning_level)
   (if __headway_warning_level (setq _headway_warning_level __headway_warning_level)) _headway_warning_level)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _suppress_sound
    1
    ;; bool _night_time
    1
    ;; bool _dusk_time
    1
    ;; uint8 _sound_type
    1
    ;; bool _headway_valid
    1
    ;; float32 _headway_measurement
    4
    ;; bool _lanes_on
    1
    ;; bool _left_ldw_on
    1
    ;; bool _right_ldw_on
    1
    ;; bool _fcw_on
    1
    ;; bool _left_crossing
    1
    ;; bool _right_crossing
    1
    ;; bool _maintenance
    1
    ;; bool _failsafe
    1
    ;; bool _ped_fcw
    1
    ;; bool _ped_in_dz
    1
    ;; uint8 _headway_warning_level
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _suppress_sound
       (if _suppress_sound (write-byte -1 s) (write-byte 0 s))
     ;; bool _night_time
       (if _night_time (write-byte -1 s) (write-byte 0 s))
     ;; bool _dusk_time
       (if _dusk_time (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _sound_type
       (write-byte _sound_type s)
     ;; bool _headway_valid
       (if _headway_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _headway_measurement
       (sys::poke _headway_measurement (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _lanes_on
       (if _lanes_on (write-byte -1 s) (write-byte 0 s))
     ;; bool _left_ldw_on
       (if _left_ldw_on (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_ldw_on
       (if _right_ldw_on (write-byte -1 s) (write-byte 0 s))
     ;; bool _fcw_on
       (if _fcw_on (write-byte -1 s) (write-byte 0 s))
     ;; bool _left_crossing
       (if _left_crossing (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_crossing
       (if _right_crossing (write-byte -1 s) (write-byte 0 s))
     ;; bool _maintenance
       (if _maintenance (write-byte -1 s) (write-byte 0 s))
     ;; bool _failsafe
       (if _failsafe (write-byte -1 s) (write-byte 0 s))
     ;; bool _ped_fcw
       (if _ped_fcw (write-byte -1 s) (write-byte 0 s))
     ;; bool _ped_in_dz
       (if _ped_in_dz (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _headway_warning_level
       (write-byte _headway_warning_level s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _suppress_sound
     (setq _suppress_sound (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _night_time
     (setq _night_time (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dusk_time
     (setq _dusk_time (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _sound_type
     (setq _sound_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _headway_valid
     (setq _headway_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _headway_measurement
     (setq _headway_measurement (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _lanes_on
     (setq _lanes_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left_ldw_on
     (setq _left_ldw_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_ldw_on
     (setq _right_ldw_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fcw_on
     (setq _fcw_on (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left_crossing
     (setq _left_crossing (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_crossing
     (setq _right_crossing (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _maintenance
     (setq _maintenance (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _failsafe
     (setq _failsafe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ped_fcw
     (setq _ped_fcw (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ped_in_dz
     (setq _ped_in_dz (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _headway_warning_level
     (setq _headway_warning_level (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::AwsDisplay :md5sum-) "7aa82a0063aa4c0e719bef3d14c24bf7")
(setf (get mobileye_560_660_msgs::AwsDisplay :datatype-) "mobileye_560_660_msgs/AwsDisplay")
(setf (get mobileye_560_660_msgs::AwsDisplay :definition-)
      "std_msgs/Header header

bool suppress_sound
bool night_time
bool dusk_time

uint8 SOUND_SILENT = 0
uint8 SOUND_LDWL = 1
uint8 SOUND_LDWR = 2
uint8 SOUND_FAR_HW = 3
uint8 SOUND_NEAR_HW = 4
uint8 SOUND_SOFT_FCW = 5
uint8 SOUND_HARD_FCW = 6
uint8 SOUND_RESERVED = 7
uint8 sound_type

bool headway_valid
float32 headway_measurement
bool lanes_on
bool left_ldw_on
bool right_ldw_on
bool fcw_on
bool left_crossing
bool right_crossing
bool maintenance
bool failsafe
bool ped_fcw
bool ped_in_dz

uint8 HEADWAY_LEVEL_OFF = 0
uint8 HEADWAY_LEVEL_GREEN = 1
uint8 HEADWAY_LEVEL_ORANGE = 2
uint8 HEADWAY_LEVEL_RED = 3
uint8 headway_warning_level

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



(provide :mobileye_560_660_msgs/AwsDisplay "7aa82a0063aa4c0e719bef3d14c24bf7")


