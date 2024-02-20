;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::Ahbc)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'Ahbc (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::AHBC")
  (make-package "MOBILEYE_560_660_MSGS::AHBC"))

(in-package "ROS")
;;//! \htmlinclude Ahbc.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION*" (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(shadow '*HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION* (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(defconstant mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION* 0)
(intern "*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF*" (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(shadow '*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF* (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(defconstant mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF* 1)
(intern "*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON*" (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(shadow '*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON* (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(defconstant mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON* 2)
(intern "*HIGH_LOW_BEAM_DECISION_INVALID*" (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(shadow '*HIGH_LOW_BEAM_DECISION_INVALID* (find-package "MOBILEYE_560_660_MSGS::AHBC"))
(defconstant mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_INVALID* 3)

(defun mobileye_560_660_msgs::Ahbc-to-symbol (const)
  (cond
        ((= const 0) 'mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION*)
        ((= const 1) 'mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF*)
        ((= const 2) 'mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON*)
        ((= const 3) 'mobileye_560_660_msgs::Ahbc::*HIGH_LOW_BEAM_DECISION_INVALID*)
        (t nil)))

(defclass mobileye_560_660_msgs::Ahbc
  :super ros::object
  :slots (_header _high_low_beam_decision _reasons_for_switch_to_low_beam ))

(defmethod mobileye_560_660_msgs::Ahbc
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:high_low_beam_decision __high_low_beam_decision) 0)
    ((:reasons_for_switch_to_low_beam __reasons_for_switch_to_low_beam) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _high_low_beam_decision (round __high_low_beam_decision))
   (setq _reasons_for_switch_to_low_beam (round __reasons_for_switch_to_low_beam))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:high_low_beam_decision
   (&optional __high_low_beam_decision)
   (if __high_low_beam_decision (setq _high_low_beam_decision __high_low_beam_decision)) _high_low_beam_decision)
  (:reasons_for_switch_to_low_beam
   (&optional __reasons_for_switch_to_low_beam)
   (if __reasons_for_switch_to_low_beam (setq _reasons_for_switch_to_low_beam __reasons_for_switch_to_low_beam)) _reasons_for_switch_to_low_beam)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _high_low_beam_decision
    1
    ;; uint16 _reasons_for_switch_to_low_beam
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _high_low_beam_decision
       (write-byte _high_low_beam_decision s)
     ;; uint16 _reasons_for_switch_to_low_beam
       (write-word _reasons_for_switch_to_low_beam s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _high_low_beam_decision
     (setq _high_low_beam_decision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _reasons_for_switch_to_low_beam
     (setq _reasons_for_switch_to_low_beam (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::Ahbc :md5sum-) "475e214fc14bee0ccbbfc2ae7aaea6ec")
(setf (get mobileye_560_660_msgs::Ahbc :datatype-) "mobileye_560_660_msgs/Ahbc")
(setf (get mobileye_560_660_msgs::Ahbc :definition-)
      "std_msgs/Header header

uint8 HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION = 0
uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF = 1
uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON = 2
uint8 HIGH_LOW_BEAM_DECISION_INVALID = 3
uint8 high_low_beam_decision

uint16 reasons_for_switch_to_low_beam

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



(provide :mobileye_560_660_msgs/Ahbc "475e214fc14bee0ccbbfc2ae7aaea6ec")


