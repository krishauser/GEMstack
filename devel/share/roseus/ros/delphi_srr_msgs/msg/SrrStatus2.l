;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrStatus2)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrStatus2 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRSTATUS2")
  (make-package "DELPHI_SRR_MSGS::SRRSTATUS2"))

(in-package "ROS")
;;//! \htmlinclude SrrStatus2.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CAN_TX_ALIGNMENT_STATUS_UNKNOWN*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(shadow '*CAN_TX_ALIGNMENT_STATUS_UNKNOWN* (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(defconstant delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_UNKNOWN* 0)
(intern "*CAN_TX_ALIGNMENT_STATUS_CONVERGED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(shadow '*CAN_TX_ALIGNMENT_STATUS_CONVERGED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(defconstant delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_CONVERGED* 1)
(intern "*CAN_TX_ALIGNMENT_STATUS_FAILED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(shadow '*CAN_TX_ALIGNMENT_STATUS_FAILED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(defconstant delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_FAILED* 2)
(intern "*CAN_TX_ALIGNMENT_STATUS_RESERVED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(shadow '*CAN_TX_ALIGNMENT_STATUS_RESERVED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS2"))
(defconstant delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_RESERVED* 3)

(defun delphi_srr_msgs::SrrStatus2-to-symbol (const)
  (cond
        ((= const 0) 'delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_UNKNOWN*)
        ((= const 1) 'delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_CONVERGED*)
        ((= const 2) 'delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_FAILED*)
        ((= const 3) 'delphi_srr_msgs::SrrStatus2::*CAN_TX_ALIGNMENT_STATUS_RESERVED*)
        (t nil)))

(defclass delphi_srr_msgs::SrrStatus2
  :super ros::object
  :slots (_header _can_tx_alignment_status _can_tx_comm_error _can_tx_steering_angle_sign _can_tx_yaw_rate_bias _can_tx_veh_spd_comp_factor _can_tx_sw_version_dsp _can_tx_temperature _can_tx_range_perf_error _can_tx_overheat_error _can_tx_internal_error _can_tx_xcvr_operational _can_tx_steering_angle _can_tx_rolling_count_2 ))

(defmethod delphi_srr_msgs::SrrStatus2
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_tx_alignment_status __can_tx_alignment_status) 0)
    ((:can_tx_comm_error __can_tx_comm_error) nil)
    ((:can_tx_steering_angle_sign __can_tx_steering_angle_sign) nil)
    ((:can_tx_yaw_rate_bias __can_tx_yaw_rate_bias) 0.0)
    ((:can_tx_veh_spd_comp_factor __can_tx_veh_spd_comp_factor) 0.0)
    ((:can_tx_sw_version_dsp __can_tx_sw_version_dsp) 0)
    ((:can_tx_temperature __can_tx_temperature) 0)
    ((:can_tx_range_perf_error __can_tx_range_perf_error) nil)
    ((:can_tx_overheat_error __can_tx_overheat_error) nil)
    ((:can_tx_internal_error __can_tx_internal_error) nil)
    ((:can_tx_xcvr_operational __can_tx_xcvr_operational) nil)
    ((:can_tx_steering_angle __can_tx_steering_angle) 0)
    ((:can_tx_rolling_count_2 __can_tx_rolling_count_2) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_tx_alignment_status (round __can_tx_alignment_status))
   (setq _can_tx_comm_error __can_tx_comm_error)
   (setq _can_tx_steering_angle_sign __can_tx_steering_angle_sign)
   (setq _can_tx_yaw_rate_bias (float __can_tx_yaw_rate_bias))
   (setq _can_tx_veh_spd_comp_factor (float __can_tx_veh_spd_comp_factor))
   (setq _can_tx_sw_version_dsp (round __can_tx_sw_version_dsp))
   (setq _can_tx_temperature (round __can_tx_temperature))
   (setq _can_tx_range_perf_error __can_tx_range_perf_error)
   (setq _can_tx_overheat_error __can_tx_overheat_error)
   (setq _can_tx_internal_error __can_tx_internal_error)
   (setq _can_tx_xcvr_operational __can_tx_xcvr_operational)
   (setq _can_tx_steering_angle (round __can_tx_steering_angle))
   (setq _can_tx_rolling_count_2 (round __can_tx_rolling_count_2))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_tx_alignment_status
   (&optional __can_tx_alignment_status)
   (if __can_tx_alignment_status (setq _can_tx_alignment_status __can_tx_alignment_status)) _can_tx_alignment_status)
  (:can_tx_comm_error
   (&optional (__can_tx_comm_error :null))
   (if (not (eq __can_tx_comm_error :null)) (setq _can_tx_comm_error __can_tx_comm_error)) _can_tx_comm_error)
  (:can_tx_steering_angle_sign
   (&optional (__can_tx_steering_angle_sign :null))
   (if (not (eq __can_tx_steering_angle_sign :null)) (setq _can_tx_steering_angle_sign __can_tx_steering_angle_sign)) _can_tx_steering_angle_sign)
  (:can_tx_yaw_rate_bias
   (&optional __can_tx_yaw_rate_bias)
   (if __can_tx_yaw_rate_bias (setq _can_tx_yaw_rate_bias __can_tx_yaw_rate_bias)) _can_tx_yaw_rate_bias)
  (:can_tx_veh_spd_comp_factor
   (&optional __can_tx_veh_spd_comp_factor)
   (if __can_tx_veh_spd_comp_factor (setq _can_tx_veh_spd_comp_factor __can_tx_veh_spd_comp_factor)) _can_tx_veh_spd_comp_factor)
  (:can_tx_sw_version_dsp
   (&optional __can_tx_sw_version_dsp)
   (if __can_tx_sw_version_dsp (setq _can_tx_sw_version_dsp __can_tx_sw_version_dsp)) _can_tx_sw_version_dsp)
  (:can_tx_temperature
   (&optional __can_tx_temperature)
   (if __can_tx_temperature (setq _can_tx_temperature __can_tx_temperature)) _can_tx_temperature)
  (:can_tx_range_perf_error
   (&optional (__can_tx_range_perf_error :null))
   (if (not (eq __can_tx_range_perf_error :null)) (setq _can_tx_range_perf_error __can_tx_range_perf_error)) _can_tx_range_perf_error)
  (:can_tx_overheat_error
   (&optional (__can_tx_overheat_error :null))
   (if (not (eq __can_tx_overheat_error :null)) (setq _can_tx_overheat_error __can_tx_overheat_error)) _can_tx_overheat_error)
  (:can_tx_internal_error
   (&optional (__can_tx_internal_error :null))
   (if (not (eq __can_tx_internal_error :null)) (setq _can_tx_internal_error __can_tx_internal_error)) _can_tx_internal_error)
  (:can_tx_xcvr_operational
   (&optional (__can_tx_xcvr_operational :null))
   (if (not (eq __can_tx_xcvr_operational :null)) (setq _can_tx_xcvr_operational __can_tx_xcvr_operational)) _can_tx_xcvr_operational)
  (:can_tx_steering_angle
   (&optional __can_tx_steering_angle)
   (if __can_tx_steering_angle (setq _can_tx_steering_angle __can_tx_steering_angle)) _can_tx_steering_angle)
  (:can_tx_rolling_count_2
   (&optional __can_tx_rolling_count_2)
   (if __can_tx_rolling_count_2 (setq _can_tx_rolling_count_2 __can_tx_rolling_count_2)) _can_tx_rolling_count_2)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_tx_alignment_status
    1
    ;; bool _can_tx_comm_error
    1
    ;; bool _can_tx_steering_angle_sign
    1
    ;; float32 _can_tx_yaw_rate_bias
    4
    ;; float32 _can_tx_veh_spd_comp_factor
    4
    ;; uint16 _can_tx_sw_version_dsp
    2
    ;; int16 _can_tx_temperature
    2
    ;; bool _can_tx_range_perf_error
    1
    ;; bool _can_tx_overheat_error
    1
    ;; bool _can_tx_internal_error
    1
    ;; bool _can_tx_xcvr_operational
    1
    ;; uint16 _can_tx_steering_angle
    2
    ;; uint8 _can_tx_rolling_count_2
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_tx_alignment_status
       (write-byte _can_tx_alignment_status s)
     ;; bool _can_tx_comm_error
       (if _can_tx_comm_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_tx_steering_angle_sign
       (if _can_tx_steering_angle_sign (write-byte -1 s) (write-byte 0 s))
     ;; float32 _can_tx_yaw_rate_bias
       (sys::poke _can_tx_yaw_rate_bias (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_veh_spd_comp_factor
       (sys::poke _can_tx_veh_spd_comp_factor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _can_tx_sw_version_dsp
       (write-word _can_tx_sw_version_dsp s)
     ;; int16 _can_tx_temperature
       (write-word _can_tx_temperature s)
     ;; bool _can_tx_range_perf_error
       (if _can_tx_range_perf_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_tx_overheat_error
       (if _can_tx_overheat_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_tx_internal_error
       (if _can_tx_internal_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_tx_xcvr_operational
       (if _can_tx_xcvr_operational (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _can_tx_steering_angle
       (write-word _can_tx_steering_angle s)
     ;; uint8 _can_tx_rolling_count_2
       (write-byte _can_tx_rolling_count_2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_tx_alignment_status
     (setq _can_tx_alignment_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_tx_comm_error
     (setq _can_tx_comm_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_tx_steering_angle_sign
     (setq _can_tx_steering_angle_sign (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _can_tx_yaw_rate_bias
     (setq _can_tx_yaw_rate_bias (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_veh_spd_comp_factor
     (setq _can_tx_veh_spd_comp_factor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _can_tx_sw_version_dsp
     (setq _can_tx_sw_version_dsp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _can_tx_temperature
     (setq _can_tx_temperature (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _can_tx_range_perf_error
     (setq _can_tx_range_perf_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_tx_overheat_error
     (setq _can_tx_overheat_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_tx_internal_error
     (setq _can_tx_internal_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_tx_xcvr_operational
     (setq _can_tx_xcvr_operational (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _can_tx_steering_angle
     (setq _can_tx_steering_angle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _can_tx_rolling_count_2
     (setq _can_tx_rolling_count_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrStatus2 :md5sum-) "2b05d1c3cfa8185e9616806113ff9b8c")
(setf (get delphi_srr_msgs::SrrStatus2 :datatype-) "delphi_srr_msgs/SrrStatus2")
(setf (get delphi_srr_msgs::SrrStatus2 :definition-)
      "# Message file for srr_status2

std_msgs/Header header

uint8     can_tx_alignment_status
uint8     CAN_TX_ALIGNMENT_STATUS_UNKNOWN=0
uint8     CAN_TX_ALIGNMENT_STATUS_CONVERGED=1
uint8     CAN_TX_ALIGNMENT_STATUS_FAILED=2
uint8     CAN_TX_ALIGNMENT_STATUS_RESERVED=3

bool      can_tx_comm_error
bool      can_tx_steering_angle_sign
float32   can_tx_yaw_rate_bias
float32   can_tx_veh_spd_comp_factor
uint16    can_tx_sw_version_dsp
int16     can_tx_temperature                       # degc
bool      can_tx_range_perf_error
bool      can_tx_overheat_error
bool      can_tx_internal_error
bool      can_tx_xcvr_operational
uint16    can_tx_steering_angle                    # deg
uint8     can_tx_rolling_count_2


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



(provide :delphi_srr_msgs/SrrStatus2 "2b05d1c3cfa8185e9616806113ff9b8c")


