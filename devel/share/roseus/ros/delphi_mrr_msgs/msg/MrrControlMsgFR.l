;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrControlMsgFR)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrControlMsgFR (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRCONTROLMSGFR")
  (make-package "DELPHI_MRR_MSGS::MRRCONTROLMSGFR"))

(in-package "ROS")
;;//! \htmlinclude MrrControlMsgFR.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrControlMsgFR
  :super ros::object
  :slots (_header _can_sensitivity_profile_select _can_stop_frequency_frml _can_stop_frequency_frll _can_prp_factor_frml _can_prp_factor_frll _can_desired_sweep_bw_frml _can_desired_sweep_bw_frll ))

(defmethod delphi_mrr_msgs::MrrControlMsgFR
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_sensitivity_profile_select __can_sensitivity_profile_select) 0)
    ((:can_stop_frequency_frml __can_stop_frequency_frml) 0)
    ((:can_stop_frequency_frll __can_stop_frequency_frll) 0)
    ((:can_prp_factor_frml __can_prp_factor_frml) 0.0)
    ((:can_prp_factor_frll __can_prp_factor_frll) 0.0)
    ((:can_desired_sweep_bw_frml __can_desired_sweep_bw_frml) 0)
    ((:can_desired_sweep_bw_frll __can_desired_sweep_bw_frll) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_sensitivity_profile_select (round __can_sensitivity_profile_select))
   (setq _can_stop_frequency_frml (round __can_stop_frequency_frml))
   (setq _can_stop_frequency_frll (round __can_stop_frequency_frll))
   (setq _can_prp_factor_frml (float __can_prp_factor_frml))
   (setq _can_prp_factor_frll (float __can_prp_factor_frll))
   (setq _can_desired_sweep_bw_frml (round __can_desired_sweep_bw_frml))
   (setq _can_desired_sweep_bw_frll (round __can_desired_sweep_bw_frll))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_sensitivity_profile_select
   (&optional __can_sensitivity_profile_select)
   (if __can_sensitivity_profile_select (setq _can_sensitivity_profile_select __can_sensitivity_profile_select)) _can_sensitivity_profile_select)
  (:can_stop_frequency_frml
   (&optional __can_stop_frequency_frml)
   (if __can_stop_frequency_frml (setq _can_stop_frequency_frml __can_stop_frequency_frml)) _can_stop_frequency_frml)
  (:can_stop_frequency_frll
   (&optional __can_stop_frequency_frll)
   (if __can_stop_frequency_frll (setq _can_stop_frequency_frll __can_stop_frequency_frll)) _can_stop_frequency_frll)
  (:can_prp_factor_frml
   (&optional __can_prp_factor_frml)
   (if __can_prp_factor_frml (setq _can_prp_factor_frml __can_prp_factor_frml)) _can_prp_factor_frml)
  (:can_prp_factor_frll
   (&optional __can_prp_factor_frll)
   (if __can_prp_factor_frll (setq _can_prp_factor_frll __can_prp_factor_frll)) _can_prp_factor_frll)
  (:can_desired_sweep_bw_frml
   (&optional __can_desired_sweep_bw_frml)
   (if __can_desired_sweep_bw_frml (setq _can_desired_sweep_bw_frml __can_desired_sweep_bw_frml)) _can_desired_sweep_bw_frml)
  (:can_desired_sweep_bw_frll
   (&optional __can_desired_sweep_bw_frll)
   (if __can_desired_sweep_bw_frll (setq _can_desired_sweep_bw_frll __can_desired_sweep_bw_frll)) _can_desired_sweep_bw_frll)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_sensitivity_profile_select
    1
    ;; uint16 _can_stop_frequency_frml
    2
    ;; uint16 _can_stop_frequency_frll
    2
    ;; float32 _can_prp_factor_frml
    4
    ;; float32 _can_prp_factor_frll
    4
    ;; uint8 _can_desired_sweep_bw_frml
    1
    ;; uint8 _can_desired_sweep_bw_frll
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_sensitivity_profile_select
       (write-byte _can_sensitivity_profile_select s)
     ;; uint16 _can_stop_frequency_frml
       (write-word _can_stop_frequency_frml s)
     ;; uint16 _can_stop_frequency_frll
       (write-word _can_stop_frequency_frll s)
     ;; float32 _can_prp_factor_frml
       (sys::poke _can_prp_factor_frml (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_prp_factor_frll
       (sys::poke _can_prp_factor_frll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _can_desired_sweep_bw_frml
       (write-byte _can_desired_sweep_bw_frml s)
     ;; uint8 _can_desired_sweep_bw_frll
       (write-byte _can_desired_sweep_bw_frll s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_sensitivity_profile_select
     (setq _can_sensitivity_profile_select (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _can_stop_frequency_frml
     (setq _can_stop_frequency_frml (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _can_stop_frequency_frll
     (setq _can_stop_frequency_frll (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _can_prp_factor_frml
     (setq _can_prp_factor_frml (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_prp_factor_frll
     (setq _can_prp_factor_frll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _can_desired_sweep_bw_frml
     (setq _can_desired_sweep_bw_frml (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_desired_sweep_bw_frll
     (setq _can_desired_sweep_bw_frll (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrControlMsgFR :md5sum-) "dba2c9fc1e4b47706ab1d6d7c85d6d53")
(setf (get delphi_mrr_msgs::MrrControlMsgFR :datatype-) "delphi_mrr_msgs/MrrControlMsgFR")
(setf (get delphi_mrr_msgs::MrrControlMsgFR :definition-)
      "std_msgs/Header header

uint8   can_sensitivity_profile_select
uint16  can_stop_frequency_frml
uint16  can_stop_frequency_frll
float32 can_prp_factor_frml
float32 can_prp_factor_frll
uint8   can_desired_sweep_bw_frml
uint8   can_desired_sweep_bw_frll

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



(provide :delphi_mrr_msgs/MrrControlMsgFR "dba2c9fc1e4b47706ab1d6d7c85d6d53")


