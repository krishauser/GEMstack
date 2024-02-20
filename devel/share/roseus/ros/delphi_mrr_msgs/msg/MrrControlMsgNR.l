;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrControlMsgNR)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrControlMsgNR (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRCONTROLMSGNR")
  (make-package "DELPHI_MRR_MSGS::MRRCONTROLMSGNR"))

(in-package "ROS")
;;//! \htmlinclude MrrControlMsgNR.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrControlMsgNR
  :super ros::object
  :slots (_header _can_stop_frequency_nrml _can_prp_factor_nrml _can_desired_sweep_bw_nrml _can_radiation_ctrl _can_stop_frequency_nrll _can_prp_factor_nrll _can_desired_sweep_bw_nrll ))

(defmethod delphi_mrr_msgs::MrrControlMsgNR
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_stop_frequency_nrml __can_stop_frequency_nrml) 0)
    ((:can_prp_factor_nrml __can_prp_factor_nrml) 0)
    ((:can_desired_sweep_bw_nrml __can_desired_sweep_bw_nrml) 0)
    ((:can_radiation_ctrl __can_radiation_ctrl) nil)
    ((:can_stop_frequency_nrll __can_stop_frequency_nrll) 0)
    ((:can_prp_factor_nrll __can_prp_factor_nrll) 0)
    ((:can_desired_sweep_bw_nrll __can_desired_sweep_bw_nrll) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_stop_frequency_nrml (round __can_stop_frequency_nrml))
   (setq _can_prp_factor_nrml (round __can_prp_factor_nrml))
   (setq _can_desired_sweep_bw_nrml (round __can_desired_sweep_bw_nrml))
   (setq _can_radiation_ctrl __can_radiation_ctrl)
   (setq _can_stop_frequency_nrll (round __can_stop_frequency_nrll))
   (setq _can_prp_factor_nrll (round __can_prp_factor_nrll))
   (setq _can_desired_sweep_bw_nrll (round __can_desired_sweep_bw_nrll))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_stop_frequency_nrml
   (&optional __can_stop_frequency_nrml)
   (if __can_stop_frequency_nrml (setq _can_stop_frequency_nrml __can_stop_frequency_nrml)) _can_stop_frequency_nrml)
  (:can_prp_factor_nrml
   (&optional __can_prp_factor_nrml)
   (if __can_prp_factor_nrml (setq _can_prp_factor_nrml __can_prp_factor_nrml)) _can_prp_factor_nrml)
  (:can_desired_sweep_bw_nrml
   (&optional __can_desired_sweep_bw_nrml)
   (if __can_desired_sweep_bw_nrml (setq _can_desired_sweep_bw_nrml __can_desired_sweep_bw_nrml)) _can_desired_sweep_bw_nrml)
  (:can_radiation_ctrl
   (&optional (__can_radiation_ctrl :null))
   (if (not (eq __can_radiation_ctrl :null)) (setq _can_radiation_ctrl __can_radiation_ctrl)) _can_radiation_ctrl)
  (:can_stop_frequency_nrll
   (&optional __can_stop_frequency_nrll)
   (if __can_stop_frequency_nrll (setq _can_stop_frequency_nrll __can_stop_frequency_nrll)) _can_stop_frequency_nrll)
  (:can_prp_factor_nrll
   (&optional __can_prp_factor_nrll)
   (if __can_prp_factor_nrll (setq _can_prp_factor_nrll __can_prp_factor_nrll)) _can_prp_factor_nrll)
  (:can_desired_sweep_bw_nrll
   (&optional __can_desired_sweep_bw_nrll)
   (if __can_desired_sweep_bw_nrll (setq _can_desired_sweep_bw_nrll __can_desired_sweep_bw_nrll)) _can_desired_sweep_bw_nrll)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _can_stop_frequency_nrml
    2
    ;; uint16 _can_prp_factor_nrml
    2
    ;; uint8 _can_desired_sweep_bw_nrml
    1
    ;; bool _can_radiation_ctrl
    1
    ;; uint16 _can_stop_frequency_nrll
    2
    ;; uint16 _can_prp_factor_nrll
    2
    ;; uint8 _can_desired_sweep_bw_nrll
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _can_stop_frequency_nrml
       (write-word _can_stop_frequency_nrml s)
     ;; uint16 _can_prp_factor_nrml
       (write-word _can_prp_factor_nrml s)
     ;; uint8 _can_desired_sweep_bw_nrml
       (write-byte _can_desired_sweep_bw_nrml s)
     ;; bool _can_radiation_ctrl
       (if _can_radiation_ctrl (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _can_stop_frequency_nrll
       (write-word _can_stop_frequency_nrll s)
     ;; uint16 _can_prp_factor_nrll
       (write-word _can_prp_factor_nrll s)
     ;; uint8 _can_desired_sweep_bw_nrll
       (write-byte _can_desired_sweep_bw_nrll s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _can_stop_frequency_nrml
     (setq _can_stop_frequency_nrml (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _can_prp_factor_nrml
     (setq _can_prp_factor_nrml (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _can_desired_sweep_bw_nrml
     (setq _can_desired_sweep_bw_nrml (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_radiation_ctrl
     (setq _can_radiation_ctrl (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _can_stop_frequency_nrll
     (setq _can_stop_frequency_nrll (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _can_prp_factor_nrll
     (setq _can_prp_factor_nrll (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _can_desired_sweep_bw_nrll
     (setq _can_desired_sweep_bw_nrll (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrControlMsgNR :md5sum-) "3100fbbd30b156c46cb7b9ae9e5d17a6")
(setf (get delphi_mrr_msgs::MrrControlMsgNR :datatype-) "delphi_mrr_msgs/MrrControlMsgNR")
(setf (get delphi_mrr_msgs::MrrControlMsgNR :definition-)
      "std_msgs/Header header

uint16 can_stop_frequency_nrml
uint16 can_prp_factor_nrml
uint8  can_desired_sweep_bw_nrml
bool   can_radiation_ctrl
uint16 can_stop_frequency_nrll
uint16 can_prp_factor_nrll
uint8  can_desired_sweep_bw_nrll

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



(provide :delphi_mrr_msgs/MrrControlMsgNR "3100fbbd30b156c46cb7b9ae9e5d17a6")


