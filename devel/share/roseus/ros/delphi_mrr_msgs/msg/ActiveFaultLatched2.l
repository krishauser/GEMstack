;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::ActiveFaultLatched2)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'ActiveFaultLatched2 (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::ACTIVEFAULTLATCHED2")
  (make-package "DELPHI_MRR_MSGS::ACTIVEFAULTLATCHED2"))

(in-package "ROS")
;;//! \htmlinclude ActiveFaultLatched2.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::ActiveFaultLatched2
  :super ros::object
  :slots (_header _ipma_pcan_data_range_check _ipma_pcan_missing_msg _vin_signal_compare_failure _module_not_configured_error _car_cfg_not_configured_error ))

(defmethod delphi_mrr_msgs::ActiveFaultLatched2
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ipma_pcan_data_range_check __ipma_pcan_data_range_check) nil)
    ((:ipma_pcan_missing_msg __ipma_pcan_missing_msg) nil)
    ((:vin_signal_compare_failure __vin_signal_compare_failure) nil)
    ((:module_not_configured_error __module_not_configured_error) nil)
    ((:car_cfg_not_configured_error __car_cfg_not_configured_error) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ipma_pcan_data_range_check __ipma_pcan_data_range_check)
   (setq _ipma_pcan_missing_msg __ipma_pcan_missing_msg)
   (setq _vin_signal_compare_failure __vin_signal_compare_failure)
   (setq _module_not_configured_error __module_not_configured_error)
   (setq _car_cfg_not_configured_error __car_cfg_not_configured_error)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ipma_pcan_data_range_check
   (&optional (__ipma_pcan_data_range_check :null))
   (if (not (eq __ipma_pcan_data_range_check :null)) (setq _ipma_pcan_data_range_check __ipma_pcan_data_range_check)) _ipma_pcan_data_range_check)
  (:ipma_pcan_missing_msg
   (&optional (__ipma_pcan_missing_msg :null))
   (if (not (eq __ipma_pcan_missing_msg :null)) (setq _ipma_pcan_missing_msg __ipma_pcan_missing_msg)) _ipma_pcan_missing_msg)
  (:vin_signal_compare_failure
   (&optional (__vin_signal_compare_failure :null))
   (if (not (eq __vin_signal_compare_failure :null)) (setq _vin_signal_compare_failure __vin_signal_compare_failure)) _vin_signal_compare_failure)
  (:module_not_configured_error
   (&optional (__module_not_configured_error :null))
   (if (not (eq __module_not_configured_error :null)) (setq _module_not_configured_error __module_not_configured_error)) _module_not_configured_error)
  (:car_cfg_not_configured_error
   (&optional (__car_cfg_not_configured_error :null))
   (if (not (eq __car_cfg_not_configured_error :null)) (setq _car_cfg_not_configured_error __car_cfg_not_configured_error)) _car_cfg_not_configured_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _ipma_pcan_data_range_check
    1
    ;; bool _ipma_pcan_missing_msg
    1
    ;; bool _vin_signal_compare_failure
    1
    ;; bool _module_not_configured_error
    1
    ;; bool _car_cfg_not_configured_error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _ipma_pcan_data_range_check
       (if _ipma_pcan_data_range_check (write-byte -1 s) (write-byte 0 s))
     ;; bool _ipma_pcan_missing_msg
       (if _ipma_pcan_missing_msg (write-byte -1 s) (write-byte 0 s))
     ;; bool _vin_signal_compare_failure
       (if _vin_signal_compare_failure (write-byte -1 s) (write-byte 0 s))
     ;; bool _module_not_configured_error
       (if _module_not_configured_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _car_cfg_not_configured_error
       (if _car_cfg_not_configured_error (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _ipma_pcan_data_range_check
     (setq _ipma_pcan_data_range_check (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ipma_pcan_missing_msg
     (setq _ipma_pcan_missing_msg (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _vin_signal_compare_failure
     (setq _vin_signal_compare_failure (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _module_not_configured_error
     (setq _module_not_configured_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _car_cfg_not_configured_error
     (setq _car_cfg_not_configured_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::ActiveFaultLatched2 :md5sum-) "fcd978d054e184e337e27e3c35da2f6d")
(setf (get delphi_mrr_msgs::ActiveFaultLatched2 :datatype-) "delphi_mrr_msgs/ActiveFaultLatched2")
(setf (get delphi_mrr_msgs::ActiveFaultLatched2 :definition-)
      "std_msgs/Header header

bool ipma_pcan_data_range_check
bool ipma_pcan_missing_msg
bool vin_signal_compare_failure
bool module_not_configured_error
bool car_cfg_not_configured_error

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



(provide :delphi_mrr_msgs/ActiveFaultLatched2 "fcd978d054e184e337e27e3c35da2f6d")


