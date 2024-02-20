;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrStatusCanVersion)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrStatusCanVersion (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRSTATUSCANVERSION")
  (make-package "DELPHI_MRR_MSGS::MRRSTATUSCANVERSION"))

(in-package "ROS")
;;//! \htmlinclude MrrStatusCanVersion.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrStatusCanVersion
  :super ros::object
  :slots (_header _can_usc_section_compatibility _can_pcan_minor_mrr _can_pcan_major_mrr ))

(defmethod delphi_mrr_msgs::MrrStatusCanVersion
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_usc_section_compatibility __can_usc_section_compatibility) 0)
    ((:can_pcan_minor_mrr __can_pcan_minor_mrr) 0)
    ((:can_pcan_major_mrr __can_pcan_major_mrr) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_usc_section_compatibility (round __can_usc_section_compatibility))
   (setq _can_pcan_minor_mrr (round __can_pcan_minor_mrr))
   (setq _can_pcan_major_mrr (round __can_pcan_major_mrr))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_usc_section_compatibility
   (&optional __can_usc_section_compatibility)
   (if __can_usc_section_compatibility (setq _can_usc_section_compatibility __can_usc_section_compatibility)) _can_usc_section_compatibility)
  (:can_pcan_minor_mrr
   (&optional __can_pcan_minor_mrr)
   (if __can_pcan_minor_mrr (setq _can_pcan_minor_mrr __can_pcan_minor_mrr)) _can_pcan_minor_mrr)
  (:can_pcan_major_mrr
   (&optional __can_pcan_major_mrr)
   (if __can_pcan_major_mrr (setq _can_pcan_major_mrr __can_pcan_major_mrr)) _can_pcan_major_mrr)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _can_usc_section_compatibility
    2
    ;; uint8 _can_pcan_minor_mrr
    1
    ;; uint8 _can_pcan_major_mrr
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _can_usc_section_compatibility
       (write-word _can_usc_section_compatibility s)
     ;; uint8 _can_pcan_minor_mrr
       (write-byte _can_pcan_minor_mrr s)
     ;; uint8 _can_pcan_major_mrr
       (write-byte _can_pcan_major_mrr s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _can_usc_section_compatibility
     (setq _can_usc_section_compatibility (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _can_pcan_minor_mrr
     (setq _can_pcan_minor_mrr (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_pcan_major_mrr
     (setq _can_pcan_major_mrr (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrStatusCanVersion :md5sum-) "a03c2b40596b39ba528bf17fb334bae6")
(setf (get delphi_mrr_msgs::MrrStatusCanVersion :datatype-) "delphi_mrr_msgs/MrrStatusCanVersion")
(setf (get delphi_mrr_msgs::MrrStatusCanVersion :definition-)
      "std_msgs/Header header

uint16 can_usc_section_compatibility
uint8  can_pcan_minor_mrr
uint8  can_pcan_major_mrr

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



(provide :delphi_mrr_msgs/MrrStatusCanVersion "a03c2b40596b39ba528bf17fb334bae6")


