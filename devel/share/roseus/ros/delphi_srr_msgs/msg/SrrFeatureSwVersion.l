;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrFeatureSwVersion)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrFeatureSwVersion (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRFEATURESWVERSION")
  (make-package "DELPHI_SRR_MSGS::SRRFEATURESWVERSION"))

(in-package "ROS")
;;//! \htmlinclude SrrFeatureSwVersion.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_srr_msgs::SrrFeatureSwVersion
  :super ros::object
  :slots (_header _lcma_sw_version _cta_sw_version ))

(defmethod delphi_srr_msgs::SrrFeatureSwVersion
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lcma_sw_version __lcma_sw_version) 0)
    ((:cta_sw_version __cta_sw_version) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lcma_sw_version (round __lcma_sw_version))
   (setq _cta_sw_version (round __cta_sw_version))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lcma_sw_version
   (&optional __lcma_sw_version)
   (if __lcma_sw_version (setq _lcma_sw_version __lcma_sw_version)) _lcma_sw_version)
  (:cta_sw_version
   (&optional __cta_sw_version)
   (if __cta_sw_version (setq _cta_sw_version __cta_sw_version)) _cta_sw_version)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _lcma_sw_version
    1
    ;; uint8 _cta_sw_version
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _lcma_sw_version
       (write-byte _lcma_sw_version s)
     ;; uint8 _cta_sw_version
       (write-byte _cta_sw_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _lcma_sw_version
     (setq _lcma_sw_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cta_sw_version
     (setq _cta_sw_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrFeatureSwVersion :md5sum-) "87f69bbbab65c94e0dda04a9e0914206")
(setf (get delphi_srr_msgs::SrrFeatureSwVersion :datatype-) "delphi_srr_msgs/SrrFeatureSwVersion")
(setf (get delphi_srr_msgs::SrrFeatureSwVersion :definition-)
      "# Message file for srr_feature_sw_version

std_msgs/Header header

uint8     lcma_sw_version
uint8     cta_sw_version

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



(provide :delphi_srr_msgs/SrrFeatureSwVersion "87f69bbbab65c94e0dda04a9e0914206")


