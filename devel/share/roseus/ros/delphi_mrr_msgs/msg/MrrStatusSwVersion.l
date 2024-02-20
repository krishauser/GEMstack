;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrStatusSwVersion)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrStatusSwVersion (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRSTATUSSWVERSION")
  (make-package "DELPHI_MRR_MSGS::MRRSTATUSSWVERSION"))

(in-package "ROS")
;;//! \htmlinclude MrrStatusSwVersion.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrStatusSwVersion
  :super ros::object
  :slots (_header _can_pbl_field_revision _can_pbl_promote_revision _can_sw_field_revision _can_sw_promote_revision _can_sw_release_revision _can_pbl_release_revision ))

(defmethod delphi_mrr_msgs::MrrStatusSwVersion
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_pbl_field_revision __can_pbl_field_revision) 0)
    ((:can_pbl_promote_revision __can_pbl_promote_revision) 0)
    ((:can_sw_field_revision __can_sw_field_revision) 0)
    ((:can_sw_promote_revision __can_sw_promote_revision) 0)
    ((:can_sw_release_revision __can_sw_release_revision) 0)
    ((:can_pbl_release_revision __can_pbl_release_revision) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_pbl_field_revision (round __can_pbl_field_revision))
   (setq _can_pbl_promote_revision (round __can_pbl_promote_revision))
   (setq _can_sw_field_revision (round __can_sw_field_revision))
   (setq _can_sw_promote_revision (round __can_sw_promote_revision))
   (setq _can_sw_release_revision (round __can_sw_release_revision))
   (setq _can_pbl_release_revision (round __can_pbl_release_revision))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_pbl_field_revision
   (&optional __can_pbl_field_revision)
   (if __can_pbl_field_revision (setq _can_pbl_field_revision __can_pbl_field_revision)) _can_pbl_field_revision)
  (:can_pbl_promote_revision
   (&optional __can_pbl_promote_revision)
   (if __can_pbl_promote_revision (setq _can_pbl_promote_revision __can_pbl_promote_revision)) _can_pbl_promote_revision)
  (:can_sw_field_revision
   (&optional __can_sw_field_revision)
   (if __can_sw_field_revision (setq _can_sw_field_revision __can_sw_field_revision)) _can_sw_field_revision)
  (:can_sw_promote_revision
   (&optional __can_sw_promote_revision)
   (if __can_sw_promote_revision (setq _can_sw_promote_revision __can_sw_promote_revision)) _can_sw_promote_revision)
  (:can_sw_release_revision
   (&optional __can_sw_release_revision)
   (if __can_sw_release_revision (setq _can_sw_release_revision __can_sw_release_revision)) _can_sw_release_revision)
  (:can_pbl_release_revision
   (&optional __can_pbl_release_revision)
   (if __can_pbl_release_revision (setq _can_pbl_release_revision __can_pbl_release_revision)) _can_pbl_release_revision)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_pbl_field_revision
    1
    ;; uint8 _can_pbl_promote_revision
    1
    ;; uint8 _can_sw_field_revision
    1
    ;; uint8 _can_sw_promote_revision
    1
    ;; uint8 _can_sw_release_revision
    1
    ;; uint8 _can_pbl_release_revision
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_pbl_field_revision
       (write-byte _can_pbl_field_revision s)
     ;; uint8 _can_pbl_promote_revision
       (write-byte _can_pbl_promote_revision s)
     ;; uint8 _can_sw_field_revision
       (write-byte _can_sw_field_revision s)
     ;; uint8 _can_sw_promote_revision
       (write-byte _can_sw_promote_revision s)
     ;; uint8 _can_sw_release_revision
       (write-byte _can_sw_release_revision s)
     ;; uint8 _can_pbl_release_revision
       (write-byte _can_pbl_release_revision s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_pbl_field_revision
     (setq _can_pbl_field_revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_pbl_promote_revision
     (setq _can_pbl_promote_revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_sw_field_revision
     (setq _can_sw_field_revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_sw_promote_revision
     (setq _can_sw_promote_revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_sw_release_revision
     (setq _can_sw_release_revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_pbl_release_revision
     (setq _can_pbl_release_revision (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrStatusSwVersion :md5sum-) "2cd8dd3a73da2efcca1c60aaa8a7f9c5")
(setf (get delphi_mrr_msgs::MrrStatusSwVersion :datatype-) "delphi_mrr_msgs/MrrStatusSwVersion")
(setf (get delphi_mrr_msgs::MrrStatusSwVersion :definition-)
      "std_msgs/Header header

uint8 can_pbl_field_revision
uint8 can_pbl_promote_revision
uint8 can_sw_field_revision
uint8 can_sw_promote_revision
uint8 can_sw_release_revision
uint8 can_pbl_release_revision

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



(provide :delphi_mrr_msgs/MrrStatusSwVersion "2cd8dd3a73da2efcca1c60aaa8a7f9c5")


