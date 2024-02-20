;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrDebug5)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrDebug5 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRDEBUG5")
  (make-package "DELPHI_SRR_MSGS::SRRDEBUG5"))

(in-package "ROS")
;;//! \htmlinclude SrrDebug5.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_srr_msgs::SrrDebug5
  :super ros::object
  :slots (_header _can_tx_align_updates ))

(defmethod delphi_srr_msgs::SrrDebug5
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_tx_align_updates __can_tx_align_updates) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_tx_align_updates (round __can_tx_align_updates))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_tx_align_updates
   (&optional __can_tx_align_updates)
   (if __can_tx_align_updates (setq _can_tx_align_updates __can_tx_align_updates)) _can_tx_align_updates)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _can_tx_align_updates
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _can_tx_align_updates
       (write-word _can_tx_align_updates s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _can_tx_align_updates
     (setq _can_tx_align_updates (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrDebug5 :md5sum-) "33cb8dd5c14e9c9dfee5824987d9b07a")
(setf (get delphi_srr_msgs::SrrDebug5 :datatype-) "delphi_srr_msgs/SrrDebug5")
(setf (get delphi_srr_msgs::SrrDebug5 :definition-)
      "# Message file for srr_debug5

std_msgs/Header header

uint16    can_tx_align_updates

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



(provide :delphi_srr_msgs/SrrDebug5 "33cb8dd5c14e9c9dfee5824987d9b07a")


