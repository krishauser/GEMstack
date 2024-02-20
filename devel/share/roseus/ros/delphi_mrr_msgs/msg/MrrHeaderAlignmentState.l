;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrHeaderAlignmentState)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrHeaderAlignmentState (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRHEADERALIGNMENTSTATE")
  (make-package "DELPHI_MRR_MSGS::MRRHEADERALIGNMENTSTATE"))

(in-package "ROS")
;;//! \htmlinclude MrrHeaderAlignmentState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrHeaderAlignmentState
  :super ros::object
  :slots (_header _can_auto_align_hangle_qf _can_alignment_status _can_alignment_state _can_auto_align_hangle_ref _can_auto_align_hangle ))

(defmethod delphi_mrr_msgs::MrrHeaderAlignmentState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_auto_align_hangle_qf __can_auto_align_hangle_qf) 0)
    ((:can_alignment_status __can_alignment_status) 0)
    ((:can_alignment_state __can_alignment_state) 0)
    ((:can_auto_align_hangle_ref __can_auto_align_hangle_ref) 0.0)
    ((:can_auto_align_hangle __can_auto_align_hangle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_auto_align_hangle_qf (round __can_auto_align_hangle_qf))
   (setq _can_alignment_status (round __can_alignment_status))
   (setq _can_alignment_state (round __can_alignment_state))
   (setq _can_auto_align_hangle_ref (float __can_auto_align_hangle_ref))
   (setq _can_auto_align_hangle (float __can_auto_align_hangle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_auto_align_hangle_qf
   (&optional __can_auto_align_hangle_qf)
   (if __can_auto_align_hangle_qf (setq _can_auto_align_hangle_qf __can_auto_align_hangle_qf)) _can_auto_align_hangle_qf)
  (:can_alignment_status
   (&optional __can_alignment_status)
   (if __can_alignment_status (setq _can_alignment_status __can_alignment_status)) _can_alignment_status)
  (:can_alignment_state
   (&optional __can_alignment_state)
   (if __can_alignment_state (setq _can_alignment_state __can_alignment_state)) _can_alignment_state)
  (:can_auto_align_hangle_ref
   (&optional __can_auto_align_hangle_ref)
   (if __can_auto_align_hangle_ref (setq _can_auto_align_hangle_ref __can_auto_align_hangle_ref)) _can_auto_align_hangle_ref)
  (:can_auto_align_hangle
   (&optional __can_auto_align_hangle)
   (if __can_auto_align_hangle (setq _can_auto_align_hangle __can_auto_align_hangle)) _can_auto_align_hangle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_auto_align_hangle_qf
    1
    ;; uint8 _can_alignment_status
    1
    ;; uint8 _can_alignment_state
    1
    ;; float32 _can_auto_align_hangle_ref
    4
    ;; float32 _can_auto_align_hangle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_auto_align_hangle_qf
       (write-byte _can_auto_align_hangle_qf s)
     ;; uint8 _can_alignment_status
       (write-byte _can_alignment_status s)
     ;; uint8 _can_alignment_state
       (write-byte _can_alignment_state s)
     ;; float32 _can_auto_align_hangle_ref
       (sys::poke _can_auto_align_hangle_ref (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_auto_align_hangle
       (sys::poke _can_auto_align_hangle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_auto_align_hangle_qf
     (setq _can_auto_align_hangle_qf (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_alignment_status
     (setq _can_alignment_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_alignment_state
     (setq _can_alignment_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _can_auto_align_hangle_ref
     (setq _can_auto_align_hangle_ref (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_auto_align_hangle
     (setq _can_auto_align_hangle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrHeaderAlignmentState :md5sum-) "ed76a328bc6693a98452aedfe696f11a")
(setf (get delphi_mrr_msgs::MrrHeaderAlignmentState :datatype-) "delphi_mrr_msgs/MrrHeaderAlignmentState")
(setf (get delphi_mrr_msgs::MrrHeaderAlignmentState :definition-)
      "std_msgs/Header header

uint8   can_auto_align_hangle_qf
uint8   can_alignment_status
uint8   can_alignment_state
float32 can_auto_align_hangle_ref
float32 can_auto_align_hangle

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



(provide :delphi_mrr_msgs/MrrHeaderAlignmentState "ed76a328bc6693a98452aedfe696f11a")


