;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrStatusRadar)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrStatusRadar (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRSTATUSRADAR")
  (make-package "DELPHI_MRR_MSGS::MRRSTATUSRADAR"))

(in-package "ROS")
;;//! \htmlinclude MrrStatusRadar.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrStatusRadar
  :super ros::object
  :slots (_header _can_interference_type _can_recommend_unconverge _can_blockage_sidelobe_filter_val _can_radar_align_incomplete _can_blockage_sidelobe _can_blockage_mnr _can_radar_ext_cond_nok _can_radar_align_out_range _can_radar_align_not_start _can_radar_overheat_error _can_radar_not_op _can_xcvr_operational ))

(defmethod delphi_mrr_msgs::MrrStatusRadar
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_interference_type __can_interference_type) 0)
    ((:can_recommend_unconverge __can_recommend_unconverge) nil)
    ((:can_blockage_sidelobe_filter_val __can_blockage_sidelobe_filter_val) 0)
    ((:can_radar_align_incomplete __can_radar_align_incomplete) nil)
    ((:can_blockage_sidelobe __can_blockage_sidelobe) nil)
    ((:can_blockage_mnr __can_blockage_mnr) nil)
    ((:can_radar_ext_cond_nok __can_radar_ext_cond_nok) nil)
    ((:can_radar_align_out_range __can_radar_align_out_range) nil)
    ((:can_radar_align_not_start __can_radar_align_not_start) nil)
    ((:can_radar_overheat_error __can_radar_overheat_error) nil)
    ((:can_radar_not_op __can_radar_not_op) nil)
    ((:can_xcvr_operational __can_xcvr_operational) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_interference_type (round __can_interference_type))
   (setq _can_recommend_unconverge __can_recommend_unconverge)
   (setq _can_blockage_sidelobe_filter_val (round __can_blockage_sidelobe_filter_val))
   (setq _can_radar_align_incomplete __can_radar_align_incomplete)
   (setq _can_blockage_sidelobe __can_blockage_sidelobe)
   (setq _can_blockage_mnr __can_blockage_mnr)
   (setq _can_radar_ext_cond_nok __can_radar_ext_cond_nok)
   (setq _can_radar_align_out_range __can_radar_align_out_range)
   (setq _can_radar_align_not_start __can_radar_align_not_start)
   (setq _can_radar_overheat_error __can_radar_overheat_error)
   (setq _can_radar_not_op __can_radar_not_op)
   (setq _can_xcvr_operational __can_xcvr_operational)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_interference_type
   (&optional __can_interference_type)
   (if __can_interference_type (setq _can_interference_type __can_interference_type)) _can_interference_type)
  (:can_recommend_unconverge
   (&optional (__can_recommend_unconverge :null))
   (if (not (eq __can_recommend_unconverge :null)) (setq _can_recommend_unconverge __can_recommend_unconverge)) _can_recommend_unconverge)
  (:can_blockage_sidelobe_filter_val
   (&optional __can_blockage_sidelobe_filter_val)
   (if __can_blockage_sidelobe_filter_val (setq _can_blockage_sidelobe_filter_val __can_blockage_sidelobe_filter_val)) _can_blockage_sidelobe_filter_val)
  (:can_radar_align_incomplete
   (&optional (__can_radar_align_incomplete :null))
   (if (not (eq __can_radar_align_incomplete :null)) (setq _can_radar_align_incomplete __can_radar_align_incomplete)) _can_radar_align_incomplete)
  (:can_blockage_sidelobe
   (&optional (__can_blockage_sidelobe :null))
   (if (not (eq __can_blockage_sidelobe :null)) (setq _can_blockage_sidelobe __can_blockage_sidelobe)) _can_blockage_sidelobe)
  (:can_blockage_mnr
   (&optional (__can_blockage_mnr :null))
   (if (not (eq __can_blockage_mnr :null)) (setq _can_blockage_mnr __can_blockage_mnr)) _can_blockage_mnr)
  (:can_radar_ext_cond_nok
   (&optional (__can_radar_ext_cond_nok :null))
   (if (not (eq __can_radar_ext_cond_nok :null)) (setq _can_radar_ext_cond_nok __can_radar_ext_cond_nok)) _can_radar_ext_cond_nok)
  (:can_radar_align_out_range
   (&optional (__can_radar_align_out_range :null))
   (if (not (eq __can_radar_align_out_range :null)) (setq _can_radar_align_out_range __can_radar_align_out_range)) _can_radar_align_out_range)
  (:can_radar_align_not_start
   (&optional (__can_radar_align_not_start :null))
   (if (not (eq __can_radar_align_not_start :null)) (setq _can_radar_align_not_start __can_radar_align_not_start)) _can_radar_align_not_start)
  (:can_radar_overheat_error
   (&optional (__can_radar_overheat_error :null))
   (if (not (eq __can_radar_overheat_error :null)) (setq _can_radar_overheat_error __can_radar_overheat_error)) _can_radar_overheat_error)
  (:can_radar_not_op
   (&optional (__can_radar_not_op :null))
   (if (not (eq __can_radar_not_op :null)) (setq _can_radar_not_op __can_radar_not_op)) _can_radar_not_op)
  (:can_xcvr_operational
   (&optional (__can_xcvr_operational :null))
   (if (not (eq __can_xcvr_operational :null)) (setq _can_xcvr_operational __can_xcvr_operational)) _can_xcvr_operational)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_interference_type
    1
    ;; bool _can_recommend_unconverge
    1
    ;; uint8 _can_blockage_sidelobe_filter_val
    1
    ;; bool _can_radar_align_incomplete
    1
    ;; bool _can_blockage_sidelobe
    1
    ;; bool _can_blockage_mnr
    1
    ;; bool _can_radar_ext_cond_nok
    1
    ;; bool _can_radar_align_out_range
    1
    ;; bool _can_radar_align_not_start
    1
    ;; bool _can_radar_overheat_error
    1
    ;; bool _can_radar_not_op
    1
    ;; bool _can_xcvr_operational
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_interference_type
       (write-byte _can_interference_type s)
     ;; bool _can_recommend_unconverge
       (if _can_recommend_unconverge (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _can_blockage_sidelobe_filter_val
       (write-byte _can_blockage_sidelobe_filter_val s)
     ;; bool _can_radar_align_incomplete
       (if _can_radar_align_incomplete (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_blockage_sidelobe
       (if _can_blockage_sidelobe (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_blockage_mnr
       (if _can_blockage_mnr (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_radar_ext_cond_nok
       (if _can_radar_ext_cond_nok (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_radar_align_out_range
       (if _can_radar_align_out_range (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_radar_align_not_start
       (if _can_radar_align_not_start (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_radar_overheat_error
       (if _can_radar_overheat_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_radar_not_op
       (if _can_radar_not_op (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_xcvr_operational
       (if _can_xcvr_operational (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_interference_type
     (setq _can_interference_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_recommend_unconverge
     (setq _can_recommend_unconverge (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _can_blockage_sidelobe_filter_val
     (setq _can_blockage_sidelobe_filter_val (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_radar_align_incomplete
     (setq _can_radar_align_incomplete (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_blockage_sidelobe
     (setq _can_blockage_sidelobe (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_blockage_mnr
     (setq _can_blockage_mnr (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_radar_ext_cond_nok
     (setq _can_radar_ext_cond_nok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_radar_align_out_range
     (setq _can_radar_align_out_range (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_radar_align_not_start
     (setq _can_radar_align_not_start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_radar_overheat_error
     (setq _can_radar_overheat_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_radar_not_op
     (setq _can_radar_not_op (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_xcvr_operational
     (setq _can_xcvr_operational (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrStatusRadar :md5sum-) "3dbdaa8c61c744a4f2863586bf997cac")
(setf (get delphi_mrr_msgs::MrrStatusRadar :datatype-) "delphi_mrr_msgs/MrrStatusRadar")
(setf (get delphi_mrr_msgs::MrrStatusRadar :definition-)
      "std_msgs/Header header

uint8 can_interference_type
bool  can_recommend_unconverge
uint8 can_blockage_sidelobe_filter_val
bool  can_radar_align_incomplete
bool  can_blockage_sidelobe
bool  can_blockage_mnr
bool  can_radar_ext_cond_nok
bool  can_radar_align_out_range
bool  can_radar_align_not_start
bool  can_radar_overheat_error
bool  can_radar_not_op
bool  can_xcvr_operational

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



(provide :delphi_mrr_msgs/MrrStatusRadar "3dbdaa8c61c744a4f2863586bf997cac")


