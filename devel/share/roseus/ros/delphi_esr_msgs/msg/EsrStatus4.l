;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrStatus4)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrStatus4 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRSTATUS4")
  (make-package "DELPHI_ESR_MSGS::ESRSTATUS4"))

(in-package "ROS")
;;//! \htmlinclude EsrStatus4.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrStatus4
  :super ros::object
  :slots (_header _canmsg _truck_target_det _lr_only_grating_lobe_det _sidelobe_blockage _partial_blockage _mr_lr_mode _rolling_count_3 _path_id_acc _path_id_cmbb_move _path_id_cmbb_stat _path_id_fcw_move _path_id_fcw_stat _auto_align_angle _path_id_acc_stat ))

(defmethod delphi_esr_msgs::EsrStatus4
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:truck_target_det __truck_target_det) nil)
    ((:lr_only_grating_lobe_det __lr_only_grating_lobe_det) nil)
    ((:sidelobe_blockage __sidelobe_blockage) nil)
    ((:partial_blockage __partial_blockage) nil)
    ((:mr_lr_mode __mr_lr_mode) 0)
    ((:rolling_count_3 __rolling_count_3) 0)
    ((:path_id_acc __path_id_acc) 0)
    ((:path_id_cmbb_move __path_id_cmbb_move) 0)
    ((:path_id_cmbb_stat __path_id_cmbb_stat) 0)
    ((:path_id_fcw_move __path_id_fcw_move) 0)
    ((:path_id_fcw_stat __path_id_fcw_stat) 0)
    ((:auto_align_angle __auto_align_angle) 0.0)
    ((:path_id_acc_stat __path_id_acc_stat) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _truck_target_det __truck_target_det)
   (setq _lr_only_grating_lobe_det __lr_only_grating_lobe_det)
   (setq _sidelobe_blockage __sidelobe_blockage)
   (setq _partial_blockage __partial_blockage)
   (setq _mr_lr_mode (round __mr_lr_mode))
   (setq _rolling_count_3 (round __rolling_count_3))
   (setq _path_id_acc (round __path_id_acc))
   (setq _path_id_cmbb_move (round __path_id_cmbb_move))
   (setq _path_id_cmbb_stat (round __path_id_cmbb_stat))
   (setq _path_id_fcw_move (round __path_id_fcw_move))
   (setq _path_id_fcw_stat (round __path_id_fcw_stat))
   (setq _auto_align_angle (float __auto_align_angle))
   (setq _path_id_acc_stat (round __path_id_acc_stat))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:canmsg
   (&optional __canmsg)
   (if __canmsg (setq _canmsg __canmsg)) _canmsg)
  (:truck_target_det
   (&optional (__truck_target_det :null))
   (if (not (eq __truck_target_det :null)) (setq _truck_target_det __truck_target_det)) _truck_target_det)
  (:lr_only_grating_lobe_det
   (&optional (__lr_only_grating_lobe_det :null))
   (if (not (eq __lr_only_grating_lobe_det :null)) (setq _lr_only_grating_lobe_det __lr_only_grating_lobe_det)) _lr_only_grating_lobe_det)
  (:sidelobe_blockage
   (&optional (__sidelobe_blockage :null))
   (if (not (eq __sidelobe_blockage :null)) (setq _sidelobe_blockage __sidelobe_blockage)) _sidelobe_blockage)
  (:partial_blockage
   (&optional (__partial_blockage :null))
   (if (not (eq __partial_blockage :null)) (setq _partial_blockage __partial_blockage)) _partial_blockage)
  (:mr_lr_mode
   (&optional __mr_lr_mode)
   (if __mr_lr_mode (setq _mr_lr_mode __mr_lr_mode)) _mr_lr_mode)
  (:rolling_count_3
   (&optional __rolling_count_3)
   (if __rolling_count_3 (setq _rolling_count_3 __rolling_count_3)) _rolling_count_3)
  (:path_id_acc
   (&optional __path_id_acc)
   (if __path_id_acc (setq _path_id_acc __path_id_acc)) _path_id_acc)
  (:path_id_cmbb_move
   (&optional __path_id_cmbb_move)
   (if __path_id_cmbb_move (setq _path_id_cmbb_move __path_id_cmbb_move)) _path_id_cmbb_move)
  (:path_id_cmbb_stat
   (&optional __path_id_cmbb_stat)
   (if __path_id_cmbb_stat (setq _path_id_cmbb_stat __path_id_cmbb_stat)) _path_id_cmbb_stat)
  (:path_id_fcw_move
   (&optional __path_id_fcw_move)
   (if __path_id_fcw_move (setq _path_id_fcw_move __path_id_fcw_move)) _path_id_fcw_move)
  (:path_id_fcw_stat
   (&optional __path_id_fcw_stat)
   (if __path_id_fcw_stat (setq _path_id_fcw_stat __path_id_fcw_stat)) _path_id_fcw_stat)
  (:auto_align_angle
   (&optional __auto_align_angle)
   (if __auto_align_angle (setq _auto_align_angle __auto_align_angle)) _auto_align_angle)
  (:path_id_acc_stat
   (&optional __path_id_acc_stat)
   (if __path_id_acc_stat (setq _path_id_acc_stat __path_id_acc_stat)) _path_id_acc_stat)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; bool _truck_target_det
    1
    ;; bool _lr_only_grating_lobe_det
    1
    ;; bool _sidelobe_blockage
    1
    ;; bool _partial_blockage
    1
    ;; uint8 _mr_lr_mode
    1
    ;; uint8 _rolling_count_3
    1
    ;; uint8 _path_id_acc
    1
    ;; uint8 _path_id_cmbb_move
    1
    ;; uint8 _path_id_cmbb_stat
    1
    ;; uint8 _path_id_fcw_move
    1
    ;; uint8 _path_id_fcw_stat
    1
    ;; float32 _auto_align_angle
    4
    ;; uint8 _path_id_acc_stat
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; bool _truck_target_det
       (if _truck_target_det (write-byte -1 s) (write-byte 0 s))
     ;; bool _lr_only_grating_lobe_det
       (if _lr_only_grating_lobe_det (write-byte -1 s) (write-byte 0 s))
     ;; bool _sidelobe_blockage
       (if _sidelobe_blockage (write-byte -1 s) (write-byte 0 s))
     ;; bool _partial_blockage
       (if _partial_blockage (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _mr_lr_mode
       (write-byte _mr_lr_mode s)
     ;; uint8 _rolling_count_3
       (write-byte _rolling_count_3 s)
     ;; uint8 _path_id_acc
       (write-byte _path_id_acc s)
     ;; uint8 _path_id_cmbb_move
       (write-byte _path_id_cmbb_move s)
     ;; uint8 _path_id_cmbb_stat
       (write-byte _path_id_cmbb_stat s)
     ;; uint8 _path_id_fcw_move
       (write-byte _path_id_fcw_move s)
     ;; uint8 _path_id_fcw_stat
       (write-byte _path_id_fcw_stat s)
     ;; float32 _auto_align_angle
       (sys::poke _auto_align_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _path_id_acc_stat
       (write-byte _path_id_acc_stat s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _truck_target_det
     (setq _truck_target_det (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _lr_only_grating_lobe_det
     (setq _lr_only_grating_lobe_det (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _sidelobe_blockage
     (setq _sidelobe_blockage (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _partial_blockage
     (setq _partial_blockage (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _mr_lr_mode
     (setq _mr_lr_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rolling_count_3
     (setq _rolling_count_3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _path_id_acc
     (setq _path_id_acc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _path_id_cmbb_move
     (setq _path_id_cmbb_move (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _path_id_cmbb_stat
     (setq _path_id_cmbb_stat (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _path_id_fcw_move
     (setq _path_id_fcw_move (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _path_id_fcw_stat
     (setq _path_id_fcw_stat (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _auto_align_angle
     (setq _auto_align_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _path_id_acc_stat
     (setq _path_id_acc_stat (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrStatus4 :md5sum-) "6d073b78c0d621fce59ffa9fb7c576de")
(setf (get delphi_esr_msgs::EsrStatus4 :datatype-) "delphi_esr_msgs/EsrStatus4")
(setf (get delphi_esr_msgs::EsrStatus4 :definition-)
      "std_msgs/Header header

# ESR Status4
string      canmsg

bool        truck_target_det
bool        lr_only_grating_lobe_det
bool        sidelobe_blockage
bool        partial_blockage
uint8       mr_lr_mode
uint8       rolling_count_3
uint8       path_id_acc
uint8       path_id_cmbb_move
uint8       path_id_cmbb_stat
uint8       path_id_fcw_move
uint8       path_id_fcw_stat
float32     auto_align_angle
uint8       path_id_acc_stat

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



(provide :delphi_esr_msgs/EsrStatus4 "6d073b78c0d621fce59ffa9fb7c576de")


