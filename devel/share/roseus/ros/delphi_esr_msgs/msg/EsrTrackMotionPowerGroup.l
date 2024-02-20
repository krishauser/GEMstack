;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrTrackMotionPowerGroup)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrTrackMotionPowerGroup (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRTRACKMOTIONPOWERGROUP")
  (make-package "DELPHI_ESR_MSGS::ESRTRACKMOTIONPOWERGROUP"))

(in-package "ROS")
;;//! \htmlinclude EsrTrackMotionPowerGroup.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrTrackMotionPowerGroup
  :super ros::object
  :slots (_header _canmsg _rolling_count_2 _can_id_group _tracks ))

(defmethod delphi_esr_msgs::EsrTrackMotionPowerGroup
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:rolling_count_2 __rolling_count_2) 0)
    ((:can_id_group __can_id_group) 0)
    ((:tracks __tracks) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _rolling_count_2 (round __rolling_count_2))
   (setq _can_id_group (round __can_id_group))
   (setq _tracks __tracks)
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
  (:rolling_count_2
   (&optional __rolling_count_2)
   (if __rolling_count_2 (setq _rolling_count_2 __rolling_count_2)) _rolling_count_2)
  (:can_id_group
   (&optional __can_id_group)
   (if __can_id_group (setq _can_id_group __can_id_group)) _can_id_group)
  (:tracks
   (&rest __tracks)
   (if (keywordp (car __tracks))
       (send* _tracks __tracks)
     (progn
       (if __tracks (setq _tracks (car __tracks)))
       _tracks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _rolling_count_2
    1
    ;; uint8 _can_id_group
    1
    ;; delphi_esr_msgs/EsrTrackMotionPowerTrack[] _tracks
    (apply #'+ (send-all _tracks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; uint8 _rolling_count_2
       (write-byte _rolling_count_2 s)
     ;; uint8 _can_id_group
       (write-byte _can_id_group s)
     ;; delphi_esr_msgs/EsrTrackMotionPowerTrack[] _tracks
     (write-long (length _tracks) s)
     (dolist (elem _tracks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _rolling_count_2
     (setq _rolling_count_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_id_group
     (setq _can_id_group (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; delphi_esr_msgs/EsrTrackMotionPowerTrack[] _tracks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tracks (let (r) (dotimes (i n) (push (instance delphi_esr_msgs::EsrTrackMotionPowerTrack :init) r)) r))
     (dolist (elem- _tracks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrTrackMotionPowerGroup :md5sum-) "58598630b679d4a2eed0f058be9b1aaa")
(setf (get delphi_esr_msgs::EsrTrackMotionPowerGroup :datatype-) "delphi_esr_msgs/EsrTrackMotionPowerGroup")
(setf (get delphi_esr_msgs::EsrTrackMotionPowerGroup :definition-)
      "std_msgs/Header header

# ESR TrackMotionPower, information common to a group
string                                      canmsg

uint8                                       rolling_count_2
uint8                                       can_id_group
delphi_esr_msgs/EsrTrackMotionPowerTrack[]  tracks

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

================================================================================
MSG: delphi_esr_msgs/EsrTrackMotionPowerTrack
# ESR TrackMotionPower, track-specific information
uint8  id
bool   movable_fast
bool   movable_slow
bool   moving
int8   power

")



(provide :delphi_esr_msgs/EsrTrackMotionPowerGroup "58598630b679d4a2eed0f058be9b1aaa")


