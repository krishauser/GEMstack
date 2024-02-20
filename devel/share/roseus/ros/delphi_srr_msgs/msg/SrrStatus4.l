;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrStatus4)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrStatus4 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRSTATUS4")
  (make-package "DELPHI_SRR_MSGS::SRRSTATUS4"))

(in-package "ROS")
;;//! \htmlinclude SrrStatus4.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_srr_msgs::SrrStatus4
  :super ros::object
  :slots (_header _can_tx_sw_version_host _can_tx_path_id_blis_ignore _can_tx_path_id_blis _can_tx_angle_misalignment _can_tx_auto_align_angle ))

(defmethod delphi_srr_msgs::SrrStatus4
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_tx_sw_version_host __can_tx_sw_version_host) 0)
    ((:can_tx_path_id_blis_ignore __can_tx_path_id_blis_ignore) 0)
    ((:can_tx_path_id_blis __can_tx_path_id_blis) 0)
    ((:can_tx_angle_misalignment __can_tx_angle_misalignment) 0.0)
    ((:can_tx_auto_align_angle __can_tx_auto_align_angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_tx_sw_version_host (round __can_tx_sw_version_host))
   (setq _can_tx_path_id_blis_ignore (round __can_tx_path_id_blis_ignore))
   (setq _can_tx_path_id_blis (round __can_tx_path_id_blis))
   (setq _can_tx_angle_misalignment (float __can_tx_angle_misalignment))
   (setq _can_tx_auto_align_angle (float __can_tx_auto_align_angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_tx_sw_version_host
   (&optional __can_tx_sw_version_host)
   (if __can_tx_sw_version_host (setq _can_tx_sw_version_host __can_tx_sw_version_host)) _can_tx_sw_version_host)
  (:can_tx_path_id_blis_ignore
   (&optional __can_tx_path_id_blis_ignore)
   (if __can_tx_path_id_blis_ignore (setq _can_tx_path_id_blis_ignore __can_tx_path_id_blis_ignore)) _can_tx_path_id_blis_ignore)
  (:can_tx_path_id_blis
   (&optional __can_tx_path_id_blis)
   (if __can_tx_path_id_blis (setq _can_tx_path_id_blis __can_tx_path_id_blis)) _can_tx_path_id_blis)
  (:can_tx_angle_misalignment
   (&optional __can_tx_angle_misalignment)
   (if __can_tx_angle_misalignment (setq _can_tx_angle_misalignment __can_tx_angle_misalignment)) _can_tx_angle_misalignment)
  (:can_tx_auto_align_angle
   (&optional __can_tx_auto_align_angle)
   (if __can_tx_auto_align_angle (setq _can_tx_auto_align_angle __can_tx_auto_align_angle)) _can_tx_auto_align_angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _can_tx_sw_version_host
    2
    ;; uint8 _can_tx_path_id_blis_ignore
    1
    ;; uint8 _can_tx_path_id_blis
    1
    ;; float32 _can_tx_angle_misalignment
    4
    ;; float32 _can_tx_auto_align_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _can_tx_sw_version_host
       (write-word _can_tx_sw_version_host s)
     ;; uint8 _can_tx_path_id_blis_ignore
       (write-byte _can_tx_path_id_blis_ignore s)
     ;; uint8 _can_tx_path_id_blis
       (write-byte _can_tx_path_id_blis s)
     ;; float32 _can_tx_angle_misalignment
       (sys::poke _can_tx_angle_misalignment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_auto_align_angle
       (sys::poke _can_tx_auto_align_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _can_tx_sw_version_host
     (setq _can_tx_sw_version_host (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _can_tx_path_id_blis_ignore
     (setq _can_tx_path_id_blis_ignore (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_tx_path_id_blis
     (setq _can_tx_path_id_blis (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _can_tx_angle_misalignment
     (setq _can_tx_angle_misalignment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_auto_align_angle
     (setq _can_tx_auto_align_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrStatus4 :md5sum-) "8f5e5c4790453e1f3bcd5507dd8162bd")
(setf (get delphi_srr_msgs::SrrStatus4 :datatype-) "delphi_srr_msgs/SrrStatus4")
(setf (get delphi_srr_msgs::SrrStatus4 :definition-)
      "# Message file for srr_status4

std_msgs/Header header

uint16    can_tx_sw_version_host
uint8     can_tx_path_id_blis_ignore
uint8     can_tx_path_id_blis
float32   can_tx_angle_misalignment
float32   can_tx_auto_align_angle

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



(provide :delphi_srr_msgs/SrrStatus4 "8f5e5c4790453e1f3bcd5507dd8162bd")


