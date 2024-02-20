;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrStatus1)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrStatus1 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRSTATUS1")
  (make-package "DELPHI_SRR_MSGS::SRRSTATUS1"))

(in-package "ROS")
;;//! \htmlinclude SrrStatus1.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_srr_msgs::SrrStatus1
  :super ros::object
  :slots (_header _can_tx_look_type _can_tx_dsp_timestamp _can_tx_yaw_rate_calc _can_tx_vehicle_speed_calc _can_tx_scan_index _can_tx_curvature ))

(defmethod delphi_srr_msgs::SrrStatus1
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_tx_look_type __can_tx_look_type) nil)
    ((:can_tx_dsp_timestamp __can_tx_dsp_timestamp) 0)
    ((:can_tx_yaw_rate_calc __can_tx_yaw_rate_calc) 0.0)
    ((:can_tx_vehicle_speed_calc __can_tx_vehicle_speed_calc) 0.0)
    ((:can_tx_scan_index __can_tx_scan_index) 0)
    ((:can_tx_curvature __can_tx_curvature) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_tx_look_type __can_tx_look_type)
   (setq _can_tx_dsp_timestamp (round __can_tx_dsp_timestamp))
   (setq _can_tx_yaw_rate_calc (float __can_tx_yaw_rate_calc))
   (setq _can_tx_vehicle_speed_calc (float __can_tx_vehicle_speed_calc))
   (setq _can_tx_scan_index (round __can_tx_scan_index))
   (setq _can_tx_curvature (float __can_tx_curvature))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_tx_look_type
   (&optional (__can_tx_look_type :null))
   (if (not (eq __can_tx_look_type :null)) (setq _can_tx_look_type __can_tx_look_type)) _can_tx_look_type)
  (:can_tx_dsp_timestamp
   (&optional __can_tx_dsp_timestamp)
   (if __can_tx_dsp_timestamp (setq _can_tx_dsp_timestamp __can_tx_dsp_timestamp)) _can_tx_dsp_timestamp)
  (:can_tx_yaw_rate_calc
   (&optional __can_tx_yaw_rate_calc)
   (if __can_tx_yaw_rate_calc (setq _can_tx_yaw_rate_calc __can_tx_yaw_rate_calc)) _can_tx_yaw_rate_calc)
  (:can_tx_vehicle_speed_calc
   (&optional __can_tx_vehicle_speed_calc)
   (if __can_tx_vehicle_speed_calc (setq _can_tx_vehicle_speed_calc __can_tx_vehicle_speed_calc)) _can_tx_vehicle_speed_calc)
  (:can_tx_scan_index
   (&optional __can_tx_scan_index)
   (if __can_tx_scan_index (setq _can_tx_scan_index __can_tx_scan_index)) _can_tx_scan_index)
  (:can_tx_curvature
   (&optional __can_tx_curvature)
   (if __can_tx_curvature (setq _can_tx_curvature __can_tx_curvature)) _can_tx_curvature)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _can_tx_look_type
    1
    ;; uint32 _can_tx_dsp_timestamp
    4
    ;; float32 _can_tx_yaw_rate_calc
    4
    ;; float32 _can_tx_vehicle_speed_calc
    4
    ;; uint16 _can_tx_scan_index
    2
    ;; float32 _can_tx_curvature
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _can_tx_look_type
       (if _can_tx_look_type (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _can_tx_dsp_timestamp
       (write-long _can_tx_dsp_timestamp s)
     ;; float32 _can_tx_yaw_rate_calc
       (sys::poke _can_tx_yaw_rate_calc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_vehicle_speed_calc
       (sys::poke _can_tx_vehicle_speed_calc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _can_tx_scan_index
       (write-word _can_tx_scan_index s)
     ;; float32 _can_tx_curvature
       (sys::poke _can_tx_curvature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _can_tx_look_type
     (setq _can_tx_look_type (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _can_tx_dsp_timestamp
     (setq _can_tx_dsp_timestamp (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _can_tx_yaw_rate_calc
     (setq _can_tx_yaw_rate_calc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_vehicle_speed_calc
     (setq _can_tx_vehicle_speed_calc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _can_tx_scan_index
     (setq _can_tx_scan_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _can_tx_curvature
     (setq _can_tx_curvature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrStatus1 :md5sum-) "585df8ad7a5b009cc9f6c14365cc686e")
(setf (get delphi_srr_msgs::SrrStatus1 :datatype-) "delphi_srr_msgs/SrrStatus1")
(setf (get delphi_srr_msgs::SrrStatus1 :definition-)
      "# Message file for srr_status1

std_msgs/Header header

bool      can_tx_look_type
uint32    can_tx_dsp_timestamp                     # ms
float32   can_tx_yaw_rate_calc                     # deg/s
float32   can_tx_vehicle_speed_calc                # m/s
uint16    can_tx_scan_index
float32   can_tx_curvature                         # 1/m

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



(provide :delphi_srr_msgs/SrrStatus1 "585df8ad7a5b009cc9f6c14365cc686e")


