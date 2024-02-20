;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::VehicleStateMsg3)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'VehicleStateMsg3 (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::VEHICLESTATEMSG3")
  (make-package "DELPHI_MRR_MSGS::VEHICLESTATEMSG3"))

(in-package "ROS")
;;//! \htmlinclude VehicleStateMsg3.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::VehicleStateMsg3
  :super ros::object
  :slots (_header _yaw_rate_reference_valid _yaw_rate_reference _can_veh_long_accel_qf _can_veh_long_accel ))

(defmethod delphi_mrr_msgs::VehicleStateMsg3
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:yaw_rate_reference_valid __yaw_rate_reference_valid) nil)
    ((:yaw_rate_reference __yaw_rate_reference) 0.0)
    ((:can_veh_long_accel_qf __can_veh_long_accel_qf) 0)
    ((:can_veh_long_accel __can_veh_long_accel) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _yaw_rate_reference_valid __yaw_rate_reference_valid)
   (setq _yaw_rate_reference (float __yaw_rate_reference))
   (setq _can_veh_long_accel_qf (round __can_veh_long_accel_qf))
   (setq _can_veh_long_accel (float __can_veh_long_accel))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:yaw_rate_reference_valid
   (&optional (__yaw_rate_reference_valid :null))
   (if (not (eq __yaw_rate_reference_valid :null)) (setq _yaw_rate_reference_valid __yaw_rate_reference_valid)) _yaw_rate_reference_valid)
  (:yaw_rate_reference
   (&optional __yaw_rate_reference)
   (if __yaw_rate_reference (setq _yaw_rate_reference __yaw_rate_reference)) _yaw_rate_reference)
  (:can_veh_long_accel_qf
   (&optional __can_veh_long_accel_qf)
   (if __can_veh_long_accel_qf (setq _can_veh_long_accel_qf __can_veh_long_accel_qf)) _can_veh_long_accel_qf)
  (:can_veh_long_accel
   (&optional __can_veh_long_accel)
   (if __can_veh_long_accel (setq _can_veh_long_accel __can_veh_long_accel)) _can_veh_long_accel)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _yaw_rate_reference_valid
    1
    ;; float32 _yaw_rate_reference
    4
    ;; uint8 _can_veh_long_accel_qf
    1
    ;; float32 _can_veh_long_accel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _yaw_rate_reference_valid
       (if _yaw_rate_reference_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _yaw_rate_reference
       (sys::poke _yaw_rate_reference (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _can_veh_long_accel_qf
       (write-byte _can_veh_long_accel_qf s)
     ;; float32 _can_veh_long_accel
       (sys::poke _can_veh_long_accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _yaw_rate_reference_valid
     (setq _yaw_rate_reference_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _yaw_rate_reference
     (setq _yaw_rate_reference (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _can_veh_long_accel_qf
     (setq _can_veh_long_accel_qf (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _can_veh_long_accel
     (setq _can_veh_long_accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::VehicleStateMsg3 :md5sum-) "ce47a927102040e8c016cbb9166e5057")
(setf (get delphi_mrr_msgs::VehicleStateMsg3 :datatype-) "delphi_mrr_msgs/VehicleStateMsg3")
(setf (get delphi_mrr_msgs::VehicleStateMsg3 :definition-)
      "std_msgs/Header header

bool    yaw_rate_reference_valid
float32 yaw_rate_reference
uint8   can_veh_long_accel_qf
float32 can_veh_long_accel

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



(provide :delphi_mrr_msgs/VehicleStateMsg3 "ce47a927102040e8c016cbb9166e5057")


