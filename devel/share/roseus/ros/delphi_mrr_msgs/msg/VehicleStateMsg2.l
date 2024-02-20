;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::VehicleStateMsg2)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'VehicleStateMsg2 (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::VEHICLESTATEMSG2")
  (make-package "DELPHI_MRR_MSGS::VEHICLESTATEMSG2"))

(in-package "ROS")
;;//! \htmlinclude VehicleStateMsg2.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::VehicleStateMsg2
  :super ros::object
  :slots (_header _fsm_yaw_rate_valid _fsm_yaw_rate _can_vehicle_index_4fa _fsm_vehicle_velocity _can_steering_whl_angle_qf _fsm_vehicle_velocity_valid _can_steering_whl_angle ))

(defmethod delphi_mrr_msgs::VehicleStateMsg2
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:fsm_yaw_rate_valid __fsm_yaw_rate_valid) nil)
    ((:fsm_yaw_rate __fsm_yaw_rate) 0.0)
    ((:can_vehicle_index_4fa __can_vehicle_index_4fa) 0)
    ((:fsm_vehicle_velocity __fsm_vehicle_velocity) 0.0)
    ((:can_steering_whl_angle_qf __can_steering_whl_angle_qf) nil)
    ((:fsm_vehicle_velocity_valid __fsm_vehicle_velocity_valid) nil)
    ((:can_steering_whl_angle __can_steering_whl_angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _fsm_yaw_rate_valid __fsm_yaw_rate_valid)
   (setq _fsm_yaw_rate (float __fsm_yaw_rate))
   (setq _can_vehicle_index_4fa (round __can_vehicle_index_4fa))
   (setq _fsm_vehicle_velocity (float __fsm_vehicle_velocity))
   (setq _can_steering_whl_angle_qf __can_steering_whl_angle_qf)
   (setq _fsm_vehicle_velocity_valid __fsm_vehicle_velocity_valid)
   (setq _can_steering_whl_angle (float __can_steering_whl_angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:fsm_yaw_rate_valid
   (&optional (__fsm_yaw_rate_valid :null))
   (if (not (eq __fsm_yaw_rate_valid :null)) (setq _fsm_yaw_rate_valid __fsm_yaw_rate_valid)) _fsm_yaw_rate_valid)
  (:fsm_yaw_rate
   (&optional __fsm_yaw_rate)
   (if __fsm_yaw_rate (setq _fsm_yaw_rate __fsm_yaw_rate)) _fsm_yaw_rate)
  (:can_vehicle_index_4fa
   (&optional __can_vehicle_index_4fa)
   (if __can_vehicle_index_4fa (setq _can_vehicle_index_4fa __can_vehicle_index_4fa)) _can_vehicle_index_4fa)
  (:fsm_vehicle_velocity
   (&optional __fsm_vehicle_velocity)
   (if __fsm_vehicle_velocity (setq _fsm_vehicle_velocity __fsm_vehicle_velocity)) _fsm_vehicle_velocity)
  (:can_steering_whl_angle_qf
   (&optional (__can_steering_whl_angle_qf :null))
   (if (not (eq __can_steering_whl_angle_qf :null)) (setq _can_steering_whl_angle_qf __can_steering_whl_angle_qf)) _can_steering_whl_angle_qf)
  (:fsm_vehicle_velocity_valid
   (&optional (__fsm_vehicle_velocity_valid :null))
   (if (not (eq __fsm_vehicle_velocity_valid :null)) (setq _fsm_vehicle_velocity_valid __fsm_vehicle_velocity_valid)) _fsm_vehicle_velocity_valid)
  (:can_steering_whl_angle
   (&optional __can_steering_whl_angle)
   (if __can_steering_whl_angle (setq _can_steering_whl_angle __can_steering_whl_angle)) _can_steering_whl_angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _fsm_yaw_rate_valid
    1
    ;; float32 _fsm_yaw_rate
    4
    ;; uint16 _can_vehicle_index_4fa
    2
    ;; float32 _fsm_vehicle_velocity
    4
    ;; bool _can_steering_whl_angle_qf
    1
    ;; bool _fsm_vehicle_velocity_valid
    1
    ;; float32 _can_steering_whl_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _fsm_yaw_rate_valid
       (if _fsm_yaw_rate_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _fsm_yaw_rate
       (sys::poke _fsm_yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _can_vehicle_index_4fa
       (write-word _can_vehicle_index_4fa s)
     ;; float32 _fsm_vehicle_velocity
       (sys::poke _fsm_vehicle_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _can_steering_whl_angle_qf
       (if _can_steering_whl_angle_qf (write-byte -1 s) (write-byte 0 s))
     ;; bool _fsm_vehicle_velocity_valid
       (if _fsm_vehicle_velocity_valid (write-byte -1 s) (write-byte 0 s))
     ;; float32 _can_steering_whl_angle
       (sys::poke _can_steering_whl_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _fsm_yaw_rate_valid
     (setq _fsm_yaw_rate_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _fsm_yaw_rate
     (setq _fsm_yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _can_vehicle_index_4fa
     (setq _can_vehicle_index_4fa (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _fsm_vehicle_velocity
     (setq _fsm_vehicle_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _can_steering_whl_angle_qf
     (setq _can_steering_whl_angle_qf (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fsm_vehicle_velocity_valid
     (setq _fsm_vehicle_velocity_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _can_steering_whl_angle
     (setq _can_steering_whl_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::VehicleStateMsg2 :md5sum-) "54463690e9fc3e6c8708c99c08ba6c62")
(setf (get delphi_mrr_msgs::VehicleStateMsg2 :datatype-) "delphi_mrr_msgs/VehicleStateMsg2")
(setf (get delphi_mrr_msgs::VehicleStateMsg2 :definition-)
      "std_msgs/Header header

bool    fsm_yaw_rate_valid
float32 fsm_yaw_rate
uint16  can_vehicle_index_4fa
float32 fsm_vehicle_velocity
bool    can_steering_whl_angle_qf
bool    fsm_vehicle_velocity_valid
float32 can_steering_whl_angle

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



(provide :delphi_mrr_msgs/VehicleStateMsg2 "54463690e9fc3e6c8708c99c08ba6c62")


