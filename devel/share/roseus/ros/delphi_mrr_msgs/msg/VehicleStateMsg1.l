;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::VehicleStateMsg1)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'VehicleStateMsg1 (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::VEHICLESTATEMSG1")
  (make-package "DELPHI_MRR_MSGS::VEHICLESTATEMSG1"))

(in-package "ROS")
;;//! \htmlinclude VehicleStateMsg1.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::VehicleStateMsg1
  :super ros::object
  :slots (_header _can_fcw_sensitivity_level _can_vehicle_stationary _can_intf_minor_version _can_intf_major_version _can_brake_pedal _can_high_wheel_slip _can_turn_signal_status _can_washer_front_cmd _can_wiper_front_cmd _can_wiper_speed_info _can_reverse_gear _can_beam_shape_actual_right _can_beam_shape_actual_left _can_main_beam_indication _can_vehicle_index ))

(defmethod delphi_mrr_msgs::VehicleStateMsg1
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_fcw_sensitivity_level __can_fcw_sensitivity_level) 0)
    ((:can_vehicle_stationary __can_vehicle_stationary) nil)
    ((:can_intf_minor_version __can_intf_minor_version) 0)
    ((:can_intf_major_version __can_intf_major_version) 0)
    ((:can_brake_pedal __can_brake_pedal) 0)
    ((:can_high_wheel_slip __can_high_wheel_slip) nil)
    ((:can_turn_signal_status __can_turn_signal_status) 0)
    ((:can_washer_front_cmd __can_washer_front_cmd) nil)
    ((:can_wiper_front_cmd __can_wiper_front_cmd) nil)
    ((:can_wiper_speed_info __can_wiper_speed_info) 0)
    ((:can_reverse_gear __can_reverse_gear) nil)
    ((:can_beam_shape_actual_right __can_beam_shape_actual_right) 0)
    ((:can_beam_shape_actual_left __can_beam_shape_actual_left) 0)
    ((:can_main_beam_indication __can_main_beam_indication) nil)
    ((:can_vehicle_index __can_vehicle_index) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_fcw_sensitivity_level (round __can_fcw_sensitivity_level))
   (setq _can_vehicle_stationary __can_vehicle_stationary)
   (setq _can_intf_minor_version (round __can_intf_minor_version))
   (setq _can_intf_major_version (round __can_intf_major_version))
   (setq _can_brake_pedal (round __can_brake_pedal))
   (setq _can_high_wheel_slip __can_high_wheel_slip)
   (setq _can_turn_signal_status (round __can_turn_signal_status))
   (setq _can_washer_front_cmd __can_washer_front_cmd)
   (setq _can_wiper_front_cmd __can_wiper_front_cmd)
   (setq _can_wiper_speed_info (round __can_wiper_speed_info))
   (setq _can_reverse_gear __can_reverse_gear)
   (setq _can_beam_shape_actual_right (round __can_beam_shape_actual_right))
   (setq _can_beam_shape_actual_left (round __can_beam_shape_actual_left))
   (setq _can_main_beam_indication __can_main_beam_indication)
   (setq _can_vehicle_index (round __can_vehicle_index))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_fcw_sensitivity_level
   (&optional __can_fcw_sensitivity_level)
   (if __can_fcw_sensitivity_level (setq _can_fcw_sensitivity_level __can_fcw_sensitivity_level)) _can_fcw_sensitivity_level)
  (:can_vehicle_stationary
   (&optional (__can_vehicle_stationary :null))
   (if (not (eq __can_vehicle_stationary :null)) (setq _can_vehicle_stationary __can_vehicle_stationary)) _can_vehicle_stationary)
  (:can_intf_minor_version
   (&optional __can_intf_minor_version)
   (if __can_intf_minor_version (setq _can_intf_minor_version __can_intf_minor_version)) _can_intf_minor_version)
  (:can_intf_major_version
   (&optional __can_intf_major_version)
   (if __can_intf_major_version (setq _can_intf_major_version __can_intf_major_version)) _can_intf_major_version)
  (:can_brake_pedal
   (&optional __can_brake_pedal)
   (if __can_brake_pedal (setq _can_brake_pedal __can_brake_pedal)) _can_brake_pedal)
  (:can_high_wheel_slip
   (&optional (__can_high_wheel_slip :null))
   (if (not (eq __can_high_wheel_slip :null)) (setq _can_high_wheel_slip __can_high_wheel_slip)) _can_high_wheel_slip)
  (:can_turn_signal_status
   (&optional __can_turn_signal_status)
   (if __can_turn_signal_status (setq _can_turn_signal_status __can_turn_signal_status)) _can_turn_signal_status)
  (:can_washer_front_cmd
   (&optional (__can_washer_front_cmd :null))
   (if (not (eq __can_washer_front_cmd :null)) (setq _can_washer_front_cmd __can_washer_front_cmd)) _can_washer_front_cmd)
  (:can_wiper_front_cmd
   (&optional (__can_wiper_front_cmd :null))
   (if (not (eq __can_wiper_front_cmd :null)) (setq _can_wiper_front_cmd __can_wiper_front_cmd)) _can_wiper_front_cmd)
  (:can_wiper_speed_info
   (&optional __can_wiper_speed_info)
   (if __can_wiper_speed_info (setq _can_wiper_speed_info __can_wiper_speed_info)) _can_wiper_speed_info)
  (:can_reverse_gear
   (&optional (__can_reverse_gear :null))
   (if (not (eq __can_reverse_gear :null)) (setq _can_reverse_gear __can_reverse_gear)) _can_reverse_gear)
  (:can_beam_shape_actual_right
   (&optional __can_beam_shape_actual_right)
   (if __can_beam_shape_actual_right (setq _can_beam_shape_actual_right __can_beam_shape_actual_right)) _can_beam_shape_actual_right)
  (:can_beam_shape_actual_left
   (&optional __can_beam_shape_actual_left)
   (if __can_beam_shape_actual_left (setq _can_beam_shape_actual_left __can_beam_shape_actual_left)) _can_beam_shape_actual_left)
  (:can_main_beam_indication
   (&optional (__can_main_beam_indication :null))
   (if (not (eq __can_main_beam_indication :null)) (setq _can_main_beam_indication __can_main_beam_indication)) _can_main_beam_indication)
  (:can_vehicle_index
   (&optional __can_vehicle_index)
   (if __can_vehicle_index (setq _can_vehicle_index __can_vehicle_index)) _can_vehicle_index)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_fcw_sensitivity_level
    1
    ;; bool _can_vehicle_stationary
    1
    ;; uint8 _can_intf_minor_version
    1
    ;; uint8 _can_intf_major_version
    1
    ;; uint8 _can_brake_pedal
    1
    ;; bool _can_high_wheel_slip
    1
    ;; uint8 _can_turn_signal_status
    1
    ;; bool _can_washer_front_cmd
    1
    ;; bool _can_wiper_front_cmd
    1
    ;; uint8 _can_wiper_speed_info
    1
    ;; bool _can_reverse_gear
    1
    ;; uint8 _can_beam_shape_actual_right
    1
    ;; uint8 _can_beam_shape_actual_left
    1
    ;; bool _can_main_beam_indication
    1
    ;; uint16 _can_vehicle_index
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_fcw_sensitivity_level
       (write-byte _can_fcw_sensitivity_level s)
     ;; bool _can_vehicle_stationary
       (if _can_vehicle_stationary (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _can_intf_minor_version
       (write-byte _can_intf_minor_version s)
     ;; uint8 _can_intf_major_version
       (write-byte _can_intf_major_version s)
     ;; uint8 _can_brake_pedal
       (write-byte _can_brake_pedal s)
     ;; bool _can_high_wheel_slip
       (if _can_high_wheel_slip (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _can_turn_signal_status
       (write-byte _can_turn_signal_status s)
     ;; bool _can_washer_front_cmd
       (if _can_washer_front_cmd (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_wiper_front_cmd
       (if _can_wiper_front_cmd (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _can_wiper_speed_info
       (write-byte _can_wiper_speed_info s)
     ;; bool _can_reverse_gear
       (if _can_reverse_gear (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _can_beam_shape_actual_right
       (write-byte _can_beam_shape_actual_right s)
     ;; uint8 _can_beam_shape_actual_left
       (write-byte _can_beam_shape_actual_left s)
     ;; bool _can_main_beam_indication
       (if _can_main_beam_indication (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _can_vehicle_index
       (write-word _can_vehicle_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_fcw_sensitivity_level
     (setq _can_fcw_sensitivity_level (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_vehicle_stationary
     (setq _can_vehicle_stationary (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _can_intf_minor_version
     (setq _can_intf_minor_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_intf_major_version
     (setq _can_intf_major_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_brake_pedal
     (setq _can_brake_pedal (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_high_wheel_slip
     (setq _can_high_wheel_slip (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _can_turn_signal_status
     (setq _can_turn_signal_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_washer_front_cmd
     (setq _can_washer_front_cmd (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_wiper_front_cmd
     (setq _can_wiper_front_cmd (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _can_wiper_speed_info
     (setq _can_wiper_speed_info (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_reverse_gear
     (setq _can_reverse_gear (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _can_beam_shape_actual_right
     (setq _can_beam_shape_actual_right (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_beam_shape_actual_left
     (setq _can_beam_shape_actual_left (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _can_main_beam_indication
     (setq _can_main_beam_indication (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _can_vehicle_index
     (setq _can_vehicle_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::VehicleStateMsg1 :md5sum-) "9bd8d57bd02218fdeffdb6496e73cbe0")
(setf (get delphi_mrr_msgs::VehicleStateMsg1 :datatype-) "delphi_mrr_msgs/VehicleStateMsg1")
(setf (get delphi_mrr_msgs::VehicleStateMsg1 :definition-)
      "std_msgs/Header header

uint8  can_fcw_sensitivity_level
bool   can_vehicle_stationary
uint8  can_intf_minor_version
uint8  can_intf_major_version
uint8  can_brake_pedal
bool   can_high_wheel_slip
uint8  can_turn_signal_status
bool   can_washer_front_cmd
bool   can_wiper_front_cmd
uint8  can_wiper_speed_info
bool   can_reverse_gear
uint8  can_beam_shape_actual_right
uint8  can_beam_shape_actual_left
bool   can_main_beam_indication
uint16 can_vehicle_index

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



(provide :delphi_mrr_msgs/VehicleStateMsg1 "9bd8d57bd02218fdeffdb6496e73cbe0")


