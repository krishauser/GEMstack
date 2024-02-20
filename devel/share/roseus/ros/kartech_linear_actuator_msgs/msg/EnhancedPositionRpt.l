;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::EnhancedPositionRpt)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'EnhancedPositionRpt (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::ENHANCEDPOSITIONRPT")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::ENHANCEDPOSITIONRPT"))

(in-package "ROS")
;;//! \htmlinclude EnhancedPositionRpt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::EnhancedPositionRpt
  :super ros::object
  :slots (_header _shaft_extension _motor_overload_error _clutch_overload_error _motor_open_load_error _clutch_open_load_error _position_reach_error _hardware_warning_1_error _hardware_warning_2_error _motor_current ))

(defmethod kartech_linear_actuator_msgs::EnhancedPositionRpt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:shaft_extension __shaft_extension) 0.0)
    ((:motor_overload_error __motor_overload_error) nil)
    ((:clutch_overload_error __clutch_overload_error) nil)
    ((:motor_open_load_error __motor_open_load_error) nil)
    ((:clutch_open_load_error __clutch_open_load_error) nil)
    ((:position_reach_error __position_reach_error) nil)
    ((:hardware_warning_1_error __hardware_warning_1_error) nil)
    ((:hardware_warning_2_error __hardware_warning_2_error) nil)
    ((:motor_current __motor_current) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _shaft_extension (float __shaft_extension))
   (setq _motor_overload_error __motor_overload_error)
   (setq _clutch_overload_error __clutch_overload_error)
   (setq _motor_open_load_error __motor_open_load_error)
   (setq _clutch_open_load_error __clutch_open_load_error)
   (setq _position_reach_error __position_reach_error)
   (setq _hardware_warning_1_error __hardware_warning_1_error)
   (setq _hardware_warning_2_error __hardware_warning_2_error)
   (setq _motor_current (round __motor_current))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:shaft_extension
   (&optional __shaft_extension)
   (if __shaft_extension (setq _shaft_extension __shaft_extension)) _shaft_extension)
  (:motor_overload_error
   (&optional (__motor_overload_error :null))
   (if (not (eq __motor_overload_error :null)) (setq _motor_overload_error __motor_overload_error)) _motor_overload_error)
  (:clutch_overload_error
   (&optional (__clutch_overload_error :null))
   (if (not (eq __clutch_overload_error :null)) (setq _clutch_overload_error __clutch_overload_error)) _clutch_overload_error)
  (:motor_open_load_error
   (&optional (__motor_open_load_error :null))
   (if (not (eq __motor_open_load_error :null)) (setq _motor_open_load_error __motor_open_load_error)) _motor_open_load_error)
  (:clutch_open_load_error
   (&optional (__clutch_open_load_error :null))
   (if (not (eq __clutch_open_load_error :null)) (setq _clutch_open_load_error __clutch_open_load_error)) _clutch_open_load_error)
  (:position_reach_error
   (&optional (__position_reach_error :null))
   (if (not (eq __position_reach_error :null)) (setq _position_reach_error __position_reach_error)) _position_reach_error)
  (:hardware_warning_1_error
   (&optional (__hardware_warning_1_error :null))
   (if (not (eq __hardware_warning_1_error :null)) (setq _hardware_warning_1_error __hardware_warning_1_error)) _hardware_warning_1_error)
  (:hardware_warning_2_error
   (&optional (__hardware_warning_2_error :null))
   (if (not (eq __hardware_warning_2_error :null)) (setq _hardware_warning_2_error __hardware_warning_2_error)) _hardware_warning_2_error)
  (:motor_current
   (&optional __motor_current)
   (if __motor_current (setq _motor_current __motor_current)) _motor_current)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _shaft_extension
    8
    ;; bool _motor_overload_error
    1
    ;; bool _clutch_overload_error
    1
    ;; bool _motor_open_load_error
    1
    ;; bool _clutch_open_load_error
    1
    ;; bool _position_reach_error
    1
    ;; bool _hardware_warning_1_error
    1
    ;; bool _hardware_warning_2_error
    1
    ;; uint16 _motor_current
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _shaft_extension
       (sys::poke _shaft_extension (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _motor_overload_error
       (if _motor_overload_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _clutch_overload_error
       (if _clutch_overload_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _motor_open_load_error
       (if _motor_open_load_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _clutch_open_load_error
       (if _clutch_open_load_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _position_reach_error
       (if _position_reach_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _hardware_warning_1_error
       (if _hardware_warning_1_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _hardware_warning_2_error
       (if _hardware_warning_2_error (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _motor_current
       (write-word _motor_current s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _shaft_extension
     (setq _shaft_extension (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _motor_overload_error
     (setq _motor_overload_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clutch_overload_error
     (setq _clutch_overload_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _motor_open_load_error
     (setq _motor_open_load_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _clutch_open_load_error
     (setq _clutch_open_load_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _position_reach_error
     (setq _position_reach_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hardware_warning_1_error
     (setq _hardware_warning_1_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hardware_warning_2_error
     (setq _hardware_warning_2_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _motor_current
     (setq _motor_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::EnhancedPositionRpt :md5sum-) "b5d14804230789155d91f65364c956fd")
(setf (get kartech_linear_actuator_msgs::EnhancedPositionRpt :datatype-) "kartech_linear_actuator_msgs/EnhancedPositionRpt")
(setf (get kartech_linear_actuator_msgs::EnhancedPositionRpt :definition-)
      "std_msgs/Header header
float64 shaft_extension     # The current shaft position in 0.001\" increments.
bool motor_overload_error
bool clutch_overload_error
bool motor_open_load_error
bool clutch_open_load_error
bool position_reach_error
bool hardware_warning_1_error
bool hardware_warning_2_error
uint16 motor_current        # The current motor current in mA.

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



(provide :kartech_linear_actuator_msgs/EnhancedPositionRpt "b5d14804230789155d91f65364c956fd")


