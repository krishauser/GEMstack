;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::PositionCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'PositionCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::POSITIONCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::POSITIONCMD"))

(in-package "ROS")
;;//! \htmlinclude PositionCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::PositionCmd
  :super ros::object
  :slots (_header _confirm _auto_reply _position _clutch_enable _motor_enable ))

(defmethod kartech_linear_actuator_msgs::PositionCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:auto_reply __auto_reply) nil)
    ((:position __position) 0.0)
    ((:clutch_enable __clutch_enable) nil)
    ((:motor_enable __motor_enable) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _auto_reply __auto_reply)
   (setq _position (float __position))
   (setq _clutch_enable __clutch_enable)
   (setq _motor_enable __motor_enable)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:confirm
   (&optional (__confirm :null))
   (if (not (eq __confirm :null)) (setq _confirm __confirm)) _confirm)
  (:auto_reply
   (&optional (__auto_reply :null))
   (if (not (eq __auto_reply :null)) (setq _auto_reply __auto_reply)) _auto_reply)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:clutch_enable
   (&optional (__clutch_enable :null))
   (if (not (eq __clutch_enable :null)) (setq _clutch_enable __clutch_enable)) _clutch_enable)
  (:motor_enable
   (&optional (__motor_enable :null))
   (if (not (eq __motor_enable :null)) (setq _motor_enable __motor_enable)) _motor_enable)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; bool _auto_reply
    1
    ;; float64 _position
    8
    ;; bool _clutch_enable
    1
    ;; bool _motor_enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;; bool _auto_reply
       (if _auto_reply (write-byte -1 s) (write-byte 0 s))
     ;; float64 _position
       (sys::poke _position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _clutch_enable
       (if _clutch_enable (write-byte -1 s) (write-byte 0 s))
     ;; bool _motor_enable
       (if _motor_enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _auto_reply
     (setq _auto_reply (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _position
     (setq _position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _clutch_enable
     (setq _clutch_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _motor_enable
     (setq _motor_enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::PositionCmd :md5sum-) "ac9ab77927289195f06ee9b42fabeac2")
(setf (get kartech_linear_actuator_msgs::PositionCmd :datatype-) "kartech_linear_actuator_msgs/PositionCmd")
(setf (get kartech_linear_actuator_msgs::PositionCmd :definition-)
      "std_msgs/Header header
bool confirm
bool auto_reply
float64 position    # Position in 0.001\" increments.
bool clutch_enable  # Disables (false) or enables (true) the built-in clutch after the position has been reached.
bool motor_enable   # Disables (false) or enables (true) the motor after the position has been reached.

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



(provide :kartech_linear_actuator_msgs/PositionCmd "ac9ab77927289195f06ee9b42fabeac2")


