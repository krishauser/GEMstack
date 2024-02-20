;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::PriorityConfigCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'PriorityConfigCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::PRIORITYCONFIGCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::PRIORITYCONFIGCMD"))

(in-package "ROS")
;;//! \htmlinclude PriorityConfigCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::PriorityConfigCmd
  :super ros::object
  :slots (_header _confirm _handshake_priority _auto_reply_priority _scheduled_priority _polled_priority ))

(defmethod kartech_linear_actuator_msgs::PriorityConfigCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:handshake_priority __handshake_priority) 0)
    ((:auto_reply_priority __auto_reply_priority) 0)
    ((:scheduled_priority __scheduled_priority) 0)
    ((:polled_priority __polled_priority) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _handshake_priority (round __handshake_priority))
   (setq _auto_reply_priority (round __auto_reply_priority))
   (setq _scheduled_priority (round __scheduled_priority))
   (setq _polled_priority (round __polled_priority))
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
  (:handshake_priority
   (&optional __handshake_priority)
   (if __handshake_priority (setq _handshake_priority __handshake_priority)) _handshake_priority)
  (:auto_reply_priority
   (&optional __auto_reply_priority)
   (if __auto_reply_priority (setq _auto_reply_priority __auto_reply_priority)) _auto_reply_priority)
  (:scheduled_priority
   (&optional __scheduled_priority)
   (if __scheduled_priority (setq _scheduled_priority __scheduled_priority)) _scheduled_priority)
  (:polled_priority
   (&optional __polled_priority)
   (if __polled_priority (setq _polled_priority __polled_priority)) _polled_priority)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint8 _handshake_priority
    1
    ;; uint8 _auto_reply_priority
    1
    ;; uint8 _scheduled_priority
    1
    ;; uint8 _polled_priority
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
     ;; uint8 _handshake_priority
       (write-byte _handshake_priority s)
     ;; uint8 _auto_reply_priority
       (write-byte _auto_reply_priority s)
     ;; uint8 _scheduled_priority
       (write-byte _scheduled_priority s)
     ;; uint8 _polled_priority
       (write-byte _polled_priority s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _handshake_priority
     (setq _handshake_priority (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _auto_reply_priority
     (setq _auto_reply_priority (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _scheduled_priority
     (setq _scheduled_priority (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _polled_priority
     (setq _polled_priority (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::PriorityConfigCmd :md5sum-) "04b16f80c8b9d73ef8343b9ba34c9b78")
(setf (get kartech_linear_actuator_msgs::PriorityConfigCmd :datatype-) "kartech_linear_actuator_msgs/PriorityConfigCmd")
(setf (get kartech_linear_actuator_msgs::PriorityConfigCmd :definition-)
      "std_msgs/Header header
bool confirm

# Priority values for different types of reports. Lower value = higher priority.
uint8 handshake_priority
uint8 auto_reply_priority
uint8 scheduled_priority
uint8 polled_priority

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



(provide :kartech_linear_actuator_msgs/PriorityConfigCmd "04b16f80c8b9d73ef8343b9ba34c9b78")


