;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::AutoZeroCalCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'AutoZeroCalCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::AUTOZEROCALCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::AUTOZEROCALCMD"))

(in-package "ROS")
;;//! \htmlinclude AutoZeroCalCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::AutoZeroCalCmd
  :super ros::object
  :slots (_header _confirm _auto_reply ))

(defmethod kartech_linear_actuator_msgs::AutoZeroCalCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:auto_reply __auto_reply) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _auto_reply __auto_reply)
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
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; bool _auto_reply
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
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::AutoZeroCalCmd :md5sum-) "a99ec518a235924191aab43dcf6c1e53")
(setf (get kartech_linear_actuator_msgs::AutoZeroCalCmd :datatype-) "kartech_linear_actuator_msgs/AutoZeroCalCmd")
(setf (get kartech_linear_actuator_msgs::AutoZeroCalCmd :definition-)
      "std_msgs/Header header
bool confirm
bool auto_reply

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



(provide :kartech_linear_actuator_msgs/AutoZeroCalCmd "a99ec518a235924191aab43dcf6c1e53")


