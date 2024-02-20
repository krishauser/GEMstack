;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ConfigureOutputsKpKiCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ConfigureOutputsKpKiCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::CONFIGUREOUTPUTSKPKICMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::CONFIGUREOUTPUTSKPKICMD"))

(in-package "ROS")
;;//! \htmlinclude ConfigureOutputsKpKiCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ConfigureOutputsKpKiCmd
  :super ros::object
  :slots (_header _confirm _kp _ki ))

(defmethod kartech_linear_actuator_msgs::ConfigureOutputsKpKiCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:kp __kp) 0)
    ((:ki __ki) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _kp (round __kp))
   (setq _ki (round __ki))
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
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:ki
   (&optional __ki)
   (if __ki (setq _ki __ki)) _ki)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint16 _kp
    2
    ;; uint16 _ki
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _kp
       (write-word _kp s)
     ;; uint16 _ki
       (write-word _ki s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _kp
     (setq _kp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _ki
     (setq _ki (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ConfigureOutputsKpKiCmd :md5sum-) "2776941ac111d4c1261066496047ed8d")
(setf (get kartech_linear_actuator_msgs::ConfigureOutputsKpKiCmd :datatype-) "kartech_linear_actuator_msgs/ConfigureOutputsKpKiCmd")
(setf (get kartech_linear_actuator_msgs::ConfigureOutputsKpKiCmd :definition-)
      "std_msgs/Header header
bool confirm
uint16 kp  # The proportial term of the closed-loop control. Default is 1000.
uint16 ki  # The integral term of the closed-loop control. Default is 1000.

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



(provide :kartech_linear_actuator_msgs/ConfigureOutputsKpKiCmd "2776941ac111d4c1261066496047ed8d")


