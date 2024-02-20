;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ReassignCommandIdCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ReassignCommandIdCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REASSIGNCOMMANDIDCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::REASSIGNCOMMANDIDCMD"))

(in-package "ROS")
;;//! \htmlinclude ReassignCommandIdCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ReassignCommandIdCmd
  :super ros::object
  :slots (_header _confirm _command_id_index _user_command_id _disable_default_command_id ))

(defmethod kartech_linear_actuator_msgs::ReassignCommandIdCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:command_id_index __command_id_index) 0)
    ((:user_command_id __user_command_id) 0)
    ((:disable_default_command_id __disable_default_command_id) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _command_id_index (round __command_id_index))
   (setq _user_command_id (round __user_command_id))
   (setq _disable_default_command_id __disable_default_command_id)
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
  (:command_id_index
   (&optional __command_id_index)
   (if __command_id_index (setq _command_id_index __command_id_index)) _command_id_index)
  (:user_command_id
   (&optional __user_command_id)
   (if __user_command_id (setq _user_command_id __user_command_id)) _user_command_id)
  (:disable_default_command_id
   (&optional (__disable_default_command_id :null))
   (if (not (eq __disable_default_command_id :null)) (setq _disable_default_command_id __disable_default_command_id)) _disable_default_command_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint8 _command_id_index
    1
    ;; uint32 _user_command_id
    4
    ;; bool _disable_default_command_id
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
     ;; uint8 _command_id_index
       (write-byte _command_id_index s)
     ;; uint32 _user_command_id
       (write-long _user_command_id s)
     ;; bool _disable_default_command_id
       (if _disable_default_command_id (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _command_id_index
     (setq _command_id_index (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _user_command_id
     (setq _user_command_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _disable_default_command_id
     (setq _disable_default_command_id (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ReassignCommandIdCmd :md5sum-) "41d43df68f42f68725a7567326abaa4d")
(setf (get kartech_linear_actuator_msgs::ReassignCommandIdCmd :datatype-) "kartech_linear_actuator_msgs/ReassignCommandIdCmd")
(setf (get kartech_linear_actuator_msgs::ReassignCommandIdCmd :definition-)
      "std_msgs/Header header
bool confirm
uint8 command_id_index            # The index of the user-defined command ID (1-4) to change.
uint32 user_command_id            # The new user-defined command ID to set. 0xFFFEXX and 0xFF00XX are reserved.
                                  # Setting this to 0xFFFFFFFF will change the disable_default_command_id flag without affecting any others.
bool disable_default_command_id   # Disables (true) or enables (false) the default command ID.

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



(provide :kartech_linear_actuator_msgs/ReassignCommandIdCmd "41d43df68f42f68725a7567326abaa4d")


