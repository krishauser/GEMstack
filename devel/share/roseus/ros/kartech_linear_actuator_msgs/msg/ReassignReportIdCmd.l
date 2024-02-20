;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ReassignReportIdCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ReassignReportIdCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REASSIGNREPORTIDCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::REASSIGNREPORTIDCMD"))

(in-package "ROS")
;;//! \htmlinclude ReassignReportIdCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ReassignReportIdCmd
  :super ros::object
  :slots (_header _confirm _user_report_id _use_user_report_id ))

(defmethod kartech_linear_actuator_msgs::ReassignReportIdCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:user_report_id __user_report_id) 0)
    ((:use_user_report_id __use_user_report_id) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _user_report_id (round __user_report_id))
   (setq _use_user_report_id __use_user_report_id)
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
  (:user_report_id
   (&optional __user_report_id)
   (if __user_report_id (setq _user_report_id __user_report_id)) _user_report_id)
  (:use_user_report_id
   (&optional (__use_user_report_id :null))
   (if (not (eq __use_user_report_id :null)) (setq _use_user_report_id __use_user_report_id)) _use_user_report_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint32 _user_report_id
    4
    ;; bool _use_user_report_id
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
     ;; uint32 _user_report_id
       (write-long _user_report_id s)
     ;; bool _use_user_report_id
       (if _use_user_report_id (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _user_report_id
     (setq _user_report_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _use_user_report_id
     (setq _use_user_report_id (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ReassignReportIdCmd :md5sum-) "fd2b8bcf6e2eaf63371bf7a3445d195c")
(setf (get kartech_linear_actuator_msgs::ReassignReportIdCmd :datatype-) "kartech_linear_actuator_msgs/ReassignReportIdCmd")
(setf (get kartech_linear_actuator_msgs::ReassignReportIdCmd :definition-)
      "std_msgs/Header header
bool confirm
uint32 user_report_id     # The new user report ID to use. Values 0xFFFEXX and 0xFF00XX are reserved. Setting this to 0xFFFFFFFF will only change the use_user_report_id flag.
bool use_user_report_id   # Whether to use the user-defined report ID (true) or the default report ID (false).

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



(provide :kartech_linear_actuator_msgs/ReassignReportIdCmd "fd2b8bcf6e2eaf63371bf7a3445d195c")


