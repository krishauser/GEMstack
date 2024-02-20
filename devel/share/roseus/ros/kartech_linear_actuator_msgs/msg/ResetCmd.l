;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ResetCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ResetCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))

(in-package "ROS")
;;//! \htmlinclude ResetCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*RESET_OUTPUTS*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_OUTPUTS* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_OUTPUTS* 0)
(intern "*RESET_USER_DEFINED_IDS*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_USER_DEFINED_IDS* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_USER_DEFINED_IDS* 1)
(intern "*RESET_REPORT_RATES*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_REPORT_RATES* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_REPORT_RATES* 2)
(intern "*RESET_HARDWARE_CONFIGURATIONS*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_HARDWARE_CONFIGURATIONS* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_HARDWARE_CONFIGURATIONS* 3)
(intern "*RESET_USER_CONFIGURATIONS*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_USER_CONFIGURATIONS* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_USER_CONFIGURATIONS* 4)
(intern "*RESET_EVERYTHING*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_EVERYTHING* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_EVERYTHING* 5)
(intern "*RESET_NONE*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(shadow '*RESET_NONE* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::RESETCMD"))
(defconstant kartech_linear_actuator_msgs::ResetCmd::*RESET_NONE* 6)

(defun kartech_linear_actuator_msgs::ResetCmd-to-symbol (const)
  (cond
        ((= const 0) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_OUTPUTS*)
        ((= const 1) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_USER_DEFINED_IDS*)
        ((= const 2) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_REPORT_RATES*)
        ((= const 3) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_HARDWARE_CONFIGURATIONS*)
        ((= const 4) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_USER_CONFIGURATIONS*)
        ((= const 5) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_EVERYTHING*)
        ((= const 6) 'kartech_linear_actuator_msgs::ResetCmd::*RESET_NONE*)
        (t nil)))

(defclass kartech_linear_actuator_msgs::ResetCmd
  :super ros::object
  :slots (_header _confirm _reset_type _reset_user_rpt_id _reset_user_cmd_id_1 _reset_user_cmd_id_2 _reset_user_cmd_id_3 _reset_user_cmd_id_4 _disable_user_rpt_id _reenable_default_cmd_id ))

(defmethod kartech_linear_actuator_msgs::ResetCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:reset_type __reset_type) 0)
    ((:reset_user_rpt_id __reset_user_rpt_id) nil)
    ((:reset_user_cmd_id_1 __reset_user_cmd_id_1) nil)
    ((:reset_user_cmd_id_2 __reset_user_cmd_id_2) nil)
    ((:reset_user_cmd_id_3 __reset_user_cmd_id_3) nil)
    ((:reset_user_cmd_id_4 __reset_user_cmd_id_4) nil)
    ((:disable_user_rpt_id __disable_user_rpt_id) nil)
    ((:reenable_default_cmd_id __reenable_default_cmd_id) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _reset_type (round __reset_type))
   (setq _reset_user_rpt_id __reset_user_rpt_id)
   (setq _reset_user_cmd_id_1 __reset_user_cmd_id_1)
   (setq _reset_user_cmd_id_2 __reset_user_cmd_id_2)
   (setq _reset_user_cmd_id_3 __reset_user_cmd_id_3)
   (setq _reset_user_cmd_id_4 __reset_user_cmd_id_4)
   (setq _disable_user_rpt_id __disable_user_rpt_id)
   (setq _reenable_default_cmd_id __reenable_default_cmd_id)
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
  (:reset_type
   (&optional __reset_type)
   (if __reset_type (setq _reset_type __reset_type)) _reset_type)
  (:reset_user_rpt_id
   (&optional (__reset_user_rpt_id :null))
   (if (not (eq __reset_user_rpt_id :null)) (setq _reset_user_rpt_id __reset_user_rpt_id)) _reset_user_rpt_id)
  (:reset_user_cmd_id_1
   (&optional (__reset_user_cmd_id_1 :null))
   (if (not (eq __reset_user_cmd_id_1 :null)) (setq _reset_user_cmd_id_1 __reset_user_cmd_id_1)) _reset_user_cmd_id_1)
  (:reset_user_cmd_id_2
   (&optional (__reset_user_cmd_id_2 :null))
   (if (not (eq __reset_user_cmd_id_2 :null)) (setq _reset_user_cmd_id_2 __reset_user_cmd_id_2)) _reset_user_cmd_id_2)
  (:reset_user_cmd_id_3
   (&optional (__reset_user_cmd_id_3 :null))
   (if (not (eq __reset_user_cmd_id_3 :null)) (setq _reset_user_cmd_id_3 __reset_user_cmd_id_3)) _reset_user_cmd_id_3)
  (:reset_user_cmd_id_4
   (&optional (__reset_user_cmd_id_4 :null))
   (if (not (eq __reset_user_cmd_id_4 :null)) (setq _reset_user_cmd_id_4 __reset_user_cmd_id_4)) _reset_user_cmd_id_4)
  (:disable_user_rpt_id
   (&optional (__disable_user_rpt_id :null))
   (if (not (eq __disable_user_rpt_id :null)) (setq _disable_user_rpt_id __disable_user_rpt_id)) _disable_user_rpt_id)
  (:reenable_default_cmd_id
   (&optional (__reenable_default_cmd_id :null))
   (if (not (eq __reenable_default_cmd_id :null)) (setq _reenable_default_cmd_id __reenable_default_cmd_id)) _reenable_default_cmd_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint8 _reset_type
    1
    ;; bool _reset_user_rpt_id
    1
    ;; bool _reset_user_cmd_id_1
    1
    ;; bool _reset_user_cmd_id_2
    1
    ;; bool _reset_user_cmd_id_3
    1
    ;; bool _reset_user_cmd_id_4
    1
    ;; bool _disable_user_rpt_id
    1
    ;; bool _reenable_default_cmd_id
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
     ;; uint8 _reset_type
       (write-byte _reset_type s)
     ;; bool _reset_user_rpt_id
       (if _reset_user_rpt_id (write-byte -1 s) (write-byte 0 s))
     ;; bool _reset_user_cmd_id_1
       (if _reset_user_cmd_id_1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _reset_user_cmd_id_2
       (if _reset_user_cmd_id_2 (write-byte -1 s) (write-byte 0 s))
     ;; bool _reset_user_cmd_id_3
       (if _reset_user_cmd_id_3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _reset_user_cmd_id_4
       (if _reset_user_cmd_id_4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _disable_user_rpt_id
       (if _disable_user_rpt_id (write-byte -1 s) (write-byte 0 s))
     ;; bool _reenable_default_cmd_id
       (if _reenable_default_cmd_id (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _reset_type
     (setq _reset_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _reset_user_rpt_id
     (setq _reset_user_rpt_id (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reset_user_cmd_id_1
     (setq _reset_user_cmd_id_1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reset_user_cmd_id_2
     (setq _reset_user_cmd_id_2 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reset_user_cmd_id_3
     (setq _reset_user_cmd_id_3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reset_user_cmd_id_4
     (setq _reset_user_cmd_id_4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _disable_user_rpt_id
     (setq _disable_user_rpt_id (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _reenable_default_cmd_id
     (setq _reenable_default_cmd_id (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ResetCmd :md5sum-) "0599fdf3d91d47c66c1ecb3a5a9d3e0c")
(setf (get kartech_linear_actuator_msgs::ResetCmd :datatype-) "kartech_linear_actuator_msgs/ResetCmd")
(setf (get kartech_linear_actuator_msgs::ResetCmd :definition-)
      "std_msgs/Header header
bool confirm

uint8 RESET_OUTPUTS = 0
uint8 RESET_USER_DEFINED_IDS = 1
uint8 RESET_REPORT_RATES = 2
uint8 RESET_HARDWARE_CONFIGURATIONS = 3
uint8 RESET_USER_CONFIGURATIONS = 4
uint8 RESET_EVERYTHING = 5
uint8 RESET_NONE = 6

uint8 reset_type

bool reset_user_rpt_id
bool reset_user_cmd_id_1
bool reset_user_cmd_id_2
bool reset_user_cmd_id_3
bool reset_user_cmd_id_4
bool disable_user_rpt_id
bool reenable_default_cmd_id

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



(provide :kartech_linear_actuator_msgs/ResetCmd "0599fdf3d91d47c66c1ecb3a5a9d3e0c")


