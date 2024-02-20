;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ConfigureOutputsPwmFreqCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::CONFIGUREOUTPUTSPWMFREQCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::CONFIGUREOUTPUTSPWMFREQCMD"))

(in-package "ROS")
;;//! \htmlinclude ConfigureOutputsPwmFreqCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd
  :super ros::object
  :slots (_header _confirm _min_pwm_pct _max_pwm_pct _pwm_freq ))

(defmethod kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:min_pwm_pct __min_pwm_pct) 0)
    ((:max_pwm_pct __max_pwm_pct) 0)
    ((:pwm_freq __pwm_freq) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _min_pwm_pct (round __min_pwm_pct))
   (setq _max_pwm_pct (round __max_pwm_pct))
   (setq _pwm_freq (round __pwm_freq))
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
  (:min_pwm_pct
   (&optional __min_pwm_pct)
   (if __min_pwm_pct (setq _min_pwm_pct __min_pwm_pct)) _min_pwm_pct)
  (:max_pwm_pct
   (&optional __max_pwm_pct)
   (if __max_pwm_pct (setq _max_pwm_pct __max_pwm_pct)) _max_pwm_pct)
  (:pwm_freq
   (&optional __pwm_freq)
   (if __pwm_freq (setq _pwm_freq __pwm_freq)) _pwm_freq)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint8 _min_pwm_pct
    1
    ;; uint8 _max_pwm_pct
    1
    ;; uint16 _pwm_freq
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
     ;; uint8 _min_pwm_pct
       (write-byte _min_pwm_pct s)
     ;; uint8 _max_pwm_pct
       (write-byte _max_pwm_pct s)
     ;; uint16 _pwm_freq
       (write-word _pwm_freq s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _min_pwm_pct
     (setq _min_pwm_pct (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _max_pwm_pct
     (setq _max_pwm_pct (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _pwm_freq
     (setq _pwm_freq (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd :md5sum-) "177ba95b80ad87cfd885201c32903f9c")
(setf (get kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd :datatype-) "kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd")
(setf (get kartech_linear_actuator_msgs::ConfigureOutputsPwmFreqCmd :definition-)
      "std_msgs/Header header
bool confirm
uint8 min_pwm_pct   # The minimum motor drive duty cycle in percent (0-100). Default is 20%.
uint8 max_pwm_pct   # The maximum motor drive duty cycle in percent (0-100). Default is 90%.
uint16 pwm_freq     # The frequency of the PWM outputs in Hz. Default is 2000Hz.

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



(provide :kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd "177ba95b80ad87cfd885201c32903f9c")


