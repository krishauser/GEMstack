;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ConfigureOutputsKdFreqDeadbandCmd)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ConfigureOutputsKdFreqDeadbandCmd (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::CONFIGUREOUTPUTSKDFREQDEADBANDCMD")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::CONFIGUREOUTPUTSKDFREQDEADBANDCMD"))

(in-package "ROS")
;;//! \htmlinclude ConfigureOutputsKdFreqDeadbandCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ConfigureOutputsKdFreqDeadbandCmd
  :super ros::object
  :slots (_header _confirm _kd _closed_loop_freq _error_dead_band ))

(defmethod kartech_linear_actuator_msgs::ConfigureOutputsKdFreqDeadbandCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:kd __kd) 0)
    ((:closed_loop_freq __closed_loop_freq) 0)
    ((:error_dead_band __error_dead_band) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _kd (round __kd))
   (setq _closed_loop_freq (round __closed_loop_freq))
   (setq _error_dead_band (float __error_dead_band))
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
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:closed_loop_freq
   (&optional __closed_loop_freq)
   (if __closed_loop_freq (setq _closed_loop_freq __closed_loop_freq)) _closed_loop_freq)
  (:error_dead_band
   (&optional __error_dead_band)
   (if __error_dead_band (setq _error_dead_band __error_dead_band)) _error_dead_band)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; uint16 _kd
    2
    ;; uint8 _closed_loop_freq
    1
    ;; float64 _error_dead_band
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _kd
       (write-word _kd s)
     ;; uint8 _closed_loop_freq
       (write-byte _closed_loop_freq s)
     ;; float64 _error_dead_band
       (sys::poke _error_dead_band (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _kd
     (setq _kd (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _closed_loop_freq
     (setq _closed_loop_freq (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _error_dead_band
     (setq _error_dead_band (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ConfigureOutputsKdFreqDeadbandCmd :md5sum-) "865fff7dce2fec39beac32ec4e1f4638")
(setf (get kartech_linear_actuator_msgs::ConfigureOutputsKdFreqDeadbandCmd :datatype-) "kartech_linear_actuator_msgs/ConfigureOutputsKdFreqDeadbandCmd")
(setf (get kartech_linear_actuator_msgs::ConfigureOutputsKdFreqDeadbandCmd :definition-)
      "std_msgs/Header header
bool confirm
uint16 kd                   # The differential term of the closed-loop control. Default is 10.
uint8 closed_loop_freq      # The frequency of closed-loop corrections in Hz. The default is 60Hz.
float64 error_dead_band     # The size of the dead-band for error correction in units of 0.001\". The default is 0.05\"

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



(provide :kartech_linear_actuator_msgs/ConfigureOutputsKdFreqDeadbandCmd "865fff7dce2fec39beac32ec4e1f4638")


