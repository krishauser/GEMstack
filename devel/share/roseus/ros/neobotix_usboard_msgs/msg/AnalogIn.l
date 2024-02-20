;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::AnalogIn)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'AnalogIn (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::ANALOGIN")
  (make-package "NEOBOTIX_USBOARD_MSGS::ANALOGIN"))

(in-package "ROS")
;;//! \htmlinclude AnalogIn.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neobotix_usboard_msgs::AnalogIn
  :super ros::object
  :slots (_header _command _analog_data_ch4_low_byte _analog_data_ch4_high_bits _analog_data_ch3_low_byte _analog_data_ch3_high_bits _analog_data_ch2_low_byte _analog_data_ch2_high_bits _analog_data_ch1_low_byte _analog_data_ch1_high_bits ))

(defmethod neobotix_usboard_msgs::AnalogIn
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    ((:analog_data_ch4_low_byte __analog_data_ch4_low_byte) 0)
    ((:analog_data_ch4_high_bits __analog_data_ch4_high_bits) 0)
    ((:analog_data_ch3_low_byte __analog_data_ch3_low_byte) 0)
    ((:analog_data_ch3_high_bits __analog_data_ch3_high_bits) 0)
    ((:analog_data_ch2_low_byte __analog_data_ch2_low_byte) 0)
    ((:analog_data_ch2_high_bits __analog_data_ch2_high_bits) 0)
    ((:analog_data_ch1_low_byte __analog_data_ch1_low_byte) 0)
    ((:analog_data_ch1_high_bits __analog_data_ch1_high_bits) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   (setq _analog_data_ch4_low_byte (round __analog_data_ch4_low_byte))
   (setq _analog_data_ch4_high_bits (round __analog_data_ch4_high_bits))
   (setq _analog_data_ch3_low_byte (round __analog_data_ch3_low_byte))
   (setq _analog_data_ch3_high_bits (round __analog_data_ch3_high_bits))
   (setq _analog_data_ch2_low_byte (round __analog_data_ch2_low_byte))
   (setq _analog_data_ch2_high_bits (round __analog_data_ch2_high_bits))
   (setq _analog_data_ch1_low_byte (round __analog_data_ch1_low_byte))
   (setq _analog_data_ch1_high_bits (round __analog_data_ch1_high_bits))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:analog_data_ch4_low_byte
   (&optional __analog_data_ch4_low_byte)
   (if __analog_data_ch4_low_byte (setq _analog_data_ch4_low_byte __analog_data_ch4_low_byte)) _analog_data_ch4_low_byte)
  (:analog_data_ch4_high_bits
   (&optional __analog_data_ch4_high_bits)
   (if __analog_data_ch4_high_bits (setq _analog_data_ch4_high_bits __analog_data_ch4_high_bits)) _analog_data_ch4_high_bits)
  (:analog_data_ch3_low_byte
   (&optional __analog_data_ch3_low_byte)
   (if __analog_data_ch3_low_byte (setq _analog_data_ch3_low_byte __analog_data_ch3_low_byte)) _analog_data_ch3_low_byte)
  (:analog_data_ch3_high_bits
   (&optional __analog_data_ch3_high_bits)
   (if __analog_data_ch3_high_bits (setq _analog_data_ch3_high_bits __analog_data_ch3_high_bits)) _analog_data_ch3_high_bits)
  (:analog_data_ch2_low_byte
   (&optional __analog_data_ch2_low_byte)
   (if __analog_data_ch2_low_byte (setq _analog_data_ch2_low_byte __analog_data_ch2_low_byte)) _analog_data_ch2_low_byte)
  (:analog_data_ch2_high_bits
   (&optional __analog_data_ch2_high_bits)
   (if __analog_data_ch2_high_bits (setq _analog_data_ch2_high_bits __analog_data_ch2_high_bits)) _analog_data_ch2_high_bits)
  (:analog_data_ch1_low_byte
   (&optional __analog_data_ch1_low_byte)
   (if __analog_data_ch1_low_byte (setq _analog_data_ch1_low_byte __analog_data_ch1_low_byte)) _analog_data_ch1_low_byte)
  (:analog_data_ch1_high_bits
   (&optional __analog_data_ch1_high_bits)
   (if __analog_data_ch1_high_bits (setq _analog_data_ch1_high_bits __analog_data_ch1_high_bits)) _analog_data_ch1_high_bits)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command
    1
    ;; uint8 _analog_data_ch4_low_byte
    1
    ;; uint8 _analog_data_ch4_high_bits
    1
    ;; uint8 _analog_data_ch3_low_byte
    1
    ;; uint8 _analog_data_ch3_high_bits
    1
    ;; uint8 _analog_data_ch2_low_byte
    1
    ;; uint8 _analog_data_ch2_high_bits
    1
    ;; uint8 _analog_data_ch1_low_byte
    1
    ;; uint8 _analog_data_ch1_high_bits
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _command
       (write-byte _command s)
     ;; uint8 _analog_data_ch4_low_byte
       (write-byte _analog_data_ch4_low_byte s)
     ;; uint8 _analog_data_ch4_high_bits
       (write-byte _analog_data_ch4_high_bits s)
     ;; uint8 _analog_data_ch3_low_byte
       (write-byte _analog_data_ch3_low_byte s)
     ;; uint8 _analog_data_ch3_high_bits
       (write-byte _analog_data_ch3_high_bits s)
     ;; uint8 _analog_data_ch2_low_byte
       (write-byte _analog_data_ch2_low_byte s)
     ;; uint8 _analog_data_ch2_high_bits
       (write-byte _analog_data_ch2_high_bits s)
     ;; uint8 _analog_data_ch1_low_byte
       (write-byte _analog_data_ch1_low_byte s)
     ;; uint8 _analog_data_ch1_high_bits
       (write-byte _analog_data_ch1_high_bits s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch4_low_byte
     (setq _analog_data_ch4_low_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch4_high_bits
     (setq _analog_data_ch4_high_bits (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch3_low_byte
     (setq _analog_data_ch3_low_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch3_high_bits
     (setq _analog_data_ch3_high_bits (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch2_low_byte
     (setq _analog_data_ch2_low_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch2_high_bits
     (setq _analog_data_ch2_high_bits (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch1_low_byte
     (setq _analog_data_ch1_low_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _analog_data_ch1_high_bits
     (setq _analog_data_ch1_high_bits (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::AnalogIn :md5sum-) "619eac438aa01d7a05701049ea57be6e")
(setf (get neobotix_usboard_msgs::AnalogIn :datatype-) "neobotix_usboard_msgs/AnalogIn")
(setf (get neobotix_usboard_msgs::AnalogIn :definition-)
      "# Message file for AnalogIn

std_msgs/Header header

uint8     command                                 
uint8     analog_data_ch4_low_byte                
uint8     analog_data_ch4_high_bits               
uint8     analog_data_ch3_low_byte                
uint8     analog_data_ch3_high_bits               
uint8     analog_data_ch2_low_byte                
uint8     analog_data_ch2_high_bits               
uint8     analog_data_ch1_low_byte                
uint8     analog_data_ch1_high_bits               

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



(provide :neobotix_usboard_msgs/AnalogIn "619eac438aa01d7a05701049ea57be6e")


