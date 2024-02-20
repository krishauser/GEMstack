;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::AnsParasetToEEPROM)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'AnsParasetToEEPROM (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::ANSPARASETTOEEPROM")
  (make-package "NEOBOTIX_USBOARD_MSGS::ANSPARASETTOEEPROM"))

(in-package "ROS")
;;//! \htmlinclude AnsParasetToEEPROM.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neobotix_usboard_msgs::AnsParasetToEEPROM
  :super ros::object
  :slots (_header _command _paraset_cksum_low_byte _paraset_cksum_high_byte ))

(defmethod neobotix_usboard_msgs::AnsParasetToEEPROM
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    ((:paraset_cksum_low_byte __paraset_cksum_low_byte) 0)
    ((:paraset_cksum_high_byte __paraset_cksum_high_byte) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   (setq _paraset_cksum_low_byte (round __paraset_cksum_low_byte))
   (setq _paraset_cksum_high_byte (round __paraset_cksum_high_byte))
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
  (:paraset_cksum_low_byte
   (&optional __paraset_cksum_low_byte)
   (if __paraset_cksum_low_byte (setq _paraset_cksum_low_byte __paraset_cksum_low_byte)) _paraset_cksum_low_byte)
  (:paraset_cksum_high_byte
   (&optional __paraset_cksum_high_byte)
   (if __paraset_cksum_high_byte (setq _paraset_cksum_high_byte __paraset_cksum_high_byte)) _paraset_cksum_high_byte)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command
    1
    ;; uint8 _paraset_cksum_low_byte
    1
    ;; uint8 _paraset_cksum_high_byte
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
     ;; uint8 _paraset_cksum_low_byte
       (write-byte _paraset_cksum_low_byte s)
     ;; uint8 _paraset_cksum_high_byte
       (write-byte _paraset_cksum_high_byte s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_cksum_low_byte
     (setq _paraset_cksum_low_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_cksum_high_byte
     (setq _paraset_cksum_high_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::AnsParasetToEEPROM :md5sum-) "ae6db4e8232618eb386bde89d0c827b5")
(setf (get neobotix_usboard_msgs::AnsParasetToEEPROM :datatype-) "neobotix_usboard_msgs/AnsParasetToEEPROM")
(setf (get neobotix_usboard_msgs::AnsParasetToEEPROM :definition-)
      "# Message file for AnsParasetToEEPROM

std_msgs/Header header

uint8     command                                 
uint8     paraset_cksum_low_byte                  
uint8     paraset_cksum_high_byte                 

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



(provide :neobotix_usboard_msgs/AnsParasetToEEPROM "ae6db4e8232618eb386bde89d0c827b5")


