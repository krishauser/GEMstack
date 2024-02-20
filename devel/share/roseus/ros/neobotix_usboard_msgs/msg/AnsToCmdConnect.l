;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::AnsToCmdConnect)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'AnsToCmdConnect (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::ANSTOCMDCONNECT")
  (make-package "NEOBOTIX_USBOARD_MSGS::ANSTOCMDCONNECT"))

(in-package "ROS")
;;//! \htmlinclude AnsToCmdConnect.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neobotix_usboard_msgs::AnsToCmdConnect
  :super ros::object
  :slots (_header _command _cmd_connect_ans_d7 _cmd_connect_ans_d6 _cmd_connect_ans_d5 _cmd_connect_ans_d4 _cmd_connect_ans_d3 _cmd_connect_ans_d2 _cmd_connect_ans_d1 ))

(defmethod neobotix_usboard_msgs::AnsToCmdConnect
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    ((:cmd_connect_ans_d7 __cmd_connect_ans_d7) 0)
    ((:cmd_connect_ans_d6 __cmd_connect_ans_d6) 0)
    ((:cmd_connect_ans_d5 __cmd_connect_ans_d5) 0)
    ((:cmd_connect_ans_d4 __cmd_connect_ans_d4) 0)
    ((:cmd_connect_ans_d3 __cmd_connect_ans_d3) 0)
    ((:cmd_connect_ans_d2 __cmd_connect_ans_d2) 0)
    ((:cmd_connect_ans_d1 __cmd_connect_ans_d1) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   (setq _cmd_connect_ans_d7 (round __cmd_connect_ans_d7))
   (setq _cmd_connect_ans_d6 (round __cmd_connect_ans_d6))
   (setq _cmd_connect_ans_d5 (round __cmd_connect_ans_d5))
   (setq _cmd_connect_ans_d4 (round __cmd_connect_ans_d4))
   (setq _cmd_connect_ans_d3 (round __cmd_connect_ans_d3))
   (setq _cmd_connect_ans_d2 (round __cmd_connect_ans_d2))
   (setq _cmd_connect_ans_d1 (round __cmd_connect_ans_d1))
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
  (:cmd_connect_ans_d7
   (&optional __cmd_connect_ans_d7)
   (if __cmd_connect_ans_d7 (setq _cmd_connect_ans_d7 __cmd_connect_ans_d7)) _cmd_connect_ans_d7)
  (:cmd_connect_ans_d6
   (&optional __cmd_connect_ans_d6)
   (if __cmd_connect_ans_d6 (setq _cmd_connect_ans_d6 __cmd_connect_ans_d6)) _cmd_connect_ans_d6)
  (:cmd_connect_ans_d5
   (&optional __cmd_connect_ans_d5)
   (if __cmd_connect_ans_d5 (setq _cmd_connect_ans_d5 __cmd_connect_ans_d5)) _cmd_connect_ans_d5)
  (:cmd_connect_ans_d4
   (&optional __cmd_connect_ans_d4)
   (if __cmd_connect_ans_d4 (setq _cmd_connect_ans_d4 __cmd_connect_ans_d4)) _cmd_connect_ans_d4)
  (:cmd_connect_ans_d3
   (&optional __cmd_connect_ans_d3)
   (if __cmd_connect_ans_d3 (setq _cmd_connect_ans_d3 __cmd_connect_ans_d3)) _cmd_connect_ans_d3)
  (:cmd_connect_ans_d2
   (&optional __cmd_connect_ans_d2)
   (if __cmd_connect_ans_d2 (setq _cmd_connect_ans_d2 __cmd_connect_ans_d2)) _cmd_connect_ans_d2)
  (:cmd_connect_ans_d1
   (&optional __cmd_connect_ans_d1)
   (if __cmd_connect_ans_d1 (setq _cmd_connect_ans_d1 __cmd_connect_ans_d1)) _cmd_connect_ans_d1)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command
    1
    ;; uint8 _cmd_connect_ans_d7
    1
    ;; uint8 _cmd_connect_ans_d6
    1
    ;; uint8 _cmd_connect_ans_d5
    1
    ;; uint8 _cmd_connect_ans_d4
    1
    ;; uint8 _cmd_connect_ans_d3
    1
    ;; uint8 _cmd_connect_ans_d2
    1
    ;; uint8 _cmd_connect_ans_d1
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
     ;; uint8 _cmd_connect_ans_d7
       (write-byte _cmd_connect_ans_d7 s)
     ;; uint8 _cmd_connect_ans_d6
       (write-byte _cmd_connect_ans_d6 s)
     ;; uint8 _cmd_connect_ans_d5
       (write-byte _cmd_connect_ans_d5 s)
     ;; uint8 _cmd_connect_ans_d4
       (write-byte _cmd_connect_ans_d4 s)
     ;; uint8 _cmd_connect_ans_d3
       (write-byte _cmd_connect_ans_d3 s)
     ;; uint8 _cmd_connect_ans_d2
       (write-byte _cmd_connect_ans_d2 s)
     ;; uint8 _cmd_connect_ans_d1
       (write-byte _cmd_connect_ans_d1 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d7
     (setq _cmd_connect_ans_d7 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d6
     (setq _cmd_connect_ans_d6 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d5
     (setq _cmd_connect_ans_d5 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d4
     (setq _cmd_connect_ans_d4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d3
     (setq _cmd_connect_ans_d3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d2
     (setq _cmd_connect_ans_d2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cmd_connect_ans_d1
     (setq _cmd_connect_ans_d1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::AnsToCmdConnect :md5sum-) "d1ef60b13020f0e599d4fbb33f17b3f1")
(setf (get neobotix_usboard_msgs::AnsToCmdConnect :datatype-) "neobotix_usboard_msgs/AnsToCmdConnect")
(setf (get neobotix_usboard_msgs::AnsToCmdConnect :definition-)
      "# Message file for AnsToCmdConnect

std_msgs/Header header

uint8     command                                 
uint8     cmd_connect_ans_d7                      
uint8     cmd_connect_ans_d6                      
uint8     cmd_connect_ans_d5                      
uint8     cmd_connect_ans_d4                      
uint8     cmd_connect_ans_d3                      
uint8     cmd_connect_ans_d2                      
uint8     cmd_connect_ans_d1                      

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



(provide :neobotix_usboard_msgs/AnsToCmdConnect "d1ef60b13020f0e599d4fbb33f17b3f1")


