;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::Command)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'Command (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::COMMAND")
  (make-package "NEOBOTIX_USBOARD_MSGS::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass neobotix_usboard_msgs::Command
  :super ros::object
  :slots (_header _command _command_data _set_num _paraset_byte6 _paraset_byte5 _paraset_byte4 _paraset_byte3 _paraset_byte2 _paraset_byte1 _set_active_9to16 _set_active_1to8 ))

(defmethod neobotix_usboard_msgs::Command
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    ((:command_data __command_data) 0)
    ((:set_num __set_num) 0)
    ((:paraset_byte6 __paraset_byte6) 0)
    ((:paraset_byte5 __paraset_byte5) 0)
    ((:paraset_byte4 __paraset_byte4) 0)
    ((:paraset_byte3 __paraset_byte3) 0)
    ((:paraset_byte2 __paraset_byte2) 0)
    ((:paraset_byte1 __paraset_byte1) 0)
    ((:set_active_9to16 __set_active_9to16) 0)
    ((:set_active_1to8 __set_active_1to8) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   (setq _command_data (round __command_data))
   (setq _set_num (round __set_num))
   (setq _paraset_byte6 (round __paraset_byte6))
   (setq _paraset_byte5 (round __paraset_byte5))
   (setq _paraset_byte4 (round __paraset_byte4))
   (setq _paraset_byte3 (round __paraset_byte3))
   (setq _paraset_byte2 (round __paraset_byte2))
   (setq _paraset_byte1 (round __paraset_byte1))
   (setq _set_active_9to16 (round __set_active_9to16))
   (setq _set_active_1to8 (round __set_active_1to8))
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
  (:command_data
   (&optional __command_data)
   (if __command_data (setq _command_data __command_data)) _command_data)
  (:set_num
   (&optional __set_num)
   (if __set_num (setq _set_num __set_num)) _set_num)
  (:paraset_byte6
   (&optional __paraset_byte6)
   (if __paraset_byte6 (setq _paraset_byte6 __paraset_byte6)) _paraset_byte6)
  (:paraset_byte5
   (&optional __paraset_byte5)
   (if __paraset_byte5 (setq _paraset_byte5 __paraset_byte5)) _paraset_byte5)
  (:paraset_byte4
   (&optional __paraset_byte4)
   (if __paraset_byte4 (setq _paraset_byte4 __paraset_byte4)) _paraset_byte4)
  (:paraset_byte3
   (&optional __paraset_byte3)
   (if __paraset_byte3 (setq _paraset_byte3 __paraset_byte3)) _paraset_byte3)
  (:paraset_byte2
   (&optional __paraset_byte2)
   (if __paraset_byte2 (setq _paraset_byte2 __paraset_byte2)) _paraset_byte2)
  (:paraset_byte1
   (&optional __paraset_byte1)
   (if __paraset_byte1 (setq _paraset_byte1 __paraset_byte1)) _paraset_byte1)
  (:set_active_9to16
   (&optional __set_active_9to16)
   (if __set_active_9to16 (setq _set_active_9to16 __set_active_9to16)) _set_active_9to16)
  (:set_active_1to8
   (&optional __set_active_1to8)
   (if __set_active_1to8 (setq _set_active_1to8 __set_active_1to8)) _set_active_1to8)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command
    1
    ;; uint64 _command_data
    8
    ;; uint8 _set_num
    1
    ;; uint8 _paraset_byte6
    1
    ;; uint8 _paraset_byte5
    1
    ;; uint8 _paraset_byte4
    1
    ;; uint8 _paraset_byte3
    1
    ;; uint8 _paraset_byte2
    1
    ;; uint8 _paraset_byte1
    1
    ;; uint8 _set_active_9to16
    1
    ;; uint8 _set_active_1to8
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
     ;; uint64 _command_data
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _command_data (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _command_data) (= (length (_command_data . bv)) 2)) ;; bignum
              (write-long (ash (elt (_command_data . bv) 0) 0) s)
              (write-long (ash (elt (_command_data . bv) 1) -1) s))
             ((and (class _command_data) (= (length (_command_data . bv)) 1)) ;; big1
              (write-long (elt (_command_data . bv) 0) s)
              (write-long (if (>= _command_data 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _command_data s)(write-long (if (>= _command_data 0) 0 #xffffffff) s)))
     ;; uint8 _set_num
       (write-byte _set_num s)
     ;; uint8 _paraset_byte6
       (write-byte _paraset_byte6 s)
     ;; uint8 _paraset_byte5
       (write-byte _paraset_byte5 s)
     ;; uint8 _paraset_byte4
       (write-byte _paraset_byte4 s)
     ;; uint8 _paraset_byte3
       (write-byte _paraset_byte3 s)
     ;; uint8 _paraset_byte2
       (write-byte _paraset_byte2 s)
     ;; uint8 _paraset_byte1
       (write-byte _paraset_byte1 s)
     ;; uint8 _set_active_9to16
       (write-byte _set_active_9to16 s)
     ;; uint8 _set_active_1to8
       (write-byte _set_active_1to8 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint64 _command_data
     
#+(or :alpha :irix6 :x86_64)
      (setf _command_data (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _command_data (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint8 _set_num
     (setq _set_num (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte6
     (setq _paraset_byte6 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte5
     (setq _paraset_byte5 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte4
     (setq _paraset_byte4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte3
     (setq _paraset_byte3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte2
     (setq _paraset_byte2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _paraset_byte1
     (setq _paraset_byte1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _set_active_9to16
     (setq _set_active_9to16 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _set_active_1to8
     (setq _set_active_1to8 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::Command :md5sum-) "aa2adac976b058480751c90ad9fb67d8")
(setf (get neobotix_usboard_msgs::Command :datatype-) "neobotix_usboard_msgs/Command")
(setf (get neobotix_usboard_msgs::Command :definition-)
      "# Message file for Command

std_msgs/Header header

uint8     command                                 
uint64    command_data                            
uint8     set_num                                 
uint8     paraset_byte6                     
uint8     paraset_byte5                     
uint8     paraset_byte4                     
uint8     paraset_byte3                     
uint8     paraset_byte2                     
uint8     paraset_byte1                     
uint8     set_active_9to16                        
uint8     set_active_1to8                         

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



(provide :neobotix_usboard_msgs/Command "aa2adac976b058480751c90ad9fb67d8")


