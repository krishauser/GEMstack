;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrStatus8)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrStatus8 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRSTATUS8")
  (make-package "DELPHI_ESR_MSGS::ESRSTATUS8"))

(in-package "ROS")
;;//! \htmlinclude EsrStatus8.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrStatus8
  :super ros::object
  :slots (_header _canmsg _history_fault_0 _history_fault_1 _history_fault_2 _history_fault_3 _history_fault_4 _history_fault_5 _history_fault_6 _history_fault_7 ))

(defmethod delphi_esr_msgs::EsrStatus8
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:history_fault_0 __history_fault_0) 0)
    ((:history_fault_1 __history_fault_1) 0)
    ((:history_fault_2 __history_fault_2) 0)
    ((:history_fault_3 __history_fault_3) 0)
    ((:history_fault_4 __history_fault_4) 0)
    ((:history_fault_5 __history_fault_5) 0)
    ((:history_fault_6 __history_fault_6) 0)
    ((:history_fault_7 __history_fault_7) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _history_fault_0 (round __history_fault_0))
   (setq _history_fault_1 (round __history_fault_1))
   (setq _history_fault_2 (round __history_fault_2))
   (setq _history_fault_3 (round __history_fault_3))
   (setq _history_fault_4 (round __history_fault_4))
   (setq _history_fault_5 (round __history_fault_5))
   (setq _history_fault_6 (round __history_fault_6))
   (setq _history_fault_7 (round __history_fault_7))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:canmsg
   (&optional __canmsg)
   (if __canmsg (setq _canmsg __canmsg)) _canmsg)
  (:history_fault_0
   (&optional __history_fault_0)
   (if __history_fault_0 (setq _history_fault_0 __history_fault_0)) _history_fault_0)
  (:history_fault_1
   (&optional __history_fault_1)
   (if __history_fault_1 (setq _history_fault_1 __history_fault_1)) _history_fault_1)
  (:history_fault_2
   (&optional __history_fault_2)
   (if __history_fault_2 (setq _history_fault_2 __history_fault_2)) _history_fault_2)
  (:history_fault_3
   (&optional __history_fault_3)
   (if __history_fault_3 (setq _history_fault_3 __history_fault_3)) _history_fault_3)
  (:history_fault_4
   (&optional __history_fault_4)
   (if __history_fault_4 (setq _history_fault_4 __history_fault_4)) _history_fault_4)
  (:history_fault_5
   (&optional __history_fault_5)
   (if __history_fault_5 (setq _history_fault_5 __history_fault_5)) _history_fault_5)
  (:history_fault_6
   (&optional __history_fault_6)
   (if __history_fault_6 (setq _history_fault_6 __history_fault_6)) _history_fault_6)
  (:history_fault_7
   (&optional __history_fault_7)
   (if __history_fault_7 (setq _history_fault_7 __history_fault_7)) _history_fault_7)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _history_fault_0
    1
    ;; uint8 _history_fault_1
    1
    ;; uint8 _history_fault_2
    1
    ;; uint8 _history_fault_3
    1
    ;; uint8 _history_fault_4
    1
    ;; uint8 _history_fault_5
    1
    ;; uint8 _history_fault_6
    1
    ;; uint8 _history_fault_7
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; uint8 _history_fault_0
       (write-byte _history_fault_0 s)
     ;; uint8 _history_fault_1
       (write-byte _history_fault_1 s)
     ;; uint8 _history_fault_2
       (write-byte _history_fault_2 s)
     ;; uint8 _history_fault_3
       (write-byte _history_fault_3 s)
     ;; uint8 _history_fault_4
       (write-byte _history_fault_4 s)
     ;; uint8 _history_fault_5
       (write-byte _history_fault_5 s)
     ;; uint8 _history_fault_6
       (write-byte _history_fault_6 s)
     ;; uint8 _history_fault_7
       (write-byte _history_fault_7 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _history_fault_0
     (setq _history_fault_0 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_1
     (setq _history_fault_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_2
     (setq _history_fault_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_3
     (setq _history_fault_3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_4
     (setq _history_fault_4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_5
     (setq _history_fault_5 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_6
     (setq _history_fault_6 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _history_fault_7
     (setq _history_fault_7 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrStatus8 :md5sum-) "c7431176d5d09e818f01fd18ce628e7e")
(setf (get delphi_esr_msgs::EsrStatus8 :datatype-) "delphi_esr_msgs/EsrStatus8")
(setf (get delphi_esr_msgs::EsrStatus8 :definition-)
      "std_msgs/Header header

# ESR Status8
string      canmsg

uint8       history_fault_0
uint8       history_fault_1
uint8       history_fault_2
uint8       history_fault_3
uint8       history_fault_4
uint8       history_fault_5
uint8       history_fault_6
uint8       history_fault_7

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



(provide :delphi_esr_msgs/EsrStatus8 "c7431176d5d09e818f01fd18ce628e7e")


