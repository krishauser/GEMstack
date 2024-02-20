;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrValid1)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrValid1 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRVALID1")
  (make-package "DELPHI_ESR_MSGS::ESRVALID1"))

(in-package "ROS")
;;//! \htmlinclude EsrValid1.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrValid1
  :super ros::object
  :slots (_header _canmsg _lr_sn _lr_range _lr_range_rate _lr_angle _lr_power ))

(defmethod delphi_esr_msgs::EsrValid1
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:lr_sn __lr_sn) 0)
    ((:lr_range __lr_range) 0.0)
    ((:lr_range_rate __lr_range_rate) 0.0)
    ((:lr_angle __lr_angle) 0.0)
    ((:lr_power __lr_power) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _lr_sn (round __lr_sn))
   (setq _lr_range (float __lr_range))
   (setq _lr_range_rate (float __lr_range_rate))
   (setq _lr_angle (float __lr_angle))
   (setq _lr_power (round __lr_power))
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
  (:lr_sn
   (&optional __lr_sn)
   (if __lr_sn (setq _lr_sn __lr_sn)) _lr_sn)
  (:lr_range
   (&optional __lr_range)
   (if __lr_range (setq _lr_range __lr_range)) _lr_range)
  (:lr_range_rate
   (&optional __lr_range_rate)
   (if __lr_range_rate (setq _lr_range_rate __lr_range_rate)) _lr_range_rate)
  (:lr_angle
   (&optional __lr_angle)
   (if __lr_angle (setq _lr_angle __lr_angle)) _lr_angle)
  (:lr_power
   (&optional __lr_power)
   (if __lr_power (setq _lr_power __lr_power)) _lr_power)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _lr_sn
    1
    ;; float32 _lr_range
    4
    ;; float32 _lr_range_rate
    4
    ;; float32 _lr_angle
    4
    ;; int8 _lr_power
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
     ;; uint8 _lr_sn
       (write-byte _lr_sn s)
     ;; float32 _lr_range
       (sys::poke _lr_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lr_range_rate
       (sys::poke _lr_range_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lr_angle
       (sys::poke _lr_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _lr_power
       (write-byte _lr_power s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _lr_sn
     (setq _lr_sn (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _lr_range
     (setq _lr_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lr_range_rate
     (setq _lr_range_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lr_angle
     (setq _lr_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _lr_power
     (setq _lr_power (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _lr_power 127) (setq _lr_power (- _lr_power 256)))
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrValid1 :md5sum-) "41a700597be629966cda02aac94ad2e1")
(setf (get delphi_esr_msgs::EsrValid1 :datatype-) "delphi_esr_msgs/EsrValid1")
(setf (get delphi_esr_msgs::EsrValid1 :definition-)
      "std_msgs/Header header

# ESR Valid1
string      canmsg

uint8       lr_sn
float32     lr_range
float32     lr_range_rate
float32     lr_angle
int8        lr_power

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



(provide :delphi_esr_msgs/EsrValid1 "41a700597be629966cda02aac94ad2e1")


