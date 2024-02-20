;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrValid2)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrValid2 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRVALID2")
  (make-package "DELPHI_ESR_MSGS::ESRVALID2"))

(in-package "ROS")
;;//! \htmlinclude EsrValid2.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrValid2
  :super ros::object
  :slots (_header _canmsg _mr_sn _mr_range _mr_range_rate _mr_angle _mr_power ))

(defmethod delphi_esr_msgs::EsrValid2
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:mr_sn __mr_sn) 0)
    ((:mr_range __mr_range) 0.0)
    ((:mr_range_rate __mr_range_rate) 0.0)
    ((:mr_angle __mr_angle) 0.0)
    ((:mr_power __mr_power) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _mr_sn (round __mr_sn))
   (setq _mr_range (float __mr_range))
   (setq _mr_range_rate (float __mr_range_rate))
   (setq _mr_angle (float __mr_angle))
   (setq _mr_power (round __mr_power))
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
  (:mr_sn
   (&optional __mr_sn)
   (if __mr_sn (setq _mr_sn __mr_sn)) _mr_sn)
  (:mr_range
   (&optional __mr_range)
   (if __mr_range (setq _mr_range __mr_range)) _mr_range)
  (:mr_range_rate
   (&optional __mr_range_rate)
   (if __mr_range_rate (setq _mr_range_rate __mr_range_rate)) _mr_range_rate)
  (:mr_angle
   (&optional __mr_angle)
   (if __mr_angle (setq _mr_angle __mr_angle)) _mr_angle)
  (:mr_power
   (&optional __mr_power)
   (if __mr_power (setq _mr_power __mr_power)) _mr_power)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _mr_sn
    1
    ;; float32 _mr_range
    4
    ;; float32 _mr_range_rate
    4
    ;; float32 _mr_angle
    4
    ;; int8 _mr_power
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
     ;; uint8 _mr_sn
       (write-byte _mr_sn s)
     ;; float32 _mr_range
       (sys::poke _mr_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mr_range_rate
       (sys::poke _mr_range_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mr_angle
       (sys::poke _mr_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _mr_power
       (write-byte _mr_power s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _mr_sn
     (setq _mr_sn (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _mr_range
     (setq _mr_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mr_range_rate
     (setq _mr_range_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mr_angle
     (setq _mr_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _mr_power
     (setq _mr_power (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mr_power 127) (setq _mr_power (- _mr_power 256)))
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrValid2 :md5sum-) "d38bb2ae1a8306a633e6d233f392ac23")
(setf (get delphi_esr_msgs::EsrValid2 :datatype-) "delphi_esr_msgs/EsrValid2")
(setf (get delphi_esr_msgs::EsrValid2 :definition-)
      "std_msgs/Header header

# ESR Valid2
string      canmsg

uint8       mr_sn
float32     mr_range
float32     mr_range_rate
float32     mr_angle
int8        mr_power

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



(provide :delphi_esr_msgs/EsrValid2 "d38bb2ae1a8306a633e6d233f392ac23")


