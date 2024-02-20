;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrStatusSerialNumber)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrStatusSerialNumber (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRSTATUSSERIALNUMBER")
  (make-package "DELPHI_MRR_MSGS::MRRSTATUSSERIALNUMBER"))

(in-package "ROS")
;;//! \htmlinclude MrrStatusSerialNumber.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrStatusSerialNumber
  :super ros::object
  :slots (_header _can_sequence_number _can_serial_number ))

(defmethod delphi_mrr_msgs::MrrStatusSerialNumber
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_sequence_number __can_sequence_number) 0)
    ((:can_serial_number __can_serial_number) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_sequence_number (round __can_sequence_number))
   (setq _can_serial_number (round __can_serial_number))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_sequence_number
   (&optional __can_sequence_number)
   (if __can_sequence_number (setq _can_sequence_number __can_sequence_number)) _can_sequence_number)
  (:can_serial_number
   (&optional __can_serial_number)
   (if __can_serial_number (setq _can_serial_number __can_serial_number)) _can_serial_number)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _can_sequence_number
    2
    ;; uint64 _can_serial_number
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _can_sequence_number
       (write-word _can_sequence_number s)
     ;; uint64 _can_serial_number
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _can_serial_number (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _can_serial_number) (= (length (_can_serial_number . bv)) 2)) ;; bignum
              (write-long (ash (elt (_can_serial_number . bv) 0) 0) s)
              (write-long (ash (elt (_can_serial_number . bv) 1) -1) s))
             ((and (class _can_serial_number) (= (length (_can_serial_number . bv)) 1)) ;; big1
              (write-long (elt (_can_serial_number . bv) 0) s)
              (write-long (if (>= _can_serial_number 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _can_serial_number s)(write-long (if (>= _can_serial_number 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _can_sequence_number
     (setq _can_sequence_number (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint64 _can_serial_number
     
#+(or :alpha :irix6 :x86_64)
      (setf _can_serial_number (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _can_serial_number (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrStatusSerialNumber :md5sum-) "5d3bfd88a3b11cee7de78d554e06de73")
(setf (get delphi_mrr_msgs::MrrStatusSerialNumber :datatype-) "delphi_mrr_msgs/MrrStatusSerialNumber")
(setf (get delphi_mrr_msgs::MrrStatusSerialNumber :definition-)
      "std_msgs/Header header

uint16 can_sequence_number
uint64 can_serial_number

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



(provide :delphi_mrr_msgs/MrrStatusSerialNumber "5d3bfd88a3b11cee7de78d554e06de73")


