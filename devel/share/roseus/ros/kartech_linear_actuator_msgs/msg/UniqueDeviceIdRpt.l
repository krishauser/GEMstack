;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::UniqueDeviceIdRpt)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'UniqueDeviceIdRpt (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::UNIQUEDEVICEIDRPT")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::UNIQUEDEVICEIDRPT"))

(in-package "ROS")
;;//! \htmlinclude UniqueDeviceIdRpt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::UniqueDeviceIdRpt
  :super ros::object
  :slots (_header _actuator_id_first_6 _actuator_id_last_6 ))

(defmethod kartech_linear_actuator_msgs::UniqueDeviceIdRpt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:actuator_id_first_6 __actuator_id_first_6) 0)
    ((:actuator_id_last_6 __actuator_id_last_6) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _actuator_id_first_6 (round __actuator_id_first_6))
   (setq _actuator_id_last_6 (round __actuator_id_last_6))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:actuator_id_first_6
   (&optional __actuator_id_first_6)
   (if __actuator_id_first_6 (setq _actuator_id_first_6 __actuator_id_first_6)) _actuator_id_first_6)
  (:actuator_id_last_6
   (&optional __actuator_id_last_6)
   (if __actuator_id_last_6 (setq _actuator_id_last_6 __actuator_id_last_6)) _actuator_id_last_6)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _actuator_id_first_6
    8
    ;; uint64 _actuator_id_last_6
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _actuator_id_first_6
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _actuator_id_first_6 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _actuator_id_first_6) (= (length (_actuator_id_first_6 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_actuator_id_first_6 . bv) 0) 0) s)
              (write-long (ash (elt (_actuator_id_first_6 . bv) 1) -1) s))
             ((and (class _actuator_id_first_6) (= (length (_actuator_id_first_6 . bv)) 1)) ;; big1
              (write-long (elt (_actuator_id_first_6 . bv) 0) s)
              (write-long (if (>= _actuator_id_first_6 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _actuator_id_first_6 s)(write-long (if (>= _actuator_id_first_6 0) 0 #xffffffff) s)))
     ;; uint64 _actuator_id_last_6
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _actuator_id_last_6 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _actuator_id_last_6) (= (length (_actuator_id_last_6 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_actuator_id_last_6 . bv) 0) 0) s)
              (write-long (ash (elt (_actuator_id_last_6 . bv) 1) -1) s))
             ((and (class _actuator_id_last_6) (= (length (_actuator_id_last_6 . bv)) 1)) ;; big1
              (write-long (elt (_actuator_id_last_6 . bv) 0) s)
              (write-long (if (>= _actuator_id_last_6 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _actuator_id_last_6 s)(write-long (if (>= _actuator_id_last_6 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _actuator_id_first_6
     
#+(or :alpha :irix6 :x86_64)
      (setf _actuator_id_first_6 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _actuator_id_first_6 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _actuator_id_last_6
     
#+(or :alpha :irix6 :x86_64)
      (setf _actuator_id_last_6 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _actuator_id_last_6 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get kartech_linear_actuator_msgs::UniqueDeviceIdRpt :md5sum-) "ea8eb311cb86c91d9fa6aff8968d0ee0")
(setf (get kartech_linear_actuator_msgs::UniqueDeviceIdRpt :datatype-) "kartech_linear_actuator_msgs/UniqueDeviceIdRpt")
(setf (get kartech_linear_actuator_msgs::UniqueDeviceIdRpt :definition-)
      "std_msgs/Header header
uint64 actuator_id_first_6    # The first six bytes of the unique ID of this actuator.
uint64 actuator_id_last_6     # The last six bytes of the unique ID of this actuator.

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



(provide :kartech_linear_actuator_msgs/UniqueDeviceIdRpt "ea8eb311cb86c91d9fa6aff8968d0ee0")


