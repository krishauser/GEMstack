; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude Time.msg.html

(cl:defclass <Time> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (clock_status
    :reader clock_status
    :initarg :clock_status
    :type cl:string
    :initform "")
   (offset
    :reader offset
    :initarg :offset
    :type cl:float
    :initform 0.0)
   (offset_std
    :reader offset_std
    :initarg :offset_std
    :type cl:float
    :initform 0.0)
   (utc_offset
    :reader utc_offset
    :initarg :utc_offset
    :type cl:float
    :initform 0.0)
   (utc_year
    :reader utc_year
    :initarg :utc_year
    :type cl:integer
    :initform 0)
   (utc_month
    :reader utc_month
    :initarg :utc_month
    :type cl:fixnum
    :initform 0)
   (utc_day
    :reader utc_day
    :initarg :utc_day
    :type cl:fixnum
    :initform 0)
   (utc_hour
    :reader utc_hour
    :initarg :utc_hour
    :type cl:fixnum
    :initform 0)
   (utc_minute
    :reader utc_minute
    :initarg :utc_minute
    :type cl:fixnum
    :initform 0)
   (utc_millisecond
    :reader utc_millisecond
    :initarg :utc_millisecond
    :type cl:integer
    :initform 0)
   (utc_status
    :reader utc_status
    :initarg :utc_status
    :type cl:string
    :initform ""))
)

(cl:defclass Time (<Time>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Time>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Time)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<Time> is deprecated: use novatel_gps_msgs-msg:Time instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:header-val is deprecated.  Use novatel_gps_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'clock_status-val :lambda-list '(m))
(cl:defmethod clock_status-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:clock_status-val is deprecated.  Use novatel_gps_msgs-msg:clock_status instead.")
  (clock_status m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:offset-val is deprecated.  Use novatel_gps_msgs-msg:offset instead.")
  (offset m))

(cl:ensure-generic-function 'offset_std-val :lambda-list '(m))
(cl:defmethod offset_std-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:offset_std-val is deprecated.  Use novatel_gps_msgs-msg:offset_std instead.")
  (offset_std m))

(cl:ensure-generic-function 'utc_offset-val :lambda-list '(m))
(cl:defmethod utc_offset-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_offset-val is deprecated.  Use novatel_gps_msgs-msg:utc_offset instead.")
  (utc_offset m))

(cl:ensure-generic-function 'utc_year-val :lambda-list '(m))
(cl:defmethod utc_year-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_year-val is deprecated.  Use novatel_gps_msgs-msg:utc_year instead.")
  (utc_year m))

(cl:ensure-generic-function 'utc_month-val :lambda-list '(m))
(cl:defmethod utc_month-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_month-val is deprecated.  Use novatel_gps_msgs-msg:utc_month instead.")
  (utc_month m))

(cl:ensure-generic-function 'utc_day-val :lambda-list '(m))
(cl:defmethod utc_day-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_day-val is deprecated.  Use novatel_gps_msgs-msg:utc_day instead.")
  (utc_day m))

(cl:ensure-generic-function 'utc_hour-val :lambda-list '(m))
(cl:defmethod utc_hour-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_hour-val is deprecated.  Use novatel_gps_msgs-msg:utc_hour instead.")
  (utc_hour m))

(cl:ensure-generic-function 'utc_minute-val :lambda-list '(m))
(cl:defmethod utc_minute-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_minute-val is deprecated.  Use novatel_gps_msgs-msg:utc_minute instead.")
  (utc_minute m))

(cl:ensure-generic-function 'utc_millisecond-val :lambda-list '(m))
(cl:defmethod utc_millisecond-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_millisecond-val is deprecated.  Use novatel_gps_msgs-msg:utc_millisecond instead.")
  (utc_millisecond m))

(cl:ensure-generic-function 'utc_status-val :lambda-list '(m))
(cl:defmethod utc_status-val ((m <Time>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:utc_status-val is deprecated.  Use novatel_gps_msgs-msg:utc_status instead.")
  (utc_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Time>) ostream)
  "Serializes a message object of type '<Time>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'clock_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'clock_status))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'offset_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'utc_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'utc_year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'utc_year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'utc_year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_month)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_day)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_minute)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_millisecond)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'utc_millisecond)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'utc_millisecond)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'utc_millisecond)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'utc_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'utc_status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Time>) istream)
  "Deserializes a message object of type '<Time>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'clock_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'clock_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset_std) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'utc_offset) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'utc_year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'utc_year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'utc_year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_month)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_day)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_minute)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'utc_millisecond)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'utc_millisecond)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'utc_millisecond)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'utc_millisecond)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'utc_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'utc_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Time>)))
  "Returns string type for a message object of type '<Time>"
  "novatel_gps_msgs/Time")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Time)))
  "Returns string type for a message object of type 'Time"
  "novatel_gps_msgs/Time")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Time>)))
  "Returns md5sum for a message object of type '<Time>"
  "65d339585d71de8242304ff93e8a4f1a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Time)))
  "Returns md5sum for a message object of type 'Time"
  "65d339585d71de8242304ff93e8a4f1a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Time>)))
  "Returns full string definition for message of type '<Time>"
  (cl:format cl:nil "# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver~%Header header~%~%# Clock model status~%# see Table 65 on pg 322 of the OEM6 Family Firmware Reference Manual, Rev3~%string clock_status~%~%# Receiver clock offset, in seconds, from GPS reference time. A positive offset~%# implies that the receiver clock is ahead of GPS reference time. To derive~%# GPS reference time, use the following formula:~%# GPS reference time = receiver time - offset~%float64 offset~%~%# Standard deviation of the offset~%float64 offset_std~%~%# The offset of the GPS reference time from UTC time, computed using almanac~%# parameters. UTC time is GPS reference time plus the current UTC offset plus~%# the receiver clock offset:~%# UTC time = GPS reference time + offset + UTC offset~%float64 utc_offset~%~%uint32 utc_year~%uint8 utc_month~%uint8 utc_day~%uint8 utc_hour~%uint8 utc_minute~%uint32 utc_millisecond~%~%string utc_status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Time)))
  "Returns full string definition for message of type 'Time"
  (cl:format cl:nil "# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver~%Header header~%~%# Clock model status~%# see Table 65 on pg 322 of the OEM6 Family Firmware Reference Manual, Rev3~%string clock_status~%~%# Receiver clock offset, in seconds, from GPS reference time. A positive offset~%# implies that the receiver clock is ahead of GPS reference time. To derive~%# GPS reference time, use the following formula:~%# GPS reference time = receiver time - offset~%float64 offset~%~%# Standard deviation of the offset~%float64 offset_std~%~%# The offset of the GPS reference time from UTC time, computed using almanac~%# parameters. UTC time is GPS reference time plus the current UTC offset plus~%# the receiver clock offset:~%# UTC time = GPS reference time + offset + UTC offset~%float64 utc_offset~%~%uint32 utc_year~%uint8 utc_month~%uint8 utc_day~%uint8 utc_hour~%uint8 utc_minute~%uint32 utc_millisecond~%~%string utc_status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Time>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'clock_status))
     8
     8
     8
     4
     1
     1
     1
     1
     4
     4 (cl:length (cl:slot-value msg 'utc_status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Time>))
  "Converts a ROS message object to a list"
  (cl:list 'Time
    (cl:cons ':header (header msg))
    (cl:cons ':clock_status (clock_status msg))
    (cl:cons ':offset (offset msg))
    (cl:cons ':offset_std (offset_std msg))
    (cl:cons ':utc_offset (utc_offset msg))
    (cl:cons ':utc_year (utc_year msg))
    (cl:cons ':utc_month (utc_month msg))
    (cl:cons ':utc_day (utc_day msg))
    (cl:cons ':utc_hour (utc_hour msg))
    (cl:cons ':utc_minute (utc_minute msg))
    (cl:cons ':utc_millisecond (utc_millisecond msg))
    (cl:cons ':utc_status (utc_status msg))
))
