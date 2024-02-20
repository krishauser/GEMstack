; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude NovatelVelocity.msg.html

(cl:defclass <NovatelVelocity> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (novatel_msg_header
    :reader novatel_msg_header
    :initarg :novatel_msg_header
    :type novatel_gps_msgs-msg:NovatelMessageHeader
    :initform (cl:make-instance 'novatel_gps_msgs-msg:NovatelMessageHeader))
   (solution_status
    :reader solution_status
    :initarg :solution_status
    :type cl:string
    :initform "")
   (velocity_type
    :reader velocity_type
    :initarg :velocity_type
    :type cl:string
    :initform "")
   (latency
    :reader latency
    :initarg :latency
    :type cl:float
    :initform 0.0)
   (age
    :reader age
    :initarg :age
    :type cl:float
    :initform 0.0)
   (horizontal_speed
    :reader horizontal_speed
    :initarg :horizontal_speed
    :type cl:float
    :initform 0.0)
   (track_ground
    :reader track_ground
    :initarg :track_ground
    :type cl:float
    :initform 0.0)
   (vertical_speed
    :reader vertical_speed
    :initarg :vertical_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass NovatelVelocity (<NovatelVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NovatelVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NovatelVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<NovatelVelocity> is deprecated: use novatel_gps_msgs-msg:NovatelVelocity instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:header-val is deprecated.  Use novatel_gps_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'novatel_msg_header-val :lambda-list '(m))
(cl:defmethod novatel_msg_header-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:novatel_msg_header-val is deprecated.  Use novatel_gps_msgs-msg:novatel_msg_header instead.")
  (novatel_msg_header m))

(cl:ensure-generic-function 'solution_status-val :lambda-list '(m))
(cl:defmethod solution_status-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:solution_status-val is deprecated.  Use novatel_gps_msgs-msg:solution_status instead.")
  (solution_status m))

(cl:ensure-generic-function 'velocity_type-val :lambda-list '(m))
(cl:defmethod velocity_type-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:velocity_type-val is deprecated.  Use novatel_gps_msgs-msg:velocity_type instead.")
  (velocity_type m))

(cl:ensure-generic-function 'latency-val :lambda-list '(m))
(cl:defmethod latency-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:latency-val is deprecated.  Use novatel_gps_msgs-msg:latency instead.")
  (latency m))

(cl:ensure-generic-function 'age-val :lambda-list '(m))
(cl:defmethod age-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:age-val is deprecated.  Use novatel_gps_msgs-msg:age instead.")
  (age m))

(cl:ensure-generic-function 'horizontal_speed-val :lambda-list '(m))
(cl:defmethod horizontal_speed-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:horizontal_speed-val is deprecated.  Use novatel_gps_msgs-msg:horizontal_speed instead.")
  (horizontal_speed m))

(cl:ensure-generic-function 'track_ground-val :lambda-list '(m))
(cl:defmethod track_ground-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:track_ground-val is deprecated.  Use novatel_gps_msgs-msg:track_ground instead.")
  (track_ground m))

(cl:ensure-generic-function 'vertical_speed-val :lambda-list '(m))
(cl:defmethod vertical_speed-val ((m <NovatelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:vertical_speed-val is deprecated.  Use novatel_gps_msgs-msg:vertical_speed instead.")
  (vertical_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NovatelVelocity>) ostream)
  "Serializes a message object of type '<NovatelVelocity>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'novatel_msg_header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'solution_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'solution_status))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'velocity_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'velocity_type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'age))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'horizontal_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'track_ground))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'vertical_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NovatelVelocity>) istream)
  "Deserializes a message object of type '<NovatelVelocity>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'novatel_msg_header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'solution_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'solution_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'velocity_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'age) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horizontal_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'track_ground) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vertical_speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NovatelVelocity>)))
  "Returns string type for a message object of type '<NovatelVelocity>"
  "novatel_gps_msgs/NovatelVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelVelocity)))
  "Returns string type for a message object of type 'NovatelVelocity"
  "novatel_gps_msgs/NovatelVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NovatelVelocity>)))
  "Returns md5sum for a message object of type '<NovatelVelocity>"
  "a8fb7d9232aaf68f98caa2b4cda2597b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NovatelVelocity)))
  "Returns md5sum for a message object of type 'NovatelVelocity"
  "a8fb7d9232aaf68f98caa2b4cda2597b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NovatelVelocity>)))
  "Returns full string definition for message of type '<NovatelVelocity>"
  (cl:format cl:nil "# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver~%Header header~%~%NovatelMessageHeader novatel_msg_header~%~%string solution_status~%string velocity_type~%~%# A measure of the latency in the velocity time tag in seconds.~%float32 latency~%~%# Differential age in seconds~%float32 age~%~%# Horizontal speed over ground, meters/second~%float64 horizontal_speed~%~%# Direction of motion over ground with respect to true North, degrees~%float64 track_ground~%~%# Vertical speed, where positive values correspond to increasing altitude, meters/second~%float64 vertical_speed~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/NovatelMessageHeader~%# Novatel Message Header~%~%string message_name~%string port~%uint32 sequence_num~%float32 percent_idle_time~%string gps_time_status~%uint32 gps_week_num~%float64 gps_seconds~%~%# Bit       Mask      Description~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%NovatelReceiverStatus receiver_status~%~%# Receiver build number (0-65535)~%uint32 receiver_software_version~%================================================================================~%MSG: novatel_gps_msgs/NovatelReceiverStatus~%# From the original Novatel receiver status message bitfield~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%uint32 original_status_code~%bool error_flag~%bool temperature_flag~%bool voltage_supply_flag~%bool antenna_powered~%bool antenna_is_open~%bool antenna_is_shorted~%bool cpu_overload_flag~%bool com1_buffer_overrun~%bool com2_buffer_overrun~%bool com3_buffer_overrun~%bool usb_buffer_overrun~%bool rf1_agc_flag~%bool rf2_agc_flag~%bool almanac_flag~%bool position_solution_flag~%bool position_fixed_flag~%bool clock_steering_status_enabled~%bool clock_model_flag~%bool oemv_external_oscillator_flag~%bool software_resource_flag~%bool aux1_status_event_flag~%bool aux2_status_event_flag~%bool aux3_status_event_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NovatelVelocity)))
  "Returns full string definition for message of type 'NovatelVelocity"
  (cl:format cl:nil "# Parsed Best Position or Omnistar XP or HP pos data from Novatel OEM6 receiver~%Header header~%~%NovatelMessageHeader novatel_msg_header~%~%string solution_status~%string velocity_type~%~%# A measure of the latency in the velocity time tag in seconds.~%float32 latency~%~%# Differential age in seconds~%float32 age~%~%# Horizontal speed over ground, meters/second~%float64 horizontal_speed~%~%# Direction of motion over ground with respect to true North, degrees~%float64 track_ground~%~%# Vertical speed, where positive values correspond to increasing altitude, meters/second~%float64 vertical_speed~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/NovatelMessageHeader~%# Novatel Message Header~%~%string message_name~%string port~%uint32 sequence_num~%float32 percent_idle_time~%string gps_time_status~%uint32 gps_week_num~%float64 gps_seconds~%~%# Bit       Mask      Description~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%NovatelReceiverStatus receiver_status~%~%# Receiver build number (0-65535)~%uint32 receiver_software_version~%================================================================================~%MSG: novatel_gps_msgs/NovatelReceiverStatus~%# From the original Novatel receiver status message bitfield~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%uint32 original_status_code~%bool error_flag~%bool temperature_flag~%bool voltage_supply_flag~%bool antenna_powered~%bool antenna_is_open~%bool antenna_is_shorted~%bool cpu_overload_flag~%bool com1_buffer_overrun~%bool com2_buffer_overrun~%bool com3_buffer_overrun~%bool usb_buffer_overrun~%bool rf1_agc_flag~%bool rf2_agc_flag~%bool almanac_flag~%bool position_solution_flag~%bool position_fixed_flag~%bool clock_steering_status_enabled~%bool clock_model_flag~%bool oemv_external_oscillator_flag~%bool software_resource_flag~%bool aux1_status_event_flag~%bool aux2_status_event_flag~%bool aux3_status_event_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NovatelVelocity>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'novatel_msg_header))
     4 (cl:length (cl:slot-value msg 'solution_status))
     4 (cl:length (cl:slot-value msg 'velocity_type))
     4
     4
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NovatelVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'NovatelVelocity
    (cl:cons ':header (header msg))
    (cl:cons ':novatel_msg_header (novatel_msg_header msg))
    (cl:cons ':solution_status (solution_status msg))
    (cl:cons ':velocity_type (velocity_type msg))
    (cl:cons ':latency (latency msg))
    (cl:cons ':age (age msg))
    (cl:cons ':horizontal_speed (horizontal_speed msg))
    (cl:cons ':track_ground (track_ground msg))
    (cl:cons ':vertical_speed (vertical_speed msg))
))
