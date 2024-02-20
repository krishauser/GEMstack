; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude ClockSteering.msg.html

(cl:defclass <ClockSteering> (roslisp-msg-protocol:ros-message)
  ((source
    :reader source
    :initarg :source
    :type cl:string
    :initform "")
   (steering_state
    :reader steering_state
    :initarg :steering_state
    :type cl:string
    :initform "")
   (period
    :reader period
    :initarg :period
    :type cl:integer
    :initform 0)
   (pulse_width
    :reader pulse_width
    :initarg :pulse_width
    :type cl:float
    :initform 0.0)
   (bandwidth
    :reader bandwidth
    :initarg :bandwidth
    :type cl:float
    :initform 0.0)
   (slope
    :reader slope
    :initarg :slope
    :type cl:float
    :initform 0.0)
   (offset
    :reader offset
    :initarg :offset
    :type cl:float
    :initform 0.0)
   (drift_rate
    :reader drift_rate
    :initarg :drift_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass ClockSteering (<ClockSteering>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ClockSteering>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ClockSteering)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<ClockSteering> is deprecated: use novatel_gps_msgs-msg:ClockSteering instead.")))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:source-val is deprecated.  Use novatel_gps_msgs-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'steering_state-val :lambda-list '(m))
(cl:defmethod steering_state-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:steering_state-val is deprecated.  Use novatel_gps_msgs-msg:steering_state instead.")
  (steering_state m))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:period-val is deprecated.  Use novatel_gps_msgs-msg:period instead.")
  (period m))

(cl:ensure-generic-function 'pulse_width-val :lambda-list '(m))
(cl:defmethod pulse_width-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:pulse_width-val is deprecated.  Use novatel_gps_msgs-msg:pulse_width instead.")
  (pulse_width m))

(cl:ensure-generic-function 'bandwidth-val :lambda-list '(m))
(cl:defmethod bandwidth-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:bandwidth-val is deprecated.  Use novatel_gps_msgs-msg:bandwidth instead.")
  (bandwidth m))

(cl:ensure-generic-function 'slope-val :lambda-list '(m))
(cl:defmethod slope-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:slope-val is deprecated.  Use novatel_gps_msgs-msg:slope instead.")
  (slope m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:offset-val is deprecated.  Use novatel_gps_msgs-msg:offset instead.")
  (offset m))

(cl:ensure-generic-function 'drift_rate-val :lambda-list '(m))
(cl:defmethod drift_rate-val ((m <ClockSteering>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:drift_rate-val is deprecated.  Use novatel_gps_msgs-msg:drift_rate instead.")
  (drift_rate m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ClockSteering>)))
    "Constants for message type '<ClockSteering>"
  '((:INTERNAL_SOURCE . 0)
    (:EXTERNAL_SOURCE . 1)
    (:FIRST_ORDER_STEERING_STATE . 0)
    (:SECOND_ORDER_STEERING_STATE . 1)
    (:CALIBRATE_HIGH_STEERING_STATE . 2)
    (:CALIBRATE_LOW_STEERING_STATE . 3)
    (:CALIBRATE_CENTER_STEERING_STATE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ClockSteering)))
    "Constants for message type 'ClockSteering"
  '((:INTERNAL_SOURCE . 0)
    (:EXTERNAL_SOURCE . 1)
    (:FIRST_ORDER_STEERING_STATE . 0)
    (:SECOND_ORDER_STEERING_STATE . 1)
    (:CALIBRATE_HIGH_STEERING_STATE . 2)
    (:CALIBRATE_LOW_STEERING_STATE . 3)
    (:CALIBRATE_CENTER_STEERING_STATE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ClockSteering>) ostream)
  "Serializes a message object of type '<ClockSteering>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'steering_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'steering_state))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'period)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'period)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pulse_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bandwidth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'drift_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ClockSteering>) istream)
  "Deserializes a message object of type '<ClockSteering>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'steering_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'period)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pulse_width) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bandwidth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'slope) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'drift_rate) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ClockSteering>)))
  "Returns string type for a message object of type '<ClockSteering>"
  "novatel_gps_msgs/ClockSteering")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ClockSteering)))
  "Returns string type for a message object of type 'ClockSteering"
  "novatel_gps_msgs/ClockSteering")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ClockSteering>)))
  "Returns md5sum for a message object of type '<ClockSteering>"
  "03024ea60365b742dd5e56411830735e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ClockSteering)))
  "Returns md5sum for a message object of type 'ClockSteering"
  "03024ea60365b742dd5e56411830735e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ClockSteering>)))
  "Returns full string definition for message of type '<ClockSteering>"
  (cl:format cl:nil "# The CLOCKSTEERING log is used to monitor the current state of the clock steering process.~%~%int8 INTERNAL_SOURCE=0~%int8 EXTERNAL_SOURCE=1~%~%int8 FIRST_ORDER_STEERING_STATE=0~%int8 SECOND_ORDER_STEERING_STATE=1~%int8 CALIBRATE_HIGH_STEERING_STATE=2~%int8 CALIBRATE_LOW_STEERING_STATE=3~%int8 CALIBRATE_CENTER_STEERING_STATE=4~%~%# Clock source~%string source~%~%# Steering state~%string steering_state~%~%# Period of the FREQUENCYOUT signal used to control the oscillator~%uint32 period~%~%# Current pulse width of the FREQUENCYOUT signal. ~%float64 pulse_width~%~%# The current band width of the clock steering tracking loop in Hz.~%float64 bandwidth~%~%# The current clock drift change in m/s/bit for a 1 LSB pulse width. ~%float32 slope~%~%# The last valid receiver clock offset computed (m).~%float64 offset~%~%# The last valid receiver clock drift rate received (m/s).~%float64 drift_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ClockSteering)))
  "Returns full string definition for message of type 'ClockSteering"
  (cl:format cl:nil "# The CLOCKSTEERING log is used to monitor the current state of the clock steering process.~%~%int8 INTERNAL_SOURCE=0~%int8 EXTERNAL_SOURCE=1~%~%int8 FIRST_ORDER_STEERING_STATE=0~%int8 SECOND_ORDER_STEERING_STATE=1~%int8 CALIBRATE_HIGH_STEERING_STATE=2~%int8 CALIBRATE_LOW_STEERING_STATE=3~%int8 CALIBRATE_CENTER_STEERING_STATE=4~%~%# Clock source~%string source~%~%# Steering state~%string steering_state~%~%# Period of the FREQUENCYOUT signal used to control the oscillator~%uint32 period~%~%# Current pulse width of the FREQUENCYOUT signal. ~%float64 pulse_width~%~%# The current band width of the clock steering tracking loop in Hz.~%float64 bandwidth~%~%# The current clock drift change in m/s/bit for a 1 LSB pulse width. ~%float32 slope~%~%# The last valid receiver clock offset computed (m).~%float64 offset~%~%# The last valid receiver clock drift rate received (m/s).~%float64 drift_rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ClockSteering>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'source))
     4 (cl:length (cl:slot-value msg 'steering_state))
     4
     8
     8
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ClockSteering>))
  "Converts a ROS message object to a list"
  (cl:list 'ClockSteering
    (cl:cons ':source (source msg))
    (cl:cons ':steering_state (steering_state msg))
    (cl:cons ':period (period msg))
    (cl:cons ':pulse_width (pulse_width msg))
    (cl:cons ':bandwidth (bandwidth msg))
    (cl:cons ':slope (slope msg))
    (cl:cons ':offset (offset msg))
    (cl:cons ':drift_rate (drift_rate msg))
))
