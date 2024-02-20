; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ConfigureOutputsKdFreqDeadbandCmd.msg.html

(cl:defclass <ConfigureOutputsKdFreqDeadbandCmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (confirm
    :reader confirm
    :initarg :confirm
    :type cl:boolean
    :initform cl:nil)
   (kd
    :reader kd
    :initarg :kd
    :type cl:fixnum
    :initform 0)
   (closed_loop_freq
    :reader closed_loop_freq
    :initarg :closed_loop_freq
    :type cl:fixnum
    :initform 0)
   (error_dead_band
    :reader error_dead_band
    :initarg :error_dead_band
    :type cl:float
    :initform 0.0))
)

(cl:defclass ConfigureOutputsKdFreqDeadbandCmd (<ConfigureOutputsKdFreqDeadbandCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigureOutputsKdFreqDeadbandCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigureOutputsKdFreqDeadbandCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ConfigureOutputsKdFreqDeadbandCmd> is deprecated: use kartech_linear_actuator_msgs-msg:ConfigureOutputsKdFreqDeadbandCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConfigureOutputsKdFreqDeadbandCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ConfigureOutputsKdFreqDeadbandCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'kd-val :lambda-list '(m))
(cl:defmethod kd-val ((m <ConfigureOutputsKdFreqDeadbandCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:kd-val is deprecated.  Use kartech_linear_actuator_msgs-msg:kd instead.")
  (kd m))

(cl:ensure-generic-function 'closed_loop_freq-val :lambda-list '(m))
(cl:defmethod closed_loop_freq-val ((m <ConfigureOutputsKdFreqDeadbandCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:closed_loop_freq-val is deprecated.  Use kartech_linear_actuator_msgs-msg:closed_loop_freq instead.")
  (closed_loop_freq m))

(cl:ensure-generic-function 'error_dead_band-val :lambda-list '(m))
(cl:defmethod error_dead_band-val ((m <ConfigureOutputsKdFreqDeadbandCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:error_dead_band-val is deprecated.  Use kartech_linear_actuator_msgs-msg:error_dead_band instead.")
  (error_dead_band m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigureOutputsKdFreqDeadbandCmd>) ostream)
  "Serializes a message object of type '<ConfigureOutputsKdFreqDeadbandCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'kd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'closed_loop_freq)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'error_dead_band))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigureOutputsKdFreqDeadbandCmd>) istream)
  "Deserializes a message object of type '<ConfigureOutputsKdFreqDeadbandCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'kd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'closed_loop_freq)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_dead_band) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigureOutputsKdFreqDeadbandCmd>)))
  "Returns string type for a message object of type '<ConfigureOutputsKdFreqDeadbandCmd>"
  "kartech_linear_actuator_msgs/ConfigureOutputsKdFreqDeadbandCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigureOutputsKdFreqDeadbandCmd)))
  "Returns string type for a message object of type 'ConfigureOutputsKdFreqDeadbandCmd"
  "kartech_linear_actuator_msgs/ConfigureOutputsKdFreqDeadbandCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigureOutputsKdFreqDeadbandCmd>)))
  "Returns md5sum for a message object of type '<ConfigureOutputsKdFreqDeadbandCmd>"
  "865fff7dce2fec39beac32ec4e1f4638")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigureOutputsKdFreqDeadbandCmd)))
  "Returns md5sum for a message object of type 'ConfigureOutputsKdFreqDeadbandCmd"
  "865fff7dce2fec39beac32ec4e1f4638")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigureOutputsKdFreqDeadbandCmd>)))
  "Returns full string definition for message of type '<ConfigureOutputsKdFreqDeadbandCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 kd                   # The differential term of the closed-loop control. Default is 10.~%uint8 closed_loop_freq      # The frequency of closed-loop corrections in Hz. The default is 60Hz.~%float64 error_dead_band     # The size of the dead-band for error correction in units of 0.001\". The default is 0.05\"~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigureOutputsKdFreqDeadbandCmd)))
  "Returns full string definition for message of type 'ConfigureOutputsKdFreqDeadbandCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 kd                   # The differential term of the closed-loop control. Default is 10.~%uint8 closed_loop_freq      # The frequency of closed-loop corrections in Hz. The default is 60Hz.~%float64 error_dead_band     # The size of the dead-band for error correction in units of 0.001\". The default is 0.05\"~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigureOutputsKdFreqDeadbandCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigureOutputsKdFreqDeadbandCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigureOutputsKdFreqDeadbandCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':kd (kd msg))
    (cl:cons ':closed_loop_freq (closed_loop_freq msg))
    (cl:cons ':error_dead_band (error_dead_band msg))
))
