; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ConfigureOutputsPwmFreqCmd.msg.html

(cl:defclass <ConfigureOutputsPwmFreqCmd> (roslisp-msg-protocol:ros-message)
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
   (min_pwm_pct
    :reader min_pwm_pct
    :initarg :min_pwm_pct
    :type cl:fixnum
    :initform 0)
   (max_pwm_pct
    :reader max_pwm_pct
    :initarg :max_pwm_pct
    :type cl:fixnum
    :initform 0)
   (pwm_freq
    :reader pwm_freq
    :initarg :pwm_freq
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ConfigureOutputsPwmFreqCmd (<ConfigureOutputsPwmFreqCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigureOutputsPwmFreqCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigureOutputsPwmFreqCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ConfigureOutputsPwmFreqCmd> is deprecated: use kartech_linear_actuator_msgs-msg:ConfigureOutputsPwmFreqCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConfigureOutputsPwmFreqCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ConfigureOutputsPwmFreqCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'min_pwm_pct-val :lambda-list '(m))
(cl:defmethod min_pwm_pct-val ((m <ConfigureOutputsPwmFreqCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:min_pwm_pct-val is deprecated.  Use kartech_linear_actuator_msgs-msg:min_pwm_pct instead.")
  (min_pwm_pct m))

(cl:ensure-generic-function 'max_pwm_pct-val :lambda-list '(m))
(cl:defmethod max_pwm_pct-val ((m <ConfigureOutputsPwmFreqCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:max_pwm_pct-val is deprecated.  Use kartech_linear_actuator_msgs-msg:max_pwm_pct instead.")
  (max_pwm_pct m))

(cl:ensure-generic-function 'pwm_freq-val :lambda-list '(m))
(cl:defmethod pwm_freq-val ((m <ConfigureOutputsPwmFreqCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:pwm_freq-val is deprecated.  Use kartech_linear_actuator_msgs-msg:pwm_freq instead.")
  (pwm_freq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigureOutputsPwmFreqCmd>) ostream)
  "Serializes a message object of type '<ConfigureOutputsPwmFreqCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min_pwm_pct)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_pwm_pct)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pwm_freq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pwm_freq)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigureOutputsPwmFreqCmd>) istream)
  "Deserializes a message object of type '<ConfigureOutputsPwmFreqCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min_pwm_pct)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_pwm_pct)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pwm_freq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pwm_freq)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigureOutputsPwmFreqCmd>)))
  "Returns string type for a message object of type '<ConfigureOutputsPwmFreqCmd>"
  "kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigureOutputsPwmFreqCmd)))
  "Returns string type for a message object of type 'ConfigureOutputsPwmFreqCmd"
  "kartech_linear_actuator_msgs/ConfigureOutputsPwmFreqCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigureOutputsPwmFreqCmd>)))
  "Returns md5sum for a message object of type '<ConfigureOutputsPwmFreqCmd>"
  "177ba95b80ad87cfd885201c32903f9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigureOutputsPwmFreqCmd)))
  "Returns md5sum for a message object of type 'ConfigureOutputsPwmFreqCmd"
  "177ba95b80ad87cfd885201c32903f9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigureOutputsPwmFreqCmd>)))
  "Returns full string definition for message of type '<ConfigureOutputsPwmFreqCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint8 min_pwm_pct   # The minimum motor drive duty cycle in percent (0-100). Default is 20%.~%uint8 max_pwm_pct   # The maximum motor drive duty cycle in percent (0-100). Default is 90%.~%uint16 pwm_freq     # The frequency of the PWM outputs in Hz. Default is 2000Hz.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigureOutputsPwmFreqCmd)))
  "Returns full string definition for message of type 'ConfigureOutputsPwmFreqCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint8 min_pwm_pct   # The minimum motor drive duty cycle in percent (0-100). Default is 20%.~%uint8 max_pwm_pct   # The maximum motor drive duty cycle in percent (0-100). Default is 90%.~%uint16 pwm_freq     # The frequency of the PWM outputs in Hz. Default is 2000Hz.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigureOutputsPwmFreqCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigureOutputsPwmFreqCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigureOutputsPwmFreqCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':min_pwm_pct (min_pwm_pct msg))
    (cl:cons ':max_pwm_pct (max_pwm_pct msg))
    (cl:cons ':pwm_freq (pwm_freq msg))
))
