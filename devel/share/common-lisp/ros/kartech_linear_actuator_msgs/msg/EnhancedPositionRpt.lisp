; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude EnhancedPositionRpt.msg.html

(cl:defclass <EnhancedPositionRpt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (shaft_extension
    :reader shaft_extension
    :initarg :shaft_extension
    :type cl:float
    :initform 0.0)
   (motor_overload_error
    :reader motor_overload_error
    :initarg :motor_overload_error
    :type cl:boolean
    :initform cl:nil)
   (clutch_overload_error
    :reader clutch_overload_error
    :initarg :clutch_overload_error
    :type cl:boolean
    :initform cl:nil)
   (motor_open_load_error
    :reader motor_open_load_error
    :initarg :motor_open_load_error
    :type cl:boolean
    :initform cl:nil)
   (clutch_open_load_error
    :reader clutch_open_load_error
    :initarg :clutch_open_load_error
    :type cl:boolean
    :initform cl:nil)
   (position_reach_error
    :reader position_reach_error
    :initarg :position_reach_error
    :type cl:boolean
    :initform cl:nil)
   (hardware_warning_1_error
    :reader hardware_warning_1_error
    :initarg :hardware_warning_1_error
    :type cl:boolean
    :initform cl:nil)
   (hardware_warning_2_error
    :reader hardware_warning_2_error
    :initarg :hardware_warning_2_error
    :type cl:boolean
    :initform cl:nil)
   (motor_current
    :reader motor_current
    :initarg :motor_current
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EnhancedPositionRpt (<EnhancedPositionRpt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnhancedPositionRpt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnhancedPositionRpt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<EnhancedPositionRpt> is deprecated: use kartech_linear_actuator_msgs-msg:EnhancedPositionRpt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'shaft_extension-val :lambda-list '(m))
(cl:defmethod shaft_extension-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:shaft_extension-val is deprecated.  Use kartech_linear_actuator_msgs-msg:shaft_extension instead.")
  (shaft_extension m))

(cl:ensure-generic-function 'motor_overload_error-val :lambda-list '(m))
(cl:defmethod motor_overload_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:motor_overload_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:motor_overload_error instead.")
  (motor_overload_error m))

(cl:ensure-generic-function 'clutch_overload_error-val :lambda-list '(m))
(cl:defmethod clutch_overload_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:clutch_overload_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:clutch_overload_error instead.")
  (clutch_overload_error m))

(cl:ensure-generic-function 'motor_open_load_error-val :lambda-list '(m))
(cl:defmethod motor_open_load_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:motor_open_load_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:motor_open_load_error instead.")
  (motor_open_load_error m))

(cl:ensure-generic-function 'clutch_open_load_error-val :lambda-list '(m))
(cl:defmethod clutch_open_load_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:clutch_open_load_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:clutch_open_load_error instead.")
  (clutch_open_load_error m))

(cl:ensure-generic-function 'position_reach_error-val :lambda-list '(m))
(cl:defmethod position_reach_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:position_reach_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:position_reach_error instead.")
  (position_reach_error m))

(cl:ensure-generic-function 'hardware_warning_1_error-val :lambda-list '(m))
(cl:defmethod hardware_warning_1_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:hardware_warning_1_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:hardware_warning_1_error instead.")
  (hardware_warning_1_error m))

(cl:ensure-generic-function 'hardware_warning_2_error-val :lambda-list '(m))
(cl:defmethod hardware_warning_2_error-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:hardware_warning_2_error-val is deprecated.  Use kartech_linear_actuator_msgs-msg:hardware_warning_2_error instead.")
  (hardware_warning_2_error m))

(cl:ensure-generic-function 'motor_current-val :lambda-list '(m))
(cl:defmethod motor_current-val ((m <EnhancedPositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:motor_current-val is deprecated.  Use kartech_linear_actuator_msgs-msg:motor_current instead.")
  (motor_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnhancedPositionRpt>) ostream)
  "Serializes a message object of type '<EnhancedPositionRpt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'shaft_extension))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motor_overload_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clutch_overload_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motor_open_load_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clutch_open_load_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'position_reach_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hardware_warning_1_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hardware_warning_2_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_current)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnhancedPositionRpt>) istream)
  "Deserializes a message object of type '<EnhancedPositionRpt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shaft_extension) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'motor_overload_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clutch_overload_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'motor_open_load_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clutch_open_load_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'position_reach_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hardware_warning_1_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hardware_warning_2_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_current)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnhancedPositionRpt>)))
  "Returns string type for a message object of type '<EnhancedPositionRpt>"
  "kartech_linear_actuator_msgs/EnhancedPositionRpt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnhancedPositionRpt)))
  "Returns string type for a message object of type 'EnhancedPositionRpt"
  "kartech_linear_actuator_msgs/EnhancedPositionRpt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnhancedPositionRpt>)))
  "Returns md5sum for a message object of type '<EnhancedPositionRpt>"
  "b5d14804230789155d91f65364c956fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnhancedPositionRpt)))
  "Returns md5sum for a message object of type 'EnhancedPositionRpt"
  "b5d14804230789155d91f65364c956fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnhancedPositionRpt>)))
  "Returns full string definition for message of type '<EnhancedPositionRpt>"
  (cl:format cl:nil "std_msgs/Header header~%float64 shaft_extension     # The current shaft position in 0.001\" increments.~%bool motor_overload_error~%bool clutch_overload_error~%bool motor_open_load_error~%bool clutch_open_load_error~%bool position_reach_error~%bool hardware_warning_1_error~%bool hardware_warning_2_error~%uint16 motor_current        # The current motor current in mA.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnhancedPositionRpt)))
  "Returns full string definition for message of type 'EnhancedPositionRpt"
  (cl:format cl:nil "std_msgs/Header header~%float64 shaft_extension     # The current shaft position in 0.001\" increments.~%bool motor_overload_error~%bool clutch_overload_error~%bool motor_open_load_error~%bool clutch_open_load_error~%bool position_reach_error~%bool hardware_warning_1_error~%bool hardware_warning_2_error~%uint16 motor_current        # The current motor current in mA.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnhancedPositionRpt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     1
     1
     1
     1
     1
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnhancedPositionRpt>))
  "Converts a ROS message object to a list"
  (cl:list 'EnhancedPositionRpt
    (cl:cons ':header (header msg))
    (cl:cons ':shaft_extension (shaft_extension msg))
    (cl:cons ':motor_overload_error (motor_overload_error msg))
    (cl:cons ':clutch_overload_error (clutch_overload_error msg))
    (cl:cons ':motor_open_load_error (motor_open_load_error msg))
    (cl:cons ':clutch_open_load_error (clutch_open_load_error msg))
    (cl:cons ':position_reach_error (position_reach_error msg))
    (cl:cons ':hardware_warning_1_error (hardware_warning_1_error msg))
    (cl:cons ':hardware_warning_2_error (hardware_warning_2_error msg))
    (cl:cons ':motor_current (motor_current msg))
))
