; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude PositionCmd.msg.html

(cl:defclass <PositionCmd> (roslisp-msg-protocol:ros-message)
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
   (auto_reply
    :reader auto_reply
    :initarg :auto_reply
    :type cl:boolean
    :initform cl:nil)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (clutch_enable
    :reader clutch_enable
    :initarg :clutch_enable
    :type cl:boolean
    :initform cl:nil)
   (motor_enable
    :reader motor_enable
    :initarg :motor_enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PositionCmd (<PositionCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<PositionCmd> is deprecated: use kartech_linear_actuator_msgs-msg:PositionCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PositionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <PositionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'auto_reply-val :lambda-list '(m))
(cl:defmethod auto_reply-val ((m <PositionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:auto_reply-val is deprecated.  Use kartech_linear_actuator_msgs-msg:auto_reply instead.")
  (auto_reply m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <PositionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:position-val is deprecated.  Use kartech_linear_actuator_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'clutch_enable-val :lambda-list '(m))
(cl:defmethod clutch_enable-val ((m <PositionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:clutch_enable-val is deprecated.  Use kartech_linear_actuator_msgs-msg:clutch_enable instead.")
  (clutch_enable m))

(cl:ensure-generic-function 'motor_enable-val :lambda-list '(m))
(cl:defmethod motor_enable-val ((m <PositionCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:motor_enable-val is deprecated.  Use kartech_linear_actuator_msgs-msg:motor_enable instead.")
  (motor_enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionCmd>) ostream)
  "Serializes a message object of type '<PositionCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auto_reply) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clutch_enable) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motor_enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionCmd>) istream)
  "Deserializes a message object of type '<PositionCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'auto_reply) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'clutch_enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'motor_enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionCmd>)))
  "Returns string type for a message object of type '<PositionCmd>"
  "kartech_linear_actuator_msgs/PositionCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionCmd)))
  "Returns string type for a message object of type 'PositionCmd"
  "kartech_linear_actuator_msgs/PositionCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionCmd>)))
  "Returns md5sum for a message object of type '<PositionCmd>"
  "ac9ab77927289195f06ee9b42fabeac2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionCmd)))
  "Returns md5sum for a message object of type 'PositionCmd"
  "ac9ab77927289195f06ee9b42fabeac2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionCmd>)))
  "Returns full string definition for message of type '<PositionCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%bool auto_reply~%float64 position    # Position in 0.001\" increments.~%bool clutch_enable  # Disables (false) or enables (true) the built-in clutch after the position has been reached.~%bool motor_enable   # Disables (false) or enables (true) the motor after the position has been reached.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionCmd)))
  "Returns full string definition for message of type 'PositionCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%bool auto_reply~%float64 position    # Position in 0.001\" increments.~%bool clutch_enable  # Disables (false) or enables (true) the built-in clutch after the position has been reached.~%bool motor_enable   # Disables (false) or enables (true) the motor after the position has been reached.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':auto_reply (auto_reply msg))
    (cl:cons ':position (position msg))
    (cl:cons ':clutch_enable (clutch_enable msg))
    (cl:cons ':motor_enable (motor_enable msg))
))
