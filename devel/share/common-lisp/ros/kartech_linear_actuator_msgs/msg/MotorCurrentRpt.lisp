; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude MotorCurrentRpt.msg.html

(cl:defclass <MotorCurrentRpt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (motor_current
    :reader motor_current
    :initarg :motor_current
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotorCurrentRpt (<MotorCurrentRpt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCurrentRpt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCurrentRpt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<MotorCurrentRpt> is deprecated: use kartech_linear_actuator_msgs-msg:MotorCurrentRpt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MotorCurrentRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'motor_current-val :lambda-list '(m))
(cl:defmethod motor_current-val ((m <MotorCurrentRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:motor_current-val is deprecated.  Use kartech_linear_actuator_msgs-msg:motor_current instead.")
  (motor_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCurrentRpt>) ostream)
  "Serializes a message object of type '<MotorCurrentRpt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_current)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCurrentRpt>) istream)
  "Deserializes a message object of type '<MotorCurrentRpt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motor_current)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCurrentRpt>)))
  "Returns string type for a message object of type '<MotorCurrentRpt>"
  "kartech_linear_actuator_msgs/MotorCurrentRpt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCurrentRpt)))
  "Returns string type for a message object of type 'MotorCurrentRpt"
  "kartech_linear_actuator_msgs/MotorCurrentRpt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCurrentRpt>)))
  "Returns md5sum for a message object of type '<MotorCurrentRpt>"
  "dee1ca7661def0cd0d6c1f63e8b5e45d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCurrentRpt)))
  "Returns md5sum for a message object of type 'MotorCurrentRpt"
  "dee1ca7661def0cd0d6c1f63e8b5e45d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCurrentRpt>)))
  "Returns full string definition for message of type '<MotorCurrentRpt>"
  (cl:format cl:nil "std_msgs/Header header~%uint16 motor_current    # The current motor current in mA.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCurrentRpt)))
  "Returns full string definition for message of type 'MotorCurrentRpt"
  (cl:format cl:nil "std_msgs/Header header~%uint16 motor_current    # The current motor current in mA.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCurrentRpt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCurrentRpt>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCurrentRpt
    (cl:cons ':header (header msg))
    (cl:cons ':motor_current (motor_current msg))
))
