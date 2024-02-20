; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude PositionReachErrorTimeConfigCmd.msg.html

(cl:defclass <PositionReachErrorTimeConfigCmd> (roslisp-msg-protocol:ros-message)
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
   (position_reach_error_time
    :reader position_reach_error_time
    :initarg :position_reach_error_time
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PositionReachErrorTimeConfigCmd (<PositionReachErrorTimeConfigCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionReachErrorTimeConfigCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionReachErrorTimeConfigCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<PositionReachErrorTimeConfigCmd> is deprecated: use kartech_linear_actuator_msgs-msg:PositionReachErrorTimeConfigCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PositionReachErrorTimeConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <PositionReachErrorTimeConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'position_reach_error_time-val :lambda-list '(m))
(cl:defmethod position_reach_error_time-val ((m <PositionReachErrorTimeConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:position_reach_error_time-val is deprecated.  Use kartech_linear_actuator_msgs-msg:position_reach_error_time instead.")
  (position_reach_error_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionReachErrorTimeConfigCmd>) ostream)
  "Serializes a message object of type '<PositionReachErrorTimeConfigCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position_reach_error_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position_reach_error_time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionReachErrorTimeConfigCmd>) istream)
  "Deserializes a message object of type '<PositionReachErrorTimeConfigCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position_reach_error_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position_reach_error_time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionReachErrorTimeConfigCmd>)))
  "Returns string type for a message object of type '<PositionReachErrorTimeConfigCmd>"
  "kartech_linear_actuator_msgs/PositionReachErrorTimeConfigCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionReachErrorTimeConfigCmd)))
  "Returns string type for a message object of type 'PositionReachErrorTimeConfigCmd"
  "kartech_linear_actuator_msgs/PositionReachErrorTimeConfigCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionReachErrorTimeConfigCmd>)))
  "Returns md5sum for a message object of type '<PositionReachErrorTimeConfigCmd>"
  "2f9859bacc9506f6b0784704786830d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionReachErrorTimeConfigCmd)))
  "Returns md5sum for a message object of type 'PositionReachErrorTimeConfigCmd"
  "2f9859bacc9506f6b0784704786830d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionReachErrorTimeConfigCmd>)))
  "Returns full string definition for message of type '<PositionReachErrorTimeConfigCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 position_reach_error_time # Time that actuator needs to be stalled before cycling motor output in ms. Default is 3000ms.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionReachErrorTimeConfigCmd)))
  "Returns full string definition for message of type 'PositionReachErrorTimeConfigCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 position_reach_error_time # Time that actuator needs to be stalled before cycling motor output in ms. Default is 3000ms.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionReachErrorTimeConfigCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionReachErrorTimeConfigCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionReachErrorTimeConfigCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':position_reach_error_time (position_reach_error_time msg))
))
