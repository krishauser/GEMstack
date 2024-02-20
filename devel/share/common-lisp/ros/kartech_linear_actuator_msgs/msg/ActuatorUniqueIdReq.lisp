; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ActuatorUniqueIdReq.msg.html

(cl:defclass <ActuatorUniqueIdReq> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (confirm
    :reader confirm
    :initarg :confirm
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ActuatorUniqueIdReq (<ActuatorUniqueIdReq>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActuatorUniqueIdReq>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActuatorUniqueIdReq)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ActuatorUniqueIdReq> is deprecated: use kartech_linear_actuator_msgs-msg:ActuatorUniqueIdReq instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ActuatorUniqueIdReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ActuatorUniqueIdReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActuatorUniqueIdReq>) ostream)
  "Serializes a message object of type '<ActuatorUniqueIdReq>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActuatorUniqueIdReq>) istream)
  "Deserializes a message object of type '<ActuatorUniqueIdReq>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActuatorUniqueIdReq>)))
  "Returns string type for a message object of type '<ActuatorUniqueIdReq>"
  "kartech_linear_actuator_msgs/ActuatorUniqueIdReq")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActuatorUniqueIdReq)))
  "Returns string type for a message object of type 'ActuatorUniqueIdReq"
  "kartech_linear_actuator_msgs/ActuatorUniqueIdReq")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActuatorUniqueIdReq>)))
  "Returns md5sum for a message object of type '<ActuatorUniqueIdReq>"
  "d73ec12e18c7d20276159c8210d67b94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActuatorUniqueIdReq)))
  "Returns md5sum for a message object of type 'ActuatorUniqueIdReq"
  "d73ec12e18c7d20276159c8210d67b94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActuatorUniqueIdReq>)))
  "Returns full string definition for message of type '<ActuatorUniqueIdReq>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActuatorUniqueIdReq)))
  "Returns full string definition for message of type 'ActuatorUniqueIdReq"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActuatorUniqueIdReq>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActuatorUniqueIdReq>))
  "Converts a ROS message object to a list"
  (cl:list 'ActuatorUniqueIdReq
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
))
