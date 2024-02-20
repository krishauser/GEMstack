; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude PositionRpt.msg.html

(cl:defclass <PositionRpt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (shaft_extension
    :reader shaft_extension
    :initarg :shaft_extension
    :type cl:float
    :initform 0.0))
)

(cl:defclass PositionRpt (<PositionRpt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PositionRpt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PositionRpt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<PositionRpt> is deprecated: use kartech_linear_actuator_msgs-msg:PositionRpt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'shaft_extension-val :lambda-list '(m))
(cl:defmethod shaft_extension-val ((m <PositionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:shaft_extension-val is deprecated.  Use kartech_linear_actuator_msgs-msg:shaft_extension instead.")
  (shaft_extension m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PositionRpt>) ostream)
  "Serializes a message object of type '<PositionRpt>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PositionRpt>) istream)
  "Deserializes a message object of type '<PositionRpt>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PositionRpt>)))
  "Returns string type for a message object of type '<PositionRpt>"
  "kartech_linear_actuator_msgs/PositionRpt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PositionRpt)))
  "Returns string type for a message object of type 'PositionRpt"
  "kartech_linear_actuator_msgs/PositionRpt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PositionRpt>)))
  "Returns md5sum for a message object of type '<PositionRpt>"
  "bc025045a75f313163c8d3ca0cedadf9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PositionRpt)))
  "Returns md5sum for a message object of type 'PositionRpt"
  "bc025045a75f313163c8d3ca0cedadf9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PositionRpt>)))
  "Returns full string definition for message of type '<PositionRpt>"
  (cl:format cl:nil "std_msgs/Header header~%float64 shaft_extension     # The current shaft position in 0.001\" increments.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PositionRpt)))
  "Returns full string definition for message of type 'PositionRpt"
  (cl:format cl:nil "std_msgs/Header header~%float64 shaft_extension     # The current shaft position in 0.001\" increments.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PositionRpt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PositionRpt>))
  "Converts a ROS message object to a list"
  (cl:list 'PositionRpt
    (cl:cons ':header (header msg))
    (cl:cons ':shaft_extension (shaft_extension msg))
))
