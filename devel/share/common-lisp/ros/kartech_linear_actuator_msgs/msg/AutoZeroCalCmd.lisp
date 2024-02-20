; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude AutoZeroCalCmd.msg.html

(cl:defclass <AutoZeroCalCmd> (roslisp-msg-protocol:ros-message)
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
    :initform cl:nil))
)

(cl:defclass AutoZeroCalCmd (<AutoZeroCalCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AutoZeroCalCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AutoZeroCalCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<AutoZeroCalCmd> is deprecated: use kartech_linear_actuator_msgs-msg:AutoZeroCalCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AutoZeroCalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <AutoZeroCalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'auto_reply-val :lambda-list '(m))
(cl:defmethod auto_reply-val ((m <AutoZeroCalCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:auto_reply-val is deprecated.  Use kartech_linear_actuator_msgs-msg:auto_reply instead.")
  (auto_reply m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AutoZeroCalCmd>) ostream)
  "Serializes a message object of type '<AutoZeroCalCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auto_reply) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AutoZeroCalCmd>) istream)
  "Deserializes a message object of type '<AutoZeroCalCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'auto_reply) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AutoZeroCalCmd>)))
  "Returns string type for a message object of type '<AutoZeroCalCmd>"
  "kartech_linear_actuator_msgs/AutoZeroCalCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AutoZeroCalCmd)))
  "Returns string type for a message object of type 'AutoZeroCalCmd"
  "kartech_linear_actuator_msgs/AutoZeroCalCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AutoZeroCalCmd>)))
  "Returns md5sum for a message object of type '<AutoZeroCalCmd>"
  "a99ec518a235924191aab43dcf6c1e53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AutoZeroCalCmd)))
  "Returns md5sum for a message object of type 'AutoZeroCalCmd"
  "a99ec518a235924191aab43dcf6c1e53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AutoZeroCalCmd>)))
  "Returns full string definition for message of type '<AutoZeroCalCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%bool auto_reply~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AutoZeroCalCmd)))
  "Returns full string definition for message of type 'AutoZeroCalCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%bool auto_reply~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AutoZeroCalCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AutoZeroCalCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'AutoZeroCalCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':auto_reply (auto_reply msg))
))
