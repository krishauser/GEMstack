; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude SoftwareVersionReq.msg.html

(cl:defclass <SoftwareVersionReq> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SoftwareVersionReq (<SoftwareVersionReq>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SoftwareVersionReq>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SoftwareVersionReq)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<SoftwareVersionReq> is deprecated: use kartech_linear_actuator_msgs-msg:SoftwareVersionReq instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SoftwareVersionReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <SoftwareVersionReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SoftwareVersionReq>) ostream)
  "Serializes a message object of type '<SoftwareVersionReq>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SoftwareVersionReq>) istream)
  "Deserializes a message object of type '<SoftwareVersionReq>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SoftwareVersionReq>)))
  "Returns string type for a message object of type '<SoftwareVersionReq>"
  "kartech_linear_actuator_msgs/SoftwareVersionReq")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SoftwareVersionReq)))
  "Returns string type for a message object of type 'SoftwareVersionReq"
  "kartech_linear_actuator_msgs/SoftwareVersionReq")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SoftwareVersionReq>)))
  "Returns md5sum for a message object of type '<SoftwareVersionReq>"
  "d73ec12e18c7d20276159c8210d67b94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SoftwareVersionReq)))
  "Returns md5sum for a message object of type 'SoftwareVersionReq"
  "d73ec12e18c7d20276159c8210d67b94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SoftwareVersionReq>)))
  "Returns full string definition for message of type '<SoftwareVersionReq>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SoftwareVersionReq)))
  "Returns full string definition for message of type 'SoftwareVersionReq"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SoftwareVersionReq>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SoftwareVersionReq>))
  "Converts a ROS message object to a list"
  (cl:list 'SoftwareVersionReq
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
))
