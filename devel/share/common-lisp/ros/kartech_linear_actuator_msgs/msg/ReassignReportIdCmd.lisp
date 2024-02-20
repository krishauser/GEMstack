; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ReassignReportIdCmd.msg.html

(cl:defclass <ReassignReportIdCmd> (roslisp-msg-protocol:ros-message)
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
   (user_report_id
    :reader user_report_id
    :initarg :user_report_id
    :type cl:integer
    :initform 0)
   (use_user_report_id
    :reader use_user_report_id
    :initarg :use_user_report_id
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReassignReportIdCmd (<ReassignReportIdCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReassignReportIdCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReassignReportIdCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ReassignReportIdCmd> is deprecated: use kartech_linear_actuator_msgs-msg:ReassignReportIdCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReassignReportIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ReassignReportIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'user_report_id-val :lambda-list '(m))
(cl:defmethod user_report_id-val ((m <ReassignReportIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:user_report_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:user_report_id instead.")
  (user_report_id m))

(cl:ensure-generic-function 'use_user_report_id-val :lambda-list '(m))
(cl:defmethod use_user_report_id-val ((m <ReassignReportIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:use_user_report_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:use_user_report_id instead.")
  (use_user_report_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReassignReportIdCmd>) ostream)
  "Serializes a message object of type '<ReassignReportIdCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_report_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_report_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_report_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_report_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_user_report_id) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReassignReportIdCmd>) istream)
  "Deserializes a message object of type '<ReassignReportIdCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_report_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_report_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_report_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_report_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'use_user_report_id) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReassignReportIdCmd>)))
  "Returns string type for a message object of type '<ReassignReportIdCmd>"
  "kartech_linear_actuator_msgs/ReassignReportIdCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReassignReportIdCmd)))
  "Returns string type for a message object of type 'ReassignReportIdCmd"
  "kartech_linear_actuator_msgs/ReassignReportIdCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReassignReportIdCmd>)))
  "Returns md5sum for a message object of type '<ReassignReportIdCmd>"
  "fd2b8bcf6e2eaf63371bf7a3445d195c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReassignReportIdCmd)))
  "Returns md5sum for a message object of type 'ReassignReportIdCmd"
  "fd2b8bcf6e2eaf63371bf7a3445d195c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReassignReportIdCmd>)))
  "Returns full string definition for message of type '<ReassignReportIdCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint32 user_report_id     # The new user report ID to use. Values 0xFFFEXX and 0xFF00XX are reserved. Setting this to 0xFFFFFFFF will only change the use_user_report_id flag.~%bool use_user_report_id   # Whether to use the user-defined report ID (true) or the default report ID (false).~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReassignReportIdCmd)))
  "Returns full string definition for message of type 'ReassignReportIdCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint32 user_report_id     # The new user report ID to use. Values 0xFFFEXX and 0xFF00XX are reserved. Setting this to 0xFFFFFFFF will only change the use_user_report_id flag.~%bool use_user_report_id   # Whether to use the user-defined report ID (true) or the default report ID (false).~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReassignReportIdCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReassignReportIdCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ReassignReportIdCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':user_report_id (user_report_id msg))
    (cl:cons ':use_user_report_id (use_user_report_id msg))
))
