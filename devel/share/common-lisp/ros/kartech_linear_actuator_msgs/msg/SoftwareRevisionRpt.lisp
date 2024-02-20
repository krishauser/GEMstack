; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude SoftwareRevisionRpt.msg.html

(cl:defclass <SoftwareRevisionRpt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (software_version_0
    :reader software_version_0
    :initarg :software_version_0
    :type cl:fixnum
    :initform 0)
   (software_version_1
    :reader software_version_1
    :initarg :software_version_1
    :type cl:fixnum
    :initform 0)
   (software_version_2
    :reader software_version_2
    :initarg :software_version_2
    :type cl:fixnum
    :initform 0)
   (software_day
    :reader software_day
    :initarg :software_day
    :type cl:fixnum
    :initform 0)
   (software_month_year
    :reader software_month_year
    :initarg :software_month_year
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SoftwareRevisionRpt (<SoftwareRevisionRpt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SoftwareRevisionRpt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SoftwareRevisionRpt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<SoftwareRevisionRpt> is deprecated: use kartech_linear_actuator_msgs-msg:SoftwareRevisionRpt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SoftwareRevisionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'software_version_0-val :lambda-list '(m))
(cl:defmethod software_version_0-val ((m <SoftwareRevisionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:software_version_0-val is deprecated.  Use kartech_linear_actuator_msgs-msg:software_version_0 instead.")
  (software_version_0 m))

(cl:ensure-generic-function 'software_version_1-val :lambda-list '(m))
(cl:defmethod software_version_1-val ((m <SoftwareRevisionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:software_version_1-val is deprecated.  Use kartech_linear_actuator_msgs-msg:software_version_1 instead.")
  (software_version_1 m))

(cl:ensure-generic-function 'software_version_2-val :lambda-list '(m))
(cl:defmethod software_version_2-val ((m <SoftwareRevisionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:software_version_2-val is deprecated.  Use kartech_linear_actuator_msgs-msg:software_version_2 instead.")
  (software_version_2 m))

(cl:ensure-generic-function 'software_day-val :lambda-list '(m))
(cl:defmethod software_day-val ((m <SoftwareRevisionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:software_day-val is deprecated.  Use kartech_linear_actuator_msgs-msg:software_day instead.")
  (software_day m))

(cl:ensure-generic-function 'software_month_year-val :lambda-list '(m))
(cl:defmethod software_month_year-val ((m <SoftwareRevisionRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:software_month_year-val is deprecated.  Use kartech_linear_actuator_msgs-msg:software_month_year instead.")
  (software_month_year m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SoftwareRevisionRpt>) ostream)
  "Serializes a message object of type '<SoftwareRevisionRpt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_version_0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_version_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_version_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_day)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_month_year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'software_month_year)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SoftwareRevisionRpt>) istream)
  "Deserializes a message object of type '<SoftwareRevisionRpt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_version_0)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_version_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_version_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_day)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software_month_year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'software_month_year)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SoftwareRevisionRpt>)))
  "Returns string type for a message object of type '<SoftwareRevisionRpt>"
  "kartech_linear_actuator_msgs/SoftwareRevisionRpt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SoftwareRevisionRpt)))
  "Returns string type for a message object of type 'SoftwareRevisionRpt"
  "kartech_linear_actuator_msgs/SoftwareRevisionRpt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SoftwareRevisionRpt>)))
  "Returns md5sum for a message object of type '<SoftwareRevisionRpt>"
  "5b4e8937bac2714ef707d040a16f320a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SoftwareRevisionRpt)))
  "Returns md5sum for a message object of type 'SoftwareRevisionRpt"
  "5b4e8937bac2714ef707d040a16f320a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SoftwareRevisionRpt>)))
  "Returns full string definition for message of type '<SoftwareRevisionRpt>"
  (cl:format cl:nil "std_msgs/Header header~%uint8 software_version_0~%uint8 software_version_1~%uint8 software_version_2~%uint8 software_day~%uint16 software_month_year~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SoftwareRevisionRpt)))
  "Returns full string definition for message of type 'SoftwareRevisionRpt"
  (cl:format cl:nil "std_msgs/Header header~%uint8 software_version_0~%uint8 software_version_1~%uint8 software_version_2~%uint8 software_day~%uint16 software_month_year~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SoftwareRevisionRpt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SoftwareRevisionRpt>))
  "Converts a ROS message object to a list"
  (cl:list 'SoftwareRevisionRpt
    (cl:cons ':header (header msg))
    (cl:cons ':software_version_0 (software_version_0 msg))
    (cl:cons ':software_version_1 (software_version_1 msg))
    (cl:cons ':software_version_2 (software_version_2 msg))
    (cl:cons ':software_day (software_day msg))
    (cl:cons ':software_month_year (software_month_year msg))
))
