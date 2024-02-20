; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude UniqueDeviceIdRpt.msg.html

(cl:defclass <UniqueDeviceIdRpt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (actuator_id_first_6
    :reader actuator_id_first_6
    :initarg :actuator_id_first_6
    :type cl:integer
    :initform 0)
   (actuator_id_last_6
    :reader actuator_id_last_6
    :initarg :actuator_id_last_6
    :type cl:integer
    :initform 0))
)

(cl:defclass UniqueDeviceIdRpt (<UniqueDeviceIdRpt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UniqueDeviceIdRpt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UniqueDeviceIdRpt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<UniqueDeviceIdRpt> is deprecated: use kartech_linear_actuator_msgs-msg:UniqueDeviceIdRpt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UniqueDeviceIdRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'actuator_id_first_6-val :lambda-list '(m))
(cl:defmethod actuator_id_first_6-val ((m <UniqueDeviceIdRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:actuator_id_first_6-val is deprecated.  Use kartech_linear_actuator_msgs-msg:actuator_id_first_6 instead.")
  (actuator_id_first_6 m))

(cl:ensure-generic-function 'actuator_id_last_6-val :lambda-list '(m))
(cl:defmethod actuator_id_last_6-val ((m <UniqueDeviceIdRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:actuator_id_last_6-val is deprecated.  Use kartech_linear_actuator_msgs-msg:actuator_id_last_6 instead.")
  (actuator_id_last_6 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UniqueDeviceIdRpt>) ostream)
  "Serializes a message object of type '<UniqueDeviceIdRpt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'actuator_id_first_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'actuator_id_last_6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'actuator_id_last_6)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UniqueDeviceIdRpt>) istream)
  "Deserializes a message object of type '<UniqueDeviceIdRpt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'actuator_id_first_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'actuator_id_last_6)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UniqueDeviceIdRpt>)))
  "Returns string type for a message object of type '<UniqueDeviceIdRpt>"
  "kartech_linear_actuator_msgs/UniqueDeviceIdRpt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UniqueDeviceIdRpt)))
  "Returns string type for a message object of type 'UniqueDeviceIdRpt"
  "kartech_linear_actuator_msgs/UniqueDeviceIdRpt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UniqueDeviceIdRpt>)))
  "Returns md5sum for a message object of type '<UniqueDeviceIdRpt>"
  "ea8eb311cb86c91d9fa6aff8968d0ee0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UniqueDeviceIdRpt)))
  "Returns md5sum for a message object of type 'UniqueDeviceIdRpt"
  "ea8eb311cb86c91d9fa6aff8968d0ee0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UniqueDeviceIdRpt>)))
  "Returns full string definition for message of type '<UniqueDeviceIdRpt>"
  (cl:format cl:nil "std_msgs/Header header~%uint64 actuator_id_first_6    # The first six bytes of the unique ID of this actuator.~%uint64 actuator_id_last_6     # The last six bytes of the unique ID of this actuator.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UniqueDeviceIdRpt)))
  "Returns full string definition for message of type 'UniqueDeviceIdRpt"
  (cl:format cl:nil "std_msgs/Header header~%uint64 actuator_id_first_6    # The first six bytes of the unique ID of this actuator.~%uint64 actuator_id_last_6     # The last six bytes of the unique ID of this actuator.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UniqueDeviceIdRpt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UniqueDeviceIdRpt>))
  "Converts a ROS message object to a list"
  (cl:list 'UniqueDeviceIdRpt
    (cl:cons ':header (header msg))
    (cl:cons ':actuator_id_first_6 (actuator_id_first_6 msg))
    (cl:cons ':actuator_id_last_6 (actuator_id_last_6 msg))
))
