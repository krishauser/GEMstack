; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ZeroingMessageRpt.msg.html

(cl:defclass <ZeroingMessageRpt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (chip_1_voltage
    :reader chip_1_voltage
    :initarg :chip_1_voltage
    :type cl:fixnum
    :initform 0)
   (chip_2_voltage
    :reader chip_2_voltage
    :initarg :chip_2_voltage
    :type cl:fixnum
    :initform 0)
   (chip_error_1
    :reader chip_error_1
    :initarg :chip_error_1
    :type cl:fixnum
    :initform 0)
   (chip_error_2
    :reader chip_error_2
    :initarg :chip_error_2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ZeroingMessageRpt (<ZeroingMessageRpt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ZeroingMessageRpt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ZeroingMessageRpt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ZeroingMessageRpt> is deprecated: use kartech_linear_actuator_msgs-msg:ZeroingMessageRpt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ZeroingMessageRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'chip_1_voltage-val :lambda-list '(m))
(cl:defmethod chip_1_voltage-val ((m <ZeroingMessageRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:chip_1_voltage-val is deprecated.  Use kartech_linear_actuator_msgs-msg:chip_1_voltage instead.")
  (chip_1_voltage m))

(cl:ensure-generic-function 'chip_2_voltage-val :lambda-list '(m))
(cl:defmethod chip_2_voltage-val ((m <ZeroingMessageRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:chip_2_voltage-val is deprecated.  Use kartech_linear_actuator_msgs-msg:chip_2_voltage instead.")
  (chip_2_voltage m))

(cl:ensure-generic-function 'chip_error_1-val :lambda-list '(m))
(cl:defmethod chip_error_1-val ((m <ZeroingMessageRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:chip_error_1-val is deprecated.  Use kartech_linear_actuator_msgs-msg:chip_error_1 instead.")
  (chip_error_1 m))

(cl:ensure-generic-function 'chip_error_2-val :lambda-list '(m))
(cl:defmethod chip_error_2-val ((m <ZeroingMessageRpt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:chip_error_2-val is deprecated.  Use kartech_linear_actuator_msgs-msg:chip_error_2 instead.")
  (chip_error_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ZeroingMessageRpt>) ostream)
  "Serializes a message object of type '<ZeroingMessageRpt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_1_voltage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'chip_1_voltage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_2_voltage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'chip_2_voltage)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_error_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_error_2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ZeroingMessageRpt>) istream)
  "Deserializes a message object of type '<ZeroingMessageRpt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_1_voltage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'chip_1_voltage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_2_voltage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'chip_2_voltage)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_error_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chip_error_2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ZeroingMessageRpt>)))
  "Returns string type for a message object of type '<ZeroingMessageRpt>"
  "kartech_linear_actuator_msgs/ZeroingMessageRpt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ZeroingMessageRpt)))
  "Returns string type for a message object of type 'ZeroingMessageRpt"
  "kartech_linear_actuator_msgs/ZeroingMessageRpt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ZeroingMessageRpt>)))
  "Returns md5sum for a message object of type '<ZeroingMessageRpt>"
  "1be34276909afaf5d9fd5f38a98c32a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ZeroingMessageRpt)))
  "Returns md5sum for a message object of type 'ZeroingMessageRpt"
  "1be34276909afaf5d9fd5f38a98c32a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ZeroingMessageRpt>)))
  "Returns full string definition for message of type '<ZeroingMessageRpt>"
  (cl:format cl:nil "std_msgs/Header header~%uint16 chip_1_voltage~%uint16 chip_2_voltage~%uint8 chip_error_1~%uint8 chip_error_2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ZeroingMessageRpt)))
  "Returns full string definition for message of type 'ZeroingMessageRpt"
  (cl:format cl:nil "std_msgs/Header header~%uint16 chip_1_voltage~%uint16 chip_2_voltage~%uint8 chip_error_1~%uint8 chip_error_2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ZeroingMessageRpt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ZeroingMessageRpt>))
  "Converts a ROS message object to a list"
  (cl:list 'ZeroingMessageRpt
    (cl:cons ':header (header msg))
    (cl:cons ':chip_1_voltage (chip_1_voltage msg))
    (cl:cons ':chip_2_voltage (chip_2_voltage msg))
    (cl:cons ':chip_error_1 (chip_error_1 msg))
    (cl:cons ':chip_error_2 (chip_error_2 msg))
))
