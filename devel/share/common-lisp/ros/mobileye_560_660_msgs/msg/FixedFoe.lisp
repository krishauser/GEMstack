; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude FixedFoe.msg.html

(cl:defclass <FixedFoe> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fixed_yaw
    :reader fixed_yaw
    :initarg :fixed_yaw
    :type cl:float
    :initform 0.0)
   (fixed_horizon
    :reader fixed_horizon
    :initarg :fixed_horizon
    :type cl:float
    :initform 0.0))
)

(cl:defclass FixedFoe (<FixedFoe>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FixedFoe>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FixedFoe)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<FixedFoe> is deprecated: use mobileye_560_660_msgs-msg:FixedFoe instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FixedFoe>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fixed_yaw-val :lambda-list '(m))
(cl:defmethod fixed_yaw-val ((m <FixedFoe>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:fixed_yaw-val is deprecated.  Use mobileye_560_660_msgs-msg:fixed_yaw instead.")
  (fixed_yaw m))

(cl:ensure-generic-function 'fixed_horizon-val :lambda-list '(m))
(cl:defmethod fixed_horizon-val ((m <FixedFoe>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:fixed_horizon-val is deprecated.  Use mobileye_560_660_msgs-msg:fixed_horizon instead.")
  (fixed_horizon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FixedFoe>) ostream)
  "Serializes a message object of type '<FixedFoe>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fixed_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fixed_horizon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FixedFoe>) istream)
  "Deserializes a message object of type '<FixedFoe>"
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
    (cl:setf (cl:slot-value msg 'fixed_yaw) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fixed_horizon) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FixedFoe>)))
  "Returns string type for a message object of type '<FixedFoe>"
  "mobileye_560_660_msgs/FixedFoe")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FixedFoe)))
  "Returns string type for a message object of type 'FixedFoe"
  "mobileye_560_660_msgs/FixedFoe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FixedFoe>)))
  "Returns md5sum for a message object of type '<FixedFoe>"
  "b4f93d021949d9d8c671473a5bedf703")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FixedFoe)))
  "Returns md5sum for a message object of type 'FixedFoe"
  "b4f93d021949d9d8c671473a5bedf703")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FixedFoe>)))
  "Returns full string definition for message of type '<FixedFoe>"
  (cl:format cl:nil "std_msgs/Header header~%~%float64 fixed_yaw~%float64 fixed_horizon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FixedFoe)))
  "Returns full string definition for message of type 'FixedFoe"
  (cl:format cl:nil "std_msgs/Header header~%~%float64 fixed_yaw~%float64 fixed_horizon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FixedFoe>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FixedFoe>))
  "Converts a ROS message object to a list"
  (cl:list 'FixedFoe
    (cl:cons ':header (header msg))
    (cl:cons ':fixed_yaw (fixed_yaw msg))
    (cl:cons ':fixed_horizon (fixed_horizon msg))
))
