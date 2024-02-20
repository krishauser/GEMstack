; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude TsrVisionOnly.msg.html

(cl:defclass <TsrVisionOnly> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vision_only_sign_type_display1
    :reader vision_only_sign_type_display1
    :initarg :vision_only_sign_type_display1
    :type cl:fixnum
    :initform 0)
   (vision_only_supplementary_sign_type_display1
    :reader vision_only_supplementary_sign_type_display1
    :initarg :vision_only_supplementary_sign_type_display1
    :type cl:fixnum
    :initform 0)
   (vision_only_sign_type_display2
    :reader vision_only_sign_type_display2
    :initarg :vision_only_sign_type_display2
    :type cl:fixnum
    :initform 0)
   (vision_only_supplementary_sign_type_display2
    :reader vision_only_supplementary_sign_type_display2
    :initarg :vision_only_supplementary_sign_type_display2
    :type cl:fixnum
    :initform 0)
   (vision_only_sign_type_display3
    :reader vision_only_sign_type_display3
    :initarg :vision_only_sign_type_display3
    :type cl:fixnum
    :initform 0)
   (vision_only_supplementary_sign_type_display3
    :reader vision_only_supplementary_sign_type_display3
    :initarg :vision_only_supplementary_sign_type_display3
    :type cl:fixnum
    :initform 0)
   (vision_only_sign_type_display4
    :reader vision_only_sign_type_display4
    :initarg :vision_only_sign_type_display4
    :type cl:fixnum
    :initform 0)
   (vision_only_supplementary_sign_type_display4
    :reader vision_only_supplementary_sign_type_display4
    :initarg :vision_only_supplementary_sign_type_display4
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TsrVisionOnly (<TsrVisionOnly>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TsrVisionOnly>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TsrVisionOnly)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<TsrVisionOnly> is deprecated: use mobileye_560_660_msgs-msg:TsrVisionOnly instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vision_only_sign_type_display1-val :lambda-list '(m))
(cl:defmethod vision_only_sign_type_display1-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_sign_type_display1-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_sign_type_display1 instead.")
  (vision_only_sign_type_display1 m))

(cl:ensure-generic-function 'vision_only_supplementary_sign_type_display1-val :lambda-list '(m))
(cl:defmethod vision_only_supplementary_sign_type_display1-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display1-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display1 instead.")
  (vision_only_supplementary_sign_type_display1 m))

(cl:ensure-generic-function 'vision_only_sign_type_display2-val :lambda-list '(m))
(cl:defmethod vision_only_sign_type_display2-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_sign_type_display2-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_sign_type_display2 instead.")
  (vision_only_sign_type_display2 m))

(cl:ensure-generic-function 'vision_only_supplementary_sign_type_display2-val :lambda-list '(m))
(cl:defmethod vision_only_supplementary_sign_type_display2-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display2-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display2 instead.")
  (vision_only_supplementary_sign_type_display2 m))

(cl:ensure-generic-function 'vision_only_sign_type_display3-val :lambda-list '(m))
(cl:defmethod vision_only_sign_type_display3-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_sign_type_display3-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_sign_type_display3 instead.")
  (vision_only_sign_type_display3 m))

(cl:ensure-generic-function 'vision_only_supplementary_sign_type_display3-val :lambda-list '(m))
(cl:defmethod vision_only_supplementary_sign_type_display3-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display3-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display3 instead.")
  (vision_only_supplementary_sign_type_display3 m))

(cl:ensure-generic-function 'vision_only_sign_type_display4-val :lambda-list '(m))
(cl:defmethod vision_only_sign_type_display4-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_sign_type_display4-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_sign_type_display4 instead.")
  (vision_only_sign_type_display4 m))

(cl:ensure-generic-function 'vision_only_supplementary_sign_type_display4-val :lambda-list '(m))
(cl:defmethod vision_only_supplementary_sign_type_display4-val ((m <TsrVisionOnly>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display4-val is deprecated.  Use mobileye_560_660_msgs-msg:vision_only_supplementary_sign_type_display4 instead.")
  (vision_only_supplementary_sign_type_display4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TsrVisionOnly>) ostream)
  "Serializes a message object of type '<TsrVisionOnly>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display4)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TsrVisionOnly>) istream)
  "Deserializes a message object of type '<TsrVisionOnly>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_sign_type_display4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vision_only_supplementary_sign_type_display4)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TsrVisionOnly>)))
  "Returns string type for a message object of type '<TsrVisionOnly>"
  "mobileye_560_660_msgs/TsrVisionOnly")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TsrVisionOnly)))
  "Returns string type for a message object of type 'TsrVisionOnly"
  "mobileye_560_660_msgs/TsrVisionOnly")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TsrVisionOnly>)))
  "Returns md5sum for a message object of type '<TsrVisionOnly>"
  "84f9582e1cda52683c53338cffe795f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TsrVisionOnly)))
  "Returns md5sum for a message object of type 'TsrVisionOnly"
  "84f9582e1cda52683c53338cffe795f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TsrVisionOnly>)))
  "Returns full string definition for message of type '<TsrVisionOnly>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 vision_only_sign_type_display1~%uint8 vision_only_supplementary_sign_type_display1~%uint8 vision_only_sign_type_display2~%uint8 vision_only_supplementary_sign_type_display2~%uint8 vision_only_sign_type_display3~%uint8 vision_only_supplementary_sign_type_display3~%uint8 vision_only_sign_type_display4~%uint8 vision_only_supplementary_sign_type_display4~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TsrVisionOnly)))
  "Returns full string definition for message of type 'TsrVisionOnly"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 vision_only_sign_type_display1~%uint8 vision_only_supplementary_sign_type_display1~%uint8 vision_only_sign_type_display2~%uint8 vision_only_supplementary_sign_type_display2~%uint8 vision_only_sign_type_display3~%uint8 vision_only_supplementary_sign_type_display3~%uint8 vision_only_sign_type_display4~%uint8 vision_only_supplementary_sign_type_display4~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TsrVisionOnly>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TsrVisionOnly>))
  "Converts a ROS message object to a list"
  (cl:list 'TsrVisionOnly
    (cl:cons ':header (header msg))
    (cl:cons ':vision_only_sign_type_display1 (vision_only_sign_type_display1 msg))
    (cl:cons ':vision_only_supplementary_sign_type_display1 (vision_only_supplementary_sign_type_display1 msg))
    (cl:cons ':vision_only_sign_type_display2 (vision_only_sign_type_display2 msg))
    (cl:cons ':vision_only_supplementary_sign_type_display2 (vision_only_supplementary_sign_type_display2 msg))
    (cl:cons ':vision_only_sign_type_display3 (vision_only_sign_type_display3 msg))
    (cl:cons ':vision_only_supplementary_sign_type_display3 (vision_only_supplementary_sign_type_display3 msg))
    (cl:cons ':vision_only_sign_type_display4 (vision_only_sign_type_display4 msg))
    (cl:cons ':vision_only_supplementary_sign_type_display4 (vision_only_supplementary_sign_type_display4 msg))
))
