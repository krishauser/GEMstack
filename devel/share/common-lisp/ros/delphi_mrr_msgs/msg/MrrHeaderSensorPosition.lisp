; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrHeaderSensorPosition.msg.html

(cl:defclass <MrrHeaderSensorPosition> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_sensor_polarity
    :reader can_sensor_polarity
    :initarg :can_sensor_polarity
    :type cl:boolean
    :initform cl:nil)
   (can_sensor_lat_offset
    :reader can_sensor_lat_offset
    :initarg :can_sensor_lat_offset
    :type cl:float
    :initform 0.0)
   (can_sensor_long_offset
    :reader can_sensor_long_offset
    :initarg :can_sensor_long_offset
    :type cl:float
    :initform 0.0)
   (can_sensor_hangle_offset
    :reader can_sensor_hangle_offset
    :initarg :can_sensor_hangle_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass MrrHeaderSensorPosition (<MrrHeaderSensorPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrHeaderSensorPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrHeaderSensorPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrHeaderSensorPosition> is deprecated: use delphi_mrr_msgs-msg:MrrHeaderSensorPosition instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrHeaderSensorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_sensor_polarity-val :lambda-list '(m))
(cl:defmethod can_sensor_polarity-val ((m <MrrHeaderSensorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensor_polarity-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensor_polarity instead.")
  (can_sensor_polarity m))

(cl:ensure-generic-function 'can_sensor_lat_offset-val :lambda-list '(m))
(cl:defmethod can_sensor_lat_offset-val ((m <MrrHeaderSensorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensor_lat_offset-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensor_lat_offset instead.")
  (can_sensor_lat_offset m))

(cl:ensure-generic-function 'can_sensor_long_offset-val :lambda-list '(m))
(cl:defmethod can_sensor_long_offset-val ((m <MrrHeaderSensorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensor_long_offset-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensor_long_offset instead.")
  (can_sensor_long_offset m))

(cl:ensure-generic-function 'can_sensor_hangle_offset-val :lambda-list '(m))
(cl:defmethod can_sensor_hangle_offset-val ((m <MrrHeaderSensorPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensor_hangle_offset-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensor_hangle_offset instead.")
  (can_sensor_hangle_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrHeaderSensorPosition>) ostream)
  "Serializes a message object of type '<MrrHeaderSensorPosition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_sensor_polarity) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_sensor_lat_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_sensor_long_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_sensor_hangle_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrHeaderSensorPosition>) istream)
  "Deserializes a message object of type '<MrrHeaderSensorPosition>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'can_sensor_polarity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_sensor_lat_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_sensor_long_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_sensor_hangle_offset) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrHeaderSensorPosition>)))
  "Returns string type for a message object of type '<MrrHeaderSensorPosition>"
  "delphi_mrr_msgs/MrrHeaderSensorPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrHeaderSensorPosition)))
  "Returns string type for a message object of type 'MrrHeaderSensorPosition"
  "delphi_mrr_msgs/MrrHeaderSensorPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrHeaderSensorPosition>)))
  "Returns md5sum for a message object of type '<MrrHeaderSensorPosition>"
  "1f8dc3bc1765d0520979335ee0f8d03a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrHeaderSensorPosition)))
  "Returns md5sum for a message object of type 'MrrHeaderSensorPosition"
  "1f8dc3bc1765d0520979335ee0f8d03a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrHeaderSensorPosition>)))
  "Returns full string definition for message of type '<MrrHeaderSensorPosition>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool    can_sensor_polarity~%float32 can_sensor_lat_offset~%float32 can_sensor_long_offset~%float32 can_sensor_hangle_offset~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrHeaderSensorPosition)))
  "Returns full string definition for message of type 'MrrHeaderSensorPosition"
  (cl:format cl:nil "std_msgs/Header header~%~%bool    can_sensor_polarity~%float32 can_sensor_lat_offset~%float32 can_sensor_long_offset~%float32 can_sensor_hangle_offset~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrHeaderSensorPosition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrHeaderSensorPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrHeaderSensorPosition
    (cl:cons ':header (header msg))
    (cl:cons ':can_sensor_polarity (can_sensor_polarity msg))
    (cl:cons ':can_sensor_lat_offset (can_sensor_lat_offset msg))
    (cl:cons ':can_sensor_long_offset (can_sensor_long_offset msg))
    (cl:cons ':can_sensor_hangle_offset (can_sensor_hangle_offset msg))
))
