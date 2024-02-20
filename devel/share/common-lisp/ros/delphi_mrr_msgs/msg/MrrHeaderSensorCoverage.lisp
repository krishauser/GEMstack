; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrHeaderSensorCoverage.msg.html

(cl:defclass <MrrHeaderSensorCoverage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_sensor_fov_hor
    :reader can_sensor_fov_hor
    :initarg :can_sensor_fov_hor
    :type cl:fixnum
    :initform 0)
   (can_doppler_coverage
    :reader can_doppler_coverage
    :initarg :can_doppler_coverage
    :type cl:fixnum
    :initform 0)
   (can_range_coverage
    :reader can_range_coverage
    :initarg :can_range_coverage
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrHeaderSensorCoverage (<MrrHeaderSensorCoverage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrHeaderSensorCoverage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrHeaderSensorCoverage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrHeaderSensorCoverage> is deprecated: use delphi_mrr_msgs-msg:MrrHeaderSensorCoverage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrHeaderSensorCoverage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_sensor_fov_hor-val :lambda-list '(m))
(cl:defmethod can_sensor_fov_hor-val ((m <MrrHeaderSensorCoverage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensor_fov_hor-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensor_fov_hor instead.")
  (can_sensor_fov_hor m))

(cl:ensure-generic-function 'can_doppler_coverage-val :lambda-list '(m))
(cl:defmethod can_doppler_coverage-val ((m <MrrHeaderSensorCoverage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_doppler_coverage-val is deprecated.  Use delphi_mrr_msgs-msg:can_doppler_coverage instead.")
  (can_doppler_coverage m))

(cl:ensure-generic-function 'can_range_coverage-val :lambda-list '(m))
(cl:defmethod can_range_coverage-val ((m <MrrHeaderSensorCoverage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_range_coverage-val is deprecated.  Use delphi_mrr_msgs-msg:can_range_coverage instead.")
  (can_range_coverage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrHeaderSensorCoverage>) ostream)
  "Serializes a message object of type '<MrrHeaderSensorCoverage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sensor_fov_hor)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'can_doppler_coverage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_range_coverage)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrHeaderSensorCoverage>) istream)
  "Deserializes a message object of type '<MrrHeaderSensorCoverage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sensor_fov_hor)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'can_doppler_coverage) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_range_coverage)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrHeaderSensorCoverage>)))
  "Returns string type for a message object of type '<MrrHeaderSensorCoverage>"
  "delphi_mrr_msgs/MrrHeaderSensorCoverage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrHeaderSensorCoverage)))
  "Returns string type for a message object of type 'MrrHeaderSensorCoverage"
  "delphi_mrr_msgs/MrrHeaderSensorCoverage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrHeaderSensorCoverage>)))
  "Returns md5sum for a message object of type '<MrrHeaderSensorCoverage>"
  "138445a2498338a847056e570b23da55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrHeaderSensorCoverage)))
  "Returns md5sum for a message object of type 'MrrHeaderSensorCoverage"
  "138445a2498338a847056e570b23da55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrHeaderSensorCoverage>)))
  "Returns full string definition for message of type '<MrrHeaderSensorCoverage>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 can_sensor_fov_hor~%int8  can_doppler_coverage~%uint8 can_range_coverage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrHeaderSensorCoverage)))
  "Returns full string definition for message of type 'MrrHeaderSensorCoverage"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 can_sensor_fov_hor~%int8  can_doppler_coverage~%uint8 can_range_coverage~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrHeaderSensorCoverage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrHeaderSensorCoverage>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrHeaderSensorCoverage
    (cl:cons ':header (header msg))
    (cl:cons ':can_sensor_fov_hor (can_sensor_fov_hor msg))
    (cl:cons ':can_doppler_coverage (can_doppler_coverage msg))
    (cl:cons ':can_range_coverage (can_range_coverage msg))
))
