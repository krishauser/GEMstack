; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrHeaderTimestamps.msg.html

(cl:defclass <MrrHeaderTimestamps> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_det_time_since_meas
    :reader can_det_time_since_meas
    :initarg :can_det_time_since_meas
    :type cl:float
    :initform 0.0)
   (can_sensor_time_stamp
    :reader can_sensor_time_stamp
    :initarg :can_sensor_time_stamp
    :type cl:float
    :initform 0.0))
)

(cl:defclass MrrHeaderTimestamps (<MrrHeaderTimestamps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrHeaderTimestamps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrHeaderTimestamps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrHeaderTimestamps> is deprecated: use delphi_mrr_msgs-msg:MrrHeaderTimestamps instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrHeaderTimestamps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_det_time_since_meas-val :lambda-list '(m))
(cl:defmethod can_det_time_since_meas-val ((m <MrrHeaderTimestamps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_det_time_since_meas-val is deprecated.  Use delphi_mrr_msgs-msg:can_det_time_since_meas instead.")
  (can_det_time_since_meas m))

(cl:ensure-generic-function 'can_sensor_time_stamp-val :lambda-list '(m))
(cl:defmethod can_sensor_time_stamp-val ((m <MrrHeaderTimestamps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensor_time_stamp-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensor_time_stamp instead.")
  (can_sensor_time_stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrHeaderTimestamps>) ostream)
  "Serializes a message object of type '<MrrHeaderTimestamps>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_det_time_since_meas))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_sensor_time_stamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrHeaderTimestamps>) istream)
  "Deserializes a message object of type '<MrrHeaderTimestamps>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_det_time_since_meas) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_sensor_time_stamp) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrHeaderTimestamps>)))
  "Returns string type for a message object of type '<MrrHeaderTimestamps>"
  "delphi_mrr_msgs/MrrHeaderTimestamps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrHeaderTimestamps)))
  "Returns string type for a message object of type 'MrrHeaderTimestamps"
  "delphi_mrr_msgs/MrrHeaderTimestamps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrHeaderTimestamps>)))
  "Returns md5sum for a message object of type '<MrrHeaderTimestamps>"
  "31560a809bee8d977f1d25fd94db961e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrHeaderTimestamps)))
  "Returns md5sum for a message object of type 'MrrHeaderTimestamps"
  "31560a809bee8d977f1d25fd94db961e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrHeaderTimestamps>)))
  "Returns full string definition for message of type '<MrrHeaderTimestamps>"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 can_det_time_since_meas~%float32 can_sensor_time_stamp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrHeaderTimestamps)))
  "Returns full string definition for message of type 'MrrHeaderTimestamps"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 can_det_time_since_meas~%float32 can_sensor_time_stamp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrHeaderTimestamps>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrHeaderTimestamps>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrHeaderTimestamps
    (cl:cons ':header (header msg))
    (cl:cons ':can_det_time_since_meas (can_det_time_since_meas msg))
    (cl:cons ':can_sensor_time_stamp (can_sensor_time_stamp msg))
))
