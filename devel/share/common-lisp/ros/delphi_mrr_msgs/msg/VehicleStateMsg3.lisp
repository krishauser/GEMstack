; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude VehicleStateMsg3.msg.html

(cl:defclass <VehicleStateMsg3> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (yaw_rate_reference_valid
    :reader yaw_rate_reference_valid
    :initarg :yaw_rate_reference_valid
    :type cl:boolean
    :initform cl:nil)
   (yaw_rate_reference
    :reader yaw_rate_reference
    :initarg :yaw_rate_reference
    :type cl:float
    :initform 0.0)
   (can_veh_long_accel_qf
    :reader can_veh_long_accel_qf
    :initarg :can_veh_long_accel_qf
    :type cl:fixnum
    :initform 0)
   (can_veh_long_accel
    :reader can_veh_long_accel
    :initarg :can_veh_long_accel
    :type cl:float
    :initform 0.0))
)

(cl:defclass VehicleStateMsg3 (<VehicleStateMsg3>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleStateMsg3>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleStateMsg3)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<VehicleStateMsg3> is deprecated: use delphi_mrr_msgs-msg:VehicleStateMsg3 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehicleStateMsg3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'yaw_rate_reference_valid-val :lambda-list '(m))
(cl:defmethod yaw_rate_reference_valid-val ((m <VehicleStateMsg3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:yaw_rate_reference_valid-val is deprecated.  Use delphi_mrr_msgs-msg:yaw_rate_reference_valid instead.")
  (yaw_rate_reference_valid m))

(cl:ensure-generic-function 'yaw_rate_reference-val :lambda-list '(m))
(cl:defmethod yaw_rate_reference-val ((m <VehicleStateMsg3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:yaw_rate_reference-val is deprecated.  Use delphi_mrr_msgs-msg:yaw_rate_reference instead.")
  (yaw_rate_reference m))

(cl:ensure-generic-function 'can_veh_long_accel_qf-val :lambda-list '(m))
(cl:defmethod can_veh_long_accel_qf-val ((m <VehicleStateMsg3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_veh_long_accel_qf-val is deprecated.  Use delphi_mrr_msgs-msg:can_veh_long_accel_qf instead.")
  (can_veh_long_accel_qf m))

(cl:ensure-generic-function 'can_veh_long_accel-val :lambda-list '(m))
(cl:defmethod can_veh_long_accel-val ((m <VehicleStateMsg3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_veh_long_accel-val is deprecated.  Use delphi_mrr_msgs-msg:can_veh_long_accel instead.")
  (can_veh_long_accel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleStateMsg3>) ostream)
  "Serializes a message object of type '<VehicleStateMsg3>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'yaw_rate_reference_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate_reference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_veh_long_accel_qf)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_veh_long_accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleStateMsg3>) istream)
  "Deserializes a message object of type '<VehicleStateMsg3>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'yaw_rate_reference_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate_reference) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_veh_long_accel_qf)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_veh_long_accel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleStateMsg3>)))
  "Returns string type for a message object of type '<VehicleStateMsg3>"
  "delphi_mrr_msgs/VehicleStateMsg3")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleStateMsg3)))
  "Returns string type for a message object of type 'VehicleStateMsg3"
  "delphi_mrr_msgs/VehicleStateMsg3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleStateMsg3>)))
  "Returns md5sum for a message object of type '<VehicleStateMsg3>"
  "ce47a927102040e8c016cbb9166e5057")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleStateMsg3)))
  "Returns md5sum for a message object of type 'VehicleStateMsg3"
  "ce47a927102040e8c016cbb9166e5057")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleStateMsg3>)))
  "Returns full string definition for message of type '<VehicleStateMsg3>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool    yaw_rate_reference_valid~%float32 yaw_rate_reference~%uint8   can_veh_long_accel_qf~%float32 can_veh_long_accel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleStateMsg3)))
  "Returns full string definition for message of type 'VehicleStateMsg3"
  (cl:format cl:nil "std_msgs/Header header~%~%bool    yaw_rate_reference_valid~%float32 yaw_rate_reference~%uint8   can_veh_long_accel_qf~%float32 can_veh_long_accel~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleStateMsg3>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleStateMsg3>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleStateMsg3
    (cl:cons ':header (header msg))
    (cl:cons ':yaw_rate_reference_valid (yaw_rate_reference_valid msg))
    (cl:cons ':yaw_rate_reference (yaw_rate_reference msg))
    (cl:cons ':can_veh_long_accel_qf (can_veh_long_accel_qf msg))
    (cl:cons ':can_veh_long_accel (can_veh_long_accel msg))
))
