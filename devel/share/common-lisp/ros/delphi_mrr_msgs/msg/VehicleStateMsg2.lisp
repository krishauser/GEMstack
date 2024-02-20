; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude VehicleStateMsg2.msg.html

(cl:defclass <VehicleStateMsg2> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fsm_yaw_rate_valid
    :reader fsm_yaw_rate_valid
    :initarg :fsm_yaw_rate_valid
    :type cl:boolean
    :initform cl:nil)
   (fsm_yaw_rate
    :reader fsm_yaw_rate
    :initarg :fsm_yaw_rate
    :type cl:float
    :initform 0.0)
   (can_vehicle_index_4fa
    :reader can_vehicle_index_4fa
    :initarg :can_vehicle_index_4fa
    :type cl:fixnum
    :initform 0)
   (fsm_vehicle_velocity
    :reader fsm_vehicle_velocity
    :initarg :fsm_vehicle_velocity
    :type cl:float
    :initform 0.0)
   (can_steering_whl_angle_qf
    :reader can_steering_whl_angle_qf
    :initarg :can_steering_whl_angle_qf
    :type cl:boolean
    :initform cl:nil)
   (fsm_vehicle_velocity_valid
    :reader fsm_vehicle_velocity_valid
    :initarg :fsm_vehicle_velocity_valid
    :type cl:boolean
    :initform cl:nil)
   (can_steering_whl_angle
    :reader can_steering_whl_angle
    :initarg :can_steering_whl_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass VehicleStateMsg2 (<VehicleStateMsg2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleStateMsg2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleStateMsg2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<VehicleStateMsg2> is deprecated: use delphi_mrr_msgs-msg:VehicleStateMsg2 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fsm_yaw_rate_valid-val :lambda-list '(m))
(cl:defmethod fsm_yaw_rate_valid-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:fsm_yaw_rate_valid-val is deprecated.  Use delphi_mrr_msgs-msg:fsm_yaw_rate_valid instead.")
  (fsm_yaw_rate_valid m))

(cl:ensure-generic-function 'fsm_yaw_rate-val :lambda-list '(m))
(cl:defmethod fsm_yaw_rate-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:fsm_yaw_rate-val is deprecated.  Use delphi_mrr_msgs-msg:fsm_yaw_rate instead.")
  (fsm_yaw_rate m))

(cl:ensure-generic-function 'can_vehicle_index_4fa-val :lambda-list '(m))
(cl:defmethod can_vehicle_index_4fa-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_vehicle_index_4fa-val is deprecated.  Use delphi_mrr_msgs-msg:can_vehicle_index_4fa instead.")
  (can_vehicle_index_4fa m))

(cl:ensure-generic-function 'fsm_vehicle_velocity-val :lambda-list '(m))
(cl:defmethod fsm_vehicle_velocity-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:fsm_vehicle_velocity-val is deprecated.  Use delphi_mrr_msgs-msg:fsm_vehicle_velocity instead.")
  (fsm_vehicle_velocity m))

(cl:ensure-generic-function 'can_steering_whl_angle_qf-val :lambda-list '(m))
(cl:defmethod can_steering_whl_angle_qf-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_steering_whl_angle_qf-val is deprecated.  Use delphi_mrr_msgs-msg:can_steering_whl_angle_qf instead.")
  (can_steering_whl_angle_qf m))

(cl:ensure-generic-function 'fsm_vehicle_velocity_valid-val :lambda-list '(m))
(cl:defmethod fsm_vehicle_velocity_valid-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:fsm_vehicle_velocity_valid-val is deprecated.  Use delphi_mrr_msgs-msg:fsm_vehicle_velocity_valid instead.")
  (fsm_vehicle_velocity_valid m))

(cl:ensure-generic-function 'can_steering_whl_angle-val :lambda-list '(m))
(cl:defmethod can_steering_whl_angle-val ((m <VehicleStateMsg2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_steering_whl_angle-val is deprecated.  Use delphi_mrr_msgs-msg:can_steering_whl_angle instead.")
  (can_steering_whl_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleStateMsg2>) ostream)
  "Serializes a message object of type '<VehicleStateMsg2>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fsm_yaw_rate_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fsm_yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_vehicle_index_4fa)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_vehicle_index_4fa)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fsm_vehicle_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_steering_whl_angle_qf) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fsm_vehicle_velocity_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_steering_whl_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleStateMsg2>) istream)
  "Deserializes a message object of type '<VehicleStateMsg2>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'fsm_yaw_rate_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fsm_yaw_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_vehicle_index_4fa)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_vehicle_index_4fa)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fsm_vehicle_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'can_steering_whl_angle_qf) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fsm_vehicle_velocity_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_steering_whl_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleStateMsg2>)))
  "Returns string type for a message object of type '<VehicleStateMsg2>"
  "delphi_mrr_msgs/VehicleStateMsg2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleStateMsg2)))
  "Returns string type for a message object of type 'VehicleStateMsg2"
  "delphi_mrr_msgs/VehicleStateMsg2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleStateMsg2>)))
  "Returns md5sum for a message object of type '<VehicleStateMsg2>"
  "54463690e9fc3e6c8708c99c08ba6c62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleStateMsg2)))
  "Returns md5sum for a message object of type 'VehicleStateMsg2"
  "54463690e9fc3e6c8708c99c08ba6c62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleStateMsg2>)))
  "Returns full string definition for message of type '<VehicleStateMsg2>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool    fsm_yaw_rate_valid~%float32 fsm_yaw_rate~%uint16  can_vehicle_index_4fa~%float32 fsm_vehicle_velocity~%bool    can_steering_whl_angle_qf~%bool    fsm_vehicle_velocity_valid~%float32 can_steering_whl_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleStateMsg2)))
  "Returns full string definition for message of type 'VehicleStateMsg2"
  (cl:format cl:nil "std_msgs/Header header~%~%bool    fsm_yaw_rate_valid~%float32 fsm_yaw_rate~%uint16  can_vehicle_index_4fa~%float32 fsm_vehicle_velocity~%bool    can_steering_whl_angle_qf~%bool    fsm_vehicle_velocity_valid~%float32 can_steering_whl_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleStateMsg2>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     2
     4
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleStateMsg2>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleStateMsg2
    (cl:cons ':header (header msg))
    (cl:cons ':fsm_yaw_rate_valid (fsm_yaw_rate_valid msg))
    (cl:cons ':fsm_yaw_rate (fsm_yaw_rate msg))
    (cl:cons ':can_vehicle_index_4fa (can_vehicle_index_4fa msg))
    (cl:cons ':fsm_vehicle_velocity (fsm_vehicle_velocity msg))
    (cl:cons ':can_steering_whl_angle_qf (can_steering_whl_angle_qf msg))
    (cl:cons ':fsm_vehicle_velocity_valid (fsm_vehicle_velocity_valid msg))
    (cl:cons ':can_steering_whl_angle (can_steering_whl_angle msg))
))
