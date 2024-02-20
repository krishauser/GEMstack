; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude VehicleStateMsg1.msg.html

(cl:defclass <VehicleStateMsg1> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_fcw_sensitivity_level
    :reader can_fcw_sensitivity_level
    :initarg :can_fcw_sensitivity_level
    :type cl:fixnum
    :initform 0)
   (can_vehicle_stationary
    :reader can_vehicle_stationary
    :initarg :can_vehicle_stationary
    :type cl:boolean
    :initform cl:nil)
   (can_intf_minor_version
    :reader can_intf_minor_version
    :initarg :can_intf_minor_version
    :type cl:fixnum
    :initform 0)
   (can_intf_major_version
    :reader can_intf_major_version
    :initarg :can_intf_major_version
    :type cl:fixnum
    :initform 0)
   (can_brake_pedal
    :reader can_brake_pedal
    :initarg :can_brake_pedal
    :type cl:fixnum
    :initform 0)
   (can_high_wheel_slip
    :reader can_high_wheel_slip
    :initarg :can_high_wheel_slip
    :type cl:boolean
    :initform cl:nil)
   (can_turn_signal_status
    :reader can_turn_signal_status
    :initarg :can_turn_signal_status
    :type cl:fixnum
    :initform 0)
   (can_washer_front_cmd
    :reader can_washer_front_cmd
    :initarg :can_washer_front_cmd
    :type cl:boolean
    :initform cl:nil)
   (can_wiper_front_cmd
    :reader can_wiper_front_cmd
    :initarg :can_wiper_front_cmd
    :type cl:boolean
    :initform cl:nil)
   (can_wiper_speed_info
    :reader can_wiper_speed_info
    :initarg :can_wiper_speed_info
    :type cl:fixnum
    :initform 0)
   (can_reverse_gear
    :reader can_reverse_gear
    :initarg :can_reverse_gear
    :type cl:boolean
    :initform cl:nil)
   (can_beam_shape_actual_right
    :reader can_beam_shape_actual_right
    :initarg :can_beam_shape_actual_right
    :type cl:fixnum
    :initform 0)
   (can_beam_shape_actual_left
    :reader can_beam_shape_actual_left
    :initarg :can_beam_shape_actual_left
    :type cl:fixnum
    :initform 0)
   (can_main_beam_indication
    :reader can_main_beam_indication
    :initarg :can_main_beam_indication
    :type cl:boolean
    :initform cl:nil)
   (can_vehicle_index
    :reader can_vehicle_index
    :initarg :can_vehicle_index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass VehicleStateMsg1 (<VehicleStateMsg1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleStateMsg1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleStateMsg1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<VehicleStateMsg1> is deprecated: use delphi_mrr_msgs-msg:VehicleStateMsg1 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_fcw_sensitivity_level-val :lambda-list '(m))
(cl:defmethod can_fcw_sensitivity_level-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_fcw_sensitivity_level-val is deprecated.  Use delphi_mrr_msgs-msg:can_fcw_sensitivity_level instead.")
  (can_fcw_sensitivity_level m))

(cl:ensure-generic-function 'can_vehicle_stationary-val :lambda-list '(m))
(cl:defmethod can_vehicle_stationary-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_vehicle_stationary-val is deprecated.  Use delphi_mrr_msgs-msg:can_vehicle_stationary instead.")
  (can_vehicle_stationary m))

(cl:ensure-generic-function 'can_intf_minor_version-val :lambda-list '(m))
(cl:defmethod can_intf_minor_version-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_intf_minor_version-val is deprecated.  Use delphi_mrr_msgs-msg:can_intf_minor_version instead.")
  (can_intf_minor_version m))

(cl:ensure-generic-function 'can_intf_major_version-val :lambda-list '(m))
(cl:defmethod can_intf_major_version-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_intf_major_version-val is deprecated.  Use delphi_mrr_msgs-msg:can_intf_major_version instead.")
  (can_intf_major_version m))

(cl:ensure-generic-function 'can_brake_pedal-val :lambda-list '(m))
(cl:defmethod can_brake_pedal-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_brake_pedal-val is deprecated.  Use delphi_mrr_msgs-msg:can_brake_pedal instead.")
  (can_brake_pedal m))

(cl:ensure-generic-function 'can_high_wheel_slip-val :lambda-list '(m))
(cl:defmethod can_high_wheel_slip-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_high_wheel_slip-val is deprecated.  Use delphi_mrr_msgs-msg:can_high_wheel_slip instead.")
  (can_high_wheel_slip m))

(cl:ensure-generic-function 'can_turn_signal_status-val :lambda-list '(m))
(cl:defmethod can_turn_signal_status-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_turn_signal_status-val is deprecated.  Use delphi_mrr_msgs-msg:can_turn_signal_status instead.")
  (can_turn_signal_status m))

(cl:ensure-generic-function 'can_washer_front_cmd-val :lambda-list '(m))
(cl:defmethod can_washer_front_cmd-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_washer_front_cmd-val is deprecated.  Use delphi_mrr_msgs-msg:can_washer_front_cmd instead.")
  (can_washer_front_cmd m))

(cl:ensure-generic-function 'can_wiper_front_cmd-val :lambda-list '(m))
(cl:defmethod can_wiper_front_cmd-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_wiper_front_cmd-val is deprecated.  Use delphi_mrr_msgs-msg:can_wiper_front_cmd instead.")
  (can_wiper_front_cmd m))

(cl:ensure-generic-function 'can_wiper_speed_info-val :lambda-list '(m))
(cl:defmethod can_wiper_speed_info-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_wiper_speed_info-val is deprecated.  Use delphi_mrr_msgs-msg:can_wiper_speed_info instead.")
  (can_wiper_speed_info m))

(cl:ensure-generic-function 'can_reverse_gear-val :lambda-list '(m))
(cl:defmethod can_reverse_gear-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_reverse_gear-val is deprecated.  Use delphi_mrr_msgs-msg:can_reverse_gear instead.")
  (can_reverse_gear m))

(cl:ensure-generic-function 'can_beam_shape_actual_right-val :lambda-list '(m))
(cl:defmethod can_beam_shape_actual_right-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_beam_shape_actual_right-val is deprecated.  Use delphi_mrr_msgs-msg:can_beam_shape_actual_right instead.")
  (can_beam_shape_actual_right m))

(cl:ensure-generic-function 'can_beam_shape_actual_left-val :lambda-list '(m))
(cl:defmethod can_beam_shape_actual_left-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_beam_shape_actual_left-val is deprecated.  Use delphi_mrr_msgs-msg:can_beam_shape_actual_left instead.")
  (can_beam_shape_actual_left m))

(cl:ensure-generic-function 'can_main_beam_indication-val :lambda-list '(m))
(cl:defmethod can_main_beam_indication-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_main_beam_indication-val is deprecated.  Use delphi_mrr_msgs-msg:can_main_beam_indication instead.")
  (can_main_beam_indication m))

(cl:ensure-generic-function 'can_vehicle_index-val :lambda-list '(m))
(cl:defmethod can_vehicle_index-val ((m <VehicleStateMsg1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_vehicle_index-val is deprecated.  Use delphi_mrr_msgs-msg:can_vehicle_index instead.")
  (can_vehicle_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleStateMsg1>) ostream)
  "Serializes a message object of type '<VehicleStateMsg1>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_fcw_sensitivity_level)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_vehicle_stationary) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_intf_minor_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_intf_major_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_brake_pedal)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_high_wheel_slip) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_turn_signal_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_washer_front_cmd) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_wiper_front_cmd) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_wiper_speed_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_reverse_gear) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_beam_shape_actual_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_beam_shape_actual_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_main_beam_indication) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_vehicle_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_vehicle_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleStateMsg1>) istream)
  "Deserializes a message object of type '<VehicleStateMsg1>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_fcw_sensitivity_level)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_vehicle_stationary) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_intf_minor_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_intf_major_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_brake_pedal)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_high_wheel_slip) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_turn_signal_status)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_washer_front_cmd) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_wiper_front_cmd) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_wiper_speed_info)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_reverse_gear) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_beam_shape_actual_right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_beam_shape_actual_left)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_main_beam_indication) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_vehicle_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_vehicle_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleStateMsg1>)))
  "Returns string type for a message object of type '<VehicleStateMsg1>"
  "delphi_mrr_msgs/VehicleStateMsg1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleStateMsg1)))
  "Returns string type for a message object of type 'VehicleStateMsg1"
  "delphi_mrr_msgs/VehicleStateMsg1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleStateMsg1>)))
  "Returns md5sum for a message object of type '<VehicleStateMsg1>"
  "9bd8d57bd02218fdeffdb6496e73cbe0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleStateMsg1)))
  "Returns md5sum for a message object of type 'VehicleStateMsg1"
  "9bd8d57bd02218fdeffdb6496e73cbe0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleStateMsg1>)))
  "Returns full string definition for message of type '<VehicleStateMsg1>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8  can_fcw_sensitivity_level~%bool   can_vehicle_stationary~%uint8  can_intf_minor_version~%uint8  can_intf_major_version~%uint8  can_brake_pedal~%bool   can_high_wheel_slip~%uint8  can_turn_signal_status~%bool   can_washer_front_cmd~%bool   can_wiper_front_cmd~%uint8  can_wiper_speed_info~%bool   can_reverse_gear~%uint8  can_beam_shape_actual_right~%uint8  can_beam_shape_actual_left~%bool   can_main_beam_indication~%uint16 can_vehicle_index~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleStateMsg1)))
  "Returns full string definition for message of type 'VehicleStateMsg1"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8  can_fcw_sensitivity_level~%bool   can_vehicle_stationary~%uint8  can_intf_minor_version~%uint8  can_intf_major_version~%uint8  can_brake_pedal~%bool   can_high_wheel_slip~%uint8  can_turn_signal_status~%bool   can_washer_front_cmd~%bool   can_wiper_front_cmd~%uint8  can_wiper_speed_info~%bool   can_reverse_gear~%uint8  can_beam_shape_actual_right~%uint8  can_beam_shape_actual_left~%bool   can_main_beam_indication~%uint16 can_vehicle_index~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleStateMsg1>))
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
     1
     1
     1
     1
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleStateMsg1>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleStateMsg1
    (cl:cons ':header (header msg))
    (cl:cons ':can_fcw_sensitivity_level (can_fcw_sensitivity_level msg))
    (cl:cons ':can_vehicle_stationary (can_vehicle_stationary msg))
    (cl:cons ':can_intf_minor_version (can_intf_minor_version msg))
    (cl:cons ':can_intf_major_version (can_intf_major_version msg))
    (cl:cons ':can_brake_pedal (can_brake_pedal msg))
    (cl:cons ':can_high_wheel_slip (can_high_wheel_slip msg))
    (cl:cons ':can_turn_signal_status (can_turn_signal_status msg))
    (cl:cons ':can_washer_front_cmd (can_washer_front_cmd msg))
    (cl:cons ':can_wiper_front_cmd (can_wiper_front_cmd msg))
    (cl:cons ':can_wiper_speed_info (can_wiper_speed_info msg))
    (cl:cons ':can_reverse_gear (can_reverse_gear msg))
    (cl:cons ':can_beam_shape_actual_right (can_beam_shape_actual_right msg))
    (cl:cons ':can_beam_shape_actual_left (can_beam_shape_actual_left msg))
    (cl:cons ':can_main_beam_indication (can_main_beam_indication msg))
    (cl:cons ':can_vehicle_index (can_vehicle_index msg))
))
