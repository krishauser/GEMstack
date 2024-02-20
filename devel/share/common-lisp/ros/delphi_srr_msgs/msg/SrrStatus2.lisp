; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrStatus2.msg.html

(cl:defclass <SrrStatus2> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_tx_alignment_status
    :reader can_tx_alignment_status
    :initarg :can_tx_alignment_status
    :type cl:fixnum
    :initform 0)
   (can_tx_comm_error
    :reader can_tx_comm_error
    :initarg :can_tx_comm_error
    :type cl:boolean
    :initform cl:nil)
   (can_tx_steering_angle_sign
    :reader can_tx_steering_angle_sign
    :initarg :can_tx_steering_angle_sign
    :type cl:boolean
    :initform cl:nil)
   (can_tx_yaw_rate_bias
    :reader can_tx_yaw_rate_bias
    :initarg :can_tx_yaw_rate_bias
    :type cl:float
    :initform 0.0)
   (can_tx_veh_spd_comp_factor
    :reader can_tx_veh_spd_comp_factor
    :initarg :can_tx_veh_spd_comp_factor
    :type cl:float
    :initform 0.0)
   (can_tx_sw_version_dsp
    :reader can_tx_sw_version_dsp
    :initarg :can_tx_sw_version_dsp
    :type cl:fixnum
    :initform 0)
   (can_tx_temperature
    :reader can_tx_temperature
    :initarg :can_tx_temperature
    :type cl:fixnum
    :initform 0)
   (can_tx_range_perf_error
    :reader can_tx_range_perf_error
    :initarg :can_tx_range_perf_error
    :type cl:boolean
    :initform cl:nil)
   (can_tx_overheat_error
    :reader can_tx_overheat_error
    :initarg :can_tx_overheat_error
    :type cl:boolean
    :initform cl:nil)
   (can_tx_internal_error
    :reader can_tx_internal_error
    :initarg :can_tx_internal_error
    :type cl:boolean
    :initform cl:nil)
   (can_tx_xcvr_operational
    :reader can_tx_xcvr_operational
    :initarg :can_tx_xcvr_operational
    :type cl:boolean
    :initform cl:nil)
   (can_tx_steering_angle
    :reader can_tx_steering_angle
    :initarg :can_tx_steering_angle
    :type cl:fixnum
    :initform 0)
   (can_tx_rolling_count_2
    :reader can_tx_rolling_count_2
    :initarg :can_tx_rolling_count_2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SrrStatus2 (<SrrStatus2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrStatus2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrStatus2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrStatus2> is deprecated: use delphi_srr_msgs-msg:SrrStatus2 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_tx_alignment_status-val :lambda-list '(m))
(cl:defmethod can_tx_alignment_status-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_alignment_status-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_alignment_status instead.")
  (can_tx_alignment_status m))

(cl:ensure-generic-function 'can_tx_comm_error-val :lambda-list '(m))
(cl:defmethod can_tx_comm_error-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_comm_error-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_comm_error instead.")
  (can_tx_comm_error m))

(cl:ensure-generic-function 'can_tx_steering_angle_sign-val :lambda-list '(m))
(cl:defmethod can_tx_steering_angle_sign-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_steering_angle_sign-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_steering_angle_sign instead.")
  (can_tx_steering_angle_sign m))

(cl:ensure-generic-function 'can_tx_yaw_rate_bias-val :lambda-list '(m))
(cl:defmethod can_tx_yaw_rate_bias-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_yaw_rate_bias-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_yaw_rate_bias instead.")
  (can_tx_yaw_rate_bias m))

(cl:ensure-generic-function 'can_tx_veh_spd_comp_factor-val :lambda-list '(m))
(cl:defmethod can_tx_veh_spd_comp_factor-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_veh_spd_comp_factor-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_veh_spd_comp_factor instead.")
  (can_tx_veh_spd_comp_factor m))

(cl:ensure-generic-function 'can_tx_sw_version_dsp-val :lambda-list '(m))
(cl:defmethod can_tx_sw_version_dsp-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_sw_version_dsp-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_sw_version_dsp instead.")
  (can_tx_sw_version_dsp m))

(cl:ensure-generic-function 'can_tx_temperature-val :lambda-list '(m))
(cl:defmethod can_tx_temperature-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_temperature-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_temperature instead.")
  (can_tx_temperature m))

(cl:ensure-generic-function 'can_tx_range_perf_error-val :lambda-list '(m))
(cl:defmethod can_tx_range_perf_error-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_range_perf_error-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_range_perf_error instead.")
  (can_tx_range_perf_error m))

(cl:ensure-generic-function 'can_tx_overheat_error-val :lambda-list '(m))
(cl:defmethod can_tx_overheat_error-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_overheat_error-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_overheat_error instead.")
  (can_tx_overheat_error m))

(cl:ensure-generic-function 'can_tx_internal_error-val :lambda-list '(m))
(cl:defmethod can_tx_internal_error-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_internal_error-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_internal_error instead.")
  (can_tx_internal_error m))

(cl:ensure-generic-function 'can_tx_xcvr_operational-val :lambda-list '(m))
(cl:defmethod can_tx_xcvr_operational-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_xcvr_operational-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_xcvr_operational instead.")
  (can_tx_xcvr_operational m))

(cl:ensure-generic-function 'can_tx_steering_angle-val :lambda-list '(m))
(cl:defmethod can_tx_steering_angle-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_steering_angle-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_steering_angle instead.")
  (can_tx_steering_angle m))

(cl:ensure-generic-function 'can_tx_rolling_count_2-val :lambda-list '(m))
(cl:defmethod can_tx_rolling_count_2-val ((m <SrrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_rolling_count_2-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_rolling_count_2 instead.")
  (can_tx_rolling_count_2 m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SrrStatus2>)))
    "Constants for message type '<SrrStatus2>"
  '((:CAN_TX_ALIGNMENT_STATUS_UNKNOWN . 0)
    (:CAN_TX_ALIGNMENT_STATUS_CONVERGED . 1)
    (:CAN_TX_ALIGNMENT_STATUS_FAILED . 2)
    (:CAN_TX_ALIGNMENT_STATUS_RESERVED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SrrStatus2)))
    "Constants for message type 'SrrStatus2"
  '((:CAN_TX_ALIGNMENT_STATUS_UNKNOWN . 0)
    (:CAN_TX_ALIGNMENT_STATUS_CONVERGED . 1)
    (:CAN_TX_ALIGNMENT_STATUS_FAILED . 2)
    (:CAN_TX_ALIGNMENT_STATUS_RESERVED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrStatus2>) ostream)
  "Serializes a message object of type '<SrrStatus2>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_alignment_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_comm_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_steering_angle_sign) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_yaw_rate_bias))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_veh_spd_comp_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_sw_version_dsp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_sw_version_dsp)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'can_tx_temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_range_perf_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_overheat_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_internal_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_xcvr_operational) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_steering_angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_steering_angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_rolling_count_2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrStatus2>) istream)
  "Deserializes a message object of type '<SrrStatus2>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_alignment_status)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_comm_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_tx_steering_angle_sign) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_yaw_rate_bias) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_veh_spd_comp_factor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_sw_version_dsp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_sw_version_dsp)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'can_tx_temperature) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'can_tx_range_perf_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_tx_overheat_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_tx_internal_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_tx_xcvr_operational) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_steering_angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_steering_angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_rolling_count_2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrStatus2>)))
  "Returns string type for a message object of type '<SrrStatus2>"
  "delphi_srr_msgs/SrrStatus2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrStatus2)))
  "Returns string type for a message object of type 'SrrStatus2"
  "delphi_srr_msgs/SrrStatus2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrStatus2>)))
  "Returns md5sum for a message object of type '<SrrStatus2>"
  "2b05d1c3cfa8185e9616806113ff9b8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrStatus2)))
  "Returns md5sum for a message object of type 'SrrStatus2"
  "2b05d1c3cfa8185e9616806113ff9b8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrStatus2>)))
  "Returns full string definition for message of type '<SrrStatus2>"
  (cl:format cl:nil "# Message file for srr_status2~%~%std_msgs/Header header~%~%uint8     can_tx_alignment_status~%uint8     CAN_TX_ALIGNMENT_STATUS_UNKNOWN=0~%uint8     CAN_TX_ALIGNMENT_STATUS_CONVERGED=1~%uint8     CAN_TX_ALIGNMENT_STATUS_FAILED=2~%uint8     CAN_TX_ALIGNMENT_STATUS_RESERVED=3~%~%bool      can_tx_comm_error~%bool      can_tx_steering_angle_sign~%float32   can_tx_yaw_rate_bias~%float32   can_tx_veh_spd_comp_factor~%uint16    can_tx_sw_version_dsp~%int16     can_tx_temperature                       # degc~%bool      can_tx_range_perf_error~%bool      can_tx_overheat_error~%bool      can_tx_internal_error~%bool      can_tx_xcvr_operational~%uint16    can_tx_steering_angle                    # deg~%uint8     can_tx_rolling_count_2~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrStatus2)))
  "Returns full string definition for message of type 'SrrStatus2"
  (cl:format cl:nil "# Message file for srr_status2~%~%std_msgs/Header header~%~%uint8     can_tx_alignment_status~%uint8     CAN_TX_ALIGNMENT_STATUS_UNKNOWN=0~%uint8     CAN_TX_ALIGNMENT_STATUS_CONVERGED=1~%uint8     CAN_TX_ALIGNMENT_STATUS_FAILED=2~%uint8     CAN_TX_ALIGNMENT_STATUS_RESERVED=3~%~%bool      can_tx_comm_error~%bool      can_tx_steering_angle_sign~%float32   can_tx_yaw_rate_bias~%float32   can_tx_veh_spd_comp_factor~%uint16    can_tx_sw_version_dsp~%int16     can_tx_temperature                       # degc~%bool      can_tx_range_perf_error~%bool      can_tx_overheat_error~%bool      can_tx_internal_error~%bool      can_tx_xcvr_operational~%uint16    can_tx_steering_angle                    # deg~%uint8     can_tx_rolling_count_2~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrStatus2>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     4
     2
     2
     1
     1
     1
     1
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrStatus2>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrStatus2
    (cl:cons ':header (header msg))
    (cl:cons ':can_tx_alignment_status (can_tx_alignment_status msg))
    (cl:cons ':can_tx_comm_error (can_tx_comm_error msg))
    (cl:cons ':can_tx_steering_angle_sign (can_tx_steering_angle_sign msg))
    (cl:cons ':can_tx_yaw_rate_bias (can_tx_yaw_rate_bias msg))
    (cl:cons ':can_tx_veh_spd_comp_factor (can_tx_veh_spd_comp_factor msg))
    (cl:cons ':can_tx_sw_version_dsp (can_tx_sw_version_dsp msg))
    (cl:cons ':can_tx_temperature (can_tx_temperature msg))
    (cl:cons ':can_tx_range_perf_error (can_tx_range_perf_error msg))
    (cl:cons ':can_tx_overheat_error (can_tx_overheat_error msg))
    (cl:cons ':can_tx_internal_error (can_tx_internal_error msg))
    (cl:cons ':can_tx_xcvr_operational (can_tx_xcvr_operational msg))
    (cl:cons ':can_tx_steering_angle (can_tx_steering_angle msg))
    (cl:cons ':can_tx_rolling_count_2 (can_tx_rolling_count_2 msg))
))
