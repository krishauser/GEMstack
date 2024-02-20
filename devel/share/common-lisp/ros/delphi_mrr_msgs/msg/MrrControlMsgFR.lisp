; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrControlMsgFR.msg.html

(cl:defclass <MrrControlMsgFR> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_sensitivity_profile_select
    :reader can_sensitivity_profile_select
    :initarg :can_sensitivity_profile_select
    :type cl:fixnum
    :initform 0)
   (can_stop_frequency_frml
    :reader can_stop_frequency_frml
    :initarg :can_stop_frequency_frml
    :type cl:fixnum
    :initform 0)
   (can_stop_frequency_frll
    :reader can_stop_frequency_frll
    :initarg :can_stop_frequency_frll
    :type cl:fixnum
    :initform 0)
   (can_prp_factor_frml
    :reader can_prp_factor_frml
    :initarg :can_prp_factor_frml
    :type cl:float
    :initform 0.0)
   (can_prp_factor_frll
    :reader can_prp_factor_frll
    :initarg :can_prp_factor_frll
    :type cl:float
    :initform 0.0)
   (can_desired_sweep_bw_frml
    :reader can_desired_sweep_bw_frml
    :initarg :can_desired_sweep_bw_frml
    :type cl:fixnum
    :initform 0)
   (can_desired_sweep_bw_frll
    :reader can_desired_sweep_bw_frll
    :initarg :can_desired_sweep_bw_frll
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrControlMsgFR (<MrrControlMsgFR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrControlMsgFR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrControlMsgFR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrControlMsgFR> is deprecated: use delphi_mrr_msgs-msg:MrrControlMsgFR instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_sensitivity_profile_select-val :lambda-list '(m))
(cl:defmethod can_sensitivity_profile_select-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sensitivity_profile_select-val is deprecated.  Use delphi_mrr_msgs-msg:can_sensitivity_profile_select instead.")
  (can_sensitivity_profile_select m))

(cl:ensure-generic-function 'can_stop_frequency_frml-val :lambda-list '(m))
(cl:defmethod can_stop_frequency_frml-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_stop_frequency_frml-val is deprecated.  Use delphi_mrr_msgs-msg:can_stop_frequency_frml instead.")
  (can_stop_frequency_frml m))

(cl:ensure-generic-function 'can_stop_frequency_frll-val :lambda-list '(m))
(cl:defmethod can_stop_frequency_frll-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_stop_frequency_frll-val is deprecated.  Use delphi_mrr_msgs-msg:can_stop_frequency_frll instead.")
  (can_stop_frequency_frll m))

(cl:ensure-generic-function 'can_prp_factor_frml-val :lambda-list '(m))
(cl:defmethod can_prp_factor_frml-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_prp_factor_frml-val is deprecated.  Use delphi_mrr_msgs-msg:can_prp_factor_frml instead.")
  (can_prp_factor_frml m))

(cl:ensure-generic-function 'can_prp_factor_frll-val :lambda-list '(m))
(cl:defmethod can_prp_factor_frll-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_prp_factor_frll-val is deprecated.  Use delphi_mrr_msgs-msg:can_prp_factor_frll instead.")
  (can_prp_factor_frll m))

(cl:ensure-generic-function 'can_desired_sweep_bw_frml-val :lambda-list '(m))
(cl:defmethod can_desired_sweep_bw_frml-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_desired_sweep_bw_frml-val is deprecated.  Use delphi_mrr_msgs-msg:can_desired_sweep_bw_frml instead.")
  (can_desired_sweep_bw_frml m))

(cl:ensure-generic-function 'can_desired_sweep_bw_frll-val :lambda-list '(m))
(cl:defmethod can_desired_sweep_bw_frll-val ((m <MrrControlMsgFR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_desired_sweep_bw_frll-val is deprecated.  Use delphi_mrr_msgs-msg:can_desired_sweep_bw_frll instead.")
  (can_desired_sweep_bw_frll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrControlMsgFR>) ostream)
  "Serializes a message object of type '<MrrControlMsgFR>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sensitivity_profile_select)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_frml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_frml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_frll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_frll)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_prp_factor_frml))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_prp_factor_frll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_frml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_frll)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrControlMsgFR>) istream)
  "Deserializes a message object of type '<MrrControlMsgFR>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sensitivity_profile_select)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_frml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_frml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_frll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_frll)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_prp_factor_frml) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_prp_factor_frll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_frml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_frll)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrControlMsgFR>)))
  "Returns string type for a message object of type '<MrrControlMsgFR>"
  "delphi_mrr_msgs/MrrControlMsgFR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrControlMsgFR)))
  "Returns string type for a message object of type 'MrrControlMsgFR"
  "delphi_mrr_msgs/MrrControlMsgFR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrControlMsgFR>)))
  "Returns md5sum for a message object of type '<MrrControlMsgFR>"
  "dba2c9fc1e4b47706ab1d6d7c85d6d53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrControlMsgFR)))
  "Returns md5sum for a message object of type 'MrrControlMsgFR"
  "dba2c9fc1e4b47706ab1d6d7c85d6d53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrControlMsgFR>)))
  "Returns full string definition for message of type '<MrrControlMsgFR>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8   can_sensitivity_profile_select~%uint16  can_stop_frequency_frml~%uint16  can_stop_frequency_frll~%float32 can_prp_factor_frml~%float32 can_prp_factor_frll~%uint8   can_desired_sweep_bw_frml~%uint8   can_desired_sweep_bw_frll~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrControlMsgFR)))
  "Returns full string definition for message of type 'MrrControlMsgFR"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8   can_sensitivity_profile_select~%uint16  can_stop_frequency_frml~%uint16  can_stop_frequency_frll~%float32 can_prp_factor_frml~%float32 can_prp_factor_frll~%uint8   can_desired_sweep_bw_frml~%uint8   can_desired_sweep_bw_frll~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrControlMsgFR>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     2
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrControlMsgFR>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrControlMsgFR
    (cl:cons ':header (header msg))
    (cl:cons ':can_sensitivity_profile_select (can_sensitivity_profile_select msg))
    (cl:cons ':can_stop_frequency_frml (can_stop_frequency_frml msg))
    (cl:cons ':can_stop_frequency_frll (can_stop_frequency_frll msg))
    (cl:cons ':can_prp_factor_frml (can_prp_factor_frml msg))
    (cl:cons ':can_prp_factor_frll (can_prp_factor_frll msg))
    (cl:cons ':can_desired_sweep_bw_frml (can_desired_sweep_bw_frml msg))
    (cl:cons ':can_desired_sweep_bw_frll (can_desired_sweep_bw_frll msg))
))
