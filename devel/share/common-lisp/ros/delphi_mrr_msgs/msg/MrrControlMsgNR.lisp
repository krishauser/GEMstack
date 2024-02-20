; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrControlMsgNR.msg.html

(cl:defclass <MrrControlMsgNR> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_stop_frequency_nrml
    :reader can_stop_frequency_nrml
    :initarg :can_stop_frequency_nrml
    :type cl:fixnum
    :initform 0)
   (can_prp_factor_nrml
    :reader can_prp_factor_nrml
    :initarg :can_prp_factor_nrml
    :type cl:fixnum
    :initform 0)
   (can_desired_sweep_bw_nrml
    :reader can_desired_sweep_bw_nrml
    :initarg :can_desired_sweep_bw_nrml
    :type cl:fixnum
    :initform 0)
   (can_radiation_ctrl
    :reader can_radiation_ctrl
    :initarg :can_radiation_ctrl
    :type cl:boolean
    :initform cl:nil)
   (can_stop_frequency_nrll
    :reader can_stop_frequency_nrll
    :initarg :can_stop_frequency_nrll
    :type cl:fixnum
    :initform 0)
   (can_prp_factor_nrll
    :reader can_prp_factor_nrll
    :initarg :can_prp_factor_nrll
    :type cl:fixnum
    :initform 0)
   (can_desired_sweep_bw_nrll
    :reader can_desired_sweep_bw_nrll
    :initarg :can_desired_sweep_bw_nrll
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrControlMsgNR (<MrrControlMsgNR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrControlMsgNR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrControlMsgNR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrControlMsgNR> is deprecated: use delphi_mrr_msgs-msg:MrrControlMsgNR instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_stop_frequency_nrml-val :lambda-list '(m))
(cl:defmethod can_stop_frequency_nrml-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_stop_frequency_nrml-val is deprecated.  Use delphi_mrr_msgs-msg:can_stop_frequency_nrml instead.")
  (can_stop_frequency_nrml m))

(cl:ensure-generic-function 'can_prp_factor_nrml-val :lambda-list '(m))
(cl:defmethod can_prp_factor_nrml-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_prp_factor_nrml-val is deprecated.  Use delphi_mrr_msgs-msg:can_prp_factor_nrml instead.")
  (can_prp_factor_nrml m))

(cl:ensure-generic-function 'can_desired_sweep_bw_nrml-val :lambda-list '(m))
(cl:defmethod can_desired_sweep_bw_nrml-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_desired_sweep_bw_nrml-val is deprecated.  Use delphi_mrr_msgs-msg:can_desired_sweep_bw_nrml instead.")
  (can_desired_sweep_bw_nrml m))

(cl:ensure-generic-function 'can_radiation_ctrl-val :lambda-list '(m))
(cl:defmethod can_radiation_ctrl-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radiation_ctrl-val is deprecated.  Use delphi_mrr_msgs-msg:can_radiation_ctrl instead.")
  (can_radiation_ctrl m))

(cl:ensure-generic-function 'can_stop_frequency_nrll-val :lambda-list '(m))
(cl:defmethod can_stop_frequency_nrll-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_stop_frequency_nrll-val is deprecated.  Use delphi_mrr_msgs-msg:can_stop_frequency_nrll instead.")
  (can_stop_frequency_nrll m))

(cl:ensure-generic-function 'can_prp_factor_nrll-val :lambda-list '(m))
(cl:defmethod can_prp_factor_nrll-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_prp_factor_nrll-val is deprecated.  Use delphi_mrr_msgs-msg:can_prp_factor_nrll instead.")
  (can_prp_factor_nrll m))

(cl:ensure-generic-function 'can_desired_sweep_bw_nrll-val :lambda-list '(m))
(cl:defmethod can_desired_sweep_bw_nrll-val ((m <MrrControlMsgNR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_desired_sweep_bw_nrll-val is deprecated.  Use delphi_mrr_msgs-msg:can_desired_sweep_bw_nrll instead.")
  (can_desired_sweep_bw_nrll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrControlMsgNR>) ostream)
  "Serializes a message object of type '<MrrControlMsgNR>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_nrml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_nrml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_prp_factor_nrml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_prp_factor_nrml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_nrml)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radiation_ctrl) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_nrll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_nrll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_prp_factor_nrll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_prp_factor_nrll)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_nrll)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrControlMsgNR>) istream)
  "Deserializes a message object of type '<MrrControlMsgNR>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_nrml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_nrml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_prp_factor_nrml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_prp_factor_nrml)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_nrml)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_radiation_ctrl) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_stop_frequency_nrll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_stop_frequency_nrll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_prp_factor_nrll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_prp_factor_nrll)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_desired_sweep_bw_nrll)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrControlMsgNR>)))
  "Returns string type for a message object of type '<MrrControlMsgNR>"
  "delphi_mrr_msgs/MrrControlMsgNR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrControlMsgNR)))
  "Returns string type for a message object of type 'MrrControlMsgNR"
  "delphi_mrr_msgs/MrrControlMsgNR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrControlMsgNR>)))
  "Returns md5sum for a message object of type '<MrrControlMsgNR>"
  "3100fbbd30b156c46cb7b9ae9e5d17a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrControlMsgNR)))
  "Returns md5sum for a message object of type 'MrrControlMsgNR"
  "3100fbbd30b156c46cb7b9ae9e5d17a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrControlMsgNR>)))
  "Returns full string definition for message of type '<MrrControlMsgNR>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 can_stop_frequency_nrml~%uint16 can_prp_factor_nrml~%uint8  can_desired_sweep_bw_nrml~%bool   can_radiation_ctrl~%uint16 can_stop_frequency_nrll~%uint16 can_prp_factor_nrll~%uint8  can_desired_sweep_bw_nrll~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrControlMsgNR)))
  "Returns full string definition for message of type 'MrrControlMsgNR"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 can_stop_frequency_nrml~%uint16 can_prp_factor_nrml~%uint8  can_desired_sweep_bw_nrml~%bool   can_radiation_ctrl~%uint16 can_stop_frequency_nrll~%uint16 can_prp_factor_nrll~%uint8  can_desired_sweep_bw_nrll~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrControlMsgNR>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     1
     1
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrControlMsgNR>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrControlMsgNR
    (cl:cons ':header (header msg))
    (cl:cons ':can_stop_frequency_nrml (can_stop_frequency_nrml msg))
    (cl:cons ':can_prp_factor_nrml (can_prp_factor_nrml msg))
    (cl:cons ':can_desired_sweep_bw_nrml (can_desired_sweep_bw_nrml msg))
    (cl:cons ':can_radiation_ctrl (can_radiation_ctrl msg))
    (cl:cons ':can_stop_frequency_nrll (can_stop_frequency_nrll msg))
    (cl:cons ':can_prp_factor_nrll (can_prp_factor_nrll msg))
    (cl:cons ':can_desired_sweep_bw_nrll (can_desired_sweep_bw_nrll msg))
))
