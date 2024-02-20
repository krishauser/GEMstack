; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrStatusRadar.msg.html

(cl:defclass <MrrStatusRadar> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_interference_type
    :reader can_interference_type
    :initarg :can_interference_type
    :type cl:fixnum
    :initform 0)
   (can_recommend_unconverge
    :reader can_recommend_unconverge
    :initarg :can_recommend_unconverge
    :type cl:boolean
    :initform cl:nil)
   (can_blockage_sidelobe_filter_val
    :reader can_blockage_sidelobe_filter_val
    :initarg :can_blockage_sidelobe_filter_val
    :type cl:fixnum
    :initform 0)
   (can_radar_align_incomplete
    :reader can_radar_align_incomplete
    :initarg :can_radar_align_incomplete
    :type cl:boolean
    :initform cl:nil)
   (can_blockage_sidelobe
    :reader can_blockage_sidelobe
    :initarg :can_blockage_sidelobe
    :type cl:boolean
    :initform cl:nil)
   (can_blockage_mnr
    :reader can_blockage_mnr
    :initarg :can_blockage_mnr
    :type cl:boolean
    :initform cl:nil)
   (can_radar_ext_cond_nok
    :reader can_radar_ext_cond_nok
    :initarg :can_radar_ext_cond_nok
    :type cl:boolean
    :initform cl:nil)
   (can_radar_align_out_range
    :reader can_radar_align_out_range
    :initarg :can_radar_align_out_range
    :type cl:boolean
    :initform cl:nil)
   (can_radar_align_not_start
    :reader can_radar_align_not_start
    :initarg :can_radar_align_not_start
    :type cl:boolean
    :initform cl:nil)
   (can_radar_overheat_error
    :reader can_radar_overheat_error
    :initarg :can_radar_overheat_error
    :type cl:boolean
    :initform cl:nil)
   (can_radar_not_op
    :reader can_radar_not_op
    :initarg :can_radar_not_op
    :type cl:boolean
    :initform cl:nil)
   (can_xcvr_operational
    :reader can_xcvr_operational
    :initarg :can_xcvr_operational
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MrrStatusRadar (<MrrStatusRadar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrStatusRadar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrStatusRadar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrStatusRadar> is deprecated: use delphi_mrr_msgs-msg:MrrStatusRadar instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_interference_type-val :lambda-list '(m))
(cl:defmethod can_interference_type-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_interference_type-val is deprecated.  Use delphi_mrr_msgs-msg:can_interference_type instead.")
  (can_interference_type m))

(cl:ensure-generic-function 'can_recommend_unconverge-val :lambda-list '(m))
(cl:defmethod can_recommend_unconverge-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_recommend_unconverge-val is deprecated.  Use delphi_mrr_msgs-msg:can_recommend_unconverge instead.")
  (can_recommend_unconverge m))

(cl:ensure-generic-function 'can_blockage_sidelobe_filter_val-val :lambda-list '(m))
(cl:defmethod can_blockage_sidelobe_filter_val-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_blockage_sidelobe_filter_val-val is deprecated.  Use delphi_mrr_msgs-msg:can_blockage_sidelobe_filter_val instead.")
  (can_blockage_sidelobe_filter_val m))

(cl:ensure-generic-function 'can_radar_align_incomplete-val :lambda-list '(m))
(cl:defmethod can_radar_align_incomplete-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radar_align_incomplete-val is deprecated.  Use delphi_mrr_msgs-msg:can_radar_align_incomplete instead.")
  (can_radar_align_incomplete m))

(cl:ensure-generic-function 'can_blockage_sidelobe-val :lambda-list '(m))
(cl:defmethod can_blockage_sidelobe-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_blockage_sidelobe-val is deprecated.  Use delphi_mrr_msgs-msg:can_blockage_sidelobe instead.")
  (can_blockage_sidelobe m))

(cl:ensure-generic-function 'can_blockage_mnr-val :lambda-list '(m))
(cl:defmethod can_blockage_mnr-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_blockage_mnr-val is deprecated.  Use delphi_mrr_msgs-msg:can_blockage_mnr instead.")
  (can_blockage_mnr m))

(cl:ensure-generic-function 'can_radar_ext_cond_nok-val :lambda-list '(m))
(cl:defmethod can_radar_ext_cond_nok-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radar_ext_cond_nok-val is deprecated.  Use delphi_mrr_msgs-msg:can_radar_ext_cond_nok instead.")
  (can_radar_ext_cond_nok m))

(cl:ensure-generic-function 'can_radar_align_out_range-val :lambda-list '(m))
(cl:defmethod can_radar_align_out_range-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radar_align_out_range-val is deprecated.  Use delphi_mrr_msgs-msg:can_radar_align_out_range instead.")
  (can_radar_align_out_range m))

(cl:ensure-generic-function 'can_radar_align_not_start-val :lambda-list '(m))
(cl:defmethod can_radar_align_not_start-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radar_align_not_start-val is deprecated.  Use delphi_mrr_msgs-msg:can_radar_align_not_start instead.")
  (can_radar_align_not_start m))

(cl:ensure-generic-function 'can_radar_overheat_error-val :lambda-list '(m))
(cl:defmethod can_radar_overheat_error-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radar_overheat_error-val is deprecated.  Use delphi_mrr_msgs-msg:can_radar_overheat_error instead.")
  (can_radar_overheat_error m))

(cl:ensure-generic-function 'can_radar_not_op-val :lambda-list '(m))
(cl:defmethod can_radar_not_op-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_radar_not_op-val is deprecated.  Use delphi_mrr_msgs-msg:can_radar_not_op instead.")
  (can_radar_not_op m))

(cl:ensure-generic-function 'can_xcvr_operational-val :lambda-list '(m))
(cl:defmethod can_xcvr_operational-val ((m <MrrStatusRadar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_xcvr_operational-val is deprecated.  Use delphi_mrr_msgs-msg:can_xcvr_operational instead.")
  (can_xcvr_operational m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrStatusRadar>) ostream)
  "Serializes a message object of type '<MrrStatusRadar>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_interference_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_recommend_unconverge) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_blockage_sidelobe_filter_val)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radar_align_incomplete) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_blockage_sidelobe) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_blockage_mnr) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radar_ext_cond_nok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radar_align_out_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radar_align_not_start) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radar_overheat_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_radar_not_op) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_xcvr_operational) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrStatusRadar>) istream)
  "Deserializes a message object of type '<MrrStatusRadar>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_interference_type)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_recommend_unconverge) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_blockage_sidelobe_filter_val)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_radar_align_incomplete) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_blockage_sidelobe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_blockage_mnr) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_radar_ext_cond_nok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_radar_align_out_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_radar_align_not_start) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_radar_overheat_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_radar_not_op) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_xcvr_operational) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrStatusRadar>)))
  "Returns string type for a message object of type '<MrrStatusRadar>"
  "delphi_mrr_msgs/MrrStatusRadar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrStatusRadar)))
  "Returns string type for a message object of type 'MrrStatusRadar"
  "delphi_mrr_msgs/MrrStatusRadar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrStatusRadar>)))
  "Returns md5sum for a message object of type '<MrrStatusRadar>"
  "3dbdaa8c61c744a4f2863586bf997cac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrStatusRadar)))
  "Returns md5sum for a message object of type 'MrrStatusRadar"
  "3dbdaa8c61c744a4f2863586bf997cac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrStatusRadar>)))
  "Returns full string definition for message of type '<MrrStatusRadar>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 can_interference_type~%bool  can_recommend_unconverge~%uint8 can_blockage_sidelobe_filter_val~%bool  can_radar_align_incomplete~%bool  can_blockage_sidelobe~%bool  can_blockage_mnr~%bool  can_radar_ext_cond_nok~%bool  can_radar_align_out_range~%bool  can_radar_align_not_start~%bool  can_radar_overheat_error~%bool  can_radar_not_op~%bool  can_xcvr_operational~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrStatusRadar)))
  "Returns full string definition for message of type 'MrrStatusRadar"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 can_interference_type~%bool  can_recommend_unconverge~%uint8 can_blockage_sidelobe_filter_val~%bool  can_radar_align_incomplete~%bool  can_blockage_sidelobe~%bool  can_blockage_mnr~%bool  can_radar_ext_cond_nok~%bool  can_radar_align_out_range~%bool  can_radar_align_not_start~%bool  can_radar_overheat_error~%bool  can_radar_not_op~%bool  can_xcvr_operational~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrStatusRadar>))
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
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrStatusRadar>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrStatusRadar
    (cl:cons ':header (header msg))
    (cl:cons ':can_interference_type (can_interference_type msg))
    (cl:cons ':can_recommend_unconverge (can_recommend_unconverge msg))
    (cl:cons ':can_blockage_sidelobe_filter_val (can_blockage_sidelobe_filter_val msg))
    (cl:cons ':can_radar_align_incomplete (can_radar_align_incomplete msg))
    (cl:cons ':can_blockage_sidelobe (can_blockage_sidelobe msg))
    (cl:cons ':can_blockage_mnr (can_blockage_mnr msg))
    (cl:cons ':can_radar_ext_cond_nok (can_radar_ext_cond_nok msg))
    (cl:cons ':can_radar_align_out_range (can_radar_align_out_range msg))
    (cl:cons ':can_radar_align_not_start (can_radar_align_not_start msg))
    (cl:cons ':can_radar_overheat_error (can_radar_overheat_error msg))
    (cl:cons ':can_radar_not_op (can_radar_not_op msg))
    (cl:cons ':can_xcvr_operational (can_xcvr_operational msg))
))
