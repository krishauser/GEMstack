; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrDetection.msg.html

(cl:defclass <MrrDetection> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detection_id
    :reader detection_id
    :initarg :detection_id
    :type cl:fixnum
    :initform 0)
   (confid_azimuth
    :reader confid_azimuth
    :initarg :confid_azimuth
    :type cl:fixnum
    :initform 0)
   (super_res_target
    :reader super_res_target
    :initarg :super_res_target
    :type cl:boolean
    :initform cl:nil)
   (nd_target
    :reader nd_target
    :initarg :nd_target
    :type cl:boolean
    :initform cl:nil)
   (host_veh_clutter
    :reader host_veh_clutter
    :initarg :host_veh_clutter
    :type cl:boolean
    :initform cl:nil)
   (valid_level
    :reader valid_level
    :initarg :valid_level
    :type cl:boolean
    :initform cl:nil)
   (azimuth
    :reader azimuth
    :initarg :azimuth
    :type cl:float
    :initform 0.0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (range_rate
    :reader range_rate
    :initarg :range_rate
    :type cl:float
    :initform 0.0)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:fixnum
    :initform 0)
   (index_2lsb
    :reader index_2lsb
    :initarg :index_2lsb
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrDetection (<MrrDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrDetection> is deprecated: use delphi_mrr_msgs-msg:MrrDetection instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detection_id-val :lambda-list '(m))
(cl:defmethod detection_id-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:detection_id-val is deprecated.  Use delphi_mrr_msgs-msg:detection_id instead.")
  (detection_id m))

(cl:ensure-generic-function 'confid_azimuth-val :lambda-list '(m))
(cl:defmethod confid_azimuth-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:confid_azimuth-val is deprecated.  Use delphi_mrr_msgs-msg:confid_azimuth instead.")
  (confid_azimuth m))

(cl:ensure-generic-function 'super_res_target-val :lambda-list '(m))
(cl:defmethod super_res_target-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:super_res_target-val is deprecated.  Use delphi_mrr_msgs-msg:super_res_target instead.")
  (super_res_target m))

(cl:ensure-generic-function 'nd_target-val :lambda-list '(m))
(cl:defmethod nd_target-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:nd_target-val is deprecated.  Use delphi_mrr_msgs-msg:nd_target instead.")
  (nd_target m))

(cl:ensure-generic-function 'host_veh_clutter-val :lambda-list '(m))
(cl:defmethod host_veh_clutter-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:host_veh_clutter-val is deprecated.  Use delphi_mrr_msgs-msg:host_veh_clutter instead.")
  (host_veh_clutter m))

(cl:ensure-generic-function 'valid_level-val :lambda-list '(m))
(cl:defmethod valid_level-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:valid_level-val is deprecated.  Use delphi_mrr_msgs-msg:valid_level instead.")
  (valid_level m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:azimuth-val is deprecated.  Use delphi_mrr_msgs-msg:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:range-val is deprecated.  Use delphi_mrr_msgs-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'range_rate-val :lambda-list '(m))
(cl:defmethod range_rate-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:range_rate-val is deprecated.  Use delphi_mrr_msgs-msg:range_rate instead.")
  (range_rate m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:amplitude-val is deprecated.  Use delphi_mrr_msgs-msg:amplitude instead.")
  (amplitude m))

(cl:ensure-generic-function 'index_2lsb-val :lambda-list '(m))
(cl:defmethod index_2lsb-val ((m <MrrDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:index_2lsb-val is deprecated.  Use delphi_mrr_msgs-msg:index_2lsb instead.")
  (index_2lsb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrDetection>) ostream)
  "Serializes a message object of type '<MrrDetection>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confid_azimuth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'super_res_target) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'nd_target) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'host_veh_clutter) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid_level) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'amplitude)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_2lsb)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrDetection>) istream)
  "Deserializes a message object of type '<MrrDetection>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'detection_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'confid_azimuth)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'super_res_target) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'nd_target) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'host_veh_clutter) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'valid_level) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azimuth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'amplitude) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_2lsb)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrDetection>)))
  "Returns string type for a message object of type '<MrrDetection>"
  "delphi_mrr_msgs/MrrDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrDetection)))
  "Returns string type for a message object of type 'MrrDetection"
  "delphi_mrr_msgs/MrrDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrDetection>)))
  "Returns md5sum for a message object of type '<MrrDetection>"
  "beed6f988400c44635b6b62be6463175")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrDetection)))
  "Returns md5sum for a message object of type 'MrrDetection"
  "beed6f988400c44635b6b62be6463175")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrDetection>)))
  "Returns full string definition for message of type '<MrrDetection>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8   detection_id~%uint8   confid_azimuth~%bool    super_res_target~%bool    nd_target~%bool    host_veh_clutter~%bool    valid_level~%float32 azimuth~%float32 range~%float32 range_rate~%int8    amplitude~%uint8   index_2lsb~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrDetection)))
  "Returns full string definition for message of type 'MrrDetection"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8   detection_id~%uint8   confid_azimuth~%bool    super_res_target~%bool    nd_target~%bool    host_veh_clutter~%bool    valid_level~%float32 azimuth~%float32 range~%float32 range_rate~%int8    amplitude~%uint8   index_2lsb~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrDetection>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrDetection
    (cl:cons ':header (header msg))
    (cl:cons ':detection_id (detection_id msg))
    (cl:cons ':confid_azimuth (confid_azimuth msg))
    (cl:cons ':super_res_target (super_res_target msg))
    (cl:cons ':nd_target (nd_target msg))
    (cl:cons ':host_veh_clutter (host_veh_clutter msg))
    (cl:cons ':valid_level (valid_level msg))
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':range (range msg))
    (cl:cons ':range_rate (range_rate msg))
    (cl:cons ':amplitude (amplitude msg))
    (cl:cons ':index_2lsb (index_2lsb msg))
))
