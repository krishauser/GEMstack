; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrFeatureAlert.msg.html

(cl:defclass <SrrFeatureAlert> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lcma_blis_ignored_track_id
    :reader lcma_blis_ignored_track_id
    :initarg :lcma_blis_ignored_track_id
    :type cl:fixnum
    :initform 0)
   (lcma_blis_track_id
    :reader lcma_blis_track_id
    :initarg :lcma_blis_track_id
    :type cl:fixnum
    :initform 0)
   (lcma_cvw_ttc
    :reader lcma_cvw_ttc
    :initarg :lcma_cvw_ttc
    :type cl:float
    :initform 0.0)
   (cta_ttc_alert
    :reader cta_ttc_alert
    :initarg :cta_ttc_alert
    :type cl:boolean
    :initform cl:nil)
   (cta_selected_track_ttc
    :reader cta_selected_track_ttc
    :initarg :cta_selected_track_ttc
    :type cl:float
    :initform 0.0)
   (cta_selected_track
    :reader cta_selected_track
    :initarg :cta_selected_track
    :type cl:fixnum
    :initform 0)
   (cta_alert
    :reader cta_alert
    :initarg :cta_alert
    :type cl:fixnum
    :initform 0)
   (cta_active
    :reader cta_active
    :initarg :cta_active
    :type cl:boolean
    :initform cl:nil)
   (lcma_cvw_cipv
    :reader lcma_cvw_cipv
    :initarg :lcma_cvw_cipv
    :type cl:fixnum
    :initform 0)
   (lcma_cvw_alert_state
    :reader lcma_cvw_alert_state
    :initarg :lcma_cvw_alert_state
    :type cl:fixnum
    :initform 0)
   (lcma_blis_alert_state
    :reader lcma_blis_alert_state
    :initarg :lcma_blis_alert_state
    :type cl:fixnum
    :initform 0)
   (lcma_active
    :reader lcma_active
    :initarg :lcma_active
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrrFeatureAlert (<SrrFeatureAlert>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrFeatureAlert>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrFeatureAlert)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrFeatureAlert> is deprecated: use delphi_srr_msgs-msg:SrrFeatureAlert instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lcma_blis_ignored_track_id-val :lambda-list '(m))
(cl:defmethod lcma_blis_ignored_track_id-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_blis_ignored_track_id-val is deprecated.  Use delphi_srr_msgs-msg:lcma_blis_ignored_track_id instead.")
  (lcma_blis_ignored_track_id m))

(cl:ensure-generic-function 'lcma_blis_track_id-val :lambda-list '(m))
(cl:defmethod lcma_blis_track_id-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_blis_track_id-val is deprecated.  Use delphi_srr_msgs-msg:lcma_blis_track_id instead.")
  (lcma_blis_track_id m))

(cl:ensure-generic-function 'lcma_cvw_ttc-val :lambda-list '(m))
(cl:defmethod lcma_cvw_ttc-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_cvw_ttc-val is deprecated.  Use delphi_srr_msgs-msg:lcma_cvw_ttc instead.")
  (lcma_cvw_ttc m))

(cl:ensure-generic-function 'cta_ttc_alert-val :lambda-list '(m))
(cl:defmethod cta_ttc_alert-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:cta_ttc_alert-val is deprecated.  Use delphi_srr_msgs-msg:cta_ttc_alert instead.")
  (cta_ttc_alert m))

(cl:ensure-generic-function 'cta_selected_track_ttc-val :lambda-list '(m))
(cl:defmethod cta_selected_track_ttc-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:cta_selected_track_ttc-val is deprecated.  Use delphi_srr_msgs-msg:cta_selected_track_ttc instead.")
  (cta_selected_track_ttc m))

(cl:ensure-generic-function 'cta_selected_track-val :lambda-list '(m))
(cl:defmethod cta_selected_track-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:cta_selected_track-val is deprecated.  Use delphi_srr_msgs-msg:cta_selected_track instead.")
  (cta_selected_track m))

(cl:ensure-generic-function 'cta_alert-val :lambda-list '(m))
(cl:defmethod cta_alert-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:cta_alert-val is deprecated.  Use delphi_srr_msgs-msg:cta_alert instead.")
  (cta_alert m))

(cl:ensure-generic-function 'cta_active-val :lambda-list '(m))
(cl:defmethod cta_active-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:cta_active-val is deprecated.  Use delphi_srr_msgs-msg:cta_active instead.")
  (cta_active m))

(cl:ensure-generic-function 'lcma_cvw_cipv-val :lambda-list '(m))
(cl:defmethod lcma_cvw_cipv-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_cvw_cipv-val is deprecated.  Use delphi_srr_msgs-msg:lcma_cvw_cipv instead.")
  (lcma_cvw_cipv m))

(cl:ensure-generic-function 'lcma_cvw_alert_state-val :lambda-list '(m))
(cl:defmethod lcma_cvw_alert_state-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_cvw_alert_state-val is deprecated.  Use delphi_srr_msgs-msg:lcma_cvw_alert_state instead.")
  (lcma_cvw_alert_state m))

(cl:ensure-generic-function 'lcma_blis_alert_state-val :lambda-list '(m))
(cl:defmethod lcma_blis_alert_state-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_blis_alert_state-val is deprecated.  Use delphi_srr_msgs-msg:lcma_blis_alert_state instead.")
  (lcma_blis_alert_state m))

(cl:ensure-generic-function 'lcma_active-val :lambda-list '(m))
(cl:defmethod lcma_active-val ((m <SrrFeatureAlert>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_active-val is deprecated.  Use delphi_srr_msgs-msg:lcma_active instead.")
  (lcma_active m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrFeatureAlert>) ostream)
  "Serializes a message object of type '<SrrFeatureAlert>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_blis_ignored_track_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_blis_track_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lcma_cvw_ttc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cta_ttc_alert) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cta_selected_track_ttc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cta_selected_track)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cta_selected_track)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cta_alert)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cta_active) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_cvw_cipv)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_cvw_alert_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_blis_alert_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lcma_active) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrFeatureAlert>) istream)
  "Deserializes a message object of type '<SrrFeatureAlert>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_blis_ignored_track_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_blis_track_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lcma_cvw_ttc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'cta_ttc_alert) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cta_selected_track_ttc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cta_selected_track)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cta_selected_track)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cta_alert)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cta_active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_cvw_cipv)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_cvw_alert_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_blis_alert_state)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lcma_active) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrFeatureAlert>)))
  "Returns string type for a message object of type '<SrrFeatureAlert>"
  "delphi_srr_msgs/SrrFeatureAlert")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrFeatureAlert)))
  "Returns string type for a message object of type 'SrrFeatureAlert"
  "delphi_srr_msgs/SrrFeatureAlert")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrFeatureAlert>)))
  "Returns md5sum for a message object of type '<SrrFeatureAlert>"
  "721bc54767b8d837fd2e98fc870215ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrFeatureAlert)))
  "Returns md5sum for a message object of type 'SrrFeatureAlert"
  "721bc54767b8d837fd2e98fc870215ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrFeatureAlert>)))
  "Returns full string definition for message of type '<SrrFeatureAlert>"
  (cl:format cl:nil "# Message file for srr_feature_alert~%~%std_msgs/Header header~%~%uint8     lcma_blis_ignored_track_id~%uint8     lcma_blis_track_id~%float32   lcma_cvw_ttc                             # seconds~%bool      cta_ttc_alert~%float32   cta_selected_track_ttc                   # seconds~%uint16    cta_selected_track~%uint8     cta_alert                                # binary~%bool      cta_active                               # binary~%uint8     lcma_cvw_cipv~%uint8     lcma_cvw_alert_state~%uint8     lcma_blis_alert_state~%bool      lcma_active~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrFeatureAlert)))
  "Returns full string definition for message of type 'SrrFeatureAlert"
  (cl:format cl:nil "# Message file for srr_feature_alert~%~%std_msgs/Header header~%~%uint8     lcma_blis_ignored_track_id~%uint8     lcma_blis_track_id~%float32   lcma_cvw_ttc                             # seconds~%bool      cta_ttc_alert~%float32   cta_selected_track_ttc                   # seconds~%uint16    cta_selected_track~%uint8     cta_alert                                # binary~%bool      cta_active                               # binary~%uint8     lcma_cvw_cipv~%uint8     lcma_cvw_alert_state~%uint8     lcma_blis_alert_state~%bool      lcma_active~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrFeatureAlert>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     1
     4
     2
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrFeatureAlert>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrFeatureAlert
    (cl:cons ':header (header msg))
    (cl:cons ':lcma_blis_ignored_track_id (lcma_blis_ignored_track_id msg))
    (cl:cons ':lcma_blis_track_id (lcma_blis_track_id msg))
    (cl:cons ':lcma_cvw_ttc (lcma_cvw_ttc msg))
    (cl:cons ':cta_ttc_alert (cta_ttc_alert msg))
    (cl:cons ':cta_selected_track_ttc (cta_selected_track_ttc msg))
    (cl:cons ':cta_selected_track (cta_selected_track msg))
    (cl:cons ':cta_alert (cta_alert msg))
    (cl:cons ':cta_active (cta_active msg))
    (cl:cons ':lcma_cvw_cipv (lcma_cvw_cipv msg))
    (cl:cons ':lcma_cvw_alert_state (lcma_cvw_alert_state msg))
    (cl:cons ':lcma_blis_alert_state (lcma_blis_alert_state msg))
    (cl:cons ':lcma_active (lcma_active msg))
))
