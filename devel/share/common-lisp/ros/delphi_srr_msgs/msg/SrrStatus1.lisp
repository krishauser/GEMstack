; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrStatus1.msg.html

(cl:defclass <SrrStatus1> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_tx_look_type
    :reader can_tx_look_type
    :initarg :can_tx_look_type
    :type cl:boolean
    :initform cl:nil)
   (can_tx_dsp_timestamp
    :reader can_tx_dsp_timestamp
    :initarg :can_tx_dsp_timestamp
    :type cl:integer
    :initform 0)
   (can_tx_yaw_rate_calc
    :reader can_tx_yaw_rate_calc
    :initarg :can_tx_yaw_rate_calc
    :type cl:float
    :initform 0.0)
   (can_tx_vehicle_speed_calc
    :reader can_tx_vehicle_speed_calc
    :initarg :can_tx_vehicle_speed_calc
    :type cl:float
    :initform 0.0)
   (can_tx_scan_index
    :reader can_tx_scan_index
    :initarg :can_tx_scan_index
    :type cl:fixnum
    :initform 0)
   (can_tx_curvature
    :reader can_tx_curvature
    :initarg :can_tx_curvature
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrrStatus1 (<SrrStatus1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrStatus1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrStatus1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrStatus1> is deprecated: use delphi_srr_msgs-msg:SrrStatus1 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_tx_look_type-val :lambda-list '(m))
(cl:defmethod can_tx_look_type-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_look_type-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_look_type instead.")
  (can_tx_look_type m))

(cl:ensure-generic-function 'can_tx_dsp_timestamp-val :lambda-list '(m))
(cl:defmethod can_tx_dsp_timestamp-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_dsp_timestamp-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_dsp_timestamp instead.")
  (can_tx_dsp_timestamp m))

(cl:ensure-generic-function 'can_tx_yaw_rate_calc-val :lambda-list '(m))
(cl:defmethod can_tx_yaw_rate_calc-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_yaw_rate_calc-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_yaw_rate_calc instead.")
  (can_tx_yaw_rate_calc m))

(cl:ensure-generic-function 'can_tx_vehicle_speed_calc-val :lambda-list '(m))
(cl:defmethod can_tx_vehicle_speed_calc-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_vehicle_speed_calc-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_vehicle_speed_calc instead.")
  (can_tx_vehicle_speed_calc m))

(cl:ensure-generic-function 'can_tx_scan_index-val :lambda-list '(m))
(cl:defmethod can_tx_scan_index-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_scan_index-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_scan_index instead.")
  (can_tx_scan_index m))

(cl:ensure-generic-function 'can_tx_curvature-val :lambda-list '(m))
(cl:defmethod can_tx_curvature-val ((m <SrrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_curvature-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_curvature instead.")
  (can_tx_curvature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrStatus1>) ostream)
  "Serializes a message object of type '<SrrStatus1>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_look_type) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_dsp_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_dsp_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'can_tx_dsp_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'can_tx_dsp_timestamp)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_yaw_rate_calc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_vehicle_speed_calc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_scan_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_scan_index)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_curvature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrStatus1>) istream)
  "Deserializes a message object of type '<SrrStatus1>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'can_tx_look_type) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_dsp_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_dsp_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'can_tx_dsp_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'can_tx_dsp_timestamp)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_yaw_rate_calc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_vehicle_speed_calc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_scan_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_scan_index)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_curvature) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrStatus1>)))
  "Returns string type for a message object of type '<SrrStatus1>"
  "delphi_srr_msgs/SrrStatus1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrStatus1)))
  "Returns string type for a message object of type 'SrrStatus1"
  "delphi_srr_msgs/SrrStatus1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrStatus1>)))
  "Returns md5sum for a message object of type '<SrrStatus1>"
  "585df8ad7a5b009cc9f6c14365cc686e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrStatus1)))
  "Returns md5sum for a message object of type 'SrrStatus1"
  "585df8ad7a5b009cc9f6c14365cc686e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrStatus1>)))
  "Returns full string definition for message of type '<SrrStatus1>"
  (cl:format cl:nil "# Message file for srr_status1~%~%std_msgs/Header header~%~%bool      can_tx_look_type~%uint32    can_tx_dsp_timestamp                     # ms~%float32   can_tx_yaw_rate_calc                     # deg/s~%float32   can_tx_vehicle_speed_calc                # m/s~%uint16    can_tx_scan_index~%float32   can_tx_curvature                         # 1/m~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrStatus1)))
  "Returns full string definition for message of type 'SrrStatus1"
  (cl:format cl:nil "# Message file for srr_status1~%~%std_msgs/Header header~%~%bool      can_tx_look_type~%uint32    can_tx_dsp_timestamp                     # ms~%float32   can_tx_yaw_rate_calc                     # deg/s~%float32   can_tx_vehicle_speed_calc                # m/s~%uint16    can_tx_scan_index~%float32   can_tx_curvature                         # 1/m~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrStatus1>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4
     4
     4
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrStatus1>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrStatus1
    (cl:cons ':header (header msg))
    (cl:cons ':can_tx_look_type (can_tx_look_type msg))
    (cl:cons ':can_tx_dsp_timestamp (can_tx_dsp_timestamp msg))
    (cl:cons ':can_tx_yaw_rate_calc (can_tx_yaw_rate_calc msg))
    (cl:cons ':can_tx_vehicle_speed_calc (can_tx_vehicle_speed_calc msg))
    (cl:cons ':can_tx_scan_index (can_tx_scan_index msg))
    (cl:cons ':can_tx_curvature (can_tx_curvature msg))
))
