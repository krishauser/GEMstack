; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrStatus4.msg.html

(cl:defclass <SrrStatus4> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_tx_sw_version_host
    :reader can_tx_sw_version_host
    :initarg :can_tx_sw_version_host
    :type cl:fixnum
    :initform 0)
   (can_tx_path_id_blis_ignore
    :reader can_tx_path_id_blis_ignore
    :initarg :can_tx_path_id_blis_ignore
    :type cl:fixnum
    :initform 0)
   (can_tx_path_id_blis
    :reader can_tx_path_id_blis
    :initarg :can_tx_path_id_blis
    :type cl:fixnum
    :initform 0)
   (can_tx_angle_misalignment
    :reader can_tx_angle_misalignment
    :initarg :can_tx_angle_misalignment
    :type cl:float
    :initform 0.0)
   (can_tx_auto_align_angle
    :reader can_tx_auto_align_angle
    :initarg :can_tx_auto_align_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrrStatus4 (<SrrStatus4>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrStatus4>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrStatus4)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrStatus4> is deprecated: use delphi_srr_msgs-msg:SrrStatus4 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_tx_sw_version_host-val :lambda-list '(m))
(cl:defmethod can_tx_sw_version_host-val ((m <SrrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_sw_version_host-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_sw_version_host instead.")
  (can_tx_sw_version_host m))

(cl:ensure-generic-function 'can_tx_path_id_blis_ignore-val :lambda-list '(m))
(cl:defmethod can_tx_path_id_blis_ignore-val ((m <SrrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_path_id_blis_ignore-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_path_id_blis_ignore instead.")
  (can_tx_path_id_blis_ignore m))

(cl:ensure-generic-function 'can_tx_path_id_blis-val :lambda-list '(m))
(cl:defmethod can_tx_path_id_blis-val ((m <SrrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_path_id_blis-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_path_id_blis instead.")
  (can_tx_path_id_blis m))

(cl:ensure-generic-function 'can_tx_angle_misalignment-val :lambda-list '(m))
(cl:defmethod can_tx_angle_misalignment-val ((m <SrrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_angle_misalignment-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_angle_misalignment instead.")
  (can_tx_angle_misalignment m))

(cl:ensure-generic-function 'can_tx_auto_align_angle-val :lambda-list '(m))
(cl:defmethod can_tx_auto_align_angle-val ((m <SrrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_auto_align_angle-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_auto_align_angle instead.")
  (can_tx_auto_align_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrStatus4>) ostream)
  "Serializes a message object of type '<SrrStatus4>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_sw_version_host)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_sw_version_host)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_path_id_blis_ignore)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_path_id_blis)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_angle_misalignment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_auto_align_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrStatus4>) istream)
  "Deserializes a message object of type '<SrrStatus4>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_sw_version_host)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_sw_version_host)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_path_id_blis_ignore)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_path_id_blis)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_angle_misalignment) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_auto_align_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrStatus4>)))
  "Returns string type for a message object of type '<SrrStatus4>"
  "delphi_srr_msgs/SrrStatus4")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrStatus4)))
  "Returns string type for a message object of type 'SrrStatus4"
  "delphi_srr_msgs/SrrStatus4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrStatus4>)))
  "Returns md5sum for a message object of type '<SrrStatus4>"
  "8f5e5c4790453e1f3bcd5507dd8162bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrStatus4)))
  "Returns md5sum for a message object of type 'SrrStatus4"
  "8f5e5c4790453e1f3bcd5507dd8162bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrStatus4>)))
  "Returns full string definition for message of type '<SrrStatus4>"
  (cl:format cl:nil "# Message file for srr_status4~%~%std_msgs/Header header~%~%uint16    can_tx_sw_version_host~%uint8     can_tx_path_id_blis_ignore~%uint8     can_tx_path_id_blis~%float32   can_tx_angle_misalignment~%float32   can_tx_auto_align_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrStatus4)))
  "Returns full string definition for message of type 'SrrStatus4"
  (cl:format cl:nil "# Message file for srr_status4~%~%std_msgs/Header header~%~%uint16    can_tx_sw_version_host~%uint8     can_tx_path_id_blis_ignore~%uint8     can_tx_path_id_blis~%float32   can_tx_angle_misalignment~%float32   can_tx_auto_align_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrStatus4>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrStatus4>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrStatus4
    (cl:cons ':header (header msg))
    (cl:cons ':can_tx_sw_version_host (can_tx_sw_version_host msg))
    (cl:cons ':can_tx_path_id_blis_ignore (can_tx_path_id_blis_ignore msg))
    (cl:cons ':can_tx_path_id_blis (can_tx_path_id_blis msg))
    (cl:cons ':can_tx_angle_misalignment (can_tx_angle_misalignment msg))
    (cl:cons ':can_tx_auto_align_angle (can_tx_auto_align_angle msg))
))
