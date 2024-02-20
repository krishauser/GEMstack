; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrStatusSwVersion.msg.html

(cl:defclass <MrrStatusSwVersion> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_pbl_field_revision
    :reader can_pbl_field_revision
    :initarg :can_pbl_field_revision
    :type cl:fixnum
    :initform 0)
   (can_pbl_promote_revision
    :reader can_pbl_promote_revision
    :initarg :can_pbl_promote_revision
    :type cl:fixnum
    :initform 0)
   (can_sw_field_revision
    :reader can_sw_field_revision
    :initarg :can_sw_field_revision
    :type cl:fixnum
    :initform 0)
   (can_sw_promote_revision
    :reader can_sw_promote_revision
    :initarg :can_sw_promote_revision
    :type cl:fixnum
    :initform 0)
   (can_sw_release_revision
    :reader can_sw_release_revision
    :initarg :can_sw_release_revision
    :type cl:fixnum
    :initform 0)
   (can_pbl_release_revision
    :reader can_pbl_release_revision
    :initarg :can_pbl_release_revision
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrStatusSwVersion (<MrrStatusSwVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrStatusSwVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrStatusSwVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrStatusSwVersion> is deprecated: use delphi_mrr_msgs-msg:MrrStatusSwVersion instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_pbl_field_revision-val :lambda-list '(m))
(cl:defmethod can_pbl_field_revision-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_pbl_field_revision-val is deprecated.  Use delphi_mrr_msgs-msg:can_pbl_field_revision instead.")
  (can_pbl_field_revision m))

(cl:ensure-generic-function 'can_pbl_promote_revision-val :lambda-list '(m))
(cl:defmethod can_pbl_promote_revision-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_pbl_promote_revision-val is deprecated.  Use delphi_mrr_msgs-msg:can_pbl_promote_revision instead.")
  (can_pbl_promote_revision m))

(cl:ensure-generic-function 'can_sw_field_revision-val :lambda-list '(m))
(cl:defmethod can_sw_field_revision-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sw_field_revision-val is deprecated.  Use delphi_mrr_msgs-msg:can_sw_field_revision instead.")
  (can_sw_field_revision m))

(cl:ensure-generic-function 'can_sw_promote_revision-val :lambda-list '(m))
(cl:defmethod can_sw_promote_revision-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sw_promote_revision-val is deprecated.  Use delphi_mrr_msgs-msg:can_sw_promote_revision instead.")
  (can_sw_promote_revision m))

(cl:ensure-generic-function 'can_sw_release_revision-val :lambda-list '(m))
(cl:defmethod can_sw_release_revision-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_sw_release_revision-val is deprecated.  Use delphi_mrr_msgs-msg:can_sw_release_revision instead.")
  (can_sw_release_revision m))

(cl:ensure-generic-function 'can_pbl_release_revision-val :lambda-list '(m))
(cl:defmethod can_pbl_release_revision-val ((m <MrrStatusSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_pbl_release_revision-val is deprecated.  Use delphi_mrr_msgs-msg:can_pbl_release_revision instead.")
  (can_pbl_release_revision m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrStatusSwVersion>) ostream)
  "Serializes a message object of type '<MrrStatusSwVersion>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pbl_field_revision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pbl_promote_revision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sw_field_revision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sw_promote_revision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sw_release_revision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pbl_release_revision)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrStatusSwVersion>) istream)
  "Deserializes a message object of type '<MrrStatusSwVersion>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pbl_field_revision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pbl_promote_revision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sw_field_revision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sw_promote_revision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_sw_release_revision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pbl_release_revision)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrStatusSwVersion>)))
  "Returns string type for a message object of type '<MrrStatusSwVersion>"
  "delphi_mrr_msgs/MrrStatusSwVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrStatusSwVersion)))
  "Returns string type for a message object of type 'MrrStatusSwVersion"
  "delphi_mrr_msgs/MrrStatusSwVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrStatusSwVersion>)))
  "Returns md5sum for a message object of type '<MrrStatusSwVersion>"
  "2cd8dd3a73da2efcca1c60aaa8a7f9c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrStatusSwVersion)))
  "Returns md5sum for a message object of type 'MrrStatusSwVersion"
  "2cd8dd3a73da2efcca1c60aaa8a7f9c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrStatusSwVersion>)))
  "Returns full string definition for message of type '<MrrStatusSwVersion>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 can_pbl_field_revision~%uint8 can_pbl_promote_revision~%uint8 can_sw_field_revision~%uint8 can_sw_promote_revision~%uint8 can_sw_release_revision~%uint8 can_pbl_release_revision~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrStatusSwVersion)))
  "Returns full string definition for message of type 'MrrStatusSwVersion"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 can_pbl_field_revision~%uint8 can_pbl_promote_revision~%uint8 can_sw_field_revision~%uint8 can_sw_promote_revision~%uint8 can_sw_release_revision~%uint8 can_pbl_release_revision~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrStatusSwVersion>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrStatusSwVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrStatusSwVersion
    (cl:cons ':header (header msg))
    (cl:cons ':can_pbl_field_revision (can_pbl_field_revision msg))
    (cl:cons ':can_pbl_promote_revision (can_pbl_promote_revision msg))
    (cl:cons ':can_sw_field_revision (can_sw_field_revision msg))
    (cl:cons ':can_sw_promote_revision (can_sw_promote_revision msg))
    (cl:cons ':can_sw_release_revision (can_sw_release_revision msg))
    (cl:cons ':can_pbl_release_revision (can_pbl_release_revision msg))
))
