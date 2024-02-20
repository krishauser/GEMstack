; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrStatusCanVersion.msg.html

(cl:defclass <MrrStatusCanVersion> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_usc_section_compatibility
    :reader can_usc_section_compatibility
    :initarg :can_usc_section_compatibility
    :type cl:fixnum
    :initform 0)
   (can_pcan_minor_mrr
    :reader can_pcan_minor_mrr
    :initarg :can_pcan_minor_mrr
    :type cl:fixnum
    :initform 0)
   (can_pcan_major_mrr
    :reader can_pcan_major_mrr
    :initarg :can_pcan_major_mrr
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrStatusCanVersion (<MrrStatusCanVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrStatusCanVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrStatusCanVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrStatusCanVersion> is deprecated: use delphi_mrr_msgs-msg:MrrStatusCanVersion instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrStatusCanVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_usc_section_compatibility-val :lambda-list '(m))
(cl:defmethod can_usc_section_compatibility-val ((m <MrrStatusCanVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_usc_section_compatibility-val is deprecated.  Use delphi_mrr_msgs-msg:can_usc_section_compatibility instead.")
  (can_usc_section_compatibility m))

(cl:ensure-generic-function 'can_pcan_minor_mrr-val :lambda-list '(m))
(cl:defmethod can_pcan_minor_mrr-val ((m <MrrStatusCanVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_pcan_minor_mrr-val is deprecated.  Use delphi_mrr_msgs-msg:can_pcan_minor_mrr instead.")
  (can_pcan_minor_mrr m))

(cl:ensure-generic-function 'can_pcan_major_mrr-val :lambda-list '(m))
(cl:defmethod can_pcan_major_mrr-val ((m <MrrStatusCanVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_pcan_major_mrr-val is deprecated.  Use delphi_mrr_msgs-msg:can_pcan_major_mrr instead.")
  (can_pcan_major_mrr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrStatusCanVersion>) ostream)
  "Serializes a message object of type '<MrrStatusCanVersion>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_usc_section_compatibility)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_usc_section_compatibility)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pcan_minor_mrr)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pcan_major_mrr)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrStatusCanVersion>) istream)
  "Deserializes a message object of type '<MrrStatusCanVersion>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_usc_section_compatibility)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_usc_section_compatibility)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pcan_minor_mrr)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_pcan_major_mrr)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrStatusCanVersion>)))
  "Returns string type for a message object of type '<MrrStatusCanVersion>"
  "delphi_mrr_msgs/MrrStatusCanVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrStatusCanVersion)))
  "Returns string type for a message object of type 'MrrStatusCanVersion"
  "delphi_mrr_msgs/MrrStatusCanVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrStatusCanVersion>)))
  "Returns md5sum for a message object of type '<MrrStatusCanVersion>"
  "a03c2b40596b39ba528bf17fb334bae6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrStatusCanVersion)))
  "Returns md5sum for a message object of type 'MrrStatusCanVersion"
  "a03c2b40596b39ba528bf17fb334bae6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrStatusCanVersion>)))
  "Returns full string definition for message of type '<MrrStatusCanVersion>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 can_usc_section_compatibility~%uint8  can_pcan_minor_mrr~%uint8  can_pcan_major_mrr~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrStatusCanVersion)))
  "Returns full string definition for message of type 'MrrStatusCanVersion"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 can_usc_section_compatibility~%uint8  can_pcan_minor_mrr~%uint8  can_pcan_major_mrr~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrStatusCanVersion>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrStatusCanVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrStatusCanVersion
    (cl:cons ':header (header msg))
    (cl:cons ':can_usc_section_compatibility (can_usc_section_compatibility msg))
    (cl:cons ':can_pcan_minor_mrr (can_pcan_minor_mrr msg))
    (cl:cons ':can_pcan_major_mrr (can_pcan_major_mrr msg))
))
