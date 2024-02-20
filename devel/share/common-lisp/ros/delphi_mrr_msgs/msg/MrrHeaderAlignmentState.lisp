; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrHeaderAlignmentState.msg.html

(cl:defclass <MrrHeaderAlignmentState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_auto_align_hangle_qf
    :reader can_auto_align_hangle_qf
    :initarg :can_auto_align_hangle_qf
    :type cl:fixnum
    :initform 0)
   (can_alignment_status
    :reader can_alignment_status
    :initarg :can_alignment_status
    :type cl:fixnum
    :initform 0)
   (can_alignment_state
    :reader can_alignment_state
    :initarg :can_alignment_state
    :type cl:fixnum
    :initform 0)
   (can_auto_align_hangle_ref
    :reader can_auto_align_hangle_ref
    :initarg :can_auto_align_hangle_ref
    :type cl:float
    :initform 0.0)
   (can_auto_align_hangle
    :reader can_auto_align_hangle
    :initarg :can_auto_align_hangle
    :type cl:float
    :initform 0.0))
)

(cl:defclass MrrHeaderAlignmentState (<MrrHeaderAlignmentState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrHeaderAlignmentState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrHeaderAlignmentState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrHeaderAlignmentState> is deprecated: use delphi_mrr_msgs-msg:MrrHeaderAlignmentState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrHeaderAlignmentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_auto_align_hangle_qf-val :lambda-list '(m))
(cl:defmethod can_auto_align_hangle_qf-val ((m <MrrHeaderAlignmentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_auto_align_hangle_qf-val is deprecated.  Use delphi_mrr_msgs-msg:can_auto_align_hangle_qf instead.")
  (can_auto_align_hangle_qf m))

(cl:ensure-generic-function 'can_alignment_status-val :lambda-list '(m))
(cl:defmethod can_alignment_status-val ((m <MrrHeaderAlignmentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_alignment_status-val is deprecated.  Use delphi_mrr_msgs-msg:can_alignment_status instead.")
  (can_alignment_status m))

(cl:ensure-generic-function 'can_alignment_state-val :lambda-list '(m))
(cl:defmethod can_alignment_state-val ((m <MrrHeaderAlignmentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_alignment_state-val is deprecated.  Use delphi_mrr_msgs-msg:can_alignment_state instead.")
  (can_alignment_state m))

(cl:ensure-generic-function 'can_auto_align_hangle_ref-val :lambda-list '(m))
(cl:defmethod can_auto_align_hangle_ref-val ((m <MrrHeaderAlignmentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_auto_align_hangle_ref-val is deprecated.  Use delphi_mrr_msgs-msg:can_auto_align_hangle_ref instead.")
  (can_auto_align_hangle_ref m))

(cl:ensure-generic-function 'can_auto_align_hangle-val :lambda-list '(m))
(cl:defmethod can_auto_align_hangle-val ((m <MrrHeaderAlignmentState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_auto_align_hangle-val is deprecated.  Use delphi_mrr_msgs-msg:can_auto_align_hangle instead.")
  (can_auto_align_hangle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrHeaderAlignmentState>) ostream)
  "Serializes a message object of type '<MrrHeaderAlignmentState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_auto_align_hangle_qf)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_alignment_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_alignment_state)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_auto_align_hangle_ref))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_auto_align_hangle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrHeaderAlignmentState>) istream)
  "Deserializes a message object of type '<MrrHeaderAlignmentState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_auto_align_hangle_qf)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_alignment_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_alignment_state)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_auto_align_hangle_ref) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_auto_align_hangle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrHeaderAlignmentState>)))
  "Returns string type for a message object of type '<MrrHeaderAlignmentState>"
  "delphi_mrr_msgs/MrrHeaderAlignmentState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrHeaderAlignmentState)))
  "Returns string type for a message object of type 'MrrHeaderAlignmentState"
  "delphi_mrr_msgs/MrrHeaderAlignmentState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrHeaderAlignmentState>)))
  "Returns md5sum for a message object of type '<MrrHeaderAlignmentState>"
  "ed76a328bc6693a98452aedfe696f11a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrHeaderAlignmentState)))
  "Returns md5sum for a message object of type 'MrrHeaderAlignmentState"
  "ed76a328bc6693a98452aedfe696f11a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrHeaderAlignmentState>)))
  "Returns full string definition for message of type '<MrrHeaderAlignmentState>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8   can_auto_align_hangle_qf~%uint8   can_alignment_status~%uint8   can_alignment_state~%float32 can_auto_align_hangle_ref~%float32 can_auto_align_hangle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrHeaderAlignmentState)))
  "Returns full string definition for message of type 'MrrHeaderAlignmentState"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8   can_auto_align_hangle_qf~%uint8   can_alignment_status~%uint8   can_alignment_state~%float32 can_auto_align_hangle_ref~%float32 can_auto_align_hangle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrHeaderAlignmentState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrHeaderAlignmentState>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrHeaderAlignmentState
    (cl:cons ':header (header msg))
    (cl:cons ':can_auto_align_hangle_qf (can_auto_align_hangle_qf msg))
    (cl:cons ':can_alignment_status (can_alignment_status msg))
    (cl:cons ':can_alignment_state (can_alignment_state msg))
    (cl:cons ':can_auto_align_hangle_ref (can_auto_align_hangle_ref msg))
    (cl:cons ':can_auto_align_hangle (can_auto_align_hangle msg))
))
