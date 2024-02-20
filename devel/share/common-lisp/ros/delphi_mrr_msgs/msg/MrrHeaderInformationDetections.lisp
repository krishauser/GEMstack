; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrHeaderInformationDetections.msg.html

(cl:defclass <MrrHeaderInformationDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_align_updates_done
    :reader can_align_updates_done
    :initarg :can_align_updates_done
    :type cl:fixnum
    :initform 0)
   (can_scan_index
    :reader can_scan_index
    :initarg :can_scan_index
    :type cl:fixnum
    :initform 0)
   (can_number_of_det
    :reader can_number_of_det
    :initarg :can_number_of_det
    :type cl:fixnum
    :initform 0)
   (can_look_id
    :reader can_look_id
    :initarg :can_look_id
    :type cl:fixnum
    :initform 0)
   (can_look_index
    :reader can_look_index
    :initarg :can_look_index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrHeaderInformationDetections (<MrrHeaderInformationDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrHeaderInformationDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrHeaderInformationDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrHeaderInformationDetections> is deprecated: use delphi_mrr_msgs-msg:MrrHeaderInformationDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrHeaderInformationDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_align_updates_done-val :lambda-list '(m))
(cl:defmethod can_align_updates_done-val ((m <MrrHeaderInformationDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_align_updates_done-val is deprecated.  Use delphi_mrr_msgs-msg:can_align_updates_done instead.")
  (can_align_updates_done m))

(cl:ensure-generic-function 'can_scan_index-val :lambda-list '(m))
(cl:defmethod can_scan_index-val ((m <MrrHeaderInformationDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_scan_index-val is deprecated.  Use delphi_mrr_msgs-msg:can_scan_index instead.")
  (can_scan_index m))

(cl:ensure-generic-function 'can_number_of_det-val :lambda-list '(m))
(cl:defmethod can_number_of_det-val ((m <MrrHeaderInformationDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_number_of_det-val is deprecated.  Use delphi_mrr_msgs-msg:can_number_of_det instead.")
  (can_number_of_det m))

(cl:ensure-generic-function 'can_look_id-val :lambda-list '(m))
(cl:defmethod can_look_id-val ((m <MrrHeaderInformationDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_look_id-val is deprecated.  Use delphi_mrr_msgs-msg:can_look_id instead.")
  (can_look_id m))

(cl:ensure-generic-function 'can_look_index-val :lambda-list '(m))
(cl:defmethod can_look_index-val ((m <MrrHeaderInformationDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_look_index-val is deprecated.  Use delphi_mrr_msgs-msg:can_look_index instead.")
  (can_look_index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrHeaderInformationDetections>) ostream)
  "Serializes a message object of type '<MrrHeaderInformationDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_align_updates_done)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_align_updates_done)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_scan_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_scan_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_number_of_det)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_look_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_look_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_look_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrHeaderInformationDetections>) istream)
  "Deserializes a message object of type '<MrrHeaderInformationDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_align_updates_done)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_align_updates_done)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_scan_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_scan_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_number_of_det)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_look_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_look_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_look_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrHeaderInformationDetections>)))
  "Returns string type for a message object of type '<MrrHeaderInformationDetections>"
  "delphi_mrr_msgs/MrrHeaderInformationDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrHeaderInformationDetections)))
  "Returns string type for a message object of type 'MrrHeaderInformationDetections"
  "delphi_mrr_msgs/MrrHeaderInformationDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrHeaderInformationDetections>)))
  "Returns md5sum for a message object of type '<MrrHeaderInformationDetections>"
  "39494a4731101be0be38c8ac22c1c084")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrHeaderInformationDetections)))
  "Returns md5sum for a message object of type 'MrrHeaderInformationDetections"
  "39494a4731101be0be38c8ac22c1c084")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrHeaderInformationDetections>)))
  "Returns full string definition for message of type '<MrrHeaderInformationDetections>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 can_align_updates_done~%uint16 can_scan_index~%uint8  can_number_of_det~%uint8  can_look_id~%uint16 can_look_index~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrHeaderInformationDetections)))
  "Returns full string definition for message of type 'MrrHeaderInformationDetections"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 can_align_updates_done~%uint16 can_scan_index~%uint8  can_number_of_det~%uint8  can_look_id~%uint16 can_look_index~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrHeaderInformationDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrHeaderInformationDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrHeaderInformationDetections
    (cl:cons ':header (header msg))
    (cl:cons ':can_align_updates_done (can_align_updates_done msg))
    (cl:cons ':can_scan_index (can_scan_index msg))
    (cl:cons ':can_number_of_det (can_number_of_det msg))
    (cl:cons ':can_look_id (can_look_id msg))
    (cl:cons ':can_look_index (can_look_index msg))
))
