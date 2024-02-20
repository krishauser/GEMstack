; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrDebug5.msg.html

(cl:defclass <SrrDebug5> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_tx_align_updates
    :reader can_tx_align_updates
    :initarg :can_tx_align_updates
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SrrDebug5 (<SrrDebug5>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrDebug5>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrDebug5)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrDebug5> is deprecated: use delphi_srr_msgs-msg:SrrDebug5 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrDebug5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_tx_align_updates-val :lambda-list '(m))
(cl:defmethod can_tx_align_updates-val ((m <SrrDebug5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_align_updates-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_align_updates instead.")
  (can_tx_align_updates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrDebug5>) ostream)
  "Serializes a message object of type '<SrrDebug5>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_align_updates)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_align_updates)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrDebug5>) istream)
  "Deserializes a message object of type '<SrrDebug5>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_align_updates)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_align_updates)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrDebug5>)))
  "Returns string type for a message object of type '<SrrDebug5>"
  "delphi_srr_msgs/SrrDebug5")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrDebug5)))
  "Returns string type for a message object of type 'SrrDebug5"
  "delphi_srr_msgs/SrrDebug5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrDebug5>)))
  "Returns md5sum for a message object of type '<SrrDebug5>"
  "33cb8dd5c14e9c9dfee5824987d9b07a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrDebug5)))
  "Returns md5sum for a message object of type 'SrrDebug5"
  "33cb8dd5c14e9c9dfee5824987d9b07a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrDebug5>)))
  "Returns full string definition for message of type '<SrrDebug5>"
  (cl:format cl:nil "# Message file for srr_debug5~%~%std_msgs/Header header~%~%uint16    can_tx_align_updates~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrDebug5)))
  "Returns full string definition for message of type 'SrrDebug5"
  (cl:format cl:nil "# Message file for srr_debug5~%~%std_msgs/Header header~%~%uint16    can_tx_align_updates~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrDebug5>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrDebug5>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrDebug5
    (cl:cons ':header (header msg))
    (cl:cons ':can_tx_align_updates (can_tx_align_updates msg))
))
