; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrFeatureSwVersion.msg.html

(cl:defclass <SrrFeatureSwVersion> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lcma_sw_version
    :reader lcma_sw_version
    :initarg :lcma_sw_version
    :type cl:fixnum
    :initform 0)
   (cta_sw_version
    :reader cta_sw_version
    :initarg :cta_sw_version
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SrrFeatureSwVersion (<SrrFeatureSwVersion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrFeatureSwVersion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrFeatureSwVersion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrFeatureSwVersion> is deprecated: use delphi_srr_msgs-msg:SrrFeatureSwVersion instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrFeatureSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lcma_sw_version-val :lambda-list '(m))
(cl:defmethod lcma_sw_version-val ((m <SrrFeatureSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:lcma_sw_version-val is deprecated.  Use delphi_srr_msgs-msg:lcma_sw_version instead.")
  (lcma_sw_version m))

(cl:ensure-generic-function 'cta_sw_version-val :lambda-list '(m))
(cl:defmethod cta_sw_version-val ((m <SrrFeatureSwVersion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:cta_sw_version-val is deprecated.  Use delphi_srr_msgs-msg:cta_sw_version instead.")
  (cta_sw_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrFeatureSwVersion>) ostream)
  "Serializes a message object of type '<SrrFeatureSwVersion>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_sw_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cta_sw_version)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrFeatureSwVersion>) istream)
  "Deserializes a message object of type '<SrrFeatureSwVersion>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lcma_sw_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cta_sw_version)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrFeatureSwVersion>)))
  "Returns string type for a message object of type '<SrrFeatureSwVersion>"
  "delphi_srr_msgs/SrrFeatureSwVersion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrFeatureSwVersion)))
  "Returns string type for a message object of type 'SrrFeatureSwVersion"
  "delphi_srr_msgs/SrrFeatureSwVersion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrFeatureSwVersion>)))
  "Returns md5sum for a message object of type '<SrrFeatureSwVersion>"
  "87f69bbbab65c94e0dda04a9e0914206")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrFeatureSwVersion)))
  "Returns md5sum for a message object of type 'SrrFeatureSwVersion"
  "87f69bbbab65c94e0dda04a9e0914206")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrFeatureSwVersion>)))
  "Returns full string definition for message of type '<SrrFeatureSwVersion>"
  (cl:format cl:nil "# Message file for srr_feature_sw_version~%~%std_msgs/Header header~%~%uint8     lcma_sw_version~%uint8     cta_sw_version~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrFeatureSwVersion)))
  "Returns full string definition for message of type 'SrrFeatureSwVersion"
  (cl:format cl:nil "# Message file for srr_feature_sw_version~%~%std_msgs/Header header~%~%uint8     lcma_sw_version~%uint8     cta_sw_version~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrFeatureSwVersion>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrFeatureSwVersion>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrFeatureSwVersion
    (cl:cons ':header (header msg))
    (cl:cons ':lcma_sw_version (lcma_sw_version msg))
    (cl:cons ':cta_sw_version (cta_sw_version msg))
))
