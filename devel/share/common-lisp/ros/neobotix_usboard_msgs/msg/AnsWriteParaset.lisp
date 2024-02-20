; Auto-generated. Do not edit!


(cl:in-package neobotix_usboard_msgs-msg)


;//! \htmlinclude AnsWriteParaset.msg.html

(cl:defclass <AnsWriteParaset> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (paramset_cksum_low_byte
    :reader paramset_cksum_low_byte
    :initarg :paramset_cksum_low_byte
    :type cl:fixnum
    :initform 0)
   (paramset_cksum_high_byte
    :reader paramset_cksum_high_byte
    :initarg :paramset_cksum_high_byte
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnsWriteParaset (<AnsWriteParaset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnsWriteParaset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnsWriteParaset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neobotix_usboard_msgs-msg:<AnsWriteParaset> is deprecated: use neobotix_usboard_msgs-msg:AnsWriteParaset instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AnsWriteParaset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:header-val is deprecated.  Use neobotix_usboard_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <AnsWriteParaset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:command-val is deprecated.  Use neobotix_usboard_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'paramset_cksum_low_byte-val :lambda-list '(m))
(cl:defmethod paramset_cksum_low_byte-val ((m <AnsWriteParaset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paramset_cksum_low_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:paramset_cksum_low_byte instead.")
  (paramset_cksum_low_byte m))

(cl:ensure-generic-function 'paramset_cksum_high_byte-val :lambda-list '(m))
(cl:defmethod paramset_cksum_high_byte-val ((m <AnsWriteParaset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paramset_cksum_high_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:paramset_cksum_high_byte instead.")
  (paramset_cksum_high_byte m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnsWriteParaset>) ostream)
  "Serializes a message object of type '<AnsWriteParaset>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paramset_cksum_low_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paramset_cksum_high_byte)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnsWriteParaset>) istream)
  "Deserializes a message object of type '<AnsWriteParaset>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paramset_cksum_low_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paramset_cksum_high_byte)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnsWriteParaset>)))
  "Returns string type for a message object of type '<AnsWriteParaset>"
  "neobotix_usboard_msgs/AnsWriteParaset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnsWriteParaset)))
  "Returns string type for a message object of type 'AnsWriteParaset"
  "neobotix_usboard_msgs/AnsWriteParaset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnsWriteParaset>)))
  "Returns md5sum for a message object of type '<AnsWriteParaset>"
  "73cd7b344457ad8ea263f2b6c0b90b01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnsWriteParaset)))
  "Returns md5sum for a message object of type 'AnsWriteParaset"
  "73cd7b344457ad8ea263f2b6c0b90b01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnsWriteParaset>)))
  "Returns full string definition for message of type '<AnsWriteParaset>"
  (cl:format cl:nil "# Message file for AnsWriteParaset~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     paramset_cksum_low_byte                  ~%uint8     paramset_cksum_high_byte                 ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnsWriteParaset)))
  "Returns full string definition for message of type 'AnsWriteParaset"
  (cl:format cl:nil "# Message file for AnsWriteParaset~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     paramset_cksum_low_byte                  ~%uint8     paramset_cksum_high_byte                 ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnsWriteParaset>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnsWriteParaset>))
  "Converts a ROS message object to a list"
  (cl:list 'AnsWriteParaset
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':paramset_cksum_low_byte (paramset_cksum_low_byte msg))
    (cl:cons ':paramset_cksum_high_byte (paramset_cksum_high_byte msg))
))
