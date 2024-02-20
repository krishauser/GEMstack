; Auto-generated. Do not edit!


(cl:in-package neobotix_usboard_msgs-msg)


;//! \htmlinclude AnsParasetToEEPROM.msg.html

(cl:defclass <AnsParasetToEEPROM> (roslisp-msg-protocol:ros-message)
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
   (paraset_cksum_low_byte
    :reader paraset_cksum_low_byte
    :initarg :paraset_cksum_low_byte
    :type cl:fixnum
    :initform 0)
   (paraset_cksum_high_byte
    :reader paraset_cksum_high_byte
    :initarg :paraset_cksum_high_byte
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnsParasetToEEPROM (<AnsParasetToEEPROM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnsParasetToEEPROM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnsParasetToEEPROM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neobotix_usboard_msgs-msg:<AnsParasetToEEPROM> is deprecated: use neobotix_usboard_msgs-msg:AnsParasetToEEPROM instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AnsParasetToEEPROM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:header-val is deprecated.  Use neobotix_usboard_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <AnsParasetToEEPROM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:command-val is deprecated.  Use neobotix_usboard_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'paraset_cksum_low_byte-val :lambda-list '(m))
(cl:defmethod paraset_cksum_low_byte-val ((m <AnsParasetToEEPROM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_cksum_low_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_cksum_low_byte instead.")
  (paraset_cksum_low_byte m))

(cl:ensure-generic-function 'paraset_cksum_high_byte-val :lambda-list '(m))
(cl:defmethod paraset_cksum_high_byte-val ((m <AnsParasetToEEPROM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_cksum_high_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_cksum_high_byte instead.")
  (paraset_cksum_high_byte m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnsParasetToEEPROM>) ostream)
  "Serializes a message object of type '<AnsParasetToEEPROM>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_cksum_low_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_cksum_high_byte)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnsParasetToEEPROM>) istream)
  "Deserializes a message object of type '<AnsParasetToEEPROM>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_cksum_low_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_cksum_high_byte)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnsParasetToEEPROM>)))
  "Returns string type for a message object of type '<AnsParasetToEEPROM>"
  "neobotix_usboard_msgs/AnsParasetToEEPROM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnsParasetToEEPROM)))
  "Returns string type for a message object of type 'AnsParasetToEEPROM"
  "neobotix_usboard_msgs/AnsParasetToEEPROM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnsParasetToEEPROM>)))
  "Returns md5sum for a message object of type '<AnsParasetToEEPROM>"
  "ae6db4e8232618eb386bde89d0c827b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnsParasetToEEPROM)))
  "Returns md5sum for a message object of type 'AnsParasetToEEPROM"
  "ae6db4e8232618eb386bde89d0c827b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnsParasetToEEPROM>)))
  "Returns full string definition for message of type '<AnsParasetToEEPROM>"
  (cl:format cl:nil "# Message file for AnsParasetToEEPROM~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     paraset_cksum_low_byte                  ~%uint8     paraset_cksum_high_byte                 ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnsParasetToEEPROM)))
  "Returns full string definition for message of type 'AnsParasetToEEPROM"
  (cl:format cl:nil "# Message file for AnsParasetToEEPROM~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     paraset_cksum_low_byte                  ~%uint8     paraset_cksum_high_byte                 ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnsParasetToEEPROM>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnsParasetToEEPROM>))
  "Converts a ROS message object to a list"
  (cl:list 'AnsParasetToEEPROM
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':paraset_cksum_low_byte (paraset_cksum_low_byte msg))
    (cl:cons ':paraset_cksum_high_byte (paraset_cksum_high_byte msg))
))
