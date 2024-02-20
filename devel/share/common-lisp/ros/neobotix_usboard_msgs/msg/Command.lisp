; Auto-generated. Do not edit!


(cl:in-package neobotix_usboard_msgs-msg)


;//! \htmlinclude Command.msg.html

(cl:defclass <Command> (roslisp-msg-protocol:ros-message)
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
   (command_data
    :reader command_data
    :initarg :command_data
    :type cl:integer
    :initform 0)
   (set_num
    :reader set_num
    :initarg :set_num
    :type cl:fixnum
    :initform 0)
   (paraset_byte6
    :reader paraset_byte6
    :initarg :paraset_byte6
    :type cl:fixnum
    :initform 0)
   (paraset_byte5
    :reader paraset_byte5
    :initarg :paraset_byte5
    :type cl:fixnum
    :initform 0)
   (paraset_byte4
    :reader paraset_byte4
    :initarg :paraset_byte4
    :type cl:fixnum
    :initform 0)
   (paraset_byte3
    :reader paraset_byte3
    :initarg :paraset_byte3
    :type cl:fixnum
    :initform 0)
   (paraset_byte2
    :reader paraset_byte2
    :initarg :paraset_byte2
    :type cl:fixnum
    :initform 0)
   (paraset_byte1
    :reader paraset_byte1
    :initarg :paraset_byte1
    :type cl:fixnum
    :initform 0)
   (set_active_9to16
    :reader set_active_9to16
    :initarg :set_active_9to16
    :type cl:fixnum
    :initform 0)
   (set_active_1to8
    :reader set_active_1to8
    :initarg :set_active_1to8
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Command (<Command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neobotix_usboard_msgs-msg:<Command> is deprecated: use neobotix_usboard_msgs-msg:Command instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:header-val is deprecated.  Use neobotix_usboard_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:command-val is deprecated.  Use neobotix_usboard_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'command_data-val :lambda-list '(m))
(cl:defmethod command_data-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:command_data-val is deprecated.  Use neobotix_usboard_msgs-msg:command_data instead.")
  (command_data m))

(cl:ensure-generic-function 'set_num-val :lambda-list '(m))
(cl:defmethod set_num-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:set_num-val is deprecated.  Use neobotix_usboard_msgs-msg:set_num instead.")
  (set_num m))

(cl:ensure-generic-function 'paraset_byte6-val :lambda-list '(m))
(cl:defmethod paraset_byte6-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_byte6-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_byte6 instead.")
  (paraset_byte6 m))

(cl:ensure-generic-function 'paraset_byte5-val :lambda-list '(m))
(cl:defmethod paraset_byte5-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_byte5-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_byte5 instead.")
  (paraset_byte5 m))

(cl:ensure-generic-function 'paraset_byte4-val :lambda-list '(m))
(cl:defmethod paraset_byte4-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_byte4-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_byte4 instead.")
  (paraset_byte4 m))

(cl:ensure-generic-function 'paraset_byte3-val :lambda-list '(m))
(cl:defmethod paraset_byte3-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_byte3-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_byte3 instead.")
  (paraset_byte3 m))

(cl:ensure-generic-function 'paraset_byte2-val :lambda-list '(m))
(cl:defmethod paraset_byte2-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_byte2-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_byte2 instead.")
  (paraset_byte2 m))

(cl:ensure-generic-function 'paraset_byte1-val :lambda-list '(m))
(cl:defmethod paraset_byte1-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:paraset_byte1-val is deprecated.  Use neobotix_usboard_msgs-msg:paraset_byte1 instead.")
  (paraset_byte1 m))

(cl:ensure-generic-function 'set_active_9to16-val :lambda-list '(m))
(cl:defmethod set_active_9to16-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:set_active_9to16-val is deprecated.  Use neobotix_usboard_msgs-msg:set_active_9to16 instead.")
  (set_active_9to16 m))

(cl:ensure-generic-function 'set_active_1to8-val :lambda-list '(m))
(cl:defmethod set_active_1to8-val ((m <Command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:set_active_1to8-val is deprecated.  Use neobotix_usboard_msgs-msg:set_active_1to8 instead.")
  (set_active_1to8 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command>) ostream)
  "Serializes a message object of type '<Command>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'command_data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'set_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'set_active_9to16)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'set_active_1to8)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command>) istream)
  "Deserializes a message object of type '<Command>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'command_data)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'set_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'paraset_byte1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'set_active_9to16)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'set_active_1to8)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command>)))
  "Returns string type for a message object of type '<Command>"
  "neobotix_usboard_msgs/Command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command)))
  "Returns string type for a message object of type 'Command"
  "neobotix_usboard_msgs/Command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command>)))
  "Returns md5sum for a message object of type '<Command>"
  "aa2adac976b058480751c90ad9fb67d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command)))
  "Returns md5sum for a message object of type 'Command"
  "aa2adac976b058480751c90ad9fb67d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command>)))
  "Returns full string definition for message of type '<Command>"
  (cl:format cl:nil "# Message file for Command~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint64    command_data                            ~%uint8     set_num                                 ~%uint8     paraset_byte6                     ~%uint8     paraset_byte5                     ~%uint8     paraset_byte4                     ~%uint8     paraset_byte3                     ~%uint8     paraset_byte2                     ~%uint8     paraset_byte1                     ~%uint8     set_active_9to16                        ~%uint8     set_active_1to8                         ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command)))
  "Returns full string definition for message of type 'Command"
  (cl:format cl:nil "# Message file for Command~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint64    command_data                            ~%uint8     set_num                                 ~%uint8     paraset_byte6                     ~%uint8     paraset_byte5                     ~%uint8     paraset_byte4                     ~%uint8     paraset_byte3                     ~%uint8     paraset_byte2                     ~%uint8     paraset_byte1                     ~%uint8     set_active_9to16                        ~%uint8     set_active_1to8                         ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     8
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command>))
  "Converts a ROS message object to a list"
  (cl:list 'Command
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':command_data (command_data msg))
    (cl:cons ':set_num (set_num msg))
    (cl:cons ':paraset_byte6 (paraset_byte6 msg))
    (cl:cons ':paraset_byte5 (paraset_byte5 msg))
    (cl:cons ':paraset_byte4 (paraset_byte4 msg))
    (cl:cons ':paraset_byte3 (paraset_byte3 msg))
    (cl:cons ':paraset_byte2 (paraset_byte2 msg))
    (cl:cons ':paraset_byte1 (paraset_byte1 msg))
    (cl:cons ':set_active_9to16 (set_active_9to16 msg))
    (cl:cons ':set_active_1to8 (set_active_1to8 msg))
))
