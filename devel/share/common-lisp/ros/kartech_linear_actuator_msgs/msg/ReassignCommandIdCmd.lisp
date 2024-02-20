; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ReassignCommandIdCmd.msg.html

(cl:defclass <ReassignCommandIdCmd> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (confirm
    :reader confirm
    :initarg :confirm
    :type cl:boolean
    :initform cl:nil)
   (command_id_index
    :reader command_id_index
    :initarg :command_id_index
    :type cl:fixnum
    :initform 0)
   (user_command_id
    :reader user_command_id
    :initarg :user_command_id
    :type cl:integer
    :initform 0)
   (disable_default_command_id
    :reader disable_default_command_id
    :initarg :disable_default_command_id
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ReassignCommandIdCmd (<ReassignCommandIdCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReassignCommandIdCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReassignCommandIdCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ReassignCommandIdCmd> is deprecated: use kartech_linear_actuator_msgs-msg:ReassignCommandIdCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReassignCommandIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ReassignCommandIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'command_id_index-val :lambda-list '(m))
(cl:defmethod command_id_index-val ((m <ReassignCommandIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:command_id_index-val is deprecated.  Use kartech_linear_actuator_msgs-msg:command_id_index instead.")
  (command_id_index m))

(cl:ensure-generic-function 'user_command_id-val :lambda-list '(m))
(cl:defmethod user_command_id-val ((m <ReassignCommandIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:user_command_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:user_command_id instead.")
  (user_command_id m))

(cl:ensure-generic-function 'disable_default_command_id-val :lambda-list '(m))
(cl:defmethod disable_default_command_id-val ((m <ReassignCommandIdCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:disable_default_command_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:disable_default_command_id instead.")
  (disable_default_command_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReassignCommandIdCmd>) ostream)
  "Serializes a message object of type '<ReassignCommandIdCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_id_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_command_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_command_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_command_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_command_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'disable_default_command_id) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReassignCommandIdCmd>) istream)
  "Deserializes a message object of type '<ReassignCommandIdCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command_id_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'user_command_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'user_command_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'user_command_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'user_command_id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'disable_default_command_id) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReassignCommandIdCmd>)))
  "Returns string type for a message object of type '<ReassignCommandIdCmd>"
  "kartech_linear_actuator_msgs/ReassignCommandIdCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReassignCommandIdCmd)))
  "Returns string type for a message object of type 'ReassignCommandIdCmd"
  "kartech_linear_actuator_msgs/ReassignCommandIdCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReassignCommandIdCmd>)))
  "Returns md5sum for a message object of type '<ReassignCommandIdCmd>"
  "41d43df68f42f68725a7567326abaa4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReassignCommandIdCmd)))
  "Returns md5sum for a message object of type 'ReassignCommandIdCmd"
  "41d43df68f42f68725a7567326abaa4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReassignCommandIdCmd>)))
  "Returns full string definition for message of type '<ReassignCommandIdCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint8 command_id_index            # The index of the user-defined command ID (1-4) to change.~%uint32 user_command_id            # The new user-defined command ID to set. 0xFFFEXX and 0xFF00XX are reserved.~%                                  # Setting this to 0xFFFFFFFF will change the disable_default_command_id flag without affecting any others.~%bool disable_default_command_id   # Disables (true) or enables (false) the default command ID.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReassignCommandIdCmd)))
  "Returns full string definition for message of type 'ReassignCommandIdCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint8 command_id_index            # The index of the user-defined command ID (1-4) to change.~%uint32 user_command_id            # The new user-defined command ID to set. 0xFFFEXX and 0xFF00XX are reserved.~%                                  # Setting this to 0xFFFFFFFF will change the disable_default_command_id flag without affecting any others.~%bool disable_default_command_id   # Disables (true) or enables (false) the default command ID.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReassignCommandIdCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReassignCommandIdCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ReassignCommandIdCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':command_id_index (command_id_index msg))
    (cl:cons ':user_command_id (user_command_id msg))
    (cl:cons ':disable_default_command_id (disable_default_command_id msg))
))
