; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude PriorityConfigCmd.msg.html

(cl:defclass <PriorityConfigCmd> (roslisp-msg-protocol:ros-message)
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
   (handshake_priority
    :reader handshake_priority
    :initarg :handshake_priority
    :type cl:fixnum
    :initform 0)
   (auto_reply_priority
    :reader auto_reply_priority
    :initarg :auto_reply_priority
    :type cl:fixnum
    :initform 0)
   (scheduled_priority
    :reader scheduled_priority
    :initarg :scheduled_priority
    :type cl:fixnum
    :initform 0)
   (polled_priority
    :reader polled_priority
    :initarg :polled_priority
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PriorityConfigCmd (<PriorityConfigCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PriorityConfigCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PriorityConfigCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<PriorityConfigCmd> is deprecated: use kartech_linear_actuator_msgs-msg:PriorityConfigCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PriorityConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <PriorityConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'handshake_priority-val :lambda-list '(m))
(cl:defmethod handshake_priority-val ((m <PriorityConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:handshake_priority-val is deprecated.  Use kartech_linear_actuator_msgs-msg:handshake_priority instead.")
  (handshake_priority m))

(cl:ensure-generic-function 'auto_reply_priority-val :lambda-list '(m))
(cl:defmethod auto_reply_priority-val ((m <PriorityConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:auto_reply_priority-val is deprecated.  Use kartech_linear_actuator_msgs-msg:auto_reply_priority instead.")
  (auto_reply_priority m))

(cl:ensure-generic-function 'scheduled_priority-val :lambda-list '(m))
(cl:defmethod scheduled_priority-val ((m <PriorityConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:scheduled_priority-val is deprecated.  Use kartech_linear_actuator_msgs-msg:scheduled_priority instead.")
  (scheduled_priority m))

(cl:ensure-generic-function 'polled_priority-val :lambda-list '(m))
(cl:defmethod polled_priority-val ((m <PriorityConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:polled_priority-val is deprecated.  Use kartech_linear_actuator_msgs-msg:polled_priority instead.")
  (polled_priority m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PriorityConfigCmd>) ostream)
  "Serializes a message object of type '<PriorityConfigCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'handshake_priority)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'auto_reply_priority)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scheduled_priority)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'polled_priority)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PriorityConfigCmd>) istream)
  "Deserializes a message object of type '<PriorityConfigCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'handshake_priority)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'auto_reply_priority)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scheduled_priority)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'polled_priority)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PriorityConfigCmd>)))
  "Returns string type for a message object of type '<PriorityConfigCmd>"
  "kartech_linear_actuator_msgs/PriorityConfigCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PriorityConfigCmd)))
  "Returns string type for a message object of type 'PriorityConfigCmd"
  "kartech_linear_actuator_msgs/PriorityConfigCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PriorityConfigCmd>)))
  "Returns md5sum for a message object of type '<PriorityConfigCmd>"
  "04b16f80c8b9d73ef8343b9ba34c9b78")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PriorityConfigCmd)))
  "Returns md5sum for a message object of type 'PriorityConfigCmd"
  "04b16f80c8b9d73ef8343b9ba34c9b78")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PriorityConfigCmd>)))
  "Returns full string definition for message of type '<PriorityConfigCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%# Priority values for different types of reports. Lower value = higher priority.~%uint8 handshake_priority~%uint8 auto_reply_priority~%uint8 scheduled_priority~%uint8 polled_priority~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PriorityConfigCmd)))
  "Returns full string definition for message of type 'PriorityConfigCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%# Priority values for different types of reports. Lower value = higher priority.~%uint8 handshake_priority~%uint8 auto_reply_priority~%uint8 scheduled_priority~%uint8 polled_priority~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PriorityConfigCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PriorityConfigCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'PriorityConfigCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':handshake_priority (handshake_priority msg))
    (cl:cons ':auto_reply_priority (auto_reply_priority msg))
    (cl:cons ':scheduled_priority (scheduled_priority msg))
    (cl:cons ':polled_priority (polled_priority msg))
))
