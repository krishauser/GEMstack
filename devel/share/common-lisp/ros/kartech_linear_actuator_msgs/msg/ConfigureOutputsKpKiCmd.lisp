; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ConfigureOutputsKpKiCmd.msg.html

(cl:defclass <ConfigureOutputsKpKiCmd> (roslisp-msg-protocol:ros-message)
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
   (kp
    :reader kp
    :initarg :kp
    :type cl:fixnum
    :initform 0)
   (ki
    :reader ki
    :initarg :ki
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ConfigureOutputsKpKiCmd (<ConfigureOutputsKpKiCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigureOutputsKpKiCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigureOutputsKpKiCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ConfigureOutputsKpKiCmd> is deprecated: use kartech_linear_actuator_msgs-msg:ConfigureOutputsKpKiCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConfigureOutputsKpKiCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ConfigureOutputsKpKiCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'kp-val :lambda-list '(m))
(cl:defmethod kp-val ((m <ConfigureOutputsKpKiCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:kp-val is deprecated.  Use kartech_linear_actuator_msgs-msg:kp instead.")
  (kp m))

(cl:ensure-generic-function 'ki-val :lambda-list '(m))
(cl:defmethod ki-val ((m <ConfigureOutputsKpKiCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:ki-val is deprecated.  Use kartech_linear_actuator_msgs-msg:ki instead.")
  (ki m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigureOutputsKpKiCmd>) ostream)
  "Serializes a message object of type '<ConfigureOutputsKpKiCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'kp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ki)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ki)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigureOutputsKpKiCmd>) istream)
  "Deserializes a message object of type '<ConfigureOutputsKpKiCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'kp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ki)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ki)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigureOutputsKpKiCmd>)))
  "Returns string type for a message object of type '<ConfigureOutputsKpKiCmd>"
  "kartech_linear_actuator_msgs/ConfigureOutputsKpKiCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigureOutputsKpKiCmd)))
  "Returns string type for a message object of type 'ConfigureOutputsKpKiCmd"
  "kartech_linear_actuator_msgs/ConfigureOutputsKpKiCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigureOutputsKpKiCmd>)))
  "Returns md5sum for a message object of type '<ConfigureOutputsKpKiCmd>"
  "2776941ac111d4c1261066496047ed8d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigureOutputsKpKiCmd)))
  "Returns md5sum for a message object of type 'ConfigureOutputsKpKiCmd"
  "2776941ac111d4c1261066496047ed8d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigureOutputsKpKiCmd>)))
  "Returns full string definition for message of type '<ConfigureOutputsKpKiCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 kp  # The proportial term of the closed-loop control. Default is 1000.~%uint16 ki  # The integral term of the closed-loop control. Default is 1000.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigureOutputsKpKiCmd)))
  "Returns full string definition for message of type 'ConfigureOutputsKpKiCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 kp  # The proportial term of the closed-loop control. Default is 1000.~%uint16 ki  # The integral term of the closed-loop control. Default is 1000.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigureOutputsKpKiCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigureOutputsKpKiCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigureOutputsKpKiCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':kp (kp msg))
    (cl:cons ':ki (ki msg))
))
