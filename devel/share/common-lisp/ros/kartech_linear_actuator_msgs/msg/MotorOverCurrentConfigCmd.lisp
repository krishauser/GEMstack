; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude MotorOverCurrentConfigCmd.msg.html

(cl:defclass <MotorOverCurrentConfigCmd> (roslisp-msg-protocol:ros-message)
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
   (over_current
    :reader over_current
    :initarg :over_current
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotorOverCurrentConfigCmd (<MotorOverCurrentConfigCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorOverCurrentConfigCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorOverCurrentConfigCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<MotorOverCurrentConfigCmd> is deprecated: use kartech_linear_actuator_msgs-msg:MotorOverCurrentConfigCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MotorOverCurrentConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <MotorOverCurrentConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'over_current-val :lambda-list '(m))
(cl:defmethod over_current-val ((m <MotorOverCurrentConfigCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:over_current-val is deprecated.  Use kartech_linear_actuator_msgs-msg:over_current instead.")
  (over_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorOverCurrentConfigCmd>) ostream)
  "Serializes a message object of type '<MotorOverCurrentConfigCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'over_current)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'over_current)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorOverCurrentConfigCmd>) istream)
  "Deserializes a message object of type '<MotorOverCurrentConfigCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'over_current)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'over_current)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorOverCurrentConfigCmd>)))
  "Returns string type for a message object of type '<MotorOverCurrentConfigCmd>"
  "kartech_linear_actuator_msgs/MotorOverCurrentConfigCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorOverCurrentConfigCmd)))
  "Returns string type for a message object of type 'MotorOverCurrentConfigCmd"
  "kartech_linear_actuator_msgs/MotorOverCurrentConfigCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorOverCurrentConfigCmd>)))
  "Returns md5sum for a message object of type '<MotorOverCurrentConfigCmd>"
  "65da7043e246e0aea73fa57120f9071a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorOverCurrentConfigCmd)))
  "Returns md5sum for a message object of type 'MotorOverCurrentConfigCmd"
  "65da7043e246e0aea73fa57120f9071a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorOverCurrentConfigCmd>)))
  "Returns full string definition for message of type '<MotorOverCurrentConfigCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 over_current # The over-current value in mA. Default is 6500mA.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorOverCurrentConfigCmd)))
  "Returns full string definition for message of type 'MotorOverCurrentConfigCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%uint16 over_current # The over-current value in mA. Default is 6500mA.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorOverCurrentConfigCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorOverCurrentConfigCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorOverCurrentConfigCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':over_current (over_current msg))
))
