; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ResetCmd.msg.html

(cl:defclass <ResetCmd> (roslisp-msg-protocol:ros-message)
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
   (reset_type
    :reader reset_type
    :initarg :reset_type
    :type cl:fixnum
    :initform 0)
   (reset_user_rpt_id
    :reader reset_user_rpt_id
    :initarg :reset_user_rpt_id
    :type cl:boolean
    :initform cl:nil)
   (reset_user_cmd_id_1
    :reader reset_user_cmd_id_1
    :initarg :reset_user_cmd_id_1
    :type cl:boolean
    :initform cl:nil)
   (reset_user_cmd_id_2
    :reader reset_user_cmd_id_2
    :initarg :reset_user_cmd_id_2
    :type cl:boolean
    :initform cl:nil)
   (reset_user_cmd_id_3
    :reader reset_user_cmd_id_3
    :initarg :reset_user_cmd_id_3
    :type cl:boolean
    :initform cl:nil)
   (reset_user_cmd_id_4
    :reader reset_user_cmd_id_4
    :initarg :reset_user_cmd_id_4
    :type cl:boolean
    :initform cl:nil)
   (disable_user_rpt_id
    :reader disable_user_rpt_id
    :initarg :disable_user_rpt_id
    :type cl:boolean
    :initform cl:nil)
   (reenable_default_cmd_id
    :reader reenable_default_cmd_id
    :initarg :reenable_default_cmd_id
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResetCmd (<ResetCmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetCmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetCmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ResetCmd> is deprecated: use kartech_linear_actuator_msgs-msg:ResetCmd instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'reset_type-val :lambda-list '(m))
(cl:defmethod reset_type-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reset_type-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reset_type instead.")
  (reset_type m))

(cl:ensure-generic-function 'reset_user_rpt_id-val :lambda-list '(m))
(cl:defmethod reset_user_rpt_id-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reset_user_rpt_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reset_user_rpt_id instead.")
  (reset_user_rpt_id m))

(cl:ensure-generic-function 'reset_user_cmd_id_1-val :lambda-list '(m))
(cl:defmethod reset_user_cmd_id_1-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reset_user_cmd_id_1-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reset_user_cmd_id_1 instead.")
  (reset_user_cmd_id_1 m))

(cl:ensure-generic-function 'reset_user_cmd_id_2-val :lambda-list '(m))
(cl:defmethod reset_user_cmd_id_2-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reset_user_cmd_id_2-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reset_user_cmd_id_2 instead.")
  (reset_user_cmd_id_2 m))

(cl:ensure-generic-function 'reset_user_cmd_id_3-val :lambda-list '(m))
(cl:defmethod reset_user_cmd_id_3-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reset_user_cmd_id_3-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reset_user_cmd_id_3 instead.")
  (reset_user_cmd_id_3 m))

(cl:ensure-generic-function 'reset_user_cmd_id_4-val :lambda-list '(m))
(cl:defmethod reset_user_cmd_id_4-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reset_user_cmd_id_4-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reset_user_cmd_id_4 instead.")
  (reset_user_cmd_id_4 m))

(cl:ensure-generic-function 'disable_user_rpt_id-val :lambda-list '(m))
(cl:defmethod disable_user_rpt_id-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:disable_user_rpt_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:disable_user_rpt_id instead.")
  (disable_user_rpt_id m))

(cl:ensure-generic-function 'reenable_default_cmd_id-val :lambda-list '(m))
(cl:defmethod reenable_default_cmd_id-val ((m <ResetCmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:reenable_default_cmd_id-val is deprecated.  Use kartech_linear_actuator_msgs-msg:reenable_default_cmd_id instead.")
  (reenable_default_cmd_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ResetCmd>)))
    "Constants for message type '<ResetCmd>"
  '((:RESET_OUTPUTS . 0)
    (:RESET_USER_DEFINED_IDS . 1)
    (:RESET_REPORT_RATES . 2)
    (:RESET_HARDWARE_CONFIGURATIONS . 3)
    (:RESET_USER_CONFIGURATIONS . 4)
    (:RESET_EVERYTHING . 5)
    (:RESET_NONE . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ResetCmd)))
    "Constants for message type 'ResetCmd"
  '((:RESET_OUTPUTS . 0)
    (:RESET_USER_DEFINED_IDS . 1)
    (:RESET_REPORT_RATES . 2)
    (:RESET_HARDWARE_CONFIGURATIONS . 3)
    (:RESET_USER_CONFIGURATIONS . 4)
    (:RESET_EVERYTHING . 5)
    (:RESET_NONE . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetCmd>) ostream)
  "Serializes a message object of type '<ResetCmd>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reset_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset_user_rpt_id) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset_user_cmd_id_1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset_user_cmd_id_2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset_user_cmd_id_3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reset_user_cmd_id_4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'disable_user_rpt_id) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reenable_default_cmd_id) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetCmd>) istream)
  "Deserializes a message object of type '<ResetCmd>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reset_type)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reset_user_rpt_id) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reset_user_cmd_id_1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reset_user_cmd_id_2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reset_user_cmd_id_3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reset_user_cmd_id_4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'disable_user_rpt_id) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'reenable_default_cmd_id) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetCmd>)))
  "Returns string type for a message object of type '<ResetCmd>"
  "kartech_linear_actuator_msgs/ResetCmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetCmd)))
  "Returns string type for a message object of type 'ResetCmd"
  "kartech_linear_actuator_msgs/ResetCmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetCmd>)))
  "Returns md5sum for a message object of type '<ResetCmd>"
  "0599fdf3d91d47c66c1ecb3a5a9d3e0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetCmd)))
  "Returns md5sum for a message object of type 'ResetCmd"
  "0599fdf3d91d47c66c1ecb3a5a9d3e0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetCmd>)))
  "Returns full string definition for message of type '<ResetCmd>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%uint8 RESET_OUTPUTS = 0~%uint8 RESET_USER_DEFINED_IDS = 1~%uint8 RESET_REPORT_RATES = 2~%uint8 RESET_HARDWARE_CONFIGURATIONS = 3~%uint8 RESET_USER_CONFIGURATIONS = 4~%uint8 RESET_EVERYTHING = 5~%uint8 RESET_NONE = 6~%~%uint8 reset_type~%~%bool reset_user_rpt_id~%bool reset_user_cmd_id_1~%bool reset_user_cmd_id_2~%bool reset_user_cmd_id_3~%bool reset_user_cmd_id_4~%bool disable_user_rpt_id~%bool reenable_default_cmd_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetCmd)))
  "Returns full string definition for message of type 'ResetCmd"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%~%uint8 RESET_OUTPUTS = 0~%uint8 RESET_USER_DEFINED_IDS = 1~%uint8 RESET_REPORT_RATES = 2~%uint8 RESET_HARDWARE_CONFIGURATIONS = 3~%uint8 RESET_USER_CONFIGURATIONS = 4~%uint8 RESET_EVERYTHING = 5~%uint8 RESET_NONE = 6~%~%uint8 reset_type~%~%bool reset_user_rpt_id~%bool reset_user_cmd_id_1~%bool reset_user_cmd_id_2~%bool reset_user_cmd_id_3~%bool reset_user_cmd_id_4~%bool disable_user_rpt_id~%bool reenable_default_cmd_id~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetCmd>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetCmd>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetCmd
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':reset_type (reset_type msg))
    (cl:cons ':reset_user_rpt_id (reset_user_rpt_id msg))
    (cl:cons ':reset_user_cmd_id_1 (reset_user_cmd_id_1 msg))
    (cl:cons ':reset_user_cmd_id_2 (reset_user_cmd_id_2 msg))
    (cl:cons ':reset_user_cmd_id_3 (reset_user_cmd_id_3 msg))
    (cl:cons ':reset_user_cmd_id_4 (reset_user_cmd_id_4 msg))
    (cl:cons ':disable_user_rpt_id (disable_user_rpt_id msg))
    (cl:cons ':reenable_default_cmd_id (reenable_default_cmd_id msg))
))
