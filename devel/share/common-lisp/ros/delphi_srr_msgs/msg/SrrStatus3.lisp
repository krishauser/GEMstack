; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrStatus3.msg.html

(cl:defclass <SrrStatus3> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_tx_alignment_state
    :reader can_tx_alignment_state
    :initarg :can_tx_alignment_state
    :type cl:fixnum
    :initform 0)
   (can_tx_interface_ver_minor
    :reader can_tx_interface_ver_minor
    :initarg :can_tx_interface_ver_minor
    :type cl:fixnum
    :initform 0)
   (can_tx_sw_version_arm
    :reader can_tx_sw_version_arm
    :initarg :can_tx_sw_version_arm
    :type cl:integer
    :initform 0)
   (can_tx_hw_version
    :reader can_tx_hw_version
    :initarg :can_tx_hw_version
    :type cl:fixnum
    :initform 0)
   (can_tx_interface_version
    :reader can_tx_interface_version
    :initarg :can_tx_interface_version
    :type cl:fixnum
    :initform 0)
   (can_tx_serial_num
    :reader can_tx_serial_num
    :initarg :can_tx_serial_num
    :type cl:integer
    :initform 0))
)

(cl:defclass SrrStatus3 (<SrrStatus3>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrStatus3>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrStatus3)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrStatus3> is deprecated: use delphi_srr_msgs-msg:SrrStatus3 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_tx_alignment_state-val :lambda-list '(m))
(cl:defmethod can_tx_alignment_state-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_alignment_state-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_alignment_state instead.")
  (can_tx_alignment_state m))

(cl:ensure-generic-function 'can_tx_interface_ver_minor-val :lambda-list '(m))
(cl:defmethod can_tx_interface_ver_minor-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_interface_ver_minor-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_interface_ver_minor instead.")
  (can_tx_interface_ver_minor m))

(cl:ensure-generic-function 'can_tx_sw_version_arm-val :lambda-list '(m))
(cl:defmethod can_tx_sw_version_arm-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_sw_version_arm-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_sw_version_arm instead.")
  (can_tx_sw_version_arm m))

(cl:ensure-generic-function 'can_tx_hw_version-val :lambda-list '(m))
(cl:defmethod can_tx_hw_version-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_hw_version-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_hw_version instead.")
  (can_tx_hw_version m))

(cl:ensure-generic-function 'can_tx_interface_version-val :lambda-list '(m))
(cl:defmethod can_tx_interface_version-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_interface_version-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_interface_version instead.")
  (can_tx_interface_version m))

(cl:ensure-generic-function 'can_tx_serial_num-val :lambda-list '(m))
(cl:defmethod can_tx_serial_num-val ((m <SrrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_serial_num-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_serial_num instead.")
  (can_tx_serial_num m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SrrStatus3>)))
    "Constants for message type '<SrrStatus3>"
  '((:CAN_TX_ALIGNMENT_STATE_OFF . 0)
    (:CAN_TX_ALIGNMENT_STATE_INIT . 1)
    (:CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT . 2)
    (:CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT . 3)
    (:CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SrrStatus3)))
    "Constants for message type 'SrrStatus3"
  '((:CAN_TX_ALIGNMENT_STATE_OFF . 0)
    (:CAN_TX_ALIGNMENT_STATE_INIT . 1)
    (:CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT . 2)
    (:CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT . 3)
    (:CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrStatus3>) ostream)
  "Serializes a message object of type '<SrrStatus3>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_alignment_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_interface_ver_minor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_sw_version_arm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_sw_version_arm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'can_tx_sw_version_arm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'can_tx_sw_version_arm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_hw_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_interface_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_serial_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_serial_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'can_tx_serial_num)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'can_tx_serial_num)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrStatus3>) istream)
  "Deserializes a message object of type '<SrrStatus3>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_alignment_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_interface_ver_minor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_sw_version_arm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_sw_version_arm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'can_tx_sw_version_arm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'can_tx_sw_version_arm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_hw_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_interface_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_serial_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'can_tx_serial_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'can_tx_serial_num)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'can_tx_serial_num)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrStatus3>)))
  "Returns string type for a message object of type '<SrrStatus3>"
  "delphi_srr_msgs/SrrStatus3")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrStatus3)))
  "Returns string type for a message object of type 'SrrStatus3"
  "delphi_srr_msgs/SrrStatus3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrStatus3>)))
  "Returns md5sum for a message object of type '<SrrStatus3>"
  "7a40100fb28cf1c5e2bf4d3c15d6aeb8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrStatus3)))
  "Returns md5sum for a message object of type 'SrrStatus3"
  "7a40100fb28cf1c5e2bf4d3c15d6aeb8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrStatus3>)))
  "Returns full string definition for message of type '<SrrStatus3>"
  (cl:format cl:nil "# Message file for srr_status3~%~%std_msgs/Header header~%~%uint8     can_tx_alignment_state~%uint8     CAN_TX_ALIGNMENT_STATE_OFF=0~%uint8     CAN_TX_ALIGNMENT_STATE_INIT=1~%uint8     CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT=2~%uint8     CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT=3~%uint8     CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT=4~%~%uint8     can_tx_interface_ver_minor~%uint32    can_tx_sw_version_arm~%uint8     can_tx_hw_version~%uint8     can_tx_interface_version~%uint32    can_tx_serial_num~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrStatus3)))
  "Returns full string definition for message of type 'SrrStatus3"
  (cl:format cl:nil "# Message file for srr_status3~%~%std_msgs/Header header~%~%uint8     can_tx_alignment_state~%uint8     CAN_TX_ALIGNMENT_STATE_OFF=0~%uint8     CAN_TX_ALIGNMENT_STATE_INIT=1~%uint8     CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT=2~%uint8     CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT=3~%uint8     CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT=4~%~%uint8     can_tx_interface_ver_minor~%uint32    can_tx_sw_version_arm~%uint8     can_tx_hw_version~%uint8     can_tx_interface_version~%uint32    can_tx_serial_num~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrStatus3>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrStatus3>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrStatus3
    (cl:cons ':header (header msg))
    (cl:cons ':can_tx_alignment_state (can_tx_alignment_state msg))
    (cl:cons ':can_tx_interface_ver_minor (can_tx_interface_ver_minor msg))
    (cl:cons ':can_tx_sw_version_arm (can_tx_sw_version_arm msg))
    (cl:cons ':can_tx_hw_version (can_tx_hw_version msg))
    (cl:cons ':can_tx_interface_version (can_tx_interface_version msg))
    (cl:cons ':can_tx_serial_num (can_tx_serial_num msg))
))
