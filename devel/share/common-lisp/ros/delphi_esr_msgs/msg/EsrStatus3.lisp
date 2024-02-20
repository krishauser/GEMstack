; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrStatus3.msg.html

(cl:defclass <EsrStatus3> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (canmsg
    :reader canmsg
    :initarg :canmsg
    :type cl:string
    :initform "")
   (interface_version
    :reader interface_version
    :initarg :interface_version
    :type cl:fixnum
    :initform 0)
   (hw_version
    :reader hw_version
    :initarg :hw_version
    :type cl:fixnum
    :initform 0)
   (sw_version_host
    :reader sw_version_host
    :initarg :sw_version_host
    :type cl:string
    :initform "")
   (serial_num
    :reader serial_num
    :initarg :serial_num
    :type cl:string
    :initform "")
   (sw_version_pld
    :reader sw_version_pld
    :initarg :sw_version_pld
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EsrStatus3 (<EsrStatus3>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrStatus3>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrStatus3)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrStatus3> is deprecated: use delphi_esr_msgs-msg:EsrStatus3 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'interface_version-val :lambda-list '(m))
(cl:defmethod interface_version-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:interface_version-val is deprecated.  Use delphi_esr_msgs-msg:interface_version instead.")
  (interface_version m))

(cl:ensure-generic-function 'hw_version-val :lambda-list '(m))
(cl:defmethod hw_version-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:hw_version-val is deprecated.  Use delphi_esr_msgs-msg:hw_version instead.")
  (hw_version m))

(cl:ensure-generic-function 'sw_version_host-val :lambda-list '(m))
(cl:defmethod sw_version_host-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:sw_version_host-val is deprecated.  Use delphi_esr_msgs-msg:sw_version_host instead.")
  (sw_version_host m))

(cl:ensure-generic-function 'serial_num-val :lambda-list '(m))
(cl:defmethod serial_num-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:serial_num-val is deprecated.  Use delphi_esr_msgs-msg:serial_num instead.")
  (serial_num m))

(cl:ensure-generic-function 'sw_version_pld-val :lambda-list '(m))
(cl:defmethod sw_version_pld-val ((m <EsrStatus3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:sw_version_pld-val is deprecated.  Use delphi_esr_msgs-msg:sw_version_pld instead.")
  (sw_version_pld m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrStatus3>) ostream)
  "Serializes a message object of type '<EsrStatus3>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interface_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hw_version)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sw_version_host))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sw_version_host))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'serial_num))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'serial_num))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sw_version_pld)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrStatus3>) istream)
  "Deserializes a message object of type '<EsrStatus3>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'interface_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hw_version)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sw_version_host) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sw_version_host) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serial_num) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'serial_num) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sw_version_pld)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrStatus3>)))
  "Returns string type for a message object of type '<EsrStatus3>"
  "delphi_esr_msgs/EsrStatus3")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrStatus3)))
  "Returns string type for a message object of type 'EsrStatus3"
  "delphi_esr_msgs/EsrStatus3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrStatus3>)))
  "Returns md5sum for a message object of type '<EsrStatus3>"
  "583f51b7628983715fa50ecaf2d76bc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrStatus3)))
  "Returns md5sum for a message object of type 'EsrStatus3"
  "583f51b7628983715fa50ecaf2d76bc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrStatus3>)))
  "Returns full string definition for message of type '<EsrStatus3>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status3~%string      canmsg~%~%uint8       interface_version~%uint8       hw_version~%string      sw_version_host~%string      serial_num~%uint8       sw_version_pld~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrStatus3)))
  "Returns full string definition for message of type 'EsrStatus3"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status3~%string      canmsg~%~%uint8       interface_version~%uint8       hw_version~%string      sw_version_host~%string      serial_num~%uint8       sw_version_pld~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrStatus3>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'canmsg))
     1
     1
     4 (cl:length (cl:slot-value msg 'sw_version_host))
     4 (cl:length (cl:slot-value msg 'serial_num))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrStatus3>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrStatus3
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':interface_version (interface_version msg))
    (cl:cons ':hw_version (hw_version msg))
    (cl:cons ':sw_version_host (sw_version_host msg))
    (cl:cons ':serial_num (serial_num msg))
    (cl:cons ':sw_version_pld (sw_version_pld msg))
))
