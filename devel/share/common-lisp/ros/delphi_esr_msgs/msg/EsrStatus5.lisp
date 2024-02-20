; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrStatus5.msg.html

(cl:defclass <EsrStatus5> (roslisp-msg-protocol:ros-message)
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
   (swbatt_a2d
    :reader swbatt_a2d
    :initarg :swbatt_a2d
    :type cl:fixnum
    :initform 0)
   (ignp_a2d
    :reader ignp_a2d
    :initarg :ignp_a2d
    :type cl:fixnum
    :initform 0)
   (temp1_a2d
    :reader temp1_a2d
    :initarg :temp1_a2d
    :type cl:fixnum
    :initform 0)
   (temp2_a2d
    :reader temp2_a2d
    :initarg :temp2_a2d
    :type cl:fixnum
    :initform 0)
   (supply_5va_a2d
    :reader supply_5va_a2d
    :initarg :supply_5va_a2d
    :type cl:fixnum
    :initform 0)
   (supply_5vdx_a2d
    :reader supply_5vdx_a2d
    :initarg :supply_5vdx_a2d
    :type cl:fixnum
    :initform 0)
   (supply_3p3v_a2d
    :reader supply_3p3v_a2d
    :initarg :supply_3p3v_a2d
    :type cl:fixnum
    :initform 0)
   (supply_10v_a2d
    :reader supply_10v_a2d
    :initarg :supply_10v_a2d
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EsrStatus5 (<EsrStatus5>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrStatus5>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrStatus5)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrStatus5> is deprecated: use delphi_esr_msgs-msg:EsrStatus5 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'swbatt_a2d-val :lambda-list '(m))
(cl:defmethod swbatt_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:swbatt_a2d-val is deprecated.  Use delphi_esr_msgs-msg:swbatt_a2d instead.")
  (swbatt_a2d m))

(cl:ensure-generic-function 'ignp_a2d-val :lambda-list '(m))
(cl:defmethod ignp_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:ignp_a2d-val is deprecated.  Use delphi_esr_msgs-msg:ignp_a2d instead.")
  (ignp_a2d m))

(cl:ensure-generic-function 'temp1_a2d-val :lambda-list '(m))
(cl:defmethod temp1_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:temp1_a2d-val is deprecated.  Use delphi_esr_msgs-msg:temp1_a2d instead.")
  (temp1_a2d m))

(cl:ensure-generic-function 'temp2_a2d-val :lambda-list '(m))
(cl:defmethod temp2_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:temp2_a2d-val is deprecated.  Use delphi_esr_msgs-msg:temp2_a2d instead.")
  (temp2_a2d m))

(cl:ensure-generic-function 'supply_5va_a2d-val :lambda-list '(m))
(cl:defmethod supply_5va_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:supply_5va_a2d-val is deprecated.  Use delphi_esr_msgs-msg:supply_5va_a2d instead.")
  (supply_5va_a2d m))

(cl:ensure-generic-function 'supply_5vdx_a2d-val :lambda-list '(m))
(cl:defmethod supply_5vdx_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:supply_5vdx_a2d-val is deprecated.  Use delphi_esr_msgs-msg:supply_5vdx_a2d instead.")
  (supply_5vdx_a2d m))

(cl:ensure-generic-function 'supply_3p3v_a2d-val :lambda-list '(m))
(cl:defmethod supply_3p3v_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:supply_3p3v_a2d-val is deprecated.  Use delphi_esr_msgs-msg:supply_3p3v_a2d instead.")
  (supply_3p3v_a2d m))

(cl:ensure-generic-function 'supply_10v_a2d-val :lambda-list '(m))
(cl:defmethod supply_10v_a2d-val ((m <EsrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:supply_10v_a2d-val is deprecated.  Use delphi_esr_msgs-msg:supply_10v_a2d instead.")
  (supply_10v_a2d m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrStatus5>) ostream)
  "Serializes a message object of type '<EsrStatus5>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swbatt_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ignp_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp1_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp2_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_5va_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_5vdx_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_3p3v_a2d)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_10v_a2d)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrStatus5>) istream)
  "Deserializes a message object of type '<EsrStatus5>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swbatt_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ignp_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp1_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp2_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_5va_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_5vdx_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_3p3v_a2d)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'supply_10v_a2d)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrStatus5>)))
  "Returns string type for a message object of type '<EsrStatus5>"
  "delphi_esr_msgs/EsrStatus5")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrStatus5)))
  "Returns string type for a message object of type 'EsrStatus5"
  "delphi_esr_msgs/EsrStatus5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrStatus5>)))
  "Returns md5sum for a message object of type '<EsrStatus5>"
  "9be5998c2855516519b427fa839cb6bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrStatus5)))
  "Returns md5sum for a message object of type 'EsrStatus5"
  "9be5998c2855516519b427fa839cb6bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrStatus5>)))
  "Returns full string definition for message of type '<EsrStatus5>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status5~%string      canmsg~%~%uint8       swbatt_a2d~%uint8       ignp_a2d~%uint8       temp1_a2d~%uint8       temp2_a2d~%uint8       supply_5va_a2d~%uint8       supply_5vdx_a2d~%uint8       supply_3p3v_a2d~%uint8       supply_10v_a2d~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrStatus5)))
  "Returns full string definition for message of type 'EsrStatus5"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status5~%string      canmsg~%~%uint8       swbatt_a2d~%uint8       ignp_a2d~%uint8       temp1_a2d~%uint8       temp2_a2d~%uint8       supply_5va_a2d~%uint8       supply_5vdx_a2d~%uint8       supply_3p3v_a2d~%uint8       supply_10v_a2d~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrStatus5>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'canmsg))
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrStatus5>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrStatus5
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':swbatt_a2d (swbatt_a2d msg))
    (cl:cons ':ignp_a2d (ignp_a2d msg))
    (cl:cons ':temp1_a2d (temp1_a2d msg))
    (cl:cons ':temp2_a2d (temp2_a2d msg))
    (cl:cons ':supply_5va_a2d (supply_5va_a2d msg))
    (cl:cons ':supply_5vdx_a2d (supply_5vdx_a2d msg))
    (cl:cons ':supply_3p3v_a2d (supply_3p3v_a2d msg))
    (cl:cons ':supply_10v_a2d (supply_10v_a2d msg))
))
