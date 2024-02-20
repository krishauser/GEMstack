; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrStatus9.msg.html

(cl:defclass <EsrStatus9> (roslisp-msg-protocol:ros-message)
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
   (avg_pwr_cwblkg
    :reader avg_pwr_cwblkg
    :initarg :avg_pwr_cwblkg
    :type cl:fixnum
    :initform 0)
   (sideslip_angle
    :reader sideslip_angle
    :initarg :sideslip_angle
    :type cl:float
    :initform 0.0)
   (serial_num_3rd_byte
    :reader serial_num_3rd_byte
    :initarg :serial_num_3rd_byte
    :type cl:fixnum
    :initform 0)
   (water_spray_target_id
    :reader water_spray_target_id
    :initarg :water_spray_target_id
    :type cl:fixnum
    :initform 0)
   (filtered_xohp_acc_cipv
    :reader filtered_xohp_acc_cipv
    :initarg :filtered_xohp_acc_cipv
    :type cl:float
    :initform 0.0)
   (path_id_acc_2
    :reader path_id_acc_2
    :initarg :path_id_acc_2
    :type cl:fixnum
    :initform 0)
   (path_id_acc_3
    :reader path_id_acc_3
    :initarg :path_id_acc_3
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EsrStatus9 (<EsrStatus9>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrStatus9>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrStatus9)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrStatus9> is deprecated: use delphi_esr_msgs-msg:EsrStatus9 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'avg_pwr_cwblkg-val :lambda-list '(m))
(cl:defmethod avg_pwr_cwblkg-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:avg_pwr_cwblkg-val is deprecated.  Use delphi_esr_msgs-msg:avg_pwr_cwblkg instead.")
  (avg_pwr_cwblkg m))

(cl:ensure-generic-function 'sideslip_angle-val :lambda-list '(m))
(cl:defmethod sideslip_angle-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:sideslip_angle-val is deprecated.  Use delphi_esr_msgs-msg:sideslip_angle instead.")
  (sideslip_angle m))

(cl:ensure-generic-function 'serial_num_3rd_byte-val :lambda-list '(m))
(cl:defmethod serial_num_3rd_byte-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:serial_num_3rd_byte-val is deprecated.  Use delphi_esr_msgs-msg:serial_num_3rd_byte instead.")
  (serial_num_3rd_byte m))

(cl:ensure-generic-function 'water_spray_target_id-val :lambda-list '(m))
(cl:defmethod water_spray_target_id-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:water_spray_target_id-val is deprecated.  Use delphi_esr_msgs-msg:water_spray_target_id instead.")
  (water_spray_target_id m))

(cl:ensure-generic-function 'filtered_xohp_acc_cipv-val :lambda-list '(m))
(cl:defmethod filtered_xohp_acc_cipv-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:filtered_xohp_acc_cipv-val is deprecated.  Use delphi_esr_msgs-msg:filtered_xohp_acc_cipv instead.")
  (filtered_xohp_acc_cipv m))

(cl:ensure-generic-function 'path_id_acc_2-val :lambda-list '(m))
(cl:defmethod path_id_acc_2-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_acc_2-val is deprecated.  Use delphi_esr_msgs-msg:path_id_acc_2 instead.")
  (path_id_acc_2 m))

(cl:ensure-generic-function 'path_id_acc_3-val :lambda-list '(m))
(cl:defmethod path_id_acc_3-val ((m <EsrStatus9>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_acc_3-val is deprecated.  Use delphi_esr_msgs-msg:path_id_acc_3 instead.")
  (path_id_acc_3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrStatus9>) ostream)
  "Serializes a message object of type '<EsrStatus9>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'avg_pwr_cwblkg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'avg_pwr_cwblkg)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sideslip_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'serial_num_3rd_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'water_spray_target_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'filtered_xohp_acc_cipv))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc_3)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrStatus9>) istream)
  "Deserializes a message object of type '<EsrStatus9>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'avg_pwr_cwblkg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'avg_pwr_cwblkg)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sideslip_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'serial_num_3rd_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'water_spray_target_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'filtered_xohp_acc_cipv) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc_3)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrStatus9>)))
  "Returns string type for a message object of type '<EsrStatus9>"
  "delphi_esr_msgs/EsrStatus9")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrStatus9)))
  "Returns string type for a message object of type 'EsrStatus9"
  "delphi_esr_msgs/EsrStatus9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrStatus9>)))
  "Returns md5sum for a message object of type '<EsrStatus9>"
  "303ebfcbdd8866a01d6c2acd5e4df496")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrStatus9)))
  "Returns md5sum for a message object of type 'EsrStatus9"
  "303ebfcbdd8866a01d6c2acd5e4df496")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrStatus9>)))
  "Returns full string definition for message of type '<EsrStatus9>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status9~%string      canmsg~%~%uint16      avg_pwr_cwblkg~%float32     sideslip_angle~%uint8       serial_num_3rd_byte~%uint8       water_spray_target_id~%float32     filtered_xohp_acc_cipv~%uint8       path_id_acc_2~%uint8       path_id_acc_3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrStatus9)))
  "Returns full string definition for message of type 'EsrStatus9"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status9~%string      canmsg~%~%uint16      avg_pwr_cwblkg~%float32     sideslip_angle~%uint8       serial_num_3rd_byte~%uint8       water_spray_target_id~%float32     filtered_xohp_acc_cipv~%uint8       path_id_acc_2~%uint8       path_id_acc_3~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrStatus9>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'canmsg))
     2
     4
     1
     1
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrStatus9>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrStatus9
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':avg_pwr_cwblkg (avg_pwr_cwblkg msg))
    (cl:cons ':sideslip_angle (sideslip_angle msg))
    (cl:cons ':serial_num_3rd_byte (serial_num_3rd_byte msg))
    (cl:cons ':water_spray_target_id (water_spray_target_id msg))
    (cl:cons ':filtered_xohp_acc_cipv (filtered_xohp_acc_cipv msg))
    (cl:cons ':path_id_acc_2 (path_id_acc_2 msg))
    (cl:cons ':path_id_acc_3 (path_id_acc_3 msg))
))
