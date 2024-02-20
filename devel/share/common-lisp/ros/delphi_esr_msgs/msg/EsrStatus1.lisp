; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrStatus1.msg.html

(cl:defclass <EsrStatus1> (roslisp-msg-protocol:ros-message)
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
   (rolling_count_1
    :reader rolling_count_1
    :initarg :rolling_count_1
    :type cl:fixnum
    :initform 0)
   (dsp_timestamp
    :reader dsp_timestamp
    :initarg :dsp_timestamp
    :type cl:fixnum
    :initform 0)
   (comm_error
    :reader comm_error
    :initarg :comm_error
    :type cl:boolean
    :initform cl:nil)
   (radius_curvature_calc
    :reader radius_curvature_calc
    :initarg :radius_curvature_calc
    :type cl:fixnum
    :initform 0)
   (scan_index
    :reader scan_index
    :initarg :scan_index
    :type cl:fixnum
    :initform 0)
   (yaw_rate_calc
    :reader yaw_rate_calc
    :initarg :yaw_rate_calc
    :type cl:float
    :initform 0.0)
   (vehicle_speed_calc
    :reader vehicle_speed_calc
    :initarg :vehicle_speed_calc
    :type cl:float
    :initform 0.0))
)

(cl:defclass EsrStatus1 (<EsrStatus1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrStatus1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrStatus1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrStatus1> is deprecated: use delphi_esr_msgs-msg:EsrStatus1 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'rolling_count_1-val :lambda-list '(m))
(cl:defmethod rolling_count_1-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:rolling_count_1-val is deprecated.  Use delphi_esr_msgs-msg:rolling_count_1 instead.")
  (rolling_count_1 m))

(cl:ensure-generic-function 'dsp_timestamp-val :lambda-list '(m))
(cl:defmethod dsp_timestamp-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:dsp_timestamp-val is deprecated.  Use delphi_esr_msgs-msg:dsp_timestamp instead.")
  (dsp_timestamp m))

(cl:ensure-generic-function 'comm_error-val :lambda-list '(m))
(cl:defmethod comm_error-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:comm_error-val is deprecated.  Use delphi_esr_msgs-msg:comm_error instead.")
  (comm_error m))

(cl:ensure-generic-function 'radius_curvature_calc-val :lambda-list '(m))
(cl:defmethod radius_curvature_calc-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:radius_curvature_calc-val is deprecated.  Use delphi_esr_msgs-msg:radius_curvature_calc instead.")
  (radius_curvature_calc m))

(cl:ensure-generic-function 'scan_index-val :lambda-list '(m))
(cl:defmethod scan_index-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:scan_index-val is deprecated.  Use delphi_esr_msgs-msg:scan_index instead.")
  (scan_index m))

(cl:ensure-generic-function 'yaw_rate_calc-val :lambda-list '(m))
(cl:defmethod yaw_rate_calc-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:yaw_rate_calc-val is deprecated.  Use delphi_esr_msgs-msg:yaw_rate_calc instead.")
  (yaw_rate_calc m))

(cl:ensure-generic-function 'vehicle_speed_calc-val :lambda-list '(m))
(cl:defmethod vehicle_speed_calc-val ((m <EsrStatus1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:vehicle_speed_calc-val is deprecated.  Use delphi_esr_msgs-msg:vehicle_speed_calc instead.")
  (vehicle_speed_calc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrStatus1>) ostream)
  "Serializes a message object of type '<EsrStatus1>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dsp_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'comm_error) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'radius_curvature_calc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scan_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'scan_index)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate_calc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vehicle_speed_calc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrStatus1>) istream)
  "Deserializes a message object of type '<EsrStatus1>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dsp_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'comm_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'radius_curvature_calc) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'scan_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'scan_index)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate_calc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vehicle_speed_calc) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrStatus1>)))
  "Returns string type for a message object of type '<EsrStatus1>"
  "delphi_esr_msgs/EsrStatus1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrStatus1)))
  "Returns string type for a message object of type 'EsrStatus1"
  "delphi_esr_msgs/EsrStatus1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrStatus1>)))
  "Returns md5sum for a message object of type '<EsrStatus1>"
  "f3f440bdd87b7ce3da2d8d915a5970b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrStatus1)))
  "Returns md5sum for a message object of type 'EsrStatus1"
  "f3f440bdd87b7ce3da2d8d915a5970b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrStatus1>)))
  "Returns full string definition for message of type '<EsrStatus1>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status1~%string      canmsg~%~%uint8       rolling_count_1~%uint8       dsp_timestamp~%bool        comm_error~%int16       radius_curvature_calc~%uint16      scan_index~%float32     yaw_rate_calc~%float32     vehicle_speed_calc~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrStatus1)))
  "Returns full string definition for message of type 'EsrStatus1"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status1~%string      canmsg~%~%uint8       rolling_count_1~%uint8       dsp_timestamp~%bool        comm_error~%int16       radius_curvature_calc~%uint16      scan_index~%float32     yaw_rate_calc~%float32     vehicle_speed_calc~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrStatus1>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'canmsg))
     1
     1
     1
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrStatus1>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrStatus1
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':rolling_count_1 (rolling_count_1 msg))
    (cl:cons ':dsp_timestamp (dsp_timestamp msg))
    (cl:cons ':comm_error (comm_error msg))
    (cl:cons ':radius_curvature_calc (radius_curvature_calc msg))
    (cl:cons ':scan_index (scan_index msg))
    (cl:cons ':yaw_rate_calc (yaw_rate_calc msg))
    (cl:cons ':vehicle_speed_calc (vehicle_speed_calc msg))
))
