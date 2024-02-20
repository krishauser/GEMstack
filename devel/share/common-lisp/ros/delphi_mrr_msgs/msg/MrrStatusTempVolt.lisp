; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude MrrStatusTempVolt.msg.html

(cl:defclass <MrrStatusTempVolt> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_batt_volts
    :reader can_batt_volts
    :initarg :can_batt_volts
    :type cl:float
    :initform 0.0)
   (can_1_25_v
    :reader can_1_25_v
    :initarg :can_1_25_v
    :type cl:float
    :initform 0.0)
   (can_5_v
    :reader can_5_v
    :initarg :can_5_v
    :type cl:float
    :initform 0.0)
   (can_3_3_v_raw
    :reader can_3_3_v_raw
    :initarg :can_3_3_v_raw
    :type cl:float
    :initform 0.0)
   (can_3_3_v_dac
    :reader can_3_3_v_dac
    :initarg :can_3_3_v_dac
    :type cl:float
    :initform 0.0)
   (can_mmic_temp1
    :reader can_mmic_temp1
    :initarg :can_mmic_temp1
    :type cl:fixnum
    :initform 0)
   (can_processor_thermistor
    :reader can_processor_thermistor
    :initarg :can_processor_thermistor
    :type cl:fixnum
    :initform 0)
   (can_processor_temp1
    :reader can_processor_temp1
    :initarg :can_processor_temp1
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MrrStatusTempVolt (<MrrStatusTempVolt>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MrrStatusTempVolt>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MrrStatusTempVolt)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<MrrStatusTempVolt> is deprecated: use delphi_mrr_msgs-msg:MrrStatusTempVolt instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_batt_volts-val :lambda-list '(m))
(cl:defmethod can_batt_volts-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_batt_volts-val is deprecated.  Use delphi_mrr_msgs-msg:can_batt_volts instead.")
  (can_batt_volts m))

(cl:ensure-generic-function 'can_1_25_v-val :lambda-list '(m))
(cl:defmethod can_1_25_v-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_1_25_v-val is deprecated.  Use delphi_mrr_msgs-msg:can_1_25_v instead.")
  (can_1_25_v m))

(cl:ensure-generic-function 'can_5_v-val :lambda-list '(m))
(cl:defmethod can_5_v-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_5_v-val is deprecated.  Use delphi_mrr_msgs-msg:can_5_v instead.")
  (can_5_v m))

(cl:ensure-generic-function 'can_3_3_v_raw-val :lambda-list '(m))
(cl:defmethod can_3_3_v_raw-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_3_3_v_raw-val is deprecated.  Use delphi_mrr_msgs-msg:can_3_3_v_raw instead.")
  (can_3_3_v_raw m))

(cl:ensure-generic-function 'can_3_3_v_dac-val :lambda-list '(m))
(cl:defmethod can_3_3_v_dac-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_3_3_v_dac-val is deprecated.  Use delphi_mrr_msgs-msg:can_3_3_v_dac instead.")
  (can_3_3_v_dac m))

(cl:ensure-generic-function 'can_mmic_temp1-val :lambda-list '(m))
(cl:defmethod can_mmic_temp1-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_mmic_temp1-val is deprecated.  Use delphi_mrr_msgs-msg:can_mmic_temp1 instead.")
  (can_mmic_temp1 m))

(cl:ensure-generic-function 'can_processor_thermistor-val :lambda-list '(m))
(cl:defmethod can_processor_thermistor-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_processor_thermistor-val is deprecated.  Use delphi_mrr_msgs-msg:can_processor_thermistor instead.")
  (can_processor_thermistor m))

(cl:ensure-generic-function 'can_processor_temp1-val :lambda-list '(m))
(cl:defmethod can_processor_temp1-val ((m <MrrStatusTempVolt>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:can_processor_temp1-val is deprecated.  Use delphi_mrr_msgs-msg:can_processor_temp1 instead.")
  (can_processor_temp1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MrrStatusTempVolt>) ostream)
  "Serializes a message object of type '<MrrStatusTempVolt>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_batt_volts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_1_25_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_5_v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_3_3_v_raw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_3_3_v_dac))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'can_mmic_temp1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'can_processor_thermistor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'can_processor_temp1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MrrStatusTempVolt>) istream)
  "Deserializes a message object of type '<MrrStatusTempVolt>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_batt_volts) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_1_25_v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_5_v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_3_3_v_raw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_3_3_v_dac) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'can_mmic_temp1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'can_processor_thermistor) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'can_processor_temp1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MrrStatusTempVolt>)))
  "Returns string type for a message object of type '<MrrStatusTempVolt>"
  "delphi_mrr_msgs/MrrStatusTempVolt")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MrrStatusTempVolt)))
  "Returns string type for a message object of type 'MrrStatusTempVolt"
  "delphi_mrr_msgs/MrrStatusTempVolt")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MrrStatusTempVolt>)))
  "Returns md5sum for a message object of type '<MrrStatusTempVolt>"
  "4200163646320431c4493e783122961d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MrrStatusTempVolt)))
  "Returns md5sum for a message object of type 'MrrStatusTempVolt"
  "4200163646320431c4493e783122961d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MrrStatusTempVolt>)))
  "Returns full string definition for message of type '<MrrStatusTempVolt>"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 can_batt_volts~%float32 can_1_25_v~%float32 can_5_v~%float32 can_3_3_v_raw~%float32 can_3_3_v_dac~%int8    can_mmic_temp1~%int8    can_processor_thermistor~%int8    can_processor_temp1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MrrStatusTempVolt)))
  "Returns full string definition for message of type 'MrrStatusTempVolt"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 can_batt_volts~%float32 can_1_25_v~%float32 can_5_v~%float32 can_3_3_v_raw~%float32 can_3_3_v_dac~%int8    can_mmic_temp1~%int8    can_processor_thermistor~%int8    can_processor_temp1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MrrStatusTempVolt>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MrrStatusTempVolt>))
  "Converts a ROS message object to a list"
  (cl:list 'MrrStatusTempVolt
    (cl:cons ':header (header msg))
    (cl:cons ':can_batt_volts (can_batt_volts msg))
    (cl:cons ':can_1_25_v (can_1_25_v msg))
    (cl:cons ':can_5_v (can_5_v msg))
    (cl:cons ':can_3_3_v_raw (can_3_3_v_raw msg))
    (cl:cons ':can_3_3_v_dac (can_3_3_v_dac msg))
    (cl:cons ':can_mmic_temp1 (can_mmic_temp1 msg))
    (cl:cons ':can_processor_thermistor (can_processor_thermistor msg))
    (cl:cons ':can_processor_temp1 (can_processor_temp1 msg))
))
