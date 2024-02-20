; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrStatus2.msg.html

(cl:defclass <EsrStatus2> (roslisp-msg-protocol:ros-message)
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
   (maximum_tracks_ack
    :reader maximum_tracks_ack
    :initarg :maximum_tracks_ack
    :type cl:fixnum
    :initform 0)
   (rolling_count_2
    :reader rolling_count_2
    :initarg :rolling_count_2
    :type cl:fixnum
    :initform 0)
   (overheat_error
    :reader overheat_error
    :initarg :overheat_error
    :type cl:boolean
    :initform cl:nil)
   (range_perf_error
    :reader range_perf_error
    :initarg :range_perf_error
    :type cl:boolean
    :initform cl:nil)
   (internal_error
    :reader internal_error
    :initarg :internal_error
    :type cl:boolean
    :initform cl:nil)
   (xcvr_operational
    :reader xcvr_operational
    :initarg :xcvr_operational
    :type cl:boolean
    :initform cl:nil)
   (raw_data_mode
    :reader raw_data_mode
    :initarg :raw_data_mode
    :type cl:boolean
    :initform cl:nil)
   (steering_angle_ack
    :reader steering_angle_ack
    :initarg :steering_angle_ack
    :type cl:fixnum
    :initform 0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:fixnum
    :initform 0)
   (veh_spd_comp_factor
    :reader veh_spd_comp_factor
    :initarg :veh_spd_comp_factor
    :type cl:float
    :initform 0.0)
   (grouping_mode
    :reader grouping_mode
    :initarg :grouping_mode
    :type cl:fixnum
    :initform 0)
   (yaw_rate_bias
    :reader yaw_rate_bias
    :initarg :yaw_rate_bias
    :type cl:float
    :initform 0.0)
   (sw_version_dsp
    :reader sw_version_dsp
    :initarg :sw_version_dsp
    :type cl:string
    :initform ""))
)

(cl:defclass EsrStatus2 (<EsrStatus2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrStatus2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrStatus2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrStatus2> is deprecated: use delphi_esr_msgs-msg:EsrStatus2 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'maximum_tracks_ack-val :lambda-list '(m))
(cl:defmethod maximum_tracks_ack-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:maximum_tracks_ack-val is deprecated.  Use delphi_esr_msgs-msg:maximum_tracks_ack instead.")
  (maximum_tracks_ack m))

(cl:ensure-generic-function 'rolling_count_2-val :lambda-list '(m))
(cl:defmethod rolling_count_2-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:rolling_count_2-val is deprecated.  Use delphi_esr_msgs-msg:rolling_count_2 instead.")
  (rolling_count_2 m))

(cl:ensure-generic-function 'overheat_error-val :lambda-list '(m))
(cl:defmethod overheat_error-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:overheat_error-val is deprecated.  Use delphi_esr_msgs-msg:overheat_error instead.")
  (overheat_error m))

(cl:ensure-generic-function 'range_perf_error-val :lambda-list '(m))
(cl:defmethod range_perf_error-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:range_perf_error-val is deprecated.  Use delphi_esr_msgs-msg:range_perf_error instead.")
  (range_perf_error m))

(cl:ensure-generic-function 'internal_error-val :lambda-list '(m))
(cl:defmethod internal_error-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:internal_error-val is deprecated.  Use delphi_esr_msgs-msg:internal_error instead.")
  (internal_error m))

(cl:ensure-generic-function 'xcvr_operational-val :lambda-list '(m))
(cl:defmethod xcvr_operational-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:xcvr_operational-val is deprecated.  Use delphi_esr_msgs-msg:xcvr_operational instead.")
  (xcvr_operational m))

(cl:ensure-generic-function 'raw_data_mode-val :lambda-list '(m))
(cl:defmethod raw_data_mode-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:raw_data_mode-val is deprecated.  Use delphi_esr_msgs-msg:raw_data_mode instead.")
  (raw_data_mode m))

(cl:ensure-generic-function 'steering_angle_ack-val :lambda-list '(m))
(cl:defmethod steering_angle_ack-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:steering_angle_ack-val is deprecated.  Use delphi_esr_msgs-msg:steering_angle_ack instead.")
  (steering_angle_ack m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:temperature-val is deprecated.  Use delphi_esr_msgs-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'veh_spd_comp_factor-val :lambda-list '(m))
(cl:defmethod veh_spd_comp_factor-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:veh_spd_comp_factor-val is deprecated.  Use delphi_esr_msgs-msg:veh_spd_comp_factor instead.")
  (veh_spd_comp_factor m))

(cl:ensure-generic-function 'grouping_mode-val :lambda-list '(m))
(cl:defmethod grouping_mode-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:grouping_mode-val is deprecated.  Use delphi_esr_msgs-msg:grouping_mode instead.")
  (grouping_mode m))

(cl:ensure-generic-function 'yaw_rate_bias-val :lambda-list '(m))
(cl:defmethod yaw_rate_bias-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:yaw_rate_bias-val is deprecated.  Use delphi_esr_msgs-msg:yaw_rate_bias instead.")
  (yaw_rate_bias m))

(cl:ensure-generic-function 'sw_version_dsp-val :lambda-list '(m))
(cl:defmethod sw_version_dsp-val ((m <EsrStatus2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:sw_version_dsp-val is deprecated.  Use delphi_esr_msgs-msg:sw_version_dsp instead.")
  (sw_version_dsp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrStatus2>) ostream)
  "Serializes a message object of type '<EsrStatus2>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'maximum_tracks_ack)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'overheat_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'range_perf_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'internal_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'xcvr_operational) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'raw_data_mode) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'steering_angle_ack)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'steering_angle_ack)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'veh_spd_comp_factor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'grouping_mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate_bias))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'sw_version_dsp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'sw_version_dsp))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrStatus2>) istream)
  "Deserializes a message object of type '<EsrStatus2>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'maximum_tracks_ack)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_2)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'overheat_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'range_perf_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'internal_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'xcvr_operational) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'raw_data_mode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'steering_angle_ack)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'steering_angle_ack)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temperature) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'veh_spd_comp_factor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'grouping_mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate_bias) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sw_version_dsp) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'sw_version_dsp) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrStatus2>)))
  "Returns string type for a message object of type '<EsrStatus2>"
  "delphi_esr_msgs/EsrStatus2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrStatus2)))
  "Returns string type for a message object of type 'EsrStatus2"
  "delphi_esr_msgs/EsrStatus2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrStatus2>)))
  "Returns md5sum for a message object of type '<EsrStatus2>"
  "5f62b9bdfb50f851bdab4bdfcce1c49b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrStatus2)))
  "Returns md5sum for a message object of type 'EsrStatus2"
  "5f62b9bdfb50f851bdab4bdfcce1c49b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrStatus2>)))
  "Returns full string definition for message of type '<EsrStatus2>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status2~%string      canmsg~%~%uint8       maximum_tracks_ack~%uint8       rolling_count_2~%bool        overheat_error~%bool        range_perf_error~%bool        internal_error~%bool        xcvr_operational~%bool        raw_data_mode~%uint16      steering_angle_ack~%int8        temperature~%float32     veh_spd_comp_factor~%uint8       grouping_mode~%float32     yaw_rate_bias~%string      sw_version_dsp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrStatus2)))
  "Returns full string definition for message of type 'EsrStatus2"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status2~%string      canmsg~%~%uint8       maximum_tracks_ack~%uint8       rolling_count_2~%bool        overheat_error~%bool        range_perf_error~%bool        internal_error~%bool        xcvr_operational~%bool        raw_data_mode~%uint16      steering_angle_ack~%int8        temperature~%float32     veh_spd_comp_factor~%uint8       grouping_mode~%float32     yaw_rate_bias~%string      sw_version_dsp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrStatus2>))
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
     2
     1
     4
     1
     4
     4 (cl:length (cl:slot-value msg 'sw_version_dsp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrStatus2>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrStatus2
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':maximum_tracks_ack (maximum_tracks_ack msg))
    (cl:cons ':rolling_count_2 (rolling_count_2 msg))
    (cl:cons ':overheat_error (overheat_error msg))
    (cl:cons ':range_perf_error (range_perf_error msg))
    (cl:cons ':internal_error (internal_error msg))
    (cl:cons ':xcvr_operational (xcvr_operational msg))
    (cl:cons ':raw_data_mode (raw_data_mode msg))
    (cl:cons ':steering_angle_ack (steering_angle_ack msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':veh_spd_comp_factor (veh_spd_comp_factor msg))
    (cl:cons ':grouping_mode (grouping_mode msg))
    (cl:cons ':yaw_rate_bias (yaw_rate_bias msg))
    (cl:cons ':sw_version_dsp (sw_version_dsp msg))
))
