; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrStatus4.msg.html

(cl:defclass <EsrStatus4> (roslisp-msg-protocol:ros-message)
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
   (truck_target_det
    :reader truck_target_det
    :initarg :truck_target_det
    :type cl:boolean
    :initform cl:nil)
   (lr_only_grating_lobe_det
    :reader lr_only_grating_lobe_det
    :initarg :lr_only_grating_lobe_det
    :type cl:boolean
    :initform cl:nil)
   (sidelobe_blockage
    :reader sidelobe_blockage
    :initarg :sidelobe_blockage
    :type cl:boolean
    :initform cl:nil)
   (partial_blockage
    :reader partial_blockage
    :initarg :partial_blockage
    :type cl:boolean
    :initform cl:nil)
   (mr_lr_mode
    :reader mr_lr_mode
    :initarg :mr_lr_mode
    :type cl:fixnum
    :initform 0)
   (rolling_count_3
    :reader rolling_count_3
    :initarg :rolling_count_3
    :type cl:fixnum
    :initform 0)
   (path_id_acc
    :reader path_id_acc
    :initarg :path_id_acc
    :type cl:fixnum
    :initform 0)
   (path_id_cmbb_move
    :reader path_id_cmbb_move
    :initarg :path_id_cmbb_move
    :type cl:fixnum
    :initform 0)
   (path_id_cmbb_stat
    :reader path_id_cmbb_stat
    :initarg :path_id_cmbb_stat
    :type cl:fixnum
    :initform 0)
   (path_id_fcw_move
    :reader path_id_fcw_move
    :initarg :path_id_fcw_move
    :type cl:fixnum
    :initform 0)
   (path_id_fcw_stat
    :reader path_id_fcw_stat
    :initarg :path_id_fcw_stat
    :type cl:fixnum
    :initform 0)
   (auto_align_angle
    :reader auto_align_angle
    :initarg :auto_align_angle
    :type cl:float
    :initform 0.0)
   (path_id_acc_stat
    :reader path_id_acc_stat
    :initarg :path_id_acc_stat
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EsrStatus4 (<EsrStatus4>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrStatus4>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrStatus4)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrStatus4> is deprecated: use delphi_esr_msgs-msg:EsrStatus4 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'truck_target_det-val :lambda-list '(m))
(cl:defmethod truck_target_det-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:truck_target_det-val is deprecated.  Use delphi_esr_msgs-msg:truck_target_det instead.")
  (truck_target_det m))

(cl:ensure-generic-function 'lr_only_grating_lobe_det-val :lambda-list '(m))
(cl:defmethod lr_only_grating_lobe_det-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:lr_only_grating_lobe_det-val is deprecated.  Use delphi_esr_msgs-msg:lr_only_grating_lobe_det instead.")
  (lr_only_grating_lobe_det m))

(cl:ensure-generic-function 'sidelobe_blockage-val :lambda-list '(m))
(cl:defmethod sidelobe_blockage-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:sidelobe_blockage-val is deprecated.  Use delphi_esr_msgs-msg:sidelobe_blockage instead.")
  (sidelobe_blockage m))

(cl:ensure-generic-function 'partial_blockage-val :lambda-list '(m))
(cl:defmethod partial_blockage-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:partial_blockage-val is deprecated.  Use delphi_esr_msgs-msg:partial_blockage instead.")
  (partial_blockage m))

(cl:ensure-generic-function 'mr_lr_mode-val :lambda-list '(m))
(cl:defmethod mr_lr_mode-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:mr_lr_mode-val is deprecated.  Use delphi_esr_msgs-msg:mr_lr_mode instead.")
  (mr_lr_mode m))

(cl:ensure-generic-function 'rolling_count_3-val :lambda-list '(m))
(cl:defmethod rolling_count_3-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:rolling_count_3-val is deprecated.  Use delphi_esr_msgs-msg:rolling_count_3 instead.")
  (rolling_count_3 m))

(cl:ensure-generic-function 'path_id_acc-val :lambda-list '(m))
(cl:defmethod path_id_acc-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_acc-val is deprecated.  Use delphi_esr_msgs-msg:path_id_acc instead.")
  (path_id_acc m))

(cl:ensure-generic-function 'path_id_cmbb_move-val :lambda-list '(m))
(cl:defmethod path_id_cmbb_move-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_cmbb_move-val is deprecated.  Use delphi_esr_msgs-msg:path_id_cmbb_move instead.")
  (path_id_cmbb_move m))

(cl:ensure-generic-function 'path_id_cmbb_stat-val :lambda-list '(m))
(cl:defmethod path_id_cmbb_stat-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_cmbb_stat-val is deprecated.  Use delphi_esr_msgs-msg:path_id_cmbb_stat instead.")
  (path_id_cmbb_stat m))

(cl:ensure-generic-function 'path_id_fcw_move-val :lambda-list '(m))
(cl:defmethod path_id_fcw_move-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_fcw_move-val is deprecated.  Use delphi_esr_msgs-msg:path_id_fcw_move instead.")
  (path_id_fcw_move m))

(cl:ensure-generic-function 'path_id_fcw_stat-val :lambda-list '(m))
(cl:defmethod path_id_fcw_stat-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_fcw_stat-val is deprecated.  Use delphi_esr_msgs-msg:path_id_fcw_stat instead.")
  (path_id_fcw_stat m))

(cl:ensure-generic-function 'auto_align_angle-val :lambda-list '(m))
(cl:defmethod auto_align_angle-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:auto_align_angle-val is deprecated.  Use delphi_esr_msgs-msg:auto_align_angle instead.")
  (auto_align_angle m))

(cl:ensure-generic-function 'path_id_acc_stat-val :lambda-list '(m))
(cl:defmethod path_id_acc_stat-val ((m <EsrStatus4>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:path_id_acc_stat-val is deprecated.  Use delphi_esr_msgs-msg:path_id_acc_stat instead.")
  (path_id_acc_stat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrStatus4>) ostream)
  "Serializes a message object of type '<EsrStatus4>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'truck_target_det) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lr_only_grating_lobe_det) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sidelobe_blockage) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'partial_blockage) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mr_lr_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_cmbb_move)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_cmbb_stat)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_fcw_move)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_fcw_stat)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'auto_align_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc_stat)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrStatus4>) istream)
  "Deserializes a message object of type '<EsrStatus4>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'truck_target_det) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'lr_only_grating_lobe_det) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'sidelobe_blockage) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'partial_blockage) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mr_lr_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_cmbb_move)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_cmbb_stat)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_fcw_move)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_fcw_stat)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'auto_align_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'path_id_acc_stat)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrStatus4>)))
  "Returns string type for a message object of type '<EsrStatus4>"
  "delphi_esr_msgs/EsrStatus4")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrStatus4)))
  "Returns string type for a message object of type 'EsrStatus4"
  "delphi_esr_msgs/EsrStatus4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrStatus4>)))
  "Returns md5sum for a message object of type '<EsrStatus4>"
  "6d073b78c0d621fce59ffa9fb7c576de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrStatus4)))
  "Returns md5sum for a message object of type 'EsrStatus4"
  "6d073b78c0d621fce59ffa9fb7c576de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrStatus4>)))
  "Returns full string definition for message of type '<EsrStatus4>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status4~%string      canmsg~%~%bool        truck_target_det~%bool        lr_only_grating_lobe_det~%bool        sidelobe_blockage~%bool        partial_blockage~%uint8       mr_lr_mode~%uint8       rolling_count_3~%uint8       path_id_acc~%uint8       path_id_cmbb_move~%uint8       path_id_cmbb_stat~%uint8       path_id_fcw_move~%uint8       path_id_fcw_stat~%float32     auto_align_angle~%uint8       path_id_acc_stat~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrStatus4)))
  "Returns full string definition for message of type 'EsrStatus4"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR Status4~%string      canmsg~%~%bool        truck_target_det~%bool        lr_only_grating_lobe_det~%bool        sidelobe_blockage~%bool        partial_blockage~%uint8       mr_lr_mode~%uint8       rolling_count_3~%uint8       path_id_acc~%uint8       path_id_cmbb_move~%uint8       path_id_cmbb_stat~%uint8       path_id_fcw_move~%uint8       path_id_fcw_stat~%float32     auto_align_angle~%uint8       path_id_acc_stat~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrStatus4>))
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
     1
     1
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrStatus4>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrStatus4
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':truck_target_det (truck_target_det msg))
    (cl:cons ':lr_only_grating_lobe_det (lr_only_grating_lobe_det msg))
    (cl:cons ':sidelobe_blockage (sidelobe_blockage msg))
    (cl:cons ':partial_blockage (partial_blockage msg))
    (cl:cons ':mr_lr_mode (mr_lr_mode msg))
    (cl:cons ':rolling_count_3 (rolling_count_3 msg))
    (cl:cons ':path_id_acc (path_id_acc msg))
    (cl:cons ':path_id_cmbb_move (path_id_cmbb_move msg))
    (cl:cons ':path_id_cmbb_stat (path_id_cmbb_stat msg))
    (cl:cons ':path_id_fcw_move (path_id_fcw_move msg))
    (cl:cons ':path_id_fcw_stat (path_id_fcw_stat msg))
    (cl:cons ':auto_align_angle (auto_align_angle msg))
    (cl:cons ':path_id_acc_stat (path_id_acc_stat msg))
))
