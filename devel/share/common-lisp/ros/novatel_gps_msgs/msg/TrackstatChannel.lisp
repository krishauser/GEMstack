; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude TrackstatChannel.msg.html

(cl:defclass <TrackstatChannel> (roslisp-msg-protocol:ros-message)
  ((prn
    :reader prn
    :initarg :prn
    :type cl:fixnum
    :initform 0)
   (glofreq
    :reader glofreq
    :initarg :glofreq
    :type cl:fixnum
    :initform 0)
   (ch_tr_status
    :reader ch_tr_status
    :initarg :ch_tr_status
    :type cl:integer
    :initform 0)
   (psr
    :reader psr
    :initarg :psr
    :type cl:float
    :initform 0.0)
   (doppler
    :reader doppler
    :initarg :doppler
    :type cl:float
    :initform 0.0)
   (c_no
    :reader c_no
    :initarg :c_no
    :type cl:float
    :initform 0.0)
   (locktime
    :reader locktime
    :initarg :locktime
    :type cl:float
    :initform 0.0)
   (psr_res
    :reader psr_res
    :initarg :psr_res
    :type cl:float
    :initform 0.0)
   (reject
    :reader reject
    :initarg :reject
    :type cl:string
    :initform "")
   (psr_weight
    :reader psr_weight
    :initarg :psr_weight
    :type cl:float
    :initform 0.0))
)

(cl:defclass TrackstatChannel (<TrackstatChannel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackstatChannel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackstatChannel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<TrackstatChannel> is deprecated: use novatel_gps_msgs-msg:TrackstatChannel instead.")))

(cl:ensure-generic-function 'prn-val :lambda-list '(m))
(cl:defmethod prn-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:prn-val is deprecated.  Use novatel_gps_msgs-msg:prn instead.")
  (prn m))

(cl:ensure-generic-function 'glofreq-val :lambda-list '(m))
(cl:defmethod glofreq-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:glofreq-val is deprecated.  Use novatel_gps_msgs-msg:glofreq instead.")
  (glofreq m))

(cl:ensure-generic-function 'ch_tr_status-val :lambda-list '(m))
(cl:defmethod ch_tr_status-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:ch_tr_status-val is deprecated.  Use novatel_gps_msgs-msg:ch_tr_status instead.")
  (ch_tr_status m))

(cl:ensure-generic-function 'psr-val :lambda-list '(m))
(cl:defmethod psr-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:psr-val is deprecated.  Use novatel_gps_msgs-msg:psr instead.")
  (psr m))

(cl:ensure-generic-function 'doppler-val :lambda-list '(m))
(cl:defmethod doppler-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:doppler-val is deprecated.  Use novatel_gps_msgs-msg:doppler instead.")
  (doppler m))

(cl:ensure-generic-function 'c_no-val :lambda-list '(m))
(cl:defmethod c_no-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:c_no-val is deprecated.  Use novatel_gps_msgs-msg:c_no instead.")
  (c_no m))

(cl:ensure-generic-function 'locktime-val :lambda-list '(m))
(cl:defmethod locktime-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:locktime-val is deprecated.  Use novatel_gps_msgs-msg:locktime instead.")
  (locktime m))

(cl:ensure-generic-function 'psr_res-val :lambda-list '(m))
(cl:defmethod psr_res-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:psr_res-val is deprecated.  Use novatel_gps_msgs-msg:psr_res instead.")
  (psr_res m))

(cl:ensure-generic-function 'reject-val :lambda-list '(m))
(cl:defmethod reject-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:reject-val is deprecated.  Use novatel_gps_msgs-msg:reject instead.")
  (reject m))

(cl:ensure-generic-function 'psr_weight-val :lambda-list '(m))
(cl:defmethod psr_weight-val ((m <TrackstatChannel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:psr_weight-val is deprecated.  Use novatel_gps_msgs-msg:psr_weight instead.")
  (psr_weight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackstatChannel>) ostream)
  "Serializes a message object of type '<TrackstatChannel>"
  (cl:let* ((signed (cl:slot-value msg 'prn)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'glofreq)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch_tr_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch_tr_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ch_tr_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ch_tr_status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'psr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'doppler))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c_no))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'locktime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psr_res))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reject))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reject))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psr_weight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackstatChannel>) istream)
  "Deserializes a message object of type '<TrackstatChannel>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'prn) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'glofreq) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ch_tr_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ch_tr_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ch_tr_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ch_tr_status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'doppler) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c_no) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'locktime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psr_res) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reject) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reject) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psr_weight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackstatChannel>)))
  "Returns string type for a message object of type '<TrackstatChannel>"
  "novatel_gps_msgs/TrackstatChannel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackstatChannel)))
  "Returns string type for a message object of type 'TrackstatChannel"
  "novatel_gps_msgs/TrackstatChannel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackstatChannel>)))
  "Returns md5sum for a message object of type '<TrackstatChannel>"
  "295831118c5ddfb83ac5b655586ae7ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackstatChannel)))
  "Returns md5sum for a message object of type 'TrackstatChannel"
  "295831118c5ddfb83ac5b655586ae7ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackstatChannel>)))
  "Returns full string definition for message of type '<TrackstatChannel>"
  (cl:format cl:nil "# A submessage of Trackstat that contains all of the data about a single hardware channel~%~%# Satellite PRN number~%int16 prn~%~%# GLONASS Frequency +7~%int16 glofreq~%~%# Channel tracking status~%uint32 ch_tr_status~%~%# Pseudorange (m)~%float64 psr~%~%# Doppler frequency (Hz)~%float32 doppler~%~%# Carrier to noise density ratio (dB-Hz)~%float32 c_no~%~%# Number of seconds of continuous tracking (no cycle slips)~%float32 locktime~%~%# Pseudorange residual from pseudorange filter (m)~%float32 psr_res~%~%# Range reject code from pseudorange filter~%string reject~%~%# Pseudorange filter weighting~%float32 psr_weight~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackstatChannel)))
  "Returns full string definition for message of type 'TrackstatChannel"
  (cl:format cl:nil "# A submessage of Trackstat that contains all of the data about a single hardware channel~%~%# Satellite PRN number~%int16 prn~%~%# GLONASS Frequency +7~%int16 glofreq~%~%# Channel tracking status~%uint32 ch_tr_status~%~%# Pseudorange (m)~%float64 psr~%~%# Doppler frequency (Hz)~%float32 doppler~%~%# Carrier to noise density ratio (dB-Hz)~%float32 c_no~%~%# Number of seconds of continuous tracking (no cycle slips)~%float32 locktime~%~%# Pseudorange residual from pseudorange filter (m)~%float32 psr_res~%~%# Range reject code from pseudorange filter~%string reject~%~%# Pseudorange filter weighting~%float32 psr_weight~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackstatChannel>))
  (cl:+ 0
     2
     2
     4
     8
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'reject))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackstatChannel>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackstatChannel
    (cl:cons ':prn (prn msg))
    (cl:cons ':glofreq (glofreq msg))
    (cl:cons ':ch_tr_status (ch_tr_status msg))
    (cl:cons ':psr (psr msg))
    (cl:cons ':doppler (doppler msg))
    (cl:cons ':c_no (c_no msg))
    (cl:cons ':locktime (locktime msg))
    (cl:cons ':psr_res (psr_res msg))
    (cl:cons ':reject (reject msg))
    (cl:cons ':psr_weight (psr_weight msg))
))
