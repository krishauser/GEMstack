; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrTrack.msg.html

(cl:defclass <SrrTrack> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (can_tx_detect_valid_level
    :reader can_tx_detect_valid_level
    :initarg :can_tx_detect_valid_level
    :type cl:fixnum
    :initform 0)
   (can_tx_detect_status
    :reader can_tx_detect_status
    :initarg :can_tx_detect_status
    :type cl:boolean
    :initform cl:nil)
   (can_tx_detect_range_rate
    :reader can_tx_detect_range_rate
    :initarg :can_tx_detect_range_rate
    :type cl:float
    :initform 0.0)
   (can_tx_detect_range
    :reader can_tx_detect_range
    :initarg :can_tx_detect_range
    :type cl:float
    :initform 0.0)
   (can_tx_detect_angle
    :reader can_tx_detect_angle
    :initarg :can_tx_detect_angle
    :type cl:float
    :initform 0.0)
   (can_tx_detect_amplitude
    :reader can_tx_detect_amplitude
    :initarg :can_tx_detect_amplitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass SrrTrack (<SrrTrack>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrTrack>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrTrack)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrTrack> is deprecated: use delphi_srr_msgs-msg:SrrTrack instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'can_tx_detect_valid_level-val :lambda-list '(m))
(cl:defmethod can_tx_detect_valid_level-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_detect_valid_level-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_detect_valid_level instead.")
  (can_tx_detect_valid_level m))

(cl:ensure-generic-function 'can_tx_detect_status-val :lambda-list '(m))
(cl:defmethod can_tx_detect_status-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_detect_status-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_detect_status instead.")
  (can_tx_detect_status m))

(cl:ensure-generic-function 'can_tx_detect_range_rate-val :lambda-list '(m))
(cl:defmethod can_tx_detect_range_rate-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_detect_range_rate-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_detect_range_rate instead.")
  (can_tx_detect_range_rate m))

(cl:ensure-generic-function 'can_tx_detect_range-val :lambda-list '(m))
(cl:defmethod can_tx_detect_range-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_detect_range-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_detect_range instead.")
  (can_tx_detect_range m))

(cl:ensure-generic-function 'can_tx_detect_angle-val :lambda-list '(m))
(cl:defmethod can_tx_detect_angle-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_detect_angle-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_detect_angle instead.")
  (can_tx_detect_angle m))

(cl:ensure-generic-function 'can_tx_detect_amplitude-val :lambda-list '(m))
(cl:defmethod can_tx_detect_amplitude-val ((m <SrrTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_detect_amplitude-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_detect_amplitude instead.")
  (can_tx_detect_amplitude m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SrrTrack>)))
    "Constants for message type '<SrrTrack>"
  '((:CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION . 0)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_1 . 1)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_2 . 2)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_3 . 3)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_4 . 4)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_5 . 5)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_6 . 6)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_7 . 7)
    (:CAN_TX_DETECT_STATUS_NO_DATA . False)
    (:CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT . True))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SrrTrack)))
    "Constants for message type 'SrrTrack"
  '((:CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION . 0)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_1 . 1)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_2 . 2)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_3 . 3)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_4 . 4)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_5 . 5)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_6 . 6)
    (:CAN_TX_DETECT_VALID_LEVEL_LEVEL_7 . 7)
    (:CAN_TX_DETECT_STATUS_NO_DATA . False)
    (:CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT . True))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrTrack>) ostream)
  "Serializes a message object of type '<SrrTrack>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_detect_valid_level)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_tx_detect_status) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_detect_range_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_detect_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_detect_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_detect_amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrTrack>) istream)
  "Deserializes a message object of type '<SrrTrack>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_detect_valid_level)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_detect_status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_detect_range_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_detect_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_detect_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_detect_amplitude) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrTrack>)))
  "Returns string type for a message object of type '<SrrTrack>"
  "delphi_srr_msgs/SrrTrack")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrTrack)))
  "Returns string type for a message object of type 'SrrTrack"
  "delphi_srr_msgs/SrrTrack")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrTrack>)))
  "Returns md5sum for a message object of type '<SrrTrack>"
  "a689930ba3ce2066d655a7425f6fdbde")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrTrack)))
  "Returns md5sum for a message object of type 'SrrTrack"
  "a689930ba3ce2066d655a7425f6fdbde")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrTrack>)))
  "Returns full string definition for message of type '<SrrTrack>"
  (cl:format cl:nil "# Message file for srr_track~%~%std_msgs/Header header~%~%uint8     can_tx_detect_valid_level~%uint8     CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION=0~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_1=1~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_2=2~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_3=3~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_4=4~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_5=5~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_6=6~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_7=7~%~%bool      can_tx_detect_status~%bool      CAN_TX_DETECT_STATUS_NO_DATA=0~%bool      CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT=1~%~%float32   can_tx_detect_range_rate                 # m/s~%float32   can_tx_detect_range                      # m~%float32   can_tx_detect_angle                      # deg~%float32   can_tx_detect_amplitude                  # dbsm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrTrack)))
  "Returns full string definition for message of type 'SrrTrack"
  (cl:format cl:nil "# Message file for srr_track~%~%std_msgs/Header header~%~%uint8     can_tx_detect_valid_level~%uint8     CAN_TX_DETECT_VALID_LEVEL_SUSPECT_DETECTION=0~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_1=1~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_2=2~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_3=3~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_4=4~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_5=5~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_6=6~%uint8     CAN_TX_DETECT_VALID_LEVEL_LEVEL_7=7~%~%bool      can_tx_detect_status~%bool      CAN_TX_DETECT_STATUS_NO_DATA=0~%bool      CAN_TX_DETECT_STATUS_VALID_DATA_PRESENT=1~%~%float32   can_tx_detect_range_rate                 # m/s~%float32   can_tx_detect_range                      # m~%float32   can_tx_detect_angle                      # deg~%float32   can_tx_detect_amplitude                  # dbsm~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrTrack>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrTrack>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrTrack
    (cl:cons ':header (header msg))
    (cl:cons ':can_tx_detect_valid_level (can_tx_detect_valid_level msg))
    (cl:cons ':can_tx_detect_status (can_tx_detect_status msg))
    (cl:cons ':can_tx_detect_range_rate (can_tx_detect_range_rate msg))
    (cl:cons ':can_tx_detect_range (can_tx_detect_range msg))
    (cl:cons ':can_tx_detect_angle (can_tx_detect_angle msg))
    (cl:cons ':can_tx_detect_amplitude (can_tx_detect_amplitude msg))
))
