; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude AwsDisplay.msg.html

(cl:defclass <AwsDisplay> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (suppress_sound
    :reader suppress_sound
    :initarg :suppress_sound
    :type cl:boolean
    :initform cl:nil)
   (night_time
    :reader night_time
    :initarg :night_time
    :type cl:boolean
    :initform cl:nil)
   (dusk_time
    :reader dusk_time
    :initarg :dusk_time
    :type cl:boolean
    :initform cl:nil)
   (sound_type
    :reader sound_type
    :initarg :sound_type
    :type cl:fixnum
    :initform 0)
   (headway_valid
    :reader headway_valid
    :initarg :headway_valid
    :type cl:boolean
    :initform cl:nil)
   (headway_measurement
    :reader headway_measurement
    :initarg :headway_measurement
    :type cl:float
    :initform 0.0)
   (lanes_on
    :reader lanes_on
    :initarg :lanes_on
    :type cl:boolean
    :initform cl:nil)
   (left_ldw_on
    :reader left_ldw_on
    :initarg :left_ldw_on
    :type cl:boolean
    :initform cl:nil)
   (right_ldw_on
    :reader right_ldw_on
    :initarg :right_ldw_on
    :type cl:boolean
    :initform cl:nil)
   (fcw_on
    :reader fcw_on
    :initarg :fcw_on
    :type cl:boolean
    :initform cl:nil)
   (left_crossing
    :reader left_crossing
    :initarg :left_crossing
    :type cl:boolean
    :initform cl:nil)
   (right_crossing
    :reader right_crossing
    :initarg :right_crossing
    :type cl:boolean
    :initform cl:nil)
   (maintenance
    :reader maintenance
    :initarg :maintenance
    :type cl:boolean
    :initform cl:nil)
   (failsafe
    :reader failsafe
    :initarg :failsafe
    :type cl:boolean
    :initform cl:nil)
   (ped_fcw
    :reader ped_fcw
    :initarg :ped_fcw
    :type cl:boolean
    :initform cl:nil)
   (ped_in_dz
    :reader ped_in_dz
    :initarg :ped_in_dz
    :type cl:boolean
    :initform cl:nil)
   (headway_warning_level
    :reader headway_warning_level
    :initarg :headway_warning_level
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AwsDisplay (<AwsDisplay>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AwsDisplay>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AwsDisplay)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<AwsDisplay> is deprecated: use mobileye_560_660_msgs-msg:AwsDisplay instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'suppress_sound-val :lambda-list '(m))
(cl:defmethod suppress_sound-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:suppress_sound-val is deprecated.  Use mobileye_560_660_msgs-msg:suppress_sound instead.")
  (suppress_sound m))

(cl:ensure-generic-function 'night_time-val :lambda-list '(m))
(cl:defmethod night_time-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:night_time-val is deprecated.  Use mobileye_560_660_msgs-msg:night_time instead.")
  (night_time m))

(cl:ensure-generic-function 'dusk_time-val :lambda-list '(m))
(cl:defmethod dusk_time-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:dusk_time-val is deprecated.  Use mobileye_560_660_msgs-msg:dusk_time instead.")
  (dusk_time m))

(cl:ensure-generic-function 'sound_type-val :lambda-list '(m))
(cl:defmethod sound_type-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:sound_type-val is deprecated.  Use mobileye_560_660_msgs-msg:sound_type instead.")
  (sound_type m))

(cl:ensure-generic-function 'headway_valid-val :lambda-list '(m))
(cl:defmethod headway_valid-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:headway_valid-val is deprecated.  Use mobileye_560_660_msgs-msg:headway_valid instead.")
  (headway_valid m))

(cl:ensure-generic-function 'headway_measurement-val :lambda-list '(m))
(cl:defmethod headway_measurement-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:headway_measurement-val is deprecated.  Use mobileye_560_660_msgs-msg:headway_measurement instead.")
  (headway_measurement m))

(cl:ensure-generic-function 'lanes_on-val :lambda-list '(m))
(cl:defmethod lanes_on-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:lanes_on-val is deprecated.  Use mobileye_560_660_msgs-msg:lanes_on instead.")
  (lanes_on m))

(cl:ensure-generic-function 'left_ldw_on-val :lambda-list '(m))
(cl:defmethod left_ldw_on-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:left_ldw_on-val is deprecated.  Use mobileye_560_660_msgs-msg:left_ldw_on instead.")
  (left_ldw_on m))

(cl:ensure-generic-function 'right_ldw_on-val :lambda-list '(m))
(cl:defmethod right_ldw_on-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:right_ldw_on-val is deprecated.  Use mobileye_560_660_msgs-msg:right_ldw_on instead.")
  (right_ldw_on m))

(cl:ensure-generic-function 'fcw_on-val :lambda-list '(m))
(cl:defmethod fcw_on-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:fcw_on-val is deprecated.  Use mobileye_560_660_msgs-msg:fcw_on instead.")
  (fcw_on m))

(cl:ensure-generic-function 'left_crossing-val :lambda-list '(m))
(cl:defmethod left_crossing-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:left_crossing-val is deprecated.  Use mobileye_560_660_msgs-msg:left_crossing instead.")
  (left_crossing m))

(cl:ensure-generic-function 'right_crossing-val :lambda-list '(m))
(cl:defmethod right_crossing-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:right_crossing-val is deprecated.  Use mobileye_560_660_msgs-msg:right_crossing instead.")
  (right_crossing m))

(cl:ensure-generic-function 'maintenance-val :lambda-list '(m))
(cl:defmethod maintenance-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:maintenance-val is deprecated.  Use mobileye_560_660_msgs-msg:maintenance instead.")
  (maintenance m))

(cl:ensure-generic-function 'failsafe-val :lambda-list '(m))
(cl:defmethod failsafe-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:failsafe-val is deprecated.  Use mobileye_560_660_msgs-msg:failsafe instead.")
  (failsafe m))

(cl:ensure-generic-function 'ped_fcw-val :lambda-list '(m))
(cl:defmethod ped_fcw-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ped_fcw-val is deprecated.  Use mobileye_560_660_msgs-msg:ped_fcw instead.")
  (ped_fcw m))

(cl:ensure-generic-function 'ped_in_dz-val :lambda-list '(m))
(cl:defmethod ped_in_dz-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ped_in_dz-val is deprecated.  Use mobileye_560_660_msgs-msg:ped_in_dz instead.")
  (ped_in_dz m))

(cl:ensure-generic-function 'headway_warning_level-val :lambda-list '(m))
(cl:defmethod headway_warning_level-val ((m <AwsDisplay>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:headway_warning_level-val is deprecated.  Use mobileye_560_660_msgs-msg:headway_warning_level instead.")
  (headway_warning_level m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AwsDisplay>)))
    "Constants for message type '<AwsDisplay>"
  '((:SOUND_SILENT . 0)
    (:SOUND_LDWL . 1)
    (:SOUND_LDWR . 2)
    (:SOUND_FAR_HW . 3)
    (:SOUND_NEAR_HW . 4)
    (:SOUND_SOFT_FCW . 5)
    (:SOUND_HARD_FCW . 6)
    (:SOUND_RESERVED . 7)
    (:HEADWAY_LEVEL_OFF . 0)
    (:HEADWAY_LEVEL_GREEN . 1)
    (:HEADWAY_LEVEL_ORANGE . 2)
    (:HEADWAY_LEVEL_RED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AwsDisplay)))
    "Constants for message type 'AwsDisplay"
  '((:SOUND_SILENT . 0)
    (:SOUND_LDWL . 1)
    (:SOUND_LDWR . 2)
    (:SOUND_FAR_HW . 3)
    (:SOUND_NEAR_HW . 4)
    (:SOUND_SOFT_FCW . 5)
    (:SOUND_HARD_FCW . 6)
    (:SOUND_RESERVED . 7)
    (:HEADWAY_LEVEL_OFF . 0)
    (:HEADWAY_LEVEL_GREEN . 1)
    (:HEADWAY_LEVEL_ORANGE . 2)
    (:HEADWAY_LEVEL_RED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AwsDisplay>) ostream)
  "Serializes a message object of type '<AwsDisplay>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'suppress_sound) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'night_time) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dusk_time) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sound_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'headway_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'headway_measurement))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lanes_on) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_ldw_on) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_ldw_on) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fcw_on) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_crossing) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_crossing) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'maintenance) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'failsafe) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ped_fcw) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ped_in_dz) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'headway_warning_level)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AwsDisplay>) istream)
  "Deserializes a message object of type '<AwsDisplay>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'suppress_sound) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'night_time) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dusk_time) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sound_type)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'headway_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'headway_measurement) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'lanes_on) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left_ldw_on) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_ldw_on) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fcw_on) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left_crossing) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_crossing) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'maintenance) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'failsafe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ped_fcw) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ped_in_dz) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'headway_warning_level)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AwsDisplay>)))
  "Returns string type for a message object of type '<AwsDisplay>"
  "mobileye_560_660_msgs/AwsDisplay")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AwsDisplay)))
  "Returns string type for a message object of type 'AwsDisplay"
  "mobileye_560_660_msgs/AwsDisplay")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AwsDisplay>)))
  "Returns md5sum for a message object of type '<AwsDisplay>"
  "7aa82a0063aa4c0e719bef3d14c24bf7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AwsDisplay)))
  "Returns md5sum for a message object of type 'AwsDisplay"
  "7aa82a0063aa4c0e719bef3d14c24bf7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AwsDisplay>)))
  "Returns full string definition for message of type '<AwsDisplay>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool suppress_sound~%bool night_time~%bool dusk_time~%~%uint8 SOUND_SILENT = 0~%uint8 SOUND_LDWL = 1~%uint8 SOUND_LDWR = 2~%uint8 SOUND_FAR_HW = 3~%uint8 SOUND_NEAR_HW = 4~%uint8 SOUND_SOFT_FCW = 5~%uint8 SOUND_HARD_FCW = 6~%uint8 SOUND_RESERVED = 7~%uint8 sound_type~%~%bool headway_valid~%float32 headway_measurement~%bool lanes_on~%bool left_ldw_on~%bool right_ldw_on~%bool fcw_on~%bool left_crossing~%bool right_crossing~%bool maintenance~%bool failsafe~%bool ped_fcw~%bool ped_in_dz~%~%uint8 HEADWAY_LEVEL_OFF = 0~%uint8 HEADWAY_LEVEL_GREEN = 1~%uint8 HEADWAY_LEVEL_ORANGE = 2~%uint8 HEADWAY_LEVEL_RED = 3~%uint8 headway_warning_level~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AwsDisplay)))
  "Returns full string definition for message of type 'AwsDisplay"
  (cl:format cl:nil "std_msgs/Header header~%~%bool suppress_sound~%bool night_time~%bool dusk_time~%~%uint8 SOUND_SILENT = 0~%uint8 SOUND_LDWL = 1~%uint8 SOUND_LDWR = 2~%uint8 SOUND_FAR_HW = 3~%uint8 SOUND_NEAR_HW = 4~%uint8 SOUND_SOFT_FCW = 5~%uint8 SOUND_HARD_FCW = 6~%uint8 SOUND_RESERVED = 7~%uint8 sound_type~%~%bool headway_valid~%float32 headway_measurement~%bool lanes_on~%bool left_ldw_on~%bool right_ldw_on~%bool fcw_on~%bool left_crossing~%bool right_crossing~%bool maintenance~%bool failsafe~%bool ped_fcw~%bool ped_in_dz~%~%uint8 HEADWAY_LEVEL_OFF = 0~%uint8 HEADWAY_LEVEL_GREEN = 1~%uint8 HEADWAY_LEVEL_ORANGE = 2~%uint8 HEADWAY_LEVEL_RED = 3~%uint8 headway_warning_level~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AwsDisplay>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     4
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
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AwsDisplay>))
  "Converts a ROS message object to a list"
  (cl:list 'AwsDisplay
    (cl:cons ':header (header msg))
    (cl:cons ':suppress_sound (suppress_sound msg))
    (cl:cons ':night_time (night_time msg))
    (cl:cons ':dusk_time (dusk_time msg))
    (cl:cons ':sound_type (sound_type msg))
    (cl:cons ':headway_valid (headway_valid msg))
    (cl:cons ':headway_measurement (headway_measurement msg))
    (cl:cons ':lanes_on (lanes_on msg))
    (cl:cons ':left_ldw_on (left_ldw_on msg))
    (cl:cons ':right_ldw_on (right_ldw_on msg))
    (cl:cons ':fcw_on (fcw_on msg))
    (cl:cons ':left_crossing (left_crossing msg))
    (cl:cons ':right_crossing (right_crossing msg))
    (cl:cons ':maintenance (maintenance msg))
    (cl:cons ':failsafe (failsafe msg))
    (cl:cons ':ped_fcw (ped_fcw msg))
    (cl:cons ':ped_in_dz (ped_in_dz msg))
    (cl:cons ':headway_warning_level (headway_warning_level msg))
))
