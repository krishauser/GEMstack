; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude Inspvax.msg.html

(cl:defclass <Inspvax> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (novatel_msg_header
    :reader novatel_msg_header
    :initarg :novatel_msg_header
    :type novatel_gps_msgs-msg:NovatelMessageHeader
    :initform (cl:make-instance 'novatel_gps_msgs-msg:NovatelMessageHeader))
   (ins_status
    :reader ins_status
    :initarg :ins_status
    :type cl:string
    :initform "")
   (position_type
    :reader position_type
    :initarg :position_type
    :type cl:string
    :initform "")
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (undulation
    :reader undulation
    :initarg :undulation
    :type cl:float
    :initform 0.0)
   (north_velocity
    :reader north_velocity
    :initarg :north_velocity
    :type cl:float
    :initform 0.0)
   (east_velocity
    :reader east_velocity
    :initarg :east_velocity
    :type cl:float
    :initform 0.0)
   (up_velocity
    :reader up_velocity
    :initarg :up_velocity
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (azimuth
    :reader azimuth
    :initarg :azimuth
    :type cl:float
    :initform 0.0)
   (latitude_std
    :reader latitude_std
    :initarg :latitude_std
    :type cl:float
    :initform 0.0)
   (longitude_std
    :reader longitude_std
    :initarg :longitude_std
    :type cl:float
    :initform 0.0)
   (altitude_std
    :reader altitude_std
    :initarg :altitude_std
    :type cl:float
    :initform 0.0)
   (north_velocity_std
    :reader north_velocity_std
    :initarg :north_velocity_std
    :type cl:float
    :initform 0.0)
   (east_velocity_std
    :reader east_velocity_std
    :initarg :east_velocity_std
    :type cl:float
    :initform 0.0)
   (up_velocity_std
    :reader up_velocity_std
    :initarg :up_velocity_std
    :type cl:float
    :initform 0.0)
   (roll_std
    :reader roll_std
    :initarg :roll_std
    :type cl:float
    :initform 0.0)
   (pitch_std
    :reader pitch_std
    :initarg :pitch_std
    :type cl:float
    :initform 0.0)
   (azimuth_std
    :reader azimuth_std
    :initarg :azimuth_std
    :type cl:float
    :initform 0.0)
   (extended_status
    :reader extended_status
    :initarg :extended_status
    :type novatel_gps_msgs-msg:NovatelExtendedSolutionStatus
    :initform (cl:make-instance 'novatel_gps_msgs-msg:NovatelExtendedSolutionStatus))
   (seconds_since_update
    :reader seconds_since_update
    :initarg :seconds_since_update
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Inspvax (<Inspvax>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Inspvax>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Inspvax)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<Inspvax> is deprecated: use novatel_gps_msgs-msg:Inspvax instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:header-val is deprecated.  Use novatel_gps_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'novatel_msg_header-val :lambda-list '(m))
(cl:defmethod novatel_msg_header-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:novatel_msg_header-val is deprecated.  Use novatel_gps_msgs-msg:novatel_msg_header instead.")
  (novatel_msg_header m))

(cl:ensure-generic-function 'ins_status-val :lambda-list '(m))
(cl:defmethod ins_status-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:ins_status-val is deprecated.  Use novatel_gps_msgs-msg:ins_status instead.")
  (ins_status m))

(cl:ensure-generic-function 'position_type-val :lambda-list '(m))
(cl:defmethod position_type-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:position_type-val is deprecated.  Use novatel_gps_msgs-msg:position_type instead.")
  (position_type m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:latitude-val is deprecated.  Use novatel_gps_msgs-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:longitude-val is deprecated.  Use novatel_gps_msgs-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:altitude-val is deprecated.  Use novatel_gps_msgs-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'undulation-val :lambda-list '(m))
(cl:defmethod undulation-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:undulation-val is deprecated.  Use novatel_gps_msgs-msg:undulation instead.")
  (undulation m))

(cl:ensure-generic-function 'north_velocity-val :lambda-list '(m))
(cl:defmethod north_velocity-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:north_velocity-val is deprecated.  Use novatel_gps_msgs-msg:north_velocity instead.")
  (north_velocity m))

(cl:ensure-generic-function 'east_velocity-val :lambda-list '(m))
(cl:defmethod east_velocity-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:east_velocity-val is deprecated.  Use novatel_gps_msgs-msg:east_velocity instead.")
  (east_velocity m))

(cl:ensure-generic-function 'up_velocity-val :lambda-list '(m))
(cl:defmethod up_velocity-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:up_velocity-val is deprecated.  Use novatel_gps_msgs-msg:up_velocity instead.")
  (up_velocity m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:roll-val is deprecated.  Use novatel_gps_msgs-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:pitch-val is deprecated.  Use novatel_gps_msgs-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:azimuth-val is deprecated.  Use novatel_gps_msgs-msg:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'latitude_std-val :lambda-list '(m))
(cl:defmethod latitude_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:latitude_std-val is deprecated.  Use novatel_gps_msgs-msg:latitude_std instead.")
  (latitude_std m))

(cl:ensure-generic-function 'longitude_std-val :lambda-list '(m))
(cl:defmethod longitude_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:longitude_std-val is deprecated.  Use novatel_gps_msgs-msg:longitude_std instead.")
  (longitude_std m))

(cl:ensure-generic-function 'altitude_std-val :lambda-list '(m))
(cl:defmethod altitude_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:altitude_std-val is deprecated.  Use novatel_gps_msgs-msg:altitude_std instead.")
  (altitude_std m))

(cl:ensure-generic-function 'north_velocity_std-val :lambda-list '(m))
(cl:defmethod north_velocity_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:north_velocity_std-val is deprecated.  Use novatel_gps_msgs-msg:north_velocity_std instead.")
  (north_velocity_std m))

(cl:ensure-generic-function 'east_velocity_std-val :lambda-list '(m))
(cl:defmethod east_velocity_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:east_velocity_std-val is deprecated.  Use novatel_gps_msgs-msg:east_velocity_std instead.")
  (east_velocity_std m))

(cl:ensure-generic-function 'up_velocity_std-val :lambda-list '(m))
(cl:defmethod up_velocity_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:up_velocity_std-val is deprecated.  Use novatel_gps_msgs-msg:up_velocity_std instead.")
  (up_velocity_std m))

(cl:ensure-generic-function 'roll_std-val :lambda-list '(m))
(cl:defmethod roll_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:roll_std-val is deprecated.  Use novatel_gps_msgs-msg:roll_std instead.")
  (roll_std m))

(cl:ensure-generic-function 'pitch_std-val :lambda-list '(m))
(cl:defmethod pitch_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:pitch_std-val is deprecated.  Use novatel_gps_msgs-msg:pitch_std instead.")
  (pitch_std m))

(cl:ensure-generic-function 'azimuth_std-val :lambda-list '(m))
(cl:defmethod azimuth_std-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:azimuth_std-val is deprecated.  Use novatel_gps_msgs-msg:azimuth_std instead.")
  (azimuth_std m))

(cl:ensure-generic-function 'extended_status-val :lambda-list '(m))
(cl:defmethod extended_status-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:extended_status-val is deprecated.  Use novatel_gps_msgs-msg:extended_status instead.")
  (extended_status m))

(cl:ensure-generic-function 'seconds_since_update-val :lambda-list '(m))
(cl:defmethod seconds_since_update-val ((m <Inspvax>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:seconds_since_update-val is deprecated.  Use novatel_gps_msgs-msg:seconds_since_update instead.")
  (seconds_since_update m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Inspvax>) ostream)
  "Serializes a message object of type '<Inspvax>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'novatel_msg_header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ins_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ins_status))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'position_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'position_type))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'undulation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'north_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'east_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'up_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'north_velocity_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'east_velocity_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'up_velocity_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'azimuth_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'extended_status) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seconds_since_update)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seconds_since_update)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Inspvax>) istream)
  "Deserializes a message object of type '<Inspvax>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'novatel_msg_header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ins_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ins_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'position_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'undulation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'north_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'east_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'up_velocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azimuth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'north_velocity_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'east_velocity_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'up_velocity_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azimuth_std) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'extended_status) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seconds_since_update)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seconds_since_update)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Inspvax>)))
  "Returns string type for a message object of type '<Inspvax>"
  "novatel_gps_msgs/Inspvax")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Inspvax)))
  "Returns string type for a message object of type 'Inspvax"
  "novatel_gps_msgs/Inspvax")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Inspvax>)))
  "Returns md5sum for a message object of type '<Inspvax>"
  "cebf3b182479d01907e3894361b97eba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Inspvax)))
  "Returns md5sum for a message object of type 'Inspvax"
  "cebf3b182479d01907e3894361b97eba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Inspvax>)))
  "Returns full string definition for message of type '<Inspvax>"
  (cl:format cl:nil "# message 1465~%~%std_msgs/Header header~%~%NovatelMessageHeader novatel_msg_header~%~%# Table 29 in the SPAN on OEM6 manual:~%# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121~%string ins_status~%~%~%# Table 30 in the SPAN on OEM6 manual:~%# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=124~%string position_type~%~%~%float64 latitude~%float64 longitude~%float64 altitude~%~%float32 undulation~%~%float64 north_velocity~%float64 east_velocity~%float64 up_velocity~%~%float64 roll~%float64 pitch~%float64 azimuth~%~%float32 latitude_std~%float32 longitude_std~%float32 altitude_std~%~%float32 north_velocity_std~%float32 east_velocity_std~%float32 up_velocity_std~%~%float32 roll_std~%float32 pitch_std~%float32 azimuth_std~%~%NovatelExtendedSolutionStatus extended_status~%~%uint16 seconds_since_update~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/NovatelMessageHeader~%# Novatel Message Header~%~%string message_name~%string port~%uint32 sequence_num~%float32 percent_idle_time~%string gps_time_status~%uint32 gps_week_num~%float64 gps_seconds~%~%# Bit       Mask      Description~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%NovatelReceiverStatus receiver_status~%~%# Receiver build number (0-65535)~%uint32 receiver_software_version~%================================================================================~%MSG: novatel_gps_msgs/NovatelReceiverStatus~%# From the original Novatel receiver status message bitfield~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%uint32 original_status_code~%bool error_flag~%bool temperature_flag~%bool voltage_supply_flag~%bool antenna_powered~%bool antenna_is_open~%bool antenna_is_shorted~%bool cpu_overload_flag~%bool com1_buffer_overrun~%bool com2_buffer_overrun~%bool com3_buffer_overrun~%bool usb_buffer_overrun~%bool rf1_agc_flag~%bool rf2_agc_flag~%bool almanac_flag~%bool position_solution_flag~%bool position_fixed_flag~%bool clock_steering_status_enabled~%bool clock_model_flag~%bool oemv_external_oscillator_flag~%bool software_resource_flag~%bool aux1_status_event_flag~%bool aux2_status_event_flag~%bool aux3_status_event_flag~%~%~%================================================================================~%MSG: novatel_gps_msgs/NovatelExtendedSolutionStatus~%# Bit    Mask      Description~%#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)~%# 1-3    0x0E      Pseudorange Ionosphere Correction~%#                    0 = unknown~%#                    1 = Klobuchar Broadcast~%#                    2 = SBAS Broadcast~%#                    3 = Multi-frequency Computed~%#                    4 = PSRDiff Correction~%#                    5 = NovaTel Blended Ionosphere Value~%# 4-7  0xF0        <Reserved>~%uint32 original_mask~%bool advance_rtk_verified~%string psuedorange_iono_correction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Inspvax)))
  "Returns full string definition for message of type 'Inspvax"
  (cl:format cl:nil "# message 1465~%~%std_msgs/Header header~%~%NovatelMessageHeader novatel_msg_header~%~%# Table 29 in the SPAN on OEM6 manual:~%# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=121~%string ins_status~%~%~%# Table 30 in the SPAN on OEM6 manual:~%# See: http://www.novatel.com/assets/Documents/Manuals/OM-20000144UM.pdf#page=124~%string position_type~%~%~%float64 latitude~%float64 longitude~%float64 altitude~%~%float32 undulation~%~%float64 north_velocity~%float64 east_velocity~%float64 up_velocity~%~%float64 roll~%float64 pitch~%float64 azimuth~%~%float32 latitude_std~%float32 longitude_std~%float32 altitude_std~%~%float32 north_velocity_std~%float32 east_velocity_std~%float32 up_velocity_std~%~%float32 roll_std~%float32 pitch_std~%float32 azimuth_std~%~%NovatelExtendedSolutionStatus extended_status~%~%uint16 seconds_since_update~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/NovatelMessageHeader~%# Novatel Message Header~%~%string message_name~%string port~%uint32 sequence_num~%float32 percent_idle_time~%string gps_time_status~%uint32 gps_week_num~%float64 gps_seconds~%~%# Bit       Mask      Description~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%NovatelReceiverStatus receiver_status~%~%# Receiver build number (0-65535)~%uint32 receiver_software_version~%================================================================================~%MSG: novatel_gps_msgs/NovatelReceiverStatus~%# From the original Novatel receiver status message bitfield~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%uint32 original_status_code~%bool error_flag~%bool temperature_flag~%bool voltage_supply_flag~%bool antenna_powered~%bool antenna_is_open~%bool antenna_is_shorted~%bool cpu_overload_flag~%bool com1_buffer_overrun~%bool com2_buffer_overrun~%bool com3_buffer_overrun~%bool usb_buffer_overrun~%bool rf1_agc_flag~%bool rf2_agc_flag~%bool almanac_flag~%bool position_solution_flag~%bool position_fixed_flag~%bool clock_steering_status_enabled~%bool clock_model_flag~%bool oemv_external_oscillator_flag~%bool software_resource_flag~%bool aux1_status_event_flag~%bool aux2_status_event_flag~%bool aux3_status_event_flag~%~%~%================================================================================~%MSG: novatel_gps_msgs/NovatelExtendedSolutionStatus~%# Bit    Mask      Description~%#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)~%# 1-3    0x0E      Pseudorange Ionosphere Correction~%#                    0 = unknown~%#                    1 = Klobuchar Broadcast~%#                    2 = SBAS Broadcast~%#                    3 = Multi-frequency Computed~%#                    4 = PSRDiff Correction~%#                    5 = NovaTel Blended Ionosphere Value~%# 4-7  0xF0        <Reserved>~%uint32 original_mask~%bool advance_rtk_verified~%string psuedorange_iono_correction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Inspvax>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'novatel_msg_header))
     4 (cl:length (cl:slot-value msg 'ins_status))
     4 (cl:length (cl:slot-value msg 'position_type))
     8
     8
     8
     4
     8
     8
     8
     8
     8
     8
     4
     4
     4
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'extended_status))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Inspvax>))
  "Converts a ROS message object to a list"
  (cl:list 'Inspvax
    (cl:cons ':header (header msg))
    (cl:cons ':novatel_msg_header (novatel_msg_header msg))
    (cl:cons ':ins_status (ins_status msg))
    (cl:cons ':position_type (position_type msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':undulation (undulation msg))
    (cl:cons ':north_velocity (north_velocity msg))
    (cl:cons ':east_velocity (east_velocity msg))
    (cl:cons ':up_velocity (up_velocity msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':latitude_std (latitude_std msg))
    (cl:cons ':longitude_std (longitude_std msg))
    (cl:cons ':altitude_std (altitude_std msg))
    (cl:cons ':north_velocity_std (north_velocity_std msg))
    (cl:cons ':east_velocity_std (east_velocity_std msg))
    (cl:cons ':up_velocity_std (up_velocity_std msg))
    (cl:cons ':roll_std (roll_std msg))
    (cl:cons ':pitch_std (pitch_std msg))
    (cl:cons ':azimuth_std (azimuth_std msg))
    (cl:cons ':extended_status (extended_status msg))
    (cl:cons ':seconds_since_update (seconds_since_update msg))
))
