; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude ObstacleData.msg.html

(cl:defclass <ObstacleData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacle_id
    :reader obstacle_id
    :initarg :obstacle_id
    :type cl:fixnum
    :initform 0)
   (obstacle_pos_x
    :reader obstacle_pos_x
    :initarg :obstacle_pos_x
    :type cl:float
    :initform 0.0)
   (obstacle_pos_y
    :reader obstacle_pos_y
    :initarg :obstacle_pos_y
    :type cl:float
    :initform 0.0)
   (blinker_info
    :reader blinker_info
    :initarg :blinker_info
    :type cl:fixnum
    :initform 0)
   (cut_in_and_out
    :reader cut_in_and_out
    :initarg :cut_in_and_out
    :type cl:fixnum
    :initform 0)
   (obstacle_rel_vel_x
    :reader obstacle_rel_vel_x
    :initarg :obstacle_rel_vel_x
    :type cl:float
    :initform 0.0)
   (obstacle_type
    :reader obstacle_type
    :initarg :obstacle_type
    :type cl:fixnum
    :initform 0)
   (obstacle_status
    :reader obstacle_status
    :initarg :obstacle_status
    :type cl:fixnum
    :initform 0)
   (obstacle_brake_lights
    :reader obstacle_brake_lights
    :initarg :obstacle_brake_lights
    :type cl:boolean
    :initform cl:nil)
   (obstacle_valid
    :reader obstacle_valid
    :initarg :obstacle_valid
    :type cl:fixnum
    :initform 0)
   (obstacle_length
    :reader obstacle_length
    :initarg :obstacle_length
    :type cl:float
    :initform 0.0)
   (obstacle_width
    :reader obstacle_width
    :initarg :obstacle_width
    :type cl:float
    :initform 0.0)
   (obstacle_age
    :reader obstacle_age
    :initarg :obstacle_age
    :type cl:fixnum
    :initform 0)
   (obstacle_lane
    :reader obstacle_lane
    :initarg :obstacle_lane
    :type cl:fixnum
    :initform 0)
   (cipv_flag
    :reader cipv_flag
    :initarg :cipv_flag
    :type cl:boolean
    :initform cl:nil)
   (radar_pos_x
    :reader radar_pos_x
    :initarg :radar_pos_x
    :type cl:float
    :initform 0.0)
   (radar_vel_x
    :reader radar_vel_x
    :initarg :radar_vel_x
    :type cl:float
    :initform 0.0)
   (radar_match_confidence
    :reader radar_match_confidence
    :initarg :radar_match_confidence
    :type cl:fixnum
    :initform 0)
   (matched_radar_id
    :reader matched_radar_id
    :initarg :matched_radar_id
    :type cl:fixnum
    :initform 0)
   (obstacle_angle_rate
    :reader obstacle_angle_rate
    :initarg :obstacle_angle_rate
    :type cl:float
    :initform 0.0)
   (obstacle_scale_change
    :reader obstacle_scale_change
    :initarg :obstacle_scale_change
    :type cl:float
    :initform 0.0)
   (object_accel_x
    :reader object_accel_x
    :initarg :object_accel_x
    :type cl:float
    :initform 0.0)
   (obstacle_replaced
    :reader obstacle_replaced
    :initarg :obstacle_replaced
    :type cl:boolean
    :initform cl:nil)
   (obstacle_angle
    :reader obstacle_angle
    :initarg :obstacle_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass ObstacleData (<ObstacleData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<ObstacleData> is deprecated: use mobileye_560_660_msgs-msg:ObstacleData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacle_id-val :lambda-list '(m))
(cl:defmethod obstacle_id-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_id-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_id instead.")
  (obstacle_id m))

(cl:ensure-generic-function 'obstacle_pos_x-val :lambda-list '(m))
(cl:defmethod obstacle_pos_x-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_pos_x-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_pos_x instead.")
  (obstacle_pos_x m))

(cl:ensure-generic-function 'obstacle_pos_y-val :lambda-list '(m))
(cl:defmethod obstacle_pos_y-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_pos_y-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_pos_y instead.")
  (obstacle_pos_y m))

(cl:ensure-generic-function 'blinker_info-val :lambda-list '(m))
(cl:defmethod blinker_info-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:blinker_info-val is deprecated.  Use mobileye_560_660_msgs-msg:blinker_info instead.")
  (blinker_info m))

(cl:ensure-generic-function 'cut_in_and_out-val :lambda-list '(m))
(cl:defmethod cut_in_and_out-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:cut_in_and_out-val is deprecated.  Use mobileye_560_660_msgs-msg:cut_in_and_out instead.")
  (cut_in_and_out m))

(cl:ensure-generic-function 'obstacle_rel_vel_x-val :lambda-list '(m))
(cl:defmethod obstacle_rel_vel_x-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_rel_vel_x-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_rel_vel_x instead.")
  (obstacle_rel_vel_x m))

(cl:ensure-generic-function 'obstacle_type-val :lambda-list '(m))
(cl:defmethod obstacle_type-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_type-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_type instead.")
  (obstacle_type m))

(cl:ensure-generic-function 'obstacle_status-val :lambda-list '(m))
(cl:defmethod obstacle_status-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_status-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_status instead.")
  (obstacle_status m))

(cl:ensure-generic-function 'obstacle_brake_lights-val :lambda-list '(m))
(cl:defmethod obstacle_brake_lights-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_brake_lights-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_brake_lights instead.")
  (obstacle_brake_lights m))

(cl:ensure-generic-function 'obstacle_valid-val :lambda-list '(m))
(cl:defmethod obstacle_valid-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_valid-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_valid instead.")
  (obstacle_valid m))

(cl:ensure-generic-function 'obstacle_length-val :lambda-list '(m))
(cl:defmethod obstacle_length-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_length-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_length instead.")
  (obstacle_length m))

(cl:ensure-generic-function 'obstacle_width-val :lambda-list '(m))
(cl:defmethod obstacle_width-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_width-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_width instead.")
  (obstacle_width m))

(cl:ensure-generic-function 'obstacle_age-val :lambda-list '(m))
(cl:defmethod obstacle_age-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_age-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_age instead.")
  (obstacle_age m))

(cl:ensure-generic-function 'obstacle_lane-val :lambda-list '(m))
(cl:defmethod obstacle_lane-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_lane-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_lane instead.")
  (obstacle_lane m))

(cl:ensure-generic-function 'cipv_flag-val :lambda-list '(m))
(cl:defmethod cipv_flag-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:cipv_flag-val is deprecated.  Use mobileye_560_660_msgs-msg:cipv_flag instead.")
  (cipv_flag m))

(cl:ensure-generic-function 'radar_pos_x-val :lambda-list '(m))
(cl:defmethod radar_pos_x-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:radar_pos_x-val is deprecated.  Use mobileye_560_660_msgs-msg:radar_pos_x instead.")
  (radar_pos_x m))

(cl:ensure-generic-function 'radar_vel_x-val :lambda-list '(m))
(cl:defmethod radar_vel_x-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:radar_vel_x-val is deprecated.  Use mobileye_560_660_msgs-msg:radar_vel_x instead.")
  (radar_vel_x m))

(cl:ensure-generic-function 'radar_match_confidence-val :lambda-list '(m))
(cl:defmethod radar_match_confidence-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:radar_match_confidence-val is deprecated.  Use mobileye_560_660_msgs-msg:radar_match_confidence instead.")
  (radar_match_confidence m))

(cl:ensure-generic-function 'matched_radar_id-val :lambda-list '(m))
(cl:defmethod matched_radar_id-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:matched_radar_id-val is deprecated.  Use mobileye_560_660_msgs-msg:matched_radar_id instead.")
  (matched_radar_id m))

(cl:ensure-generic-function 'obstacle_angle_rate-val :lambda-list '(m))
(cl:defmethod obstacle_angle_rate-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_angle_rate-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_angle_rate instead.")
  (obstacle_angle_rate m))

(cl:ensure-generic-function 'obstacle_scale_change-val :lambda-list '(m))
(cl:defmethod obstacle_scale_change-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_scale_change-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_scale_change instead.")
  (obstacle_scale_change m))

(cl:ensure-generic-function 'object_accel_x-val :lambda-list '(m))
(cl:defmethod object_accel_x-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:object_accel_x-val is deprecated.  Use mobileye_560_660_msgs-msg:object_accel_x instead.")
  (object_accel_x m))

(cl:ensure-generic-function 'obstacle_replaced-val :lambda-list '(m))
(cl:defmethod obstacle_replaced-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_replaced-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_replaced instead.")
  (obstacle_replaced m))

(cl:ensure-generic-function 'obstacle_angle-val :lambda-list '(m))
(cl:defmethod obstacle_angle-val ((m <ObstacleData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:obstacle_angle-val is deprecated.  Use mobileye_560_660_msgs-msg:obstacle_angle instead.")
  (obstacle_angle m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ObstacleData>)))
    "Constants for message type '<ObstacleData>"
  '((:BLINKER_INFO_UNAVAILABLE . 0)
    (:BLINKER_INFO_OFF . 1)
    (:BLINKER_INFO_LEFT . 2)
    (:BLINKER_INFO_RIGHT . 3)
    (:BLINKER_INFO_BOTH . 4)
    (:CUT_IN_AND_OUT_UNDEFINED . 0)
    (:CUT_IN_AND_OUT_IN_HOST_LANE . 1)
    (:CUT_IN_AND_OUT_OUT_HOST_LANE . 2)
    (:CUT_IN_AND_OUT_CUT_IN . 3)
    (:CUT_IN_AND_OUT_CUT_OUT . 4)
    (:OBSTACLE_TYPE_VEHICLE . 0)
    (:OBSTACLE_TYPE_TRUCK . 1)
    (:OBSTACLE_TYPE_BIKE . 2)
    (:OBSTACLE_TYPE_PED . 3)
    (:OBSTACLE_TYPE_BICYCLE . 4)
    (:OBSTACLE_STATUS_UNDEFINED . 0)
    (:OBSTACLE_STATUS_STANDING . 1)
    (:OBSTACLE_STATUS_STOPPED . 2)
    (:OBSTACLE_STATUS_MOVING . 3)
    (:OBSTACLE_STATUS_ONCOMING . 4)
    (:OBSTACLE_STATUS_PARKED . 5)
    (:OBSTACLE_VALID_INVALID . 0)
    (:OBSTACLE_VALID_NEW . 1)
    (:OBSTACLE_VALID_OLDER . 2)
    (:OBSTACLE_LANE_NOT_ASSIGNED . 0)
    (:OBSTACLE_LANE_EGO_LANE . 1)
    (:OBSTACLE_LANE_NEXT_LANE . 2)
    (:OBSTACLE_LANE_INVALID . 3)
    (:RADAR_MATCH_CONFIDENCE_NO_MATCH . 0)
    (:RADAR_MATCH_CONFIDENCE_MULTI_MATCH . 1)
    (:RADAR_MATCH_CONFIDENCE_BOUNDED_LOW . 2)
    (:RADAR_MATCH_CONFIDENCE_BOUNDED_MED . 3)
    (:RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH . 4)
    (:RADAR_MATCH_CONFIDENCE_HIGH . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ObstacleData)))
    "Constants for message type 'ObstacleData"
  '((:BLINKER_INFO_UNAVAILABLE . 0)
    (:BLINKER_INFO_OFF . 1)
    (:BLINKER_INFO_LEFT . 2)
    (:BLINKER_INFO_RIGHT . 3)
    (:BLINKER_INFO_BOTH . 4)
    (:CUT_IN_AND_OUT_UNDEFINED . 0)
    (:CUT_IN_AND_OUT_IN_HOST_LANE . 1)
    (:CUT_IN_AND_OUT_OUT_HOST_LANE . 2)
    (:CUT_IN_AND_OUT_CUT_IN . 3)
    (:CUT_IN_AND_OUT_CUT_OUT . 4)
    (:OBSTACLE_TYPE_VEHICLE . 0)
    (:OBSTACLE_TYPE_TRUCK . 1)
    (:OBSTACLE_TYPE_BIKE . 2)
    (:OBSTACLE_TYPE_PED . 3)
    (:OBSTACLE_TYPE_BICYCLE . 4)
    (:OBSTACLE_STATUS_UNDEFINED . 0)
    (:OBSTACLE_STATUS_STANDING . 1)
    (:OBSTACLE_STATUS_STOPPED . 2)
    (:OBSTACLE_STATUS_MOVING . 3)
    (:OBSTACLE_STATUS_ONCOMING . 4)
    (:OBSTACLE_STATUS_PARKED . 5)
    (:OBSTACLE_VALID_INVALID . 0)
    (:OBSTACLE_VALID_NEW . 1)
    (:OBSTACLE_VALID_OLDER . 2)
    (:OBSTACLE_LANE_NOT_ASSIGNED . 0)
    (:OBSTACLE_LANE_EGO_LANE . 1)
    (:OBSTACLE_LANE_NEXT_LANE . 2)
    (:OBSTACLE_LANE_INVALID . 3)
    (:RADAR_MATCH_CONFIDENCE_NO_MATCH . 0)
    (:RADAR_MATCH_CONFIDENCE_MULTI_MATCH . 1)
    (:RADAR_MATCH_CONFIDENCE_BOUNDED_LOW . 2)
    (:RADAR_MATCH_CONFIDENCE_BOUNDED_MED . 3)
    (:RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH . 4)
    (:RADAR_MATCH_CONFIDENCE_HIGH . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleData>) ostream)
  "Serializes a message object of type '<ObstacleData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'obstacle_pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'obstacle_pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'blinker_info)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cut_in_and_out)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'obstacle_rel_vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'obstacle_brake_lights) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_valid)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obstacle_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obstacle_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_age)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_lane)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cipv_flag) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radar_pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radar_vel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_match_confidence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'matched_radar_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'matched_radar_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obstacle_angle_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'obstacle_scale_change))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'object_accel_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'obstacle_replaced) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obstacle_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleData>) istream)
  "Deserializes a message object of type '<ObstacleData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_pos_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_pos_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'blinker_info)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cut_in_and_out)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_rel_vel_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_status)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_brake_lights) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_valid)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_length) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'obstacle_age)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'obstacle_lane)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cipv_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radar_pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radar_vel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'radar_match_confidence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'matched_radar_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'matched_radar_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_angle_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_scale_change) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'object_accel_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'obstacle_replaced) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obstacle_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleData>)))
  "Returns string type for a message object of type '<ObstacleData>"
  "mobileye_560_660_msgs/ObstacleData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleData)))
  "Returns string type for a message object of type 'ObstacleData"
  "mobileye_560_660_msgs/ObstacleData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleData>)))
  "Returns md5sum for a message object of type '<ObstacleData>"
  "ff75c75f79e1f472d5b0086caa5c286f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleData)))
  "Returns md5sum for a message object of type 'ObstacleData"
  "ff75c75f79e1f472d5b0086caa5c286f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleData>)))
  "Returns full string definition for message of type '<ObstacleData>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 obstacle_id~%float64 obstacle_pos_x~%float64 obstacle_pos_y~%~%uint8 BLINKER_INFO_UNAVAILABLE = 0~%uint8 BLINKER_INFO_OFF = 1~%uint8 BLINKER_INFO_LEFT = 2~%uint8 BLINKER_INFO_RIGHT = 3~%uint8 BLINKER_INFO_BOTH = 4~%uint8 blinker_info~%~%uint8 CUT_IN_AND_OUT_UNDEFINED = 0~%uint8 CUT_IN_AND_OUT_IN_HOST_LANE = 1~%uint8 CUT_IN_AND_OUT_OUT_HOST_LANE = 2~%uint8 CUT_IN_AND_OUT_CUT_IN = 3~%uint8 CUT_IN_AND_OUT_CUT_OUT = 4~%uint8 cut_in_and_out~%~%float64 obstacle_rel_vel_x~%~%uint8 OBSTACLE_TYPE_VEHICLE = 0~%uint8 OBSTACLE_TYPE_TRUCK = 1~%uint8 OBSTACLE_TYPE_BIKE = 2~%uint8 OBSTACLE_TYPE_PED = 3~%uint8 OBSTACLE_TYPE_BICYCLE = 4~%uint8 obstacle_type~%~%uint8 OBSTACLE_STATUS_UNDEFINED = 0~%uint8 OBSTACLE_STATUS_STANDING = 1~%uint8 OBSTACLE_STATUS_STOPPED = 2~%uint8 OBSTACLE_STATUS_MOVING = 3~%uint8 OBSTACLE_STATUS_ONCOMING = 4~%uint8 OBSTACLE_STATUS_PARKED = 5~%uint8 obstacle_status~%~%bool obstacle_brake_lights~%~%uint8 OBSTACLE_VALID_INVALID = 0~%uint8 OBSTACLE_VALID_NEW = 1~%uint8 OBSTACLE_VALID_OLDER = 2~%uint8 obstacle_valid~%~%float32 obstacle_length~%float32 obstacle_width~%uint16 obstacle_age~%~%uint8 OBSTACLE_LANE_NOT_ASSIGNED = 0~%uint8 OBSTACLE_LANE_EGO_LANE = 1~%uint8 OBSTACLE_LANE_NEXT_LANE = 2~%uint8 OBSTACLE_LANE_INVALID = 3~%uint8 obstacle_lane~%~%bool cipv_flag~%float32 radar_pos_x~%float32 radar_vel_x~%~%uint8 RADAR_MATCH_CONFIDENCE_NO_MATCH = 0~%uint8 RADAR_MATCH_CONFIDENCE_MULTI_MATCH = 1~%uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_LOW = 2~%uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_MED = 3~%uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH = 4~%uint8 RADAR_MATCH_CONFIDENCE_HIGH = 5~%uint8 radar_match_confidence~%~%uint16 matched_radar_id~%float32 obstacle_angle_rate~%float64 obstacle_scale_change~%float32 object_accel_x~%bool obstacle_replaced~%float32 obstacle_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleData)))
  "Returns full string definition for message of type 'ObstacleData"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 obstacle_id~%float64 obstacle_pos_x~%float64 obstacle_pos_y~%~%uint8 BLINKER_INFO_UNAVAILABLE = 0~%uint8 BLINKER_INFO_OFF = 1~%uint8 BLINKER_INFO_LEFT = 2~%uint8 BLINKER_INFO_RIGHT = 3~%uint8 BLINKER_INFO_BOTH = 4~%uint8 blinker_info~%~%uint8 CUT_IN_AND_OUT_UNDEFINED = 0~%uint8 CUT_IN_AND_OUT_IN_HOST_LANE = 1~%uint8 CUT_IN_AND_OUT_OUT_HOST_LANE = 2~%uint8 CUT_IN_AND_OUT_CUT_IN = 3~%uint8 CUT_IN_AND_OUT_CUT_OUT = 4~%uint8 cut_in_and_out~%~%float64 obstacle_rel_vel_x~%~%uint8 OBSTACLE_TYPE_VEHICLE = 0~%uint8 OBSTACLE_TYPE_TRUCK = 1~%uint8 OBSTACLE_TYPE_BIKE = 2~%uint8 OBSTACLE_TYPE_PED = 3~%uint8 OBSTACLE_TYPE_BICYCLE = 4~%uint8 obstacle_type~%~%uint8 OBSTACLE_STATUS_UNDEFINED = 0~%uint8 OBSTACLE_STATUS_STANDING = 1~%uint8 OBSTACLE_STATUS_STOPPED = 2~%uint8 OBSTACLE_STATUS_MOVING = 3~%uint8 OBSTACLE_STATUS_ONCOMING = 4~%uint8 OBSTACLE_STATUS_PARKED = 5~%uint8 obstacle_status~%~%bool obstacle_brake_lights~%~%uint8 OBSTACLE_VALID_INVALID = 0~%uint8 OBSTACLE_VALID_NEW = 1~%uint8 OBSTACLE_VALID_OLDER = 2~%uint8 obstacle_valid~%~%float32 obstacle_length~%float32 obstacle_width~%uint16 obstacle_age~%~%uint8 OBSTACLE_LANE_NOT_ASSIGNED = 0~%uint8 OBSTACLE_LANE_EGO_LANE = 1~%uint8 OBSTACLE_LANE_NEXT_LANE = 2~%uint8 OBSTACLE_LANE_INVALID = 3~%uint8 obstacle_lane~%~%bool cipv_flag~%float32 radar_pos_x~%float32 radar_vel_x~%~%uint8 RADAR_MATCH_CONFIDENCE_NO_MATCH = 0~%uint8 RADAR_MATCH_CONFIDENCE_MULTI_MATCH = 1~%uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_LOW = 2~%uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_MED = 3~%uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH = 4~%uint8 RADAR_MATCH_CONFIDENCE_HIGH = 5~%uint8 radar_match_confidence~%~%uint16 matched_radar_id~%float32 obstacle_angle_rate~%float64 obstacle_scale_change~%float32 object_accel_x~%bool obstacle_replaced~%float32 obstacle_angle~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     8
     8
     1
     1
     8
     1
     1
     1
     1
     4
     4
     2
     1
     1
     4
     4
     1
     2
     4
     8
     4
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleData>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleData
    (cl:cons ':header (header msg))
    (cl:cons ':obstacle_id (obstacle_id msg))
    (cl:cons ':obstacle_pos_x (obstacle_pos_x msg))
    (cl:cons ':obstacle_pos_y (obstacle_pos_y msg))
    (cl:cons ':blinker_info (blinker_info msg))
    (cl:cons ':cut_in_and_out (cut_in_and_out msg))
    (cl:cons ':obstacle_rel_vel_x (obstacle_rel_vel_x msg))
    (cl:cons ':obstacle_type (obstacle_type msg))
    (cl:cons ':obstacle_status (obstacle_status msg))
    (cl:cons ':obstacle_brake_lights (obstacle_brake_lights msg))
    (cl:cons ':obstacle_valid (obstacle_valid msg))
    (cl:cons ':obstacle_length (obstacle_length msg))
    (cl:cons ':obstacle_width (obstacle_width msg))
    (cl:cons ':obstacle_age (obstacle_age msg))
    (cl:cons ':obstacle_lane (obstacle_lane msg))
    (cl:cons ':cipv_flag (cipv_flag msg))
    (cl:cons ':radar_pos_x (radar_pos_x msg))
    (cl:cons ':radar_vel_x (radar_vel_x msg))
    (cl:cons ':radar_match_confidence (radar_match_confidence msg))
    (cl:cons ':matched_radar_id (matched_radar_id msg))
    (cl:cons ':obstacle_angle_rate (obstacle_angle_rate msg))
    (cl:cons ':obstacle_scale_change (obstacle_scale_change msg))
    (cl:cons ':object_accel_x (object_accel_x msg))
    (cl:cons ':obstacle_replaced (obstacle_replaced msg))
    (cl:cons ':obstacle_angle (obstacle_angle msg))
))
