; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude LkaLane.msg.html

(cl:defclass <LkaLane> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lane_type
    :reader lane_type
    :initarg :lane_type
    :type cl:fixnum
    :initform 0)
   (quality
    :reader quality
    :initarg :quality
    :type cl:fixnum
    :initform 0)
   (model_degree
    :reader model_degree
    :initarg :model_degree
    :type cl:fixnum
    :initform 0)
   (position_parameter_c0
    :reader position_parameter_c0
    :initarg :position_parameter_c0
    :type cl:float
    :initform 0.0)
   (curvature_parameter_c2
    :reader curvature_parameter_c2
    :initarg :curvature_parameter_c2
    :type cl:float
    :initform 0.0)
   (curvature_derivative_parameter_c3
    :reader curvature_derivative_parameter_c3
    :initarg :curvature_derivative_parameter_c3
    :type cl:float
    :initform 0.0)
   (marking_width
    :reader marking_width
    :initarg :marking_width
    :type cl:float
    :initform 0.0)
   (heading_angle_parameter_c1
    :reader heading_angle_parameter_c1
    :initarg :heading_angle_parameter_c1
    :type cl:float
    :initform 0.0)
   (view_range
    :reader view_range
    :initarg :view_range
    :type cl:float
    :initform 0.0)
   (view_range_availability
    :reader view_range_availability
    :initarg :view_range_availability
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LkaLane (<LkaLane>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LkaLane>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LkaLane)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<LkaLane> is deprecated: use mobileye_560_660_msgs-msg:LkaLane instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lane_type-val :lambda-list '(m))
(cl:defmethod lane_type-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:lane_type-val is deprecated.  Use mobileye_560_660_msgs-msg:lane_type instead.")
  (lane_type m))

(cl:ensure-generic-function 'quality-val :lambda-list '(m))
(cl:defmethod quality-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:quality-val is deprecated.  Use mobileye_560_660_msgs-msg:quality instead.")
  (quality m))

(cl:ensure-generic-function 'model_degree-val :lambda-list '(m))
(cl:defmethod model_degree-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:model_degree-val is deprecated.  Use mobileye_560_660_msgs-msg:model_degree instead.")
  (model_degree m))

(cl:ensure-generic-function 'position_parameter_c0-val :lambda-list '(m))
(cl:defmethod position_parameter_c0-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:position_parameter_c0-val is deprecated.  Use mobileye_560_660_msgs-msg:position_parameter_c0 instead.")
  (position_parameter_c0 m))

(cl:ensure-generic-function 'curvature_parameter_c2-val :lambda-list '(m))
(cl:defmethod curvature_parameter_c2-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:curvature_parameter_c2-val is deprecated.  Use mobileye_560_660_msgs-msg:curvature_parameter_c2 instead.")
  (curvature_parameter_c2 m))

(cl:ensure-generic-function 'curvature_derivative_parameter_c3-val :lambda-list '(m))
(cl:defmethod curvature_derivative_parameter_c3-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:curvature_derivative_parameter_c3-val is deprecated.  Use mobileye_560_660_msgs-msg:curvature_derivative_parameter_c3 instead.")
  (curvature_derivative_parameter_c3 m))

(cl:ensure-generic-function 'marking_width-val :lambda-list '(m))
(cl:defmethod marking_width-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:marking_width-val is deprecated.  Use mobileye_560_660_msgs-msg:marking_width instead.")
  (marking_width m))

(cl:ensure-generic-function 'heading_angle_parameter_c1-val :lambda-list '(m))
(cl:defmethod heading_angle_parameter_c1-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:heading_angle_parameter_c1-val is deprecated.  Use mobileye_560_660_msgs-msg:heading_angle_parameter_c1 instead.")
  (heading_angle_parameter_c1 m))

(cl:ensure-generic-function 'view_range-val :lambda-list '(m))
(cl:defmethod view_range-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:view_range-val is deprecated.  Use mobileye_560_660_msgs-msg:view_range instead.")
  (view_range m))

(cl:ensure-generic-function 'view_range_availability-val :lambda-list '(m))
(cl:defmethod view_range_availability-val ((m <LkaLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:view_range_availability-val is deprecated.  Use mobileye_560_660_msgs-msg:view_range_availability instead.")
  (view_range_availability m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<LkaLane>)))
    "Constants for message type '<LkaLane>"
  '((:LANE_CONFIDENCE_NONE . 0)
    (:LANE_CONFIDENCE_LOW . 1)
    (:LANE_CONFIDENCE_MED . 2)
    (:LANE_CONFIDENCE_HIGH . 3)
    (:LANE_TYPE_DASHED . 0)
    (:LANE_TYPE_SOLID . 1)
    (:LANE_TYPE_NONE . 2)
    (:LANE_TYPE_ROAD_EDGE . 3)
    (:LANE_TYPE_DOUBLE_LANE_MARK . 4)
    (:LANE_TYPE_BOTTS_DOTS . 5)
    (:LANE_TYPE_INVALID . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'LkaLane)))
    "Constants for message type 'LkaLane"
  '((:LANE_CONFIDENCE_NONE . 0)
    (:LANE_CONFIDENCE_LOW . 1)
    (:LANE_CONFIDENCE_MED . 2)
    (:LANE_CONFIDENCE_HIGH . 3)
    (:LANE_TYPE_DASHED . 0)
    (:LANE_TYPE_SOLID . 1)
    (:LANE_TYPE_NONE . 2)
    (:LANE_TYPE_ROAD_EDGE . 3)
    (:LANE_TYPE_DOUBLE_LANE_MARK . 4)
    (:LANE_TYPE_BOTTS_DOTS . 5)
    (:LANE_TYPE_INVALID . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LkaLane>) ostream)
  "Serializes a message object of type '<LkaLane>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'model_degree)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position_parameter_c0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'curvature_parameter_c2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'curvature_derivative_parameter_c3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'marking_width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_angle_parameter_c1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'view_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'view_range_availability) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LkaLane>) istream)
  "Deserializes a message object of type '<LkaLane>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_type)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'model_degree)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position_parameter_c0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'curvature_parameter_c2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'curvature_derivative_parameter_c3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'marking_width) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_angle_parameter_c1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'view_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'view_range_availability) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LkaLane>)))
  "Returns string type for a message object of type '<LkaLane>"
  "mobileye_560_660_msgs/LkaLane")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LkaLane)))
  "Returns string type for a message object of type 'LkaLane"
  "mobileye_560_660_msgs/LkaLane")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LkaLane>)))
  "Returns md5sum for a message object of type '<LkaLane>"
  "13c7b357c14488be92473cab7e5461ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LkaLane)))
  "Returns md5sum for a message object of type 'LkaLane"
  "13c7b357c14488be92473cab7e5461ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LkaLane>)))
  "Returns full string definition for message of type '<LkaLane>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 LANE_CONFIDENCE_NONE = 0~%uint8 LANE_CONFIDENCE_LOW = 1~%uint8 LANE_CONFIDENCE_MED = 2~%uint8 LANE_CONFIDENCE_HIGH = 3~%~%uint8 LANE_TYPE_DASHED = 0~%uint8 LANE_TYPE_SOLID = 1~%uint8 LANE_TYPE_NONE = 2~%uint8 LANE_TYPE_ROAD_EDGE = 3~%uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4~%uint8 LANE_TYPE_BOTTS_DOTS = 5~%uint8 LANE_TYPE_INVALID = 6~%~%uint8 lane_type~%uint8 quality~%uint8 model_degree~%float64 position_parameter_c0~%float64 curvature_parameter_c2~%float64 curvature_derivative_parameter_c3~%float32 marking_width~%float64 heading_angle_parameter_c1~%float32 view_range~%bool view_range_availability~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LkaLane)))
  "Returns full string definition for message of type 'LkaLane"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 LANE_CONFIDENCE_NONE = 0~%uint8 LANE_CONFIDENCE_LOW = 1~%uint8 LANE_CONFIDENCE_MED = 2~%uint8 LANE_CONFIDENCE_HIGH = 3~%~%uint8 LANE_TYPE_DASHED = 0~%uint8 LANE_TYPE_SOLID = 1~%uint8 LANE_TYPE_NONE = 2~%uint8 LANE_TYPE_ROAD_EDGE = 3~%uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4~%uint8 LANE_TYPE_BOTTS_DOTS = 5~%uint8 LANE_TYPE_INVALID = 6~%~%uint8 lane_type~%uint8 quality~%uint8 model_degree~%float64 position_parameter_c0~%float64 curvature_parameter_c2~%float64 curvature_derivative_parameter_c3~%float32 marking_width~%float64 heading_angle_parameter_c1~%float32 view_range~%bool view_range_availability~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LkaLane>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     8
     8
     8
     4
     8
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LkaLane>))
  "Converts a ROS message object to a list"
  (cl:list 'LkaLane
    (cl:cons ':header (header msg))
    (cl:cons ':lane_type (lane_type msg))
    (cl:cons ':quality (quality msg))
    (cl:cons ':model_degree (model_degree msg))
    (cl:cons ':position_parameter_c0 (position_parameter_c0 msg))
    (cl:cons ':curvature_parameter_c2 (curvature_parameter_c2 msg))
    (cl:cons ':curvature_derivative_parameter_c3 (curvature_derivative_parameter_c3 msg))
    (cl:cons ':marking_width (marking_width msg))
    (cl:cons ':heading_angle_parameter_c1 (heading_angle_parameter_c1 msg))
    (cl:cons ':view_range (view_range msg))
    (cl:cons ':view_range_availability (view_range_availability msg))
))
