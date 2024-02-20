; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude AftermarketLane.msg.html

(cl:defclass <AftermarketLane> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lane_confidence_left
    :reader lane_confidence_left
    :initarg :lane_confidence_left
    :type cl:fixnum
    :initform 0)
   (ldw_available_left
    :reader ldw_available_left
    :initarg :ldw_available_left
    :type cl:boolean
    :initform cl:nil)
   (lane_type_left
    :reader lane_type_left
    :initarg :lane_type_left
    :type cl:fixnum
    :initform 0)
   (distance_to_left_lane
    :reader distance_to_left_lane
    :initarg :distance_to_left_lane
    :type cl:float
    :initform 0.0)
   (lane_confidence_right
    :reader lane_confidence_right
    :initarg :lane_confidence_right
    :type cl:fixnum
    :initform 0)
   (ldw_available_right
    :reader ldw_available_right
    :initarg :ldw_available_right
    :type cl:boolean
    :initform cl:nil)
   (lane_type_right
    :reader lane_type_right
    :initarg :lane_type_right
    :type cl:fixnum
    :initform 0)
   (distance_to_right_lane
    :reader distance_to_right_lane
    :initarg :distance_to_right_lane
    :type cl:float
    :initform 0.0))
)

(cl:defclass AftermarketLane (<AftermarketLane>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AftermarketLane>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AftermarketLane)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<AftermarketLane> is deprecated: use mobileye_560_660_msgs-msg:AftermarketLane instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lane_confidence_left-val :lambda-list '(m))
(cl:defmethod lane_confidence_left-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:lane_confidence_left-val is deprecated.  Use mobileye_560_660_msgs-msg:lane_confidence_left instead.")
  (lane_confidence_left m))

(cl:ensure-generic-function 'ldw_available_left-val :lambda-list '(m))
(cl:defmethod ldw_available_left-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ldw_available_left-val is deprecated.  Use mobileye_560_660_msgs-msg:ldw_available_left instead.")
  (ldw_available_left m))

(cl:ensure-generic-function 'lane_type_left-val :lambda-list '(m))
(cl:defmethod lane_type_left-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:lane_type_left-val is deprecated.  Use mobileye_560_660_msgs-msg:lane_type_left instead.")
  (lane_type_left m))

(cl:ensure-generic-function 'distance_to_left_lane-val :lambda-list '(m))
(cl:defmethod distance_to_left_lane-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:distance_to_left_lane-val is deprecated.  Use mobileye_560_660_msgs-msg:distance_to_left_lane instead.")
  (distance_to_left_lane m))

(cl:ensure-generic-function 'lane_confidence_right-val :lambda-list '(m))
(cl:defmethod lane_confidence_right-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:lane_confidence_right-val is deprecated.  Use mobileye_560_660_msgs-msg:lane_confidence_right instead.")
  (lane_confidence_right m))

(cl:ensure-generic-function 'ldw_available_right-val :lambda-list '(m))
(cl:defmethod ldw_available_right-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ldw_available_right-val is deprecated.  Use mobileye_560_660_msgs-msg:ldw_available_right instead.")
  (ldw_available_right m))

(cl:ensure-generic-function 'lane_type_right-val :lambda-list '(m))
(cl:defmethod lane_type_right-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:lane_type_right-val is deprecated.  Use mobileye_560_660_msgs-msg:lane_type_right instead.")
  (lane_type_right m))

(cl:ensure-generic-function 'distance_to_right_lane-val :lambda-list '(m))
(cl:defmethod distance_to_right_lane-val ((m <AftermarketLane>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:distance_to_right_lane-val is deprecated.  Use mobileye_560_660_msgs-msg:distance_to_right_lane instead.")
  (distance_to_right_lane m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AftermarketLane>)))
    "Constants for message type '<AftermarketLane>"
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
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AftermarketLane)))
    "Constants for message type 'AftermarketLane"
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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AftermarketLane>) ostream)
  "Serializes a message object of type '<AftermarketLane>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_confidence_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ldw_available_left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_type_left)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_to_left_lane))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_confidence_right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ldw_available_right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_type_right)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_to_right_lane))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AftermarketLane>) istream)
  "Deserializes a message object of type '<AftermarketLane>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_confidence_left)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ldw_available_left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_type_left)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_left_lane) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_confidence_right)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ldw_available_right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lane_type_right)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_right_lane) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AftermarketLane>)))
  "Returns string type for a message object of type '<AftermarketLane>"
  "mobileye_560_660_msgs/AftermarketLane")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AftermarketLane)))
  "Returns string type for a message object of type 'AftermarketLane"
  "mobileye_560_660_msgs/AftermarketLane")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AftermarketLane>)))
  "Returns md5sum for a message object of type '<AftermarketLane>"
  "8a56b7a5f0247252831a59dfc0910af7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AftermarketLane)))
  "Returns md5sum for a message object of type 'AftermarketLane"
  "8a56b7a5f0247252831a59dfc0910af7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AftermarketLane>)))
  "Returns full string definition for message of type '<AftermarketLane>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 LANE_CONFIDENCE_NONE = 0~%uint8 LANE_CONFIDENCE_LOW = 1~%uint8 LANE_CONFIDENCE_MED = 2~%uint8 LANE_CONFIDENCE_HIGH = 3~%~%uint8 LANE_TYPE_DASHED = 0~%uint8 LANE_TYPE_SOLID = 1~%uint8 LANE_TYPE_NONE = 2~%uint8 LANE_TYPE_ROAD_EDGE = 3~%uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4~%uint8 LANE_TYPE_BOTTS_DOTS = 5~%uint8 LANE_TYPE_INVALID = 6~%~%uint8 lane_confidence_left~%bool ldw_available_left~%uint8 lane_type_left~%float32 distance_to_left_lane~%uint8 lane_confidence_right~%bool ldw_available_right~%uint8 lane_type_right~%float32 distance_to_right_lane~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AftermarketLane)))
  "Returns full string definition for message of type 'AftermarketLane"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 LANE_CONFIDENCE_NONE = 0~%uint8 LANE_CONFIDENCE_LOW = 1~%uint8 LANE_CONFIDENCE_MED = 2~%uint8 LANE_CONFIDENCE_HIGH = 3~%~%uint8 LANE_TYPE_DASHED = 0~%uint8 LANE_TYPE_SOLID = 1~%uint8 LANE_TYPE_NONE = 2~%uint8 LANE_TYPE_ROAD_EDGE = 3~%uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4~%uint8 LANE_TYPE_BOTTS_DOTS = 5~%uint8 LANE_TYPE_INVALID = 6~%~%uint8 lane_confidence_left~%bool ldw_available_left~%uint8 lane_type_left~%float32 distance_to_left_lane~%uint8 lane_confidence_right~%bool ldw_available_right~%uint8 lane_type_right~%float32 distance_to_right_lane~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AftermarketLane>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     1
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AftermarketLane>))
  "Converts a ROS message object to a list"
  (cl:list 'AftermarketLane
    (cl:cons ':header (header msg))
    (cl:cons ':lane_confidence_left (lane_confidence_left msg))
    (cl:cons ':ldw_available_left (ldw_available_left msg))
    (cl:cons ':lane_type_left (lane_type_left msg))
    (cl:cons ':distance_to_left_lane (distance_to_left_lane msg))
    (cl:cons ':lane_confidence_right (lane_confidence_right msg))
    (cl:cons ':ldw_available_right (ldw_available_right msg))
    (cl:cons ':lane_type_right (lane_type_right msg))
    (cl:cons ':distance_to_right_lane (distance_to_right_lane msg))
))
