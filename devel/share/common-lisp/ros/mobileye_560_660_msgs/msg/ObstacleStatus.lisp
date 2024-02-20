; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude ObstacleStatus.msg.html

(cl:defclass <ObstacleStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_obstacles
    :reader num_obstacles
    :initarg :num_obstacles
    :type cl:fixnum
    :initform 0)
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:fixnum
    :initform 0)
   (application_version
    :reader application_version
    :initarg :application_version
    :type cl:fixnum
    :initform 0)
   (active_version_number_section
    :reader active_version_number_section
    :initarg :active_version_number_section
    :type cl:fixnum
    :initform 0)
   (left_close_range_cut_in
    :reader left_close_range_cut_in
    :initarg :left_close_range_cut_in
    :type cl:boolean
    :initform cl:nil)
   (right_close_range_cut_in
    :reader right_close_range_cut_in
    :initarg :right_close_range_cut_in
    :type cl:boolean
    :initform cl:nil)
   (stop_go
    :reader stop_go
    :initarg :stop_go
    :type cl:fixnum
    :initform 0)
   (protocol_version
    :reader protocol_version
    :initarg :protocol_version
    :type cl:fixnum
    :initform 0)
   (close_car
    :reader close_car
    :initarg :close_car
    :type cl:boolean
    :initform cl:nil)
   (failsafe
    :reader failsafe
    :initarg :failsafe
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ObstacleStatus (<ObstacleStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<ObstacleStatus> is deprecated: use mobileye_560_660_msgs-msg:ObstacleStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_obstacles-val :lambda-list '(m))
(cl:defmethod num_obstacles-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:num_obstacles-val is deprecated.  Use mobileye_560_660_msgs-msg:num_obstacles instead.")
  (num_obstacles m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:timestamp-val is deprecated.  Use mobileye_560_660_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'application_version-val :lambda-list '(m))
(cl:defmethod application_version-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:application_version-val is deprecated.  Use mobileye_560_660_msgs-msg:application_version instead.")
  (application_version m))

(cl:ensure-generic-function 'active_version_number_section-val :lambda-list '(m))
(cl:defmethod active_version_number_section-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:active_version_number_section-val is deprecated.  Use mobileye_560_660_msgs-msg:active_version_number_section instead.")
  (active_version_number_section m))

(cl:ensure-generic-function 'left_close_range_cut_in-val :lambda-list '(m))
(cl:defmethod left_close_range_cut_in-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:left_close_range_cut_in-val is deprecated.  Use mobileye_560_660_msgs-msg:left_close_range_cut_in instead.")
  (left_close_range_cut_in m))

(cl:ensure-generic-function 'right_close_range_cut_in-val :lambda-list '(m))
(cl:defmethod right_close_range_cut_in-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:right_close_range_cut_in-val is deprecated.  Use mobileye_560_660_msgs-msg:right_close_range_cut_in instead.")
  (right_close_range_cut_in m))

(cl:ensure-generic-function 'stop_go-val :lambda-list '(m))
(cl:defmethod stop_go-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:stop_go-val is deprecated.  Use mobileye_560_660_msgs-msg:stop_go instead.")
  (stop_go m))

(cl:ensure-generic-function 'protocol_version-val :lambda-list '(m))
(cl:defmethod protocol_version-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:protocol_version-val is deprecated.  Use mobileye_560_660_msgs-msg:protocol_version instead.")
  (protocol_version m))

(cl:ensure-generic-function 'close_car-val :lambda-list '(m))
(cl:defmethod close_car-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:close_car-val is deprecated.  Use mobileye_560_660_msgs-msg:close_car instead.")
  (close_car m))

(cl:ensure-generic-function 'failsafe-val :lambda-list '(m))
(cl:defmethod failsafe-val ((m <ObstacleStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:failsafe-val is deprecated.  Use mobileye_560_660_msgs-msg:failsafe instead.")
  (failsafe m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ObstacleStatus>)))
    "Constants for message type '<ObstacleStatus>"
  '((:STOP_GO_STOP . 0)
    (:STOP_GO_GO . 1)
    (:STOP_GO_UNDECIDED . 2)
    (:STOP_GO_DRIVER_DECISION_REQUIRED . 3)
    (:STOP_GO_NOT_CALCULATED . 15)
    (:FAILSAFE_NONE . 0)
    (:FAILSAFE_LOW_SUN . 1)
    (:FAILSAFE_BLUR_IMAGE . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ObstacleStatus)))
    "Constants for message type 'ObstacleStatus"
  '((:STOP_GO_STOP . 0)
    (:STOP_GO_GO . 1)
    (:STOP_GO_UNDECIDED . 2)
    (:STOP_GO_DRIVER_DECISION_REQUIRED . 3)
    (:STOP_GO_NOT_CALCULATED . 15)
    (:FAILSAFE_NONE . 0)
    (:FAILSAFE_LOW_SUN . 1)
    (:FAILSAFE_BLUR_IMAGE . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleStatus>) ostream)
  "Serializes a message object of type '<ObstacleStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_obstacles)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_obstacles)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'application_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'application_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_version_number_section)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_version_number_section)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_close_range_cut_in) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_close_range_cut_in) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stop_go)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'protocol_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'close_car) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failsafe)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleStatus>) istream)
  "Deserializes a message object of type '<ObstacleStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_obstacles)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_obstacles)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'application_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'application_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_version_number_section)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'active_version_number_section)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_close_range_cut_in) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_close_range_cut_in) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stop_go)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'protocol_version)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'close_car) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failsafe)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleStatus>)))
  "Returns string type for a message object of type '<ObstacleStatus>"
  "mobileye_560_660_msgs/ObstacleStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleStatus)))
  "Returns string type for a message object of type 'ObstacleStatus"
  "mobileye_560_660_msgs/ObstacleStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleStatus>)))
  "Returns md5sum for a message object of type '<ObstacleStatus>"
  "b963ecf49d557c90935e49005018b9ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleStatus)))
  "Returns md5sum for a message object of type 'ObstacleStatus"
  "b963ecf49d557c90935e49005018b9ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleStatus>)))
  "Returns full string definition for message of type '<ObstacleStatus>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 num_obstacles~%uint16 timestamp~%uint16 application_version~%uint16 active_version_number_section~%bool left_close_range_cut_in~%bool right_close_range_cut_in~%~%uint8 STOP_GO_STOP = 0~%uint8 STOP_GO_GO = 1~%uint8 STOP_GO_UNDECIDED = 2~%uint8 STOP_GO_DRIVER_DECISION_REQUIRED = 3~%uint8 STOP_GO_NOT_CALCULATED = 15~%uint8 stop_go~%~%uint8 protocol_version~%bool close_car~%~%uint8 FAILSAFE_NONE = 0~%uint8 FAILSAFE_LOW_SUN = 1~%uint8 FAILSAFE_BLUR_IMAGE = 2~%uint8 failsafe~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleStatus)))
  "Returns full string definition for message of type 'ObstacleStatus"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 num_obstacles~%uint16 timestamp~%uint16 application_version~%uint16 active_version_number_section~%bool left_close_range_cut_in~%bool right_close_range_cut_in~%~%uint8 STOP_GO_STOP = 0~%uint8 STOP_GO_GO = 1~%uint8 STOP_GO_UNDECIDED = 2~%uint8 STOP_GO_DRIVER_DECISION_REQUIRED = 3~%uint8 STOP_GO_NOT_CALCULATED = 15~%uint8 stop_go~%~%uint8 protocol_version~%bool close_car~%~%uint8 FAILSAFE_NONE = 0~%uint8 FAILSAFE_LOW_SUN = 1~%uint8 FAILSAFE_BLUR_IMAGE = 2~%uint8 failsafe~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     2
     2
     2
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleStatus
    (cl:cons ':header (header msg))
    (cl:cons ':num_obstacles (num_obstacles msg))
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':application_version (application_version msg))
    (cl:cons ':active_version_number_section (active_version_number_section msg))
    (cl:cons ':left_close_range_cut_in (left_close_range_cut_in msg))
    (cl:cons ':right_close_range_cut_in (right_close_range_cut_in msg))
    (cl:cons ':stop_go (stop_go msg))
    (cl:cons ':protocol_version (protocol_version msg))
    (cl:cons ':close_car (close_car msg))
    (cl:cons ':failsafe (failsafe msg))
))
