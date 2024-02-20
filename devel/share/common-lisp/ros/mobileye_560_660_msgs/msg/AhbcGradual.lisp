; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude AhbcGradual.msg.html

(cl:defclass <AhbcGradual> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (boundary_domain_bottom_non_glare_hlb
    :reader boundary_domain_bottom_non_glare_hlb
    :initarg :boundary_domain_bottom_non_glare_hlb
    :type cl:float
    :initform 0.0)
   (boundary_domain_non_glare_left_hand_hlb
    :reader boundary_domain_non_glare_left_hand_hlb
    :initarg :boundary_domain_non_glare_left_hand_hlb
    :type cl:float
    :initform 0.0)
   (boundary_domain_non_glare_right_hand_hlb
    :reader boundary_domain_non_glare_right_hand_hlb
    :initarg :boundary_domain_non_glare_right_hand_hlb
    :type cl:float
    :initform 0.0)
   (object_distance_hlb
    :reader object_distance_hlb
    :initarg :object_distance_hlb
    :type cl:fixnum
    :initform 0)
   (status_boundary_domain_bottom_non_glare_hlb
    :reader status_boundary_domain_bottom_non_glare_hlb
    :initarg :status_boundary_domain_bottom_non_glare_hlb
    :type cl:fixnum
    :initform 0)
   (status_boundary_domain_non_glare_left_hand_hlb
    :reader status_boundary_domain_non_glare_left_hand_hlb
    :initarg :status_boundary_domain_non_glare_left_hand_hlb
    :type cl:fixnum
    :initform 0)
   (status_boundary_domain_non_glare_right_hand_hlb
    :reader status_boundary_domain_non_glare_right_hand_hlb
    :initarg :status_boundary_domain_non_glare_right_hand_hlb
    :type cl:fixnum
    :initform 0)
   (status_object_distance_hlb
    :reader status_object_distance_hlb
    :initarg :status_object_distance_hlb
    :type cl:fixnum
    :initform 0)
   (left_target_change
    :reader left_target_change
    :initarg :left_target_change
    :type cl:boolean
    :initform cl:nil)
   (right_target_change
    :reader right_target_change
    :initarg :right_target_change
    :type cl:boolean
    :initform cl:nil)
   (too_many_cars
    :reader too_many_cars
    :initarg :too_many_cars
    :type cl:boolean
    :initform cl:nil)
   (busy_scene
    :reader busy_scene
    :initarg :busy_scene
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AhbcGradual (<AhbcGradual>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AhbcGradual>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AhbcGradual)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<AhbcGradual> is deprecated: use mobileye_560_660_msgs-msg:AhbcGradual instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'boundary_domain_bottom_non_glare_hlb-val :lambda-list '(m))
(cl:defmethod boundary_domain_bottom_non_glare_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:boundary_domain_bottom_non_glare_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:boundary_domain_bottom_non_glare_hlb instead.")
  (boundary_domain_bottom_non_glare_hlb m))

(cl:ensure-generic-function 'boundary_domain_non_glare_left_hand_hlb-val :lambda-list '(m))
(cl:defmethod boundary_domain_non_glare_left_hand_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:boundary_domain_non_glare_left_hand_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:boundary_domain_non_glare_left_hand_hlb instead.")
  (boundary_domain_non_glare_left_hand_hlb m))

(cl:ensure-generic-function 'boundary_domain_non_glare_right_hand_hlb-val :lambda-list '(m))
(cl:defmethod boundary_domain_non_glare_right_hand_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:boundary_domain_non_glare_right_hand_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:boundary_domain_non_glare_right_hand_hlb instead.")
  (boundary_domain_non_glare_right_hand_hlb m))

(cl:ensure-generic-function 'object_distance_hlb-val :lambda-list '(m))
(cl:defmethod object_distance_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:object_distance_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:object_distance_hlb instead.")
  (object_distance_hlb m))

(cl:ensure-generic-function 'status_boundary_domain_bottom_non_glare_hlb-val :lambda-list '(m))
(cl:defmethod status_boundary_domain_bottom_non_glare_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:status_boundary_domain_bottom_non_glare_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:status_boundary_domain_bottom_non_glare_hlb instead.")
  (status_boundary_domain_bottom_non_glare_hlb m))

(cl:ensure-generic-function 'status_boundary_domain_non_glare_left_hand_hlb-val :lambda-list '(m))
(cl:defmethod status_boundary_domain_non_glare_left_hand_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:status_boundary_domain_non_glare_left_hand_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:status_boundary_domain_non_glare_left_hand_hlb instead.")
  (status_boundary_domain_non_glare_left_hand_hlb m))

(cl:ensure-generic-function 'status_boundary_domain_non_glare_right_hand_hlb-val :lambda-list '(m))
(cl:defmethod status_boundary_domain_non_glare_right_hand_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:status_boundary_domain_non_glare_right_hand_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:status_boundary_domain_non_glare_right_hand_hlb instead.")
  (status_boundary_domain_non_glare_right_hand_hlb m))

(cl:ensure-generic-function 'status_object_distance_hlb-val :lambda-list '(m))
(cl:defmethod status_object_distance_hlb-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:status_object_distance_hlb-val is deprecated.  Use mobileye_560_660_msgs-msg:status_object_distance_hlb instead.")
  (status_object_distance_hlb m))

(cl:ensure-generic-function 'left_target_change-val :lambda-list '(m))
(cl:defmethod left_target_change-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:left_target_change-val is deprecated.  Use mobileye_560_660_msgs-msg:left_target_change instead.")
  (left_target_change m))

(cl:ensure-generic-function 'right_target_change-val :lambda-list '(m))
(cl:defmethod right_target_change-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:right_target_change-val is deprecated.  Use mobileye_560_660_msgs-msg:right_target_change instead.")
  (right_target_change m))

(cl:ensure-generic-function 'too_many_cars-val :lambda-list '(m))
(cl:defmethod too_many_cars-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:too_many_cars-val is deprecated.  Use mobileye_560_660_msgs-msg:too_many_cars instead.")
  (too_many_cars m))

(cl:ensure-generic-function 'busy_scene-val :lambda-list '(m))
(cl:defmethod busy_scene-val ((m <AhbcGradual>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:busy_scene-val is deprecated.  Use mobileye_560_660_msgs-msg:busy_scene instead.")
  (busy_scene m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AhbcGradual>) ostream)
  "Serializes a message object of type '<AhbcGradual>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'boundary_domain_bottom_non_glare_hlb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'boundary_domain_non_glare_left_hand_hlb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'boundary_domain_non_glare_right_hand_hlb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_distance_hlb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_distance_hlb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_boundary_domain_bottom_non_glare_hlb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_boundary_domain_non_glare_left_hand_hlb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_boundary_domain_non_glare_right_hand_hlb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_object_distance_hlb)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_target_change) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_target_change) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'too_many_cars) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'busy_scene) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AhbcGradual>) istream)
  "Deserializes a message object of type '<AhbcGradual>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'boundary_domain_bottom_non_glare_hlb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'boundary_domain_non_glare_left_hand_hlb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'boundary_domain_non_glare_right_hand_hlb) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_distance_hlb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_distance_hlb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_boundary_domain_bottom_non_glare_hlb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_boundary_domain_non_glare_left_hand_hlb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_boundary_domain_non_glare_right_hand_hlb)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status_object_distance_hlb)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_target_change) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_target_change) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'too_many_cars) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'busy_scene) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AhbcGradual>)))
  "Returns string type for a message object of type '<AhbcGradual>"
  "mobileye_560_660_msgs/AhbcGradual")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AhbcGradual)))
  "Returns string type for a message object of type 'AhbcGradual"
  "mobileye_560_660_msgs/AhbcGradual")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AhbcGradual>)))
  "Returns md5sum for a message object of type '<AhbcGradual>"
  "06801ea66cd7dc52de9867c12dbfa5bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AhbcGradual)))
  "Returns md5sum for a message object of type 'AhbcGradual"
  "06801ea66cd7dc52de9867c12dbfa5bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AhbcGradual>)))
  "Returns full string definition for message of type '<AhbcGradual>"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 boundary_domain_bottom_non_glare_hlb~%float32 boundary_domain_non_glare_left_hand_hlb~%float32 boundary_domain_non_glare_right_hand_hlb~%uint16 object_distance_hlb~%uint8 status_boundary_domain_bottom_non_glare_hlb~%uint8 status_boundary_domain_non_glare_left_hand_hlb~%uint8 status_boundary_domain_non_glare_right_hand_hlb~%uint8 status_object_distance_hlb~%bool left_target_change~%bool right_target_change~%bool too_many_cars~%bool busy_scene~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AhbcGradual)))
  "Returns full string definition for message of type 'AhbcGradual"
  (cl:format cl:nil "std_msgs/Header header~%~%float32 boundary_domain_bottom_non_glare_hlb~%float32 boundary_domain_non_glare_left_hand_hlb~%float32 boundary_domain_non_glare_right_hand_hlb~%uint16 object_distance_hlb~%uint8 status_boundary_domain_bottom_non_glare_hlb~%uint8 status_boundary_domain_non_glare_left_hand_hlb~%uint8 status_boundary_domain_non_glare_right_hand_hlb~%uint8 status_object_distance_hlb~%bool left_target_change~%bool right_target_change~%bool too_many_cars~%bool busy_scene~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AhbcGradual>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     2
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AhbcGradual>))
  "Converts a ROS message object to a list"
  (cl:list 'AhbcGradual
    (cl:cons ':header (header msg))
    (cl:cons ':boundary_domain_bottom_non_glare_hlb (boundary_domain_bottom_non_glare_hlb msg))
    (cl:cons ':boundary_domain_non_glare_left_hand_hlb (boundary_domain_non_glare_left_hand_hlb msg))
    (cl:cons ':boundary_domain_non_glare_right_hand_hlb (boundary_domain_non_glare_right_hand_hlb msg))
    (cl:cons ':object_distance_hlb (object_distance_hlb msg))
    (cl:cons ':status_boundary_domain_bottom_non_glare_hlb (status_boundary_domain_bottom_non_glare_hlb msg))
    (cl:cons ':status_boundary_domain_non_glare_left_hand_hlb (status_boundary_domain_non_glare_left_hand_hlb msg))
    (cl:cons ':status_boundary_domain_non_glare_right_hand_hlb (status_boundary_domain_non_glare_right_hand_hlb msg))
    (cl:cons ':status_object_distance_hlb (status_object_distance_hlb msg))
    (cl:cons ':left_target_change (left_target_change msg))
    (cl:cons ':right_target_change (right_target_change msg))
    (cl:cons ':too_many_cars (too_many_cars msg))
    (cl:cons ':busy_scene (busy_scene msg))
))
