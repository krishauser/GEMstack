; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude LkaReferencePoints.msg.html

(cl:defclass <LkaReferencePoints> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ref_point_1_position
    :reader ref_point_1_position
    :initarg :ref_point_1_position
    :type cl:float
    :initform 0.0)
   (ref_point_1_distance
    :reader ref_point_1_distance
    :initarg :ref_point_1_distance
    :type cl:float
    :initform 0.0)
   (ref_point_1_validity
    :reader ref_point_1_validity
    :initarg :ref_point_1_validity
    :type cl:boolean
    :initform cl:nil)
   (ref_point_2_position
    :reader ref_point_2_position
    :initarg :ref_point_2_position
    :type cl:float
    :initform 0.0)
   (ref_point_2_distance
    :reader ref_point_2_distance
    :initarg :ref_point_2_distance
    :type cl:float
    :initform 0.0)
   (ref_point_2_validity
    :reader ref_point_2_validity
    :initarg :ref_point_2_validity
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LkaReferencePoints (<LkaReferencePoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LkaReferencePoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LkaReferencePoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<LkaReferencePoints> is deprecated: use mobileye_560_660_msgs-msg:LkaReferencePoints instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ref_point_1_position-val :lambda-list '(m))
(cl:defmethod ref_point_1_position-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ref_point_1_position-val is deprecated.  Use mobileye_560_660_msgs-msg:ref_point_1_position instead.")
  (ref_point_1_position m))

(cl:ensure-generic-function 'ref_point_1_distance-val :lambda-list '(m))
(cl:defmethod ref_point_1_distance-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ref_point_1_distance-val is deprecated.  Use mobileye_560_660_msgs-msg:ref_point_1_distance instead.")
  (ref_point_1_distance m))

(cl:ensure-generic-function 'ref_point_1_validity-val :lambda-list '(m))
(cl:defmethod ref_point_1_validity-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ref_point_1_validity-val is deprecated.  Use mobileye_560_660_msgs-msg:ref_point_1_validity instead.")
  (ref_point_1_validity m))

(cl:ensure-generic-function 'ref_point_2_position-val :lambda-list '(m))
(cl:defmethod ref_point_2_position-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ref_point_2_position-val is deprecated.  Use mobileye_560_660_msgs-msg:ref_point_2_position instead.")
  (ref_point_2_position m))

(cl:ensure-generic-function 'ref_point_2_distance-val :lambda-list '(m))
(cl:defmethod ref_point_2_distance-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ref_point_2_distance-val is deprecated.  Use mobileye_560_660_msgs-msg:ref_point_2_distance instead.")
  (ref_point_2_distance m))

(cl:ensure-generic-function 'ref_point_2_validity-val :lambda-list '(m))
(cl:defmethod ref_point_2_validity-val ((m <LkaReferencePoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:ref_point_2_validity-val is deprecated.  Use mobileye_560_660_msgs-msg:ref_point_2_validity instead.")
  (ref_point_2_validity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LkaReferencePoints>) ostream)
  "Serializes a message object of type '<LkaReferencePoints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ref_point_1_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ref_point_1_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ref_point_1_validity) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ref_point_2_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ref_point_2_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ref_point_2_validity) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LkaReferencePoints>) istream)
  "Deserializes a message object of type '<LkaReferencePoints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ref_point_1_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ref_point_1_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'ref_point_1_validity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ref_point_2_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ref_point_2_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'ref_point_2_validity) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LkaReferencePoints>)))
  "Returns string type for a message object of type '<LkaReferencePoints>"
  "mobileye_560_660_msgs/LkaReferencePoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LkaReferencePoints)))
  "Returns string type for a message object of type 'LkaReferencePoints"
  "mobileye_560_660_msgs/LkaReferencePoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LkaReferencePoints>)))
  "Returns md5sum for a message object of type '<LkaReferencePoints>"
  "0da833fa4330bb296afc10246a88cb60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LkaReferencePoints)))
  "Returns md5sum for a message object of type 'LkaReferencePoints"
  "0da833fa4330bb296afc10246a88cb60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LkaReferencePoints>)))
  "Returns full string definition for message of type '<LkaReferencePoints>"
  (cl:format cl:nil "std_msgs/Header header~%~%float64 ref_point_1_position~%float64 ref_point_1_distance~%bool ref_point_1_validity~%float64 ref_point_2_position~%float64 ref_point_2_distance~%bool ref_point_2_validity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LkaReferencePoints)))
  "Returns full string definition for message of type 'LkaReferencePoints"
  (cl:format cl:nil "std_msgs/Header header~%~%float64 ref_point_1_position~%float64 ref_point_1_distance~%bool ref_point_1_validity~%float64 ref_point_2_position~%float64 ref_point_2_distance~%bool ref_point_2_validity~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LkaReferencePoints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     1
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LkaReferencePoints>))
  "Converts a ROS message object to a list"
  (cl:list 'LkaReferencePoints
    (cl:cons ':header (header msg))
    (cl:cons ':ref_point_1_position (ref_point_1_position msg))
    (cl:cons ':ref_point_1_distance (ref_point_1_distance msg))
    (cl:cons ':ref_point_1_validity (ref_point_1_validity msg))
    (cl:cons ':ref_point_2_position (ref_point_2_position msg))
    (cl:cons ':ref_point_2_distance (ref_point_2_distance msg))
    (cl:cons ':ref_point_2_validity (ref_point_2_validity msg))
))
