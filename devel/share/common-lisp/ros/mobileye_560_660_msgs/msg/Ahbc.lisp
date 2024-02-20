; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude Ahbc.msg.html

(cl:defclass <Ahbc> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (high_low_beam_decision
    :reader high_low_beam_decision
    :initarg :high_low_beam_decision
    :type cl:fixnum
    :initform 0)
   (reasons_for_switch_to_low_beam
    :reader reasons_for_switch_to_low_beam
    :initarg :reasons_for_switch_to_low_beam
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Ahbc (<Ahbc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ahbc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ahbc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<Ahbc> is deprecated: use mobileye_560_660_msgs-msg:Ahbc instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Ahbc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'high_low_beam_decision-val :lambda-list '(m))
(cl:defmethod high_low_beam_decision-val ((m <Ahbc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:high_low_beam_decision-val is deprecated.  Use mobileye_560_660_msgs-msg:high_low_beam_decision instead.")
  (high_low_beam_decision m))

(cl:ensure-generic-function 'reasons_for_switch_to_low_beam-val :lambda-list '(m))
(cl:defmethod reasons_for_switch_to_low_beam-val ((m <Ahbc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:reasons_for_switch_to_low_beam-val is deprecated.  Use mobileye_560_660_msgs-msg:reasons_for_switch_to_low_beam instead.")
  (reasons_for_switch_to_low_beam m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Ahbc>)))
    "Constants for message type '<Ahbc>"
  '((:HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION . 0)
    (:HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF . 1)
    (:HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON . 2)
    (:HIGH_LOW_BEAM_DECISION_INVALID . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Ahbc)))
    "Constants for message type 'Ahbc"
  '((:HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION . 0)
    (:HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF . 1)
    (:HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON . 2)
    (:HIGH_LOW_BEAM_DECISION_INVALID . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ahbc>) ostream)
  "Serializes a message object of type '<Ahbc>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'high_low_beam_decision)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reasons_for_switch_to_low_beam)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reasons_for_switch_to_low_beam)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ahbc>) istream)
  "Deserializes a message object of type '<Ahbc>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'high_low_beam_decision)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'reasons_for_switch_to_low_beam)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'reasons_for_switch_to_low_beam)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ahbc>)))
  "Returns string type for a message object of type '<Ahbc>"
  "mobileye_560_660_msgs/Ahbc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ahbc)))
  "Returns string type for a message object of type 'Ahbc"
  "mobileye_560_660_msgs/Ahbc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ahbc>)))
  "Returns md5sum for a message object of type '<Ahbc>"
  "475e214fc14bee0ccbbfc2ae7aaea6ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ahbc)))
  "Returns md5sum for a message object of type 'Ahbc"
  "475e214fc14bee0ccbbfc2ae7aaea6ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ahbc>)))
  "Returns full string definition for message of type '<Ahbc>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION = 0~%uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF = 1~%uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON = 2~%uint8 HIGH_LOW_BEAM_DECISION_INVALID = 3~%uint8 high_low_beam_decision~%~%uint16 reasons_for_switch_to_low_beam~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ahbc)))
  "Returns full string definition for message of type 'Ahbc"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 HIGH_LOW_BEAM_DECISION_NO_RECOMMENDATION = 0~%uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_OFF = 1~%uint8 HIGH_LOW_BEAM_DECISION_RECOMMENDATION_ON = 2~%uint8 HIGH_LOW_BEAM_DECISION_INVALID = 3~%uint8 high_low_beam_decision~%~%uint16 reasons_for_switch_to_low_beam~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ahbc>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ahbc>))
  "Converts a ROS message object to a list"
  (cl:list 'Ahbc
    (cl:cons ':header (header msg))
    (cl:cons ':high_low_beam_decision (high_low_beam_decision msg))
    (cl:cons ':reasons_for_switch_to_low_beam (reasons_for_switch_to_low_beam msg))
))
