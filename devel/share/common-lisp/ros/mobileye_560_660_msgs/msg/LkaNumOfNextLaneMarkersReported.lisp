; Auto-generated. Do not edit!


(cl:in-package mobileye_560_660_msgs-msg)


;//! \htmlinclude LkaNumOfNextLaneMarkersReported.msg.html

(cl:defclass <LkaNumOfNextLaneMarkersReported> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_of_next_lane_markers_reported
    :reader num_of_next_lane_markers_reported
    :initarg :num_of_next_lane_markers_reported
    :type cl:fixnum
    :initform 0))
)

(cl:defclass LkaNumOfNextLaneMarkersReported (<LkaNumOfNextLaneMarkersReported>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LkaNumOfNextLaneMarkersReported>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LkaNumOfNextLaneMarkersReported)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobileye_560_660_msgs-msg:<LkaNumOfNextLaneMarkersReported> is deprecated: use mobileye_560_660_msgs-msg:LkaNumOfNextLaneMarkersReported instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LkaNumOfNextLaneMarkersReported>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:header-val is deprecated.  Use mobileye_560_660_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_of_next_lane_markers_reported-val :lambda-list '(m))
(cl:defmethod num_of_next_lane_markers_reported-val ((m <LkaNumOfNextLaneMarkersReported>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobileye_560_660_msgs-msg:num_of_next_lane_markers_reported-val is deprecated.  Use mobileye_560_660_msgs-msg:num_of_next_lane_markers_reported instead.")
  (num_of_next_lane_markers_reported m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LkaNumOfNextLaneMarkersReported>) ostream)
  "Serializes a message object of type '<LkaNumOfNextLaneMarkersReported>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_next_lane_markers_reported)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_of_next_lane_markers_reported)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LkaNumOfNextLaneMarkersReported>) istream)
  "Deserializes a message object of type '<LkaNumOfNextLaneMarkersReported>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_of_next_lane_markers_reported)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_of_next_lane_markers_reported)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LkaNumOfNextLaneMarkersReported>)))
  "Returns string type for a message object of type '<LkaNumOfNextLaneMarkersReported>"
  "mobileye_560_660_msgs/LkaNumOfNextLaneMarkersReported")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LkaNumOfNextLaneMarkersReported)))
  "Returns string type for a message object of type 'LkaNumOfNextLaneMarkersReported"
  "mobileye_560_660_msgs/LkaNumOfNextLaneMarkersReported")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LkaNumOfNextLaneMarkersReported>)))
  "Returns md5sum for a message object of type '<LkaNumOfNextLaneMarkersReported>"
  "0313c1cecbae25d684c324d160d9925e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LkaNumOfNextLaneMarkersReported)))
  "Returns md5sum for a message object of type 'LkaNumOfNextLaneMarkersReported"
  "0313c1cecbae25d684c324d160d9925e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LkaNumOfNextLaneMarkersReported>)))
  "Returns full string definition for message of type '<LkaNumOfNextLaneMarkersReported>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 num_of_next_lane_markers_reported~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LkaNumOfNextLaneMarkersReported)))
  "Returns full string definition for message of type 'LkaNumOfNextLaneMarkersReported"
  (cl:format cl:nil "std_msgs/Header header~%~%uint16 num_of_next_lane_markers_reported~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LkaNumOfNextLaneMarkersReported>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LkaNumOfNextLaneMarkersReported>))
  "Converts a ROS message object to a list"
  (cl:list 'LkaNumOfNextLaneMarkersReported
    (cl:cons ':header (header msg))
    (cl:cons ':num_of_next_lane_markers_reported (num_of_next_lane_markers_reported msg))
))
