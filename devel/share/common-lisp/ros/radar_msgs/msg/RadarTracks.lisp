; Auto-generated. Do not edit!


(cl:in-package radar_msgs-msg)


;//! \htmlinclude RadarTracks.msg.html

(cl:defclass <RadarTracks> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (tracks
    :reader tracks
    :initarg :tracks
    :type (cl:vector radar_msgs-msg:RadarTrack)
   :initform (cl:make-array 0 :element-type 'radar_msgs-msg:RadarTrack :initial-element (cl:make-instance 'radar_msgs-msg:RadarTrack))))
)

(cl:defclass RadarTracks (<RadarTracks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarTracks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarTracks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name radar_msgs-msg:<RadarTracks> is deprecated: use radar_msgs-msg:RadarTracks instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RadarTracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:header-val is deprecated.  Use radar_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'tracks-val :lambda-list '(m))
(cl:defmethod tracks-val ((m <RadarTracks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:tracks-val is deprecated.  Use radar_msgs-msg:tracks instead.")
  (tracks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarTracks>) ostream)
  "Serializes a message object of type '<RadarTracks>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarTracks>) istream)
  "Deserializes a message object of type '<RadarTracks>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'radar_msgs-msg:RadarTrack))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarTracks>)))
  "Returns string type for a message object of type '<RadarTracks>"
  "radar_msgs/RadarTracks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarTracks)))
  "Returns string type for a message object of type 'RadarTracks"
  "radar_msgs/RadarTracks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarTracks>)))
  "Returns md5sum for a message object of type '<RadarTracks>"
  "d068321616577632690aba69b8985e75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarTracks)))
  "Returns md5sum for a message object of type 'RadarTracks"
  "d068321616577632690aba69b8985e75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarTracks>)))
  "Returns full string definition for message of type '<RadarTracks>"
  (cl:format cl:nil "std_msgs/Header header~%~%radar_msgs/RadarTrack[] tracks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: radar_msgs/RadarTrack~%# This message relates only to FMCW radar.  ~%# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%# Object classifications (Additional vendor-specific classifications are permitted starting from 32000 eg. Car)~%uint16 NO_CLASSIFICATION=0~%uint16 STATIC=1~%uint16 DYNAMIC=2~%~%~%uuid_msgs/UniqueID uuid                     # A unique ID of the object generated by the radar.~%~%                                            # Note: The z component of these fields is ignored for 2D tracking.~%geometry_msgs/Point position                # x, y, z coordinates of the centroid of the object being tracked.~%geometry_msgs/Vector3 velocity              # The velocity of the object in each spatial dimension.~%geometry_msgs/Vector3 acceleration          # The acceleration of the object in each spatial dimension.~%geometry_msgs/Vector3 size                  # The object size as represented by the radar sensor eg. length, width, height OR the diameter of an ellipsoid in the x, y, z, dimensions~%                                            # and is from the sensor frame's view.~%uint16 classification                       # An optional classification of the object (see above)~%float32[6] position_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] velocity_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] acceleration_covariance          # Upper-triangle covariance about the x, y, z axes~%float32[6] size_covariance                  # Upper-triangle covariance about the x, y, z axes~%~%================================================================================~%MSG: uuid_msgs/UniqueID~%# A universally unique identifier (UUID).~%#~%#  http://en.wikipedia.org/wiki/Universally_unique_identifier~%#  http://tools.ietf.org/html/rfc4122.html~%~%uint8[16] uuid~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarTracks)))
  "Returns full string definition for message of type 'RadarTracks"
  (cl:format cl:nil "std_msgs/Header header~%~%radar_msgs/RadarTrack[] tracks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: radar_msgs/RadarTrack~%# This message relates only to FMCW radar.  ~%# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%# Object classifications (Additional vendor-specific classifications are permitted starting from 32000 eg. Car)~%uint16 NO_CLASSIFICATION=0~%uint16 STATIC=1~%uint16 DYNAMIC=2~%~%~%uuid_msgs/UniqueID uuid                     # A unique ID of the object generated by the radar.~%~%                                            # Note: The z component of these fields is ignored for 2D tracking.~%geometry_msgs/Point position                # x, y, z coordinates of the centroid of the object being tracked.~%geometry_msgs/Vector3 velocity              # The velocity of the object in each spatial dimension.~%geometry_msgs/Vector3 acceleration          # The acceleration of the object in each spatial dimension.~%geometry_msgs/Vector3 size                  # The object size as represented by the radar sensor eg. length, width, height OR the diameter of an ellipsoid in the x, y, z, dimensions~%                                            # and is from the sensor frame's view.~%uint16 classification                       # An optional classification of the object (see above)~%float32[6] position_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] velocity_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] acceleration_covariance          # Upper-triangle covariance about the x, y, z axes~%float32[6] size_covariance                  # Upper-triangle covariance about the x, y, z axes~%~%================================================================================~%MSG: uuid_msgs/UniqueID~%# A universally unique identifier (UUID).~%#~%#  http://en.wikipedia.org/wiki/Universally_unique_identifier~%#  http://tools.ietf.org/html/rfc4122.html~%~%uint8[16] uuid~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarTracks>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarTracks>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarTracks
    (cl:cons ':header (header msg))
    (cl:cons ':tracks (tracks msg))
))
