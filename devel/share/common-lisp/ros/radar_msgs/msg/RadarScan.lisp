; Auto-generated. Do not edit!


(cl:in-package radar_msgs-msg)


;//! \htmlinclude RadarScan.msg.html

(cl:defclass <RadarScan> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (returns
    :reader returns
    :initarg :returns
    :type (cl:vector radar_msgs-msg:RadarReturn)
   :initform (cl:make-array 0 :element-type 'radar_msgs-msg:RadarReturn :initial-element (cl:make-instance 'radar_msgs-msg:RadarReturn))))
)

(cl:defclass RadarScan (<RadarScan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarScan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarScan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name radar_msgs-msg:<RadarScan> is deprecated: use radar_msgs-msg:RadarScan instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RadarScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:header-val is deprecated.  Use radar_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'returns-val :lambda-list '(m))
(cl:defmethod returns-val ((m <RadarScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:returns-val is deprecated.  Use radar_msgs-msg:returns instead.")
  (returns m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarScan>) ostream)
  "Serializes a message object of type '<RadarScan>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'returns))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'returns))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarScan>) istream)
  "Deserializes a message object of type '<RadarScan>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'returns) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'returns)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'radar_msgs-msg:RadarReturn))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarScan>)))
  "Returns string type for a message object of type '<RadarScan>"
  "radar_msgs/RadarScan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarScan)))
  "Returns string type for a message object of type 'RadarScan"
  "radar_msgs/RadarScan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarScan>)))
  "Returns md5sum for a message object of type '<RadarScan>"
  "6dfacef1e665538dbd8e159d5ce7a97a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarScan)))
  "Returns md5sum for a message object of type 'RadarScan"
  "6dfacef1e665538dbd8e159d5ce7a97a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarScan>)))
  "Returns full string definition for message of type '<RadarScan>"
  (cl:format cl:nil "std_msgs/Header header~%~%radar_msgs/RadarReturn[] returns~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: radar_msgs/RadarReturn~%# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%float32 range                            # Distance (m) from the sensor to the detected return.~%float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.~%                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.~%float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.~%                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.~%float32 doppler_velocity                 # The doppler speeds (m/s) of the return.~%float32 amplitude                        # The amplitude of the of the return (dB)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarScan)))
  "Returns full string definition for message of type 'RadarScan"
  (cl:format cl:nil "std_msgs/Header header~%~%radar_msgs/RadarReturn[] returns~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: radar_msgs/RadarReturn~%# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%float32 range                            # Distance (m) from the sensor to the detected return.~%float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.~%                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.~%float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.~%                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.~%float32 doppler_velocity                 # The doppler speeds (m/s) of the return.~%float32 amplitude                        # The amplitude of the of the return (dB)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarScan>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'returns) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarScan>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarScan
    (cl:cons ':header (header msg))
    (cl:cons ':returns (returns msg))
))
