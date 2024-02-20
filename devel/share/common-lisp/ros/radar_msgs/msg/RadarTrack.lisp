; Auto-generated. Do not edit!


(cl:in-package radar_msgs-msg)


;//! \htmlinclude RadarTrack.msg.html

(cl:defclass <RadarTrack> (roslisp-msg-protocol:ros-message)
  ((uuid
    :reader uuid
    :initarg :uuid
    :type uuid_msgs-msg:UniqueID
    :initform (cl:make-instance 'uuid_msgs-msg:UniqueID))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (size
    :reader size
    :initarg :size
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (classification
    :reader classification
    :initarg :classification
    :type cl:fixnum
    :initform 0)
   (position_covariance
    :reader position_covariance
    :initarg :position_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (velocity_covariance
    :reader velocity_covariance
    :initarg :velocity_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (acceleration_covariance
    :reader acceleration_covariance
    :initarg :acceleration_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (size_covariance
    :reader size_covariance
    :initarg :size_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass RadarTrack (<RadarTrack>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarTrack>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarTrack)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name radar_msgs-msg:<RadarTrack> is deprecated: use radar_msgs-msg:RadarTrack instead.")))

(cl:ensure-generic-function 'uuid-val :lambda-list '(m))
(cl:defmethod uuid-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:uuid-val is deprecated.  Use radar_msgs-msg:uuid instead.")
  (uuid m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:position-val is deprecated.  Use radar_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:velocity-val is deprecated.  Use radar_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:acceleration-val is deprecated.  Use radar_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:size-val is deprecated.  Use radar_msgs-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'classification-val :lambda-list '(m))
(cl:defmethod classification-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:classification-val is deprecated.  Use radar_msgs-msg:classification instead.")
  (classification m))

(cl:ensure-generic-function 'position_covariance-val :lambda-list '(m))
(cl:defmethod position_covariance-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:position_covariance-val is deprecated.  Use radar_msgs-msg:position_covariance instead.")
  (position_covariance m))

(cl:ensure-generic-function 'velocity_covariance-val :lambda-list '(m))
(cl:defmethod velocity_covariance-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:velocity_covariance-val is deprecated.  Use radar_msgs-msg:velocity_covariance instead.")
  (velocity_covariance m))

(cl:ensure-generic-function 'acceleration_covariance-val :lambda-list '(m))
(cl:defmethod acceleration_covariance-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:acceleration_covariance-val is deprecated.  Use radar_msgs-msg:acceleration_covariance instead.")
  (acceleration_covariance m))

(cl:ensure-generic-function 'size_covariance-val :lambda-list '(m))
(cl:defmethod size_covariance-val ((m <RadarTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:size_covariance-val is deprecated.  Use radar_msgs-msg:size_covariance instead.")
  (size_covariance m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RadarTrack>)))
    "Constants for message type '<RadarTrack>"
  '((:NO_CLASSIFICATION . 0)
    (:STATIC . 1)
    (:DYNAMIC . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RadarTrack)))
    "Constants for message type 'RadarTrack"
  '((:NO_CLASSIFICATION . 0)
    (:STATIC . 1)
    (:DYNAMIC . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarTrack>) ostream)
  "Serializes a message object of type '<RadarTrack>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'uuid) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'classification)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'classification)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'position_covariance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'velocity_covariance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'acceleration_covariance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'size_covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarTrack>) istream)
  "Deserializes a message object of type '<RadarTrack>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'uuid) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'classification)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'classification)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'position_covariance) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'position_covariance)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'velocity_covariance) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'velocity_covariance)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'acceleration_covariance) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'acceleration_covariance)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'size_covariance) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'size_covariance)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarTrack>)))
  "Returns string type for a message object of type '<RadarTrack>"
  "radar_msgs/RadarTrack")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarTrack)))
  "Returns string type for a message object of type 'RadarTrack"
  "radar_msgs/RadarTrack")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarTrack>)))
  "Returns md5sum for a message object of type '<RadarTrack>"
  "3344659e36aff40bd4f09e82be663ec5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarTrack)))
  "Returns md5sum for a message object of type 'RadarTrack"
  "3344659e36aff40bd4f09e82be663ec5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarTrack>)))
  "Returns full string definition for message of type '<RadarTrack>"
  (cl:format cl:nil "# This message relates only to FMCW radar.  ~%# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%# Object classifications (Additional vendor-specific classifications are permitted starting from 32000 eg. Car)~%uint16 NO_CLASSIFICATION=0~%uint16 STATIC=1~%uint16 DYNAMIC=2~%~%~%uuid_msgs/UniqueID uuid                     # A unique ID of the object generated by the radar.~%~%                                            # Note: The z component of these fields is ignored for 2D tracking.~%geometry_msgs/Point position                # x, y, z coordinates of the centroid of the object being tracked.~%geometry_msgs/Vector3 velocity              # The velocity of the object in each spatial dimension.~%geometry_msgs/Vector3 acceleration          # The acceleration of the object in each spatial dimension.~%geometry_msgs/Vector3 size                  # The object size as represented by the radar sensor eg. length, width, height OR the diameter of an ellipsoid in the x, y, z, dimensions~%                                            # and is from the sensor frame's view.~%uint16 classification                       # An optional classification of the object (see above)~%float32[6] position_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] velocity_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] acceleration_covariance          # Upper-triangle covariance about the x, y, z axes~%float32[6] size_covariance                  # Upper-triangle covariance about the x, y, z axes~%~%================================================================================~%MSG: uuid_msgs/UniqueID~%# A universally unique identifier (UUID).~%#~%#  http://en.wikipedia.org/wiki/Universally_unique_identifier~%#  http://tools.ietf.org/html/rfc4122.html~%~%uint8[16] uuid~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarTrack)))
  "Returns full string definition for message of type 'RadarTrack"
  (cl:format cl:nil "# This message relates only to FMCW radar.  ~%# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%# Object classifications (Additional vendor-specific classifications are permitted starting from 32000 eg. Car)~%uint16 NO_CLASSIFICATION=0~%uint16 STATIC=1~%uint16 DYNAMIC=2~%~%~%uuid_msgs/UniqueID uuid                     # A unique ID of the object generated by the radar.~%~%                                            # Note: The z component of these fields is ignored for 2D tracking.~%geometry_msgs/Point position                # x, y, z coordinates of the centroid of the object being tracked.~%geometry_msgs/Vector3 velocity              # The velocity of the object in each spatial dimension.~%geometry_msgs/Vector3 acceleration          # The acceleration of the object in each spatial dimension.~%geometry_msgs/Vector3 size                  # The object size as represented by the radar sensor eg. length, width, height OR the diameter of an ellipsoid in the x, y, z, dimensions~%                                            # and is from the sensor frame's view.~%uint16 classification                       # An optional classification of the object (see above)~%float32[6] position_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] velocity_covariance              # Upper-triangle covariance about the x, y, z axes~%float32[6] acceleration_covariance          # Upper-triangle covariance about the x, y, z axes~%float32[6] size_covariance                  # Upper-triangle covariance about the x, y, z axes~%~%================================================================================~%MSG: uuid_msgs/UniqueID~%# A universally unique identifier (UUID).~%#~%#  http://en.wikipedia.org/wiki/Universally_unique_identifier~%#  http://tools.ietf.org/html/rfc4122.html~%~%uint8[16] uuid~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarTrack>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'uuid))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size))
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'velocity_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acceleration_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'size_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarTrack>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarTrack
    (cl:cons ':uuid (uuid msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':size (size msg))
    (cl:cons ':classification (classification msg))
    (cl:cons ':position_covariance (position_covariance msg))
    (cl:cons ':velocity_covariance (velocity_covariance msg))
    (cl:cons ':acceleration_covariance (acceleration_covariance msg))
    (cl:cons ':size_covariance (size_covariance msg))
))
