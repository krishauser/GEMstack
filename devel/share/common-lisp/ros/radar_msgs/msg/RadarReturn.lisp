; Auto-generated. Do not edit!


(cl:in-package radar_msgs-msg)


;//! \htmlinclude RadarReturn.msg.html

(cl:defclass <RadarReturn> (roslisp-msg-protocol:ros-message)
  ((range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (azimuth
    :reader azimuth
    :initarg :azimuth
    :type cl:float
    :initform 0.0)
   (elevation
    :reader elevation
    :initarg :elevation
    :type cl:float
    :initform 0.0)
   (doppler_velocity
    :reader doppler_velocity
    :initarg :doppler_velocity
    :type cl:float
    :initform 0.0)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass RadarReturn (<RadarReturn>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarReturn>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarReturn)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name radar_msgs-msg:<RadarReturn> is deprecated: use radar_msgs-msg:RadarReturn instead.")))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <RadarReturn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:range-val is deprecated.  Use radar_msgs-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <RadarReturn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:azimuth-val is deprecated.  Use radar_msgs-msg:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'elevation-val :lambda-list '(m))
(cl:defmethod elevation-val ((m <RadarReturn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:elevation-val is deprecated.  Use radar_msgs-msg:elevation instead.")
  (elevation m))

(cl:ensure-generic-function 'doppler_velocity-val :lambda-list '(m))
(cl:defmethod doppler_velocity-val ((m <RadarReturn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:doppler_velocity-val is deprecated.  Use radar_msgs-msg:doppler_velocity instead.")
  (doppler_velocity m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <RadarReturn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader radar_msgs-msg:amplitude-val is deprecated.  Use radar_msgs-msg:amplitude instead.")
  (amplitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarReturn>) ostream)
  "Serializes a message object of type '<RadarReturn>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'elevation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'doppler_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarReturn>) istream)
  "Deserializes a message object of type '<RadarReturn>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'azimuth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elevation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'doppler_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amplitude) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarReturn>)))
  "Returns string type for a message object of type '<RadarReturn>"
  "radar_msgs/RadarReturn")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarReturn)))
  "Returns string type for a message object of type 'RadarReturn"
  "radar_msgs/RadarReturn")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarReturn>)))
  "Returns md5sum for a message object of type '<RadarReturn>"
  "d2fa6f7b9af80adc27de1892e316aaf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarReturn)))
  "Returns md5sum for a message object of type 'RadarReturn"
  "d2fa6f7b9af80adc27de1892e316aaf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarReturn>)))
  "Returns full string definition for message of type '<RadarReturn>"
  (cl:format cl:nil "# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%float32 range                            # Distance (m) from the sensor to the detected return.~%float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.~%                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.~%float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.~%                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.~%float32 doppler_velocity                 # The doppler speeds (m/s) of the return.~%float32 amplitude                        # The amplitude of the of the return (dB)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarReturn)))
  "Returns full string definition for message of type 'RadarReturn"
  (cl:format cl:nil "# All variables below are relative to the radar's frame of reference.~%# This message is not meant to be used alone but as part of a stamped or array message.~%~%float32 range                            # Distance (m) from the sensor to the detected return.~%float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.~%                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.~%float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.~%                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.~%float32 doppler_velocity                 # The doppler speeds (m/s) of the return.~%float32 amplitude                        # The amplitude of the of the return (dB)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarReturn>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarReturn>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarReturn
    (cl:cons ':range (range msg))
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':elevation (elevation msg))
    (cl:cons ':doppler_velocity (doppler_velocity msg))
    (cl:cons ':amplitude (amplitude msg))
))
