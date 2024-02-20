; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrTrackMotionPowerTrack.msg.html

(cl:defclass <EsrTrackMotionPowerTrack> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (movable_fast
    :reader movable_fast
    :initarg :movable_fast
    :type cl:boolean
    :initform cl:nil)
   (movable_slow
    :reader movable_slow
    :initarg :movable_slow
    :type cl:boolean
    :initform cl:nil)
   (moving
    :reader moving
    :initarg :moving
    :type cl:boolean
    :initform cl:nil)
   (power
    :reader power
    :initarg :power
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EsrTrackMotionPowerTrack (<EsrTrackMotionPowerTrack>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrTrackMotionPowerTrack>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrTrackMotionPowerTrack)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrTrackMotionPowerTrack> is deprecated: use delphi_esr_msgs-msg:EsrTrackMotionPowerTrack instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <EsrTrackMotionPowerTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:id-val is deprecated.  Use delphi_esr_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'movable_fast-val :lambda-list '(m))
(cl:defmethod movable_fast-val ((m <EsrTrackMotionPowerTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:movable_fast-val is deprecated.  Use delphi_esr_msgs-msg:movable_fast instead.")
  (movable_fast m))

(cl:ensure-generic-function 'movable_slow-val :lambda-list '(m))
(cl:defmethod movable_slow-val ((m <EsrTrackMotionPowerTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:movable_slow-val is deprecated.  Use delphi_esr_msgs-msg:movable_slow instead.")
  (movable_slow m))

(cl:ensure-generic-function 'moving-val :lambda-list '(m))
(cl:defmethod moving-val ((m <EsrTrackMotionPowerTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:moving-val is deprecated.  Use delphi_esr_msgs-msg:moving instead.")
  (moving m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <EsrTrackMotionPowerTrack>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:power-val is deprecated.  Use delphi_esr_msgs-msg:power instead.")
  (power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrTrackMotionPowerTrack>) ostream)
  "Serializes a message object of type '<EsrTrackMotionPowerTrack>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'movable_fast) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'movable_slow) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'moving) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'power)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrTrackMotionPowerTrack>) istream)
  "Deserializes a message object of type '<EsrTrackMotionPowerTrack>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'movable_fast) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'movable_slow) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'moving) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'power) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrTrackMotionPowerTrack>)))
  "Returns string type for a message object of type '<EsrTrackMotionPowerTrack>"
  "delphi_esr_msgs/EsrTrackMotionPowerTrack")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrTrackMotionPowerTrack)))
  "Returns string type for a message object of type 'EsrTrackMotionPowerTrack"
  "delphi_esr_msgs/EsrTrackMotionPowerTrack")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrTrackMotionPowerTrack>)))
  "Returns md5sum for a message object of type '<EsrTrackMotionPowerTrack>"
  "3cb7ee3e17f03f833bf47e59a4267646")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrTrackMotionPowerTrack)))
  "Returns md5sum for a message object of type 'EsrTrackMotionPowerTrack"
  "3cb7ee3e17f03f833bf47e59a4267646")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrTrackMotionPowerTrack>)))
  "Returns full string definition for message of type '<EsrTrackMotionPowerTrack>"
  (cl:format cl:nil "# ESR TrackMotionPower, track-specific information~%uint8  id~%bool   movable_fast~%bool   movable_slow~%bool   moving~%int8   power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrTrackMotionPowerTrack)))
  "Returns full string definition for message of type 'EsrTrackMotionPowerTrack"
  (cl:format cl:nil "# ESR TrackMotionPower, track-specific information~%uint8  id~%bool   movable_fast~%bool   movable_slow~%bool   moving~%int8   power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrTrackMotionPowerTrack>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrTrackMotionPowerTrack>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrTrackMotionPowerTrack
    (cl:cons ':id (id msg))
    (cl:cons ':movable_fast (movable_fast msg))
    (cl:cons ':movable_slow (movable_slow msg))
    (cl:cons ':moving (moving msg))
    (cl:cons ':power (power msg))
))
