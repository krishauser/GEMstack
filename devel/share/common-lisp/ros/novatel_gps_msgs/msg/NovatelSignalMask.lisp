; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude NovatelSignalMask.msg.html

(cl:defclass <NovatelSignalMask> (roslisp-msg-protocol:ros-message)
  ((original_mask
    :reader original_mask
    :initarg :original_mask
    :type cl:integer
    :initform 0)
   (gps_L1_used_in_solution
    :reader gps_L1_used_in_solution
    :initarg :gps_L1_used_in_solution
    :type cl:boolean
    :initform cl:nil)
   (gps_L2_used_in_solution
    :reader gps_L2_used_in_solution
    :initarg :gps_L2_used_in_solution
    :type cl:boolean
    :initform cl:nil)
   (gps_L3_used_in_solution
    :reader gps_L3_used_in_solution
    :initarg :gps_L3_used_in_solution
    :type cl:boolean
    :initform cl:nil)
   (glonass_L1_used_in_solution
    :reader glonass_L1_used_in_solution
    :initarg :glonass_L1_used_in_solution
    :type cl:boolean
    :initform cl:nil)
   (glonass_L2_used_in_solution
    :reader glonass_L2_used_in_solution
    :initarg :glonass_L2_used_in_solution
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass NovatelSignalMask (<NovatelSignalMask>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NovatelSignalMask>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NovatelSignalMask)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<NovatelSignalMask> is deprecated: use novatel_gps_msgs-msg:NovatelSignalMask instead.")))

(cl:ensure-generic-function 'original_mask-val :lambda-list '(m))
(cl:defmethod original_mask-val ((m <NovatelSignalMask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:original_mask-val is deprecated.  Use novatel_gps_msgs-msg:original_mask instead.")
  (original_mask m))

(cl:ensure-generic-function 'gps_L1_used_in_solution-val :lambda-list '(m))
(cl:defmethod gps_L1_used_in_solution-val ((m <NovatelSignalMask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:gps_L1_used_in_solution-val is deprecated.  Use novatel_gps_msgs-msg:gps_L1_used_in_solution instead.")
  (gps_L1_used_in_solution m))

(cl:ensure-generic-function 'gps_L2_used_in_solution-val :lambda-list '(m))
(cl:defmethod gps_L2_used_in_solution-val ((m <NovatelSignalMask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:gps_L2_used_in_solution-val is deprecated.  Use novatel_gps_msgs-msg:gps_L2_used_in_solution instead.")
  (gps_L2_used_in_solution m))

(cl:ensure-generic-function 'gps_L3_used_in_solution-val :lambda-list '(m))
(cl:defmethod gps_L3_used_in_solution-val ((m <NovatelSignalMask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:gps_L3_used_in_solution-val is deprecated.  Use novatel_gps_msgs-msg:gps_L3_used_in_solution instead.")
  (gps_L3_used_in_solution m))

(cl:ensure-generic-function 'glonass_L1_used_in_solution-val :lambda-list '(m))
(cl:defmethod glonass_L1_used_in_solution-val ((m <NovatelSignalMask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:glonass_L1_used_in_solution-val is deprecated.  Use novatel_gps_msgs-msg:glonass_L1_used_in_solution instead.")
  (glonass_L1_used_in_solution m))

(cl:ensure-generic-function 'glonass_L2_used_in_solution-val :lambda-list '(m))
(cl:defmethod glonass_L2_used_in_solution-val ((m <NovatelSignalMask>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:glonass_L2_used_in_solution-val is deprecated.  Use novatel_gps_msgs-msg:glonass_L2_used_in_solution instead.")
  (glonass_L2_used_in_solution m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NovatelSignalMask>) ostream)
  "Serializes a message object of type '<NovatelSignalMask>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gps_L1_used_in_solution) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gps_L2_used_in_solution) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gps_L3_used_in_solution) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'glonass_L1_used_in_solution) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'glonass_L2_used_in_solution) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NovatelSignalMask>) istream)
  "Deserializes a message object of type '<NovatelSignalMask>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gps_L1_used_in_solution) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gps_L2_used_in_solution) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gps_L3_used_in_solution) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'glonass_L1_used_in_solution) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'glonass_L2_used_in_solution) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NovatelSignalMask>)))
  "Returns string type for a message object of type '<NovatelSignalMask>"
  "novatel_gps_msgs/NovatelSignalMask")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelSignalMask)))
  "Returns string type for a message object of type 'NovatelSignalMask"
  "novatel_gps_msgs/NovatelSignalMask")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NovatelSignalMask>)))
  "Returns md5sum for a message object of type '<NovatelSignalMask>"
  "007d687355f8f3c12ea4e18109172710")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NovatelSignalMask)))
  "Returns md5sum for a message object of type 'NovatelSignalMask"
  "007d687355f8f3c12ea4e18109172710")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NovatelSignalMask>)))
  "Returns full string definition for message of type '<NovatelSignalMask>"
  (cl:format cl:nil "# Bit    Mask      Description~%#  0     0x01      GPS L1 used in Solution~%#  1     0x02      GPS L2 used in Solution~%#  2     0x04      GPS L5 used in Solution~%#  3     0x08      <Reserved>~%#  4     0x10      GLONASS L1 used in Solution~%#  5     0x20      GLONASS L2 used in Solution~%# 6-7  0x40-0x80   <Reserved>~%uint32 original_mask~%bool gps_L1_used_in_solution~%bool gps_L2_used_in_solution~%bool gps_L3_used_in_solution~%bool glonass_L1_used_in_solution~%bool glonass_L2_used_in_solution~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NovatelSignalMask)))
  "Returns full string definition for message of type 'NovatelSignalMask"
  (cl:format cl:nil "# Bit    Mask      Description~%#  0     0x01      GPS L1 used in Solution~%#  1     0x02      GPS L2 used in Solution~%#  2     0x04      GPS L5 used in Solution~%#  3     0x08      <Reserved>~%#  4     0x10      GLONASS L1 used in Solution~%#  5     0x20      GLONASS L2 used in Solution~%# 6-7  0x40-0x80   <Reserved>~%uint32 original_mask~%bool gps_L1_used_in_solution~%bool gps_L2_used_in_solution~%bool gps_L3_used_in_solution~%bool glonass_L1_used_in_solution~%bool glonass_L2_used_in_solution~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NovatelSignalMask>))
  (cl:+ 0
     4
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NovatelSignalMask>))
  "Converts a ROS message object to a list"
  (cl:list 'NovatelSignalMask
    (cl:cons ':original_mask (original_mask msg))
    (cl:cons ':gps_L1_used_in_solution (gps_L1_used_in_solution msg))
    (cl:cons ':gps_L2_used_in_solution (gps_L2_used_in_solution msg))
    (cl:cons ':gps_L3_used_in_solution (gps_L3_used_in_solution msg))
    (cl:cons ':glonass_L1_used_in_solution (glonass_L1_used_in_solution msg))
    (cl:cons ':glonass_L2_used_in_solution (glonass_L2_used_in_solution msg))
))
