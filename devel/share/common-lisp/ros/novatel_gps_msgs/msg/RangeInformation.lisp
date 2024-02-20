; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude RangeInformation.msg.html

(cl:defclass <RangeInformation> (roslisp-msg-protocol:ros-message)
  ((prn_number
    :reader prn_number
    :initarg :prn_number
    :type cl:fixnum
    :initform 0)
   (glofreq
    :reader glofreq
    :initarg :glofreq
    :type cl:fixnum
    :initform 0)
   (psr
    :reader psr
    :initarg :psr
    :type cl:float
    :initform 0.0)
   (psr_std
    :reader psr_std
    :initarg :psr_std
    :type cl:float
    :initform 0.0)
   (adr
    :reader adr
    :initarg :adr
    :type cl:float
    :initform 0.0)
   (adr_std
    :reader adr_std
    :initarg :adr_std
    :type cl:float
    :initform 0.0)
   (dopp
    :reader dopp
    :initarg :dopp
    :type cl:float
    :initform 0.0)
   (noise_density_ratio
    :reader noise_density_ratio
    :initarg :noise_density_ratio
    :type cl:float
    :initform 0.0)
   (locktime
    :reader locktime
    :initarg :locktime
    :type cl:float
    :initform 0.0)
   (tracking_status
    :reader tracking_status
    :initarg :tracking_status
    :type cl:integer
    :initform 0))
)

(cl:defclass RangeInformation (<RangeInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RangeInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RangeInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<RangeInformation> is deprecated: use novatel_gps_msgs-msg:RangeInformation instead.")))

(cl:ensure-generic-function 'prn_number-val :lambda-list '(m))
(cl:defmethod prn_number-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:prn_number-val is deprecated.  Use novatel_gps_msgs-msg:prn_number instead.")
  (prn_number m))

(cl:ensure-generic-function 'glofreq-val :lambda-list '(m))
(cl:defmethod glofreq-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:glofreq-val is deprecated.  Use novatel_gps_msgs-msg:glofreq instead.")
  (glofreq m))

(cl:ensure-generic-function 'psr-val :lambda-list '(m))
(cl:defmethod psr-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:psr-val is deprecated.  Use novatel_gps_msgs-msg:psr instead.")
  (psr m))

(cl:ensure-generic-function 'psr_std-val :lambda-list '(m))
(cl:defmethod psr_std-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:psr_std-val is deprecated.  Use novatel_gps_msgs-msg:psr_std instead.")
  (psr_std m))

(cl:ensure-generic-function 'adr-val :lambda-list '(m))
(cl:defmethod adr-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:adr-val is deprecated.  Use novatel_gps_msgs-msg:adr instead.")
  (adr m))

(cl:ensure-generic-function 'adr_std-val :lambda-list '(m))
(cl:defmethod adr_std-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:adr_std-val is deprecated.  Use novatel_gps_msgs-msg:adr_std instead.")
  (adr_std m))

(cl:ensure-generic-function 'dopp-val :lambda-list '(m))
(cl:defmethod dopp-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:dopp-val is deprecated.  Use novatel_gps_msgs-msg:dopp instead.")
  (dopp m))

(cl:ensure-generic-function 'noise_density_ratio-val :lambda-list '(m))
(cl:defmethod noise_density_ratio-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:noise_density_ratio-val is deprecated.  Use novatel_gps_msgs-msg:noise_density_ratio instead.")
  (noise_density_ratio m))

(cl:ensure-generic-function 'locktime-val :lambda-list '(m))
(cl:defmethod locktime-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:locktime-val is deprecated.  Use novatel_gps_msgs-msg:locktime instead.")
  (locktime m))

(cl:ensure-generic-function 'tracking_status-val :lambda-list '(m))
(cl:defmethod tracking_status-val ((m <RangeInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:tracking_status-val is deprecated.  Use novatel_gps_msgs-msg:tracking_status instead.")
  (tracking_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RangeInformation>) ostream)
  "Serializes a message object of type '<RangeInformation>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'prn_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'prn_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'glofreq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'glofreq)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'psr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psr_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'adr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'adr_std))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dopp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'noise_density_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'locktime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tracking_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tracking_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tracking_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tracking_status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RangeInformation>) istream)
  "Deserializes a message object of type '<RangeInformation>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'prn_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'prn_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'glofreq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'glofreq)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psr_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'adr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'adr_std) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dopp) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'noise_density_ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'locktime) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tracking_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tracking_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'tracking_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'tracking_status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RangeInformation>)))
  "Returns string type for a message object of type '<RangeInformation>"
  "novatel_gps_msgs/RangeInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RangeInformation)))
  "Returns string type for a message object of type 'RangeInformation"
  "novatel_gps_msgs/RangeInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RangeInformation>)))
  "Returns md5sum for a message object of type '<RangeInformation>"
  "2c29299d245fc707e8f7544af871f110")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RangeInformation)))
  "Returns md5sum for a message object of type 'RangeInformation"
  "2c29299d245fc707e8f7544af871f110")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RangeInformation>)))
  "Returns full string definition for message of type '<RangeInformation>"
  (cl:format cl:nil "#Satellite Range information structure used in range messages~%~%#Satelite PRN number of range measurement~%uint16 prn_number~%~%#GLONASS Frequency~%uint16 glofreq~%~%#Pseudorange measurement(m)~%float64 psr~%~%#Pseudorange measurement standard deviation(m)~%float32 psr_std~%~%#Carrier phase, in cycles~%float64 adr~%~%#Estimated carrier phase standard deviation(cycles)~%float32 adr_std~%~%#Instantaneous carrier Doppler frequency(Hz)~%float32 dopp~%~%#Carrier to noise density ratio~%float32 noise_density_ratio~%~%## of seconds of continous tracking~%float32 locktime~%~%#Tracking status~%uint32 tracking_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RangeInformation)))
  "Returns full string definition for message of type 'RangeInformation"
  (cl:format cl:nil "#Satellite Range information structure used in range messages~%~%#Satelite PRN number of range measurement~%uint16 prn_number~%~%#GLONASS Frequency~%uint16 glofreq~%~%#Pseudorange measurement(m)~%float64 psr~%~%#Pseudorange measurement standard deviation(m)~%float32 psr_std~%~%#Carrier phase, in cycles~%float64 adr~%~%#Estimated carrier phase standard deviation(cycles)~%float32 adr_std~%~%#Instantaneous carrier Doppler frequency(Hz)~%float32 dopp~%~%#Carrier to noise density ratio~%float32 noise_density_ratio~%~%## of seconds of continous tracking~%float32 locktime~%~%#Tracking status~%uint32 tracking_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RangeInformation>))
  (cl:+ 0
     2
     2
     8
     4
     8
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RangeInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'RangeInformation
    (cl:cons ':prn_number (prn_number msg))
    (cl:cons ':glofreq (glofreq msg))
    (cl:cons ':psr (psr msg))
    (cl:cons ':psr_std (psr_std msg))
    (cl:cons ':adr (adr msg))
    (cl:cons ':adr_std (adr_std msg))
    (cl:cons ':dopp (dopp msg))
    (cl:cons ':noise_density_ratio (noise_density_ratio msg))
    (cl:cons ':locktime (locktime msg))
    (cl:cons ':tracking_status (tracking_status msg))
))
