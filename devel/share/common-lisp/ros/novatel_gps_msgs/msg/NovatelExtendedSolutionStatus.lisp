; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude NovatelExtendedSolutionStatus.msg.html

(cl:defclass <NovatelExtendedSolutionStatus> (roslisp-msg-protocol:ros-message)
  ((original_mask
    :reader original_mask
    :initarg :original_mask
    :type cl:integer
    :initform 0)
   (advance_rtk_verified
    :reader advance_rtk_verified
    :initarg :advance_rtk_verified
    :type cl:boolean
    :initform cl:nil)
   (psuedorange_iono_correction
    :reader psuedorange_iono_correction
    :initarg :psuedorange_iono_correction
    :type cl:string
    :initform ""))
)

(cl:defclass NovatelExtendedSolutionStatus (<NovatelExtendedSolutionStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NovatelExtendedSolutionStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NovatelExtendedSolutionStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<NovatelExtendedSolutionStatus> is deprecated: use novatel_gps_msgs-msg:NovatelExtendedSolutionStatus instead.")))

(cl:ensure-generic-function 'original_mask-val :lambda-list '(m))
(cl:defmethod original_mask-val ((m <NovatelExtendedSolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:original_mask-val is deprecated.  Use novatel_gps_msgs-msg:original_mask instead.")
  (original_mask m))

(cl:ensure-generic-function 'advance_rtk_verified-val :lambda-list '(m))
(cl:defmethod advance_rtk_verified-val ((m <NovatelExtendedSolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:advance_rtk_verified-val is deprecated.  Use novatel_gps_msgs-msg:advance_rtk_verified instead.")
  (advance_rtk_verified m))

(cl:ensure-generic-function 'psuedorange_iono_correction-val :lambda-list '(m))
(cl:defmethod psuedorange_iono_correction-val ((m <NovatelExtendedSolutionStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:psuedorange_iono_correction-val is deprecated.  Use novatel_gps_msgs-msg:psuedorange_iono_correction instead.")
  (psuedorange_iono_correction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NovatelExtendedSolutionStatus>) ostream)
  "Serializes a message object of type '<NovatelExtendedSolutionStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'original_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'advance_rtk_verified) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'psuedorange_iono_correction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'psuedorange_iono_correction))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NovatelExtendedSolutionStatus>) istream)
  "Deserializes a message object of type '<NovatelExtendedSolutionStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'original_mask)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'advance_rtk_verified) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'psuedorange_iono_correction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'psuedorange_iono_correction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NovatelExtendedSolutionStatus>)))
  "Returns string type for a message object of type '<NovatelExtendedSolutionStatus>"
  "novatel_gps_msgs/NovatelExtendedSolutionStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelExtendedSolutionStatus)))
  "Returns string type for a message object of type 'NovatelExtendedSolutionStatus"
  "novatel_gps_msgs/NovatelExtendedSolutionStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NovatelExtendedSolutionStatus>)))
  "Returns md5sum for a message object of type '<NovatelExtendedSolutionStatus>"
  "f0e19d53094c207c4dafdfbde750c4b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NovatelExtendedSolutionStatus)))
  "Returns md5sum for a message object of type 'NovatelExtendedSolutionStatus"
  "f0e19d53094c207c4dafdfbde750c4b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NovatelExtendedSolutionStatus>)))
  "Returns full string definition for message of type '<NovatelExtendedSolutionStatus>"
  (cl:format cl:nil "# Bit    Mask      Description~%#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)~%# 1-3    0x0E      Pseudorange Ionosphere Correction~%#                    0 = unknown~%#                    1 = Klobuchar Broadcast~%#                    2 = SBAS Broadcast~%#                    3 = Multi-frequency Computed~%#                    4 = PSRDiff Correction~%#                    5 = NovaTel Blended Ionosphere Value~%# 4-7  0xF0        <Reserved>~%uint32 original_mask~%bool advance_rtk_verified~%string psuedorange_iono_correction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NovatelExtendedSolutionStatus)))
  "Returns full string definition for message of type 'NovatelExtendedSolutionStatus"
  (cl:format cl:nil "# Bit    Mask      Description~%#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)~%# 1-3    0x0E      Pseudorange Ionosphere Correction~%#                    0 = unknown~%#                    1 = Klobuchar Broadcast~%#                    2 = SBAS Broadcast~%#                    3 = Multi-frequency Computed~%#                    4 = PSRDiff Correction~%#                    5 = NovaTel Blended Ionosphere Value~%# 4-7  0xF0        <Reserved>~%uint32 original_mask~%bool advance_rtk_verified~%string psuedorange_iono_correction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NovatelExtendedSolutionStatus>))
  (cl:+ 0
     4
     1
     4 (cl:length (cl:slot-value msg 'psuedorange_iono_correction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NovatelExtendedSolutionStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'NovatelExtendedSolutionStatus
    (cl:cons ':original_mask (original_mask msg))
    (cl:cons ':advance_rtk_verified (advance_rtk_verified msg))
    (cl:cons ':psuedorange_iono_correction (psuedorange_iono_correction msg))
))
