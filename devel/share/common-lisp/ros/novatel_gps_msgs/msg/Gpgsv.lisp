; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude Gpgsv.msg.html

(cl:defclass <Gpgsv> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (message_id
    :reader message_id
    :initarg :message_id
    :type cl:string
    :initform "")
   (n_msgs
    :reader n_msgs
    :initarg :n_msgs
    :type cl:fixnum
    :initform 0)
   (msg_number
    :reader msg_number
    :initarg :msg_number
    :type cl:fixnum
    :initform 0)
   (n_satellites
    :reader n_satellites
    :initarg :n_satellites
    :type cl:fixnum
    :initform 0)
   (satellites
    :reader satellites
    :initarg :satellites
    :type (cl:vector novatel_gps_msgs-msg:Satellite)
   :initform (cl:make-array 0 :element-type 'novatel_gps_msgs-msg:Satellite :initial-element (cl:make-instance 'novatel_gps_msgs-msg:Satellite))))
)

(cl:defclass Gpgsv (<Gpgsv>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gpgsv>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gpgsv)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<Gpgsv> is deprecated: use novatel_gps_msgs-msg:Gpgsv instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Gpgsv>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:header-val is deprecated.  Use novatel_gps_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'message_id-val :lambda-list '(m))
(cl:defmethod message_id-val ((m <Gpgsv>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:message_id-val is deprecated.  Use novatel_gps_msgs-msg:message_id instead.")
  (message_id m))

(cl:ensure-generic-function 'n_msgs-val :lambda-list '(m))
(cl:defmethod n_msgs-val ((m <Gpgsv>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:n_msgs-val is deprecated.  Use novatel_gps_msgs-msg:n_msgs instead.")
  (n_msgs m))

(cl:ensure-generic-function 'msg_number-val :lambda-list '(m))
(cl:defmethod msg_number-val ((m <Gpgsv>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:msg_number-val is deprecated.  Use novatel_gps_msgs-msg:msg_number instead.")
  (msg_number m))

(cl:ensure-generic-function 'n_satellites-val :lambda-list '(m))
(cl:defmethod n_satellites-val ((m <Gpgsv>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:n_satellites-val is deprecated.  Use novatel_gps_msgs-msg:n_satellites instead.")
  (n_satellites m))

(cl:ensure-generic-function 'satellites-val :lambda-list '(m))
(cl:defmethod satellites-val ((m <Gpgsv>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:satellites-val is deprecated.  Use novatel_gps_msgs-msg:satellites instead.")
  (satellites m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gpgsv>) ostream)
  "Serializes a message object of type '<Gpgsv>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_msgs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_satellites)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'satellites))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'satellites))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gpgsv>) istream)
  "Deserializes a message object of type '<Gpgsv>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_msgs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_satellites)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'satellites) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'satellites)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'novatel_gps_msgs-msg:Satellite))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gpgsv>)))
  "Returns string type for a message object of type '<Gpgsv>"
  "novatel_gps_msgs/Gpgsv")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gpgsv)))
  "Returns string type for a message object of type 'Gpgsv"
  "novatel_gps_msgs/Gpgsv")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gpgsv>)))
  "Returns md5sum for a message object of type '<Gpgsv>"
  "6f34bebc32fe085313c942a96fd39c77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gpgsv)))
  "Returns md5sum for a message object of type 'Gpgsv"
  "6f34bebc32fe085313c942a96fd39c77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gpgsv>)))
  "Returns full string definition for message of type '<Gpgsv>"
  (cl:format cl:nil "# Total number of satellites in view and data about satellites~%# Because the NMEA sentence is limited to 4 satellites per message, several~%# of these messages may need to be synthesized to get data about all visible~%# satellites.~%~%Header header~%~%string message_id~%~%# Number of messages in this sequence~%uint8 n_msgs~%# This messages number in its sequence. The first message is number 1.~%uint8 msg_number~%~%# Number of satellites currently visible~%uint8 n_satellites~%~%# Up to 4 satellites are described in each message~%Satellite[] satellites~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/Satellite~%# Satellite data structure used in GPGSV messages~%~%# PRN number of the satellite~%# GPS = 1..32~%# SBAS = 33..64~%# GLO = 65..96~%uint8 prn~%~%# Elevation, degrees. Maximum 90~%uint8 elevation~%~%# Azimuth, True North degrees. [0, 359]~%uint16 azimuth~%~%# Signal to noise ratio, 0-99 dB. -1 when null in NMEA sentence (not tracking)~%int8 snr~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gpgsv)))
  "Returns full string definition for message of type 'Gpgsv"
  (cl:format cl:nil "# Total number of satellites in view and data about satellites~%# Because the NMEA sentence is limited to 4 satellites per message, several~%# of these messages may need to be synthesized to get data about all visible~%# satellites.~%~%Header header~%~%string message_id~%~%# Number of messages in this sequence~%uint8 n_msgs~%# This messages number in its sequence. The first message is number 1.~%uint8 msg_number~%~%# Number of satellites currently visible~%uint8 n_satellites~%~%# Up to 4 satellites are described in each message~%Satellite[] satellites~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/Satellite~%# Satellite data structure used in GPGSV messages~%~%# PRN number of the satellite~%# GPS = 1..32~%# SBAS = 33..64~%# GLO = 65..96~%uint8 prn~%~%# Elevation, degrees. Maximum 90~%uint8 elevation~%~%# Azimuth, True North degrees. [0, 359]~%uint16 azimuth~%~%# Signal to noise ratio, 0-99 dB. -1 when null in NMEA sentence (not tracking)~%int8 snr~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gpgsv>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'message_id))
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'satellites) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gpgsv>))
  "Converts a ROS message object to a list"
  (cl:list 'Gpgsv
    (cl:cons ':header (header msg))
    (cl:cons ':message_id (message_id msg))
    (cl:cons ':n_msgs (n_msgs msg))
    (cl:cons ':msg_number (msg_number msg))
    (cl:cons ':n_satellites (n_satellites msg))
    (cl:cons ':satellites (satellites msg))
))
