; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude Trackstat.msg.html

(cl:defclass <Trackstat> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (solution_status
    :reader solution_status
    :initarg :solution_status
    :type cl:string
    :initform "")
   (position_type
    :reader position_type
    :initarg :position_type
    :type cl:string
    :initform "")
   (cutoff
    :reader cutoff
    :initarg :cutoff
    :type cl:float
    :initform 0.0)
   (channels
    :reader channels
    :initarg :channels
    :type (cl:vector novatel_gps_msgs-msg:TrackstatChannel)
   :initform (cl:make-array 0 :element-type 'novatel_gps_msgs-msg:TrackstatChannel :initial-element (cl:make-instance 'novatel_gps_msgs-msg:TrackstatChannel))))
)

(cl:defclass Trackstat (<Trackstat>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trackstat>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trackstat)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<Trackstat> is deprecated: use novatel_gps_msgs-msg:Trackstat instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Trackstat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:header-val is deprecated.  Use novatel_gps_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'solution_status-val :lambda-list '(m))
(cl:defmethod solution_status-val ((m <Trackstat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:solution_status-val is deprecated.  Use novatel_gps_msgs-msg:solution_status instead.")
  (solution_status m))

(cl:ensure-generic-function 'position_type-val :lambda-list '(m))
(cl:defmethod position_type-val ((m <Trackstat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:position_type-val is deprecated.  Use novatel_gps_msgs-msg:position_type instead.")
  (position_type m))

(cl:ensure-generic-function 'cutoff-val :lambda-list '(m))
(cl:defmethod cutoff-val ((m <Trackstat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:cutoff-val is deprecated.  Use novatel_gps_msgs-msg:cutoff instead.")
  (cutoff m))

(cl:ensure-generic-function 'channels-val :lambda-list '(m))
(cl:defmethod channels-val ((m <Trackstat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:channels-val is deprecated.  Use novatel_gps_msgs-msg:channels instead.")
  (channels m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trackstat>) ostream)
  "Serializes a message object of type '<Trackstat>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'solution_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'solution_status))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'position_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'position_type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cutoff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'channels))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'channels))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trackstat>) istream)
  "Deserializes a message object of type '<Trackstat>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'solution_status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'solution_status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'position_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cutoff) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'channels) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'channels)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'novatel_gps_msgs-msg:TrackstatChannel))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trackstat>)))
  "Returns string type for a message object of type '<Trackstat>"
  "novatel_gps_msgs/Trackstat")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trackstat)))
  "Returns string type for a message object of type 'Trackstat"
  "novatel_gps_msgs/Trackstat")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trackstat>)))
  "Returns md5sum for a message object of type '<Trackstat>"
  "10e52c1ea54daca4de3c8cdda3a79817")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trackstat)))
  "Returns md5sum for a message object of type 'Trackstat"
  "10e52c1ea54daca4de3c8cdda3a79817")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trackstat>)))
  "Returns full string definition for message of type '<Trackstat>"
  (cl:format cl:nil "# Channel tracking status information for each of the receiver parallel channels~%~%Header header~%~%string solution_status~%string position_type~%~%# Tracking elevation cutff-off angle~%float32 cutoff~%~%TrackstatChannel[] channels~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/TrackstatChannel~%# A submessage of Trackstat that contains all of the data about a single hardware channel~%~%# Satellite PRN number~%int16 prn~%~%# GLONASS Frequency +7~%int16 glofreq~%~%# Channel tracking status~%uint32 ch_tr_status~%~%# Pseudorange (m)~%float64 psr~%~%# Doppler frequency (Hz)~%float32 doppler~%~%# Carrier to noise density ratio (dB-Hz)~%float32 c_no~%~%# Number of seconds of continuous tracking (no cycle slips)~%float32 locktime~%~%# Pseudorange residual from pseudorange filter (m)~%float32 psr_res~%~%# Range reject code from pseudorange filter~%string reject~%~%# Pseudorange filter weighting~%float32 psr_weight~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trackstat)))
  "Returns full string definition for message of type 'Trackstat"
  (cl:format cl:nil "# Channel tracking status information for each of the receiver parallel channels~%~%Header header~%~%string solution_status~%string position_type~%~%# Tracking elevation cutff-off angle~%float32 cutoff~%~%TrackstatChannel[] channels~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: novatel_gps_msgs/TrackstatChannel~%# A submessage of Trackstat that contains all of the data about a single hardware channel~%~%# Satellite PRN number~%int16 prn~%~%# GLONASS Frequency +7~%int16 glofreq~%~%# Channel tracking status~%uint32 ch_tr_status~%~%# Pseudorange (m)~%float64 psr~%~%# Doppler frequency (Hz)~%float32 doppler~%~%# Carrier to noise density ratio (dB-Hz)~%float32 c_no~%~%# Number of seconds of continuous tracking (no cycle slips)~%float32 locktime~%~%# Pseudorange residual from pseudorange filter (m)~%float32 psr_res~%~%# Range reject code from pseudorange filter~%string reject~%~%# Pseudorange filter weighting~%float32 psr_weight~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trackstat>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'solution_status))
     4 (cl:length (cl:slot-value msg 'position_type))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'channels) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trackstat>))
  "Converts a ROS message object to a list"
  (cl:list 'Trackstat
    (cl:cons ':header (header msg))
    (cl:cons ':solution_status (solution_status msg))
    (cl:cons ':position_type (position_type msg))
    (cl:cons ':cutoff (cutoff msg))
    (cl:cons ':channels (channels msg))
))
