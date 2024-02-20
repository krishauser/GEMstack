; Auto-generated. Do not edit!


(cl:in-package delphi_esr_msgs-msg)


;//! \htmlinclude EsrTrackMotionPowerGroup.msg.html

(cl:defclass <EsrTrackMotionPowerGroup> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (canmsg
    :reader canmsg
    :initarg :canmsg
    :type cl:string
    :initform "")
   (rolling_count_2
    :reader rolling_count_2
    :initarg :rolling_count_2
    :type cl:fixnum
    :initform 0)
   (can_id_group
    :reader can_id_group
    :initarg :can_id_group
    :type cl:fixnum
    :initform 0)
   (tracks
    :reader tracks
    :initarg :tracks
    :type (cl:vector delphi_esr_msgs-msg:EsrTrackMotionPowerTrack)
   :initform (cl:make-array 0 :element-type 'delphi_esr_msgs-msg:EsrTrackMotionPowerTrack :initial-element (cl:make-instance 'delphi_esr_msgs-msg:EsrTrackMotionPowerTrack))))
)

(cl:defclass EsrTrackMotionPowerGroup (<EsrTrackMotionPowerGroup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EsrTrackMotionPowerGroup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EsrTrackMotionPowerGroup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_esr_msgs-msg:<EsrTrackMotionPowerGroup> is deprecated: use delphi_esr_msgs-msg:EsrTrackMotionPowerGroup instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EsrTrackMotionPowerGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:header-val is deprecated.  Use delphi_esr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'canmsg-val :lambda-list '(m))
(cl:defmethod canmsg-val ((m <EsrTrackMotionPowerGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:canmsg-val is deprecated.  Use delphi_esr_msgs-msg:canmsg instead.")
  (canmsg m))

(cl:ensure-generic-function 'rolling_count_2-val :lambda-list '(m))
(cl:defmethod rolling_count_2-val ((m <EsrTrackMotionPowerGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:rolling_count_2-val is deprecated.  Use delphi_esr_msgs-msg:rolling_count_2 instead.")
  (rolling_count_2 m))

(cl:ensure-generic-function 'can_id_group-val :lambda-list '(m))
(cl:defmethod can_id_group-val ((m <EsrTrackMotionPowerGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:can_id_group-val is deprecated.  Use delphi_esr_msgs-msg:can_id_group instead.")
  (can_id_group m))

(cl:ensure-generic-function 'tracks-val :lambda-list '(m))
(cl:defmethod tracks-val ((m <EsrTrackMotionPowerGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_esr_msgs-msg:tracks-val is deprecated.  Use delphi_esr_msgs-msg:tracks instead.")
  (tracks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EsrTrackMotionPowerGroup>) ostream)
  "Serializes a message object of type '<EsrTrackMotionPowerGroup>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'canmsg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'canmsg))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_id_group)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EsrTrackMotionPowerGroup>) istream)
  "Deserializes a message object of type '<EsrTrackMotionPowerGroup>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'canmsg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'canmsg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rolling_count_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_id_group)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'delphi_esr_msgs-msg:EsrTrackMotionPowerTrack))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EsrTrackMotionPowerGroup>)))
  "Returns string type for a message object of type '<EsrTrackMotionPowerGroup>"
  "delphi_esr_msgs/EsrTrackMotionPowerGroup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EsrTrackMotionPowerGroup)))
  "Returns string type for a message object of type 'EsrTrackMotionPowerGroup"
  "delphi_esr_msgs/EsrTrackMotionPowerGroup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EsrTrackMotionPowerGroup>)))
  "Returns md5sum for a message object of type '<EsrTrackMotionPowerGroup>"
  "58598630b679d4a2eed0f058be9b1aaa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EsrTrackMotionPowerGroup)))
  "Returns md5sum for a message object of type 'EsrTrackMotionPowerGroup"
  "58598630b679d4a2eed0f058be9b1aaa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EsrTrackMotionPowerGroup>)))
  "Returns full string definition for message of type '<EsrTrackMotionPowerGroup>"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR TrackMotionPower, information common to a group~%string                                      canmsg~%~%uint8                                       rolling_count_2~%uint8                                       can_id_group~%delphi_esr_msgs/EsrTrackMotionPowerTrack[]  tracks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: delphi_esr_msgs/EsrTrackMotionPowerTrack~%# ESR TrackMotionPower, track-specific information~%uint8  id~%bool   movable_fast~%bool   movable_slow~%bool   moving~%int8   power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EsrTrackMotionPowerGroup)))
  "Returns full string definition for message of type 'EsrTrackMotionPowerGroup"
  (cl:format cl:nil "std_msgs/Header header~%~%# ESR TrackMotionPower, information common to a group~%string                                      canmsg~%~%uint8                                       rolling_count_2~%uint8                                       can_id_group~%delphi_esr_msgs/EsrTrackMotionPowerTrack[]  tracks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: delphi_esr_msgs/EsrTrackMotionPowerTrack~%# ESR TrackMotionPower, track-specific information~%uint8  id~%bool   movable_fast~%bool   movable_slow~%bool   moving~%int8   power~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EsrTrackMotionPowerGroup>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'canmsg))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EsrTrackMotionPowerGroup>))
  "Converts a ROS message object to a list"
  (cl:list 'EsrTrackMotionPowerGroup
    (cl:cons ':header (header msg))
    (cl:cons ':canmsg (canmsg msg))
    (cl:cons ':rolling_count_2 (rolling_count_2 msg))
    (cl:cons ':can_id_group (can_id_group msg))
    (cl:cons ':tracks (tracks msg))
))
