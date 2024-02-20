; Auto-generated. Do not edit!


(cl:in-package neobotix_usboard_msgs-msg)


;//! \htmlinclude AnsToCmdConnect.msg.html

(cl:defclass <AnsToCmdConnect> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d7
    :reader cmd_connect_ans_d7
    :initarg :cmd_connect_ans_d7
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d6
    :reader cmd_connect_ans_d6
    :initarg :cmd_connect_ans_d6
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d5
    :reader cmd_connect_ans_d5
    :initarg :cmd_connect_ans_d5
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d4
    :reader cmd_connect_ans_d4
    :initarg :cmd_connect_ans_d4
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d3
    :reader cmd_connect_ans_d3
    :initarg :cmd_connect_ans_d3
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d2
    :reader cmd_connect_ans_d2
    :initarg :cmd_connect_ans_d2
    :type cl:fixnum
    :initform 0)
   (cmd_connect_ans_d1
    :reader cmd_connect_ans_d1
    :initarg :cmd_connect_ans_d1
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnsToCmdConnect (<AnsToCmdConnect>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnsToCmdConnect>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnsToCmdConnect)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neobotix_usboard_msgs-msg:<AnsToCmdConnect> is deprecated: use neobotix_usboard_msgs-msg:AnsToCmdConnect instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:header-val is deprecated.  Use neobotix_usboard_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:command-val is deprecated.  Use neobotix_usboard_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'cmd_connect_ans_d7-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d7-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d7-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d7 instead.")
  (cmd_connect_ans_d7 m))

(cl:ensure-generic-function 'cmd_connect_ans_d6-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d6-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d6-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d6 instead.")
  (cmd_connect_ans_d6 m))

(cl:ensure-generic-function 'cmd_connect_ans_d5-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d5-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d5-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d5 instead.")
  (cmd_connect_ans_d5 m))

(cl:ensure-generic-function 'cmd_connect_ans_d4-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d4-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d4-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d4 instead.")
  (cmd_connect_ans_d4 m))

(cl:ensure-generic-function 'cmd_connect_ans_d3-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d3-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d3-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d3 instead.")
  (cmd_connect_ans_d3 m))

(cl:ensure-generic-function 'cmd_connect_ans_d2-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d2-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d2-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d2 instead.")
  (cmd_connect_ans_d2 m))

(cl:ensure-generic-function 'cmd_connect_ans_d1-val :lambda-list '(m))
(cl:defmethod cmd_connect_ans_d1-val ((m <AnsToCmdConnect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:cmd_connect_ans_d1-val is deprecated.  Use neobotix_usboard_msgs-msg:cmd_connect_ans_d1 instead.")
  (cmd_connect_ans_d1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnsToCmdConnect>) ostream)
  "Serializes a message object of type '<AnsToCmdConnect>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d7)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d1)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnsToCmdConnect>) istream)
  "Deserializes a message object of type '<AnsToCmdConnect>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d7)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd_connect_ans_d1)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnsToCmdConnect>)))
  "Returns string type for a message object of type '<AnsToCmdConnect>"
  "neobotix_usboard_msgs/AnsToCmdConnect")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnsToCmdConnect)))
  "Returns string type for a message object of type 'AnsToCmdConnect"
  "neobotix_usboard_msgs/AnsToCmdConnect")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnsToCmdConnect>)))
  "Returns md5sum for a message object of type '<AnsToCmdConnect>"
  "d1ef60b13020f0e599d4fbb33f17b3f1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnsToCmdConnect)))
  "Returns md5sum for a message object of type 'AnsToCmdConnect"
  "d1ef60b13020f0e599d4fbb33f17b3f1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnsToCmdConnect>)))
  "Returns full string definition for message of type '<AnsToCmdConnect>"
  (cl:format cl:nil "# Message file for AnsToCmdConnect~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     cmd_connect_ans_d7                      ~%uint8     cmd_connect_ans_d6                      ~%uint8     cmd_connect_ans_d5                      ~%uint8     cmd_connect_ans_d4                      ~%uint8     cmd_connect_ans_d3                      ~%uint8     cmd_connect_ans_d2                      ~%uint8     cmd_connect_ans_d1                      ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnsToCmdConnect)))
  "Returns full string definition for message of type 'AnsToCmdConnect"
  (cl:format cl:nil "# Message file for AnsToCmdConnect~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     cmd_connect_ans_d7                      ~%uint8     cmd_connect_ans_d6                      ~%uint8     cmd_connect_ans_d5                      ~%uint8     cmd_connect_ans_d4                      ~%uint8     cmd_connect_ans_d3                      ~%uint8     cmd_connect_ans_d2                      ~%uint8     cmd_connect_ans_d1                      ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnsToCmdConnect>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnsToCmdConnect>))
  "Converts a ROS message object to a list"
  (cl:list 'AnsToCmdConnect
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':cmd_connect_ans_d7 (cmd_connect_ans_d7 msg))
    (cl:cons ':cmd_connect_ans_d6 (cmd_connect_ans_d6 msg))
    (cl:cons ':cmd_connect_ans_d5 (cmd_connect_ans_d5 msg))
    (cl:cons ':cmd_connect_ans_d4 (cmd_connect_ans_d4 msg))
    (cl:cons ':cmd_connect_ans_d3 (cmd_connect_ans_d3 msg))
    (cl:cons ':cmd_connect_ans_d2 (cmd_connect_ans_d2 msg))
    (cl:cons ':cmd_connect_ans_d1 (cmd_connect_ans_d1 msg))
))
