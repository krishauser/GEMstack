; Auto-generated. Do not edit!


(cl:in-package neobotix_usboard_msgs-msg)


;//! \htmlinclude AnalogIn.msg.html

(cl:defclass <AnalogIn> (roslisp-msg-protocol:ros-message)
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
   (analog_data_ch4_low_byte
    :reader analog_data_ch4_low_byte
    :initarg :analog_data_ch4_low_byte
    :type cl:fixnum
    :initform 0)
   (analog_data_ch4_high_bits
    :reader analog_data_ch4_high_bits
    :initarg :analog_data_ch4_high_bits
    :type cl:fixnum
    :initform 0)
   (analog_data_ch3_low_byte
    :reader analog_data_ch3_low_byte
    :initarg :analog_data_ch3_low_byte
    :type cl:fixnum
    :initform 0)
   (analog_data_ch3_high_bits
    :reader analog_data_ch3_high_bits
    :initarg :analog_data_ch3_high_bits
    :type cl:fixnum
    :initform 0)
   (analog_data_ch2_low_byte
    :reader analog_data_ch2_low_byte
    :initarg :analog_data_ch2_low_byte
    :type cl:fixnum
    :initform 0)
   (analog_data_ch2_high_bits
    :reader analog_data_ch2_high_bits
    :initarg :analog_data_ch2_high_bits
    :type cl:fixnum
    :initform 0)
   (analog_data_ch1_low_byte
    :reader analog_data_ch1_low_byte
    :initarg :analog_data_ch1_low_byte
    :type cl:fixnum
    :initform 0)
   (analog_data_ch1_high_bits
    :reader analog_data_ch1_high_bits
    :initarg :analog_data_ch1_high_bits
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AnalogIn (<AnalogIn>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AnalogIn>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AnalogIn)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name neobotix_usboard_msgs-msg:<AnalogIn> is deprecated: use neobotix_usboard_msgs-msg:AnalogIn instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:header-val is deprecated.  Use neobotix_usboard_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:command-val is deprecated.  Use neobotix_usboard_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'analog_data_ch4_low_byte-val :lambda-list '(m))
(cl:defmethod analog_data_ch4_low_byte-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch4_low_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch4_low_byte instead.")
  (analog_data_ch4_low_byte m))

(cl:ensure-generic-function 'analog_data_ch4_high_bits-val :lambda-list '(m))
(cl:defmethod analog_data_ch4_high_bits-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch4_high_bits-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch4_high_bits instead.")
  (analog_data_ch4_high_bits m))

(cl:ensure-generic-function 'analog_data_ch3_low_byte-val :lambda-list '(m))
(cl:defmethod analog_data_ch3_low_byte-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch3_low_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch3_low_byte instead.")
  (analog_data_ch3_low_byte m))

(cl:ensure-generic-function 'analog_data_ch3_high_bits-val :lambda-list '(m))
(cl:defmethod analog_data_ch3_high_bits-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch3_high_bits-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch3_high_bits instead.")
  (analog_data_ch3_high_bits m))

(cl:ensure-generic-function 'analog_data_ch2_low_byte-val :lambda-list '(m))
(cl:defmethod analog_data_ch2_low_byte-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch2_low_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch2_low_byte instead.")
  (analog_data_ch2_low_byte m))

(cl:ensure-generic-function 'analog_data_ch2_high_bits-val :lambda-list '(m))
(cl:defmethod analog_data_ch2_high_bits-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch2_high_bits-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch2_high_bits instead.")
  (analog_data_ch2_high_bits m))

(cl:ensure-generic-function 'analog_data_ch1_low_byte-val :lambda-list '(m))
(cl:defmethod analog_data_ch1_low_byte-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch1_low_byte-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch1_low_byte instead.")
  (analog_data_ch1_low_byte m))

(cl:ensure-generic-function 'analog_data_ch1_high_bits-val :lambda-list '(m))
(cl:defmethod analog_data_ch1_high_bits-val ((m <AnalogIn>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader neobotix_usboard_msgs-msg:analog_data_ch1_high_bits-val is deprecated.  Use neobotix_usboard_msgs-msg:analog_data_ch1_high_bits instead.")
  (analog_data_ch1_high_bits m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AnalogIn>) ostream)
  "Serializes a message object of type '<AnalogIn>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch4_low_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch4_high_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch3_low_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch3_high_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch2_low_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch2_high_bits)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch1_low_byte)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch1_high_bits)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AnalogIn>) istream)
  "Deserializes a message object of type '<AnalogIn>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch4_low_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch4_high_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch3_low_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch3_high_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch2_low_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch2_high_bits)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch1_low_byte)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'analog_data_ch1_high_bits)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AnalogIn>)))
  "Returns string type for a message object of type '<AnalogIn>"
  "neobotix_usboard_msgs/AnalogIn")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AnalogIn)))
  "Returns string type for a message object of type 'AnalogIn"
  "neobotix_usboard_msgs/AnalogIn")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AnalogIn>)))
  "Returns md5sum for a message object of type '<AnalogIn>"
  "619eac438aa01d7a05701049ea57be6e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AnalogIn)))
  "Returns md5sum for a message object of type 'AnalogIn"
  "619eac438aa01d7a05701049ea57be6e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AnalogIn>)))
  "Returns full string definition for message of type '<AnalogIn>"
  (cl:format cl:nil "# Message file for AnalogIn~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     analog_data_ch4_low_byte                ~%uint8     analog_data_ch4_high_bits               ~%uint8     analog_data_ch3_low_byte                ~%uint8     analog_data_ch3_high_bits               ~%uint8     analog_data_ch2_low_byte                ~%uint8     analog_data_ch2_high_bits               ~%uint8     analog_data_ch1_low_byte                ~%uint8     analog_data_ch1_high_bits               ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AnalogIn)))
  "Returns full string definition for message of type 'AnalogIn"
  (cl:format cl:nil "# Message file for AnalogIn~%~%std_msgs/Header header~%~%uint8     command                                 ~%uint8     analog_data_ch4_low_byte                ~%uint8     analog_data_ch4_high_bits               ~%uint8     analog_data_ch3_low_byte                ~%uint8     analog_data_ch3_high_bits               ~%uint8     analog_data_ch2_low_byte                ~%uint8     analog_data_ch2_high_bits               ~%uint8     analog_data_ch1_low_byte                ~%uint8     analog_data_ch1_high_bits               ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AnalogIn>))
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
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AnalogIn>))
  "Converts a ROS message object to a list"
  (cl:list 'AnalogIn
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':analog_data_ch4_low_byte (analog_data_ch4_low_byte msg))
    (cl:cons ':analog_data_ch4_high_bits (analog_data_ch4_high_bits msg))
    (cl:cons ':analog_data_ch3_low_byte (analog_data_ch3_low_byte msg))
    (cl:cons ':analog_data_ch3_high_bits (analog_data_ch3_high_bits msg))
    (cl:cons ':analog_data_ch2_low_byte (analog_data_ch2_low_byte msg))
    (cl:cons ':analog_data_ch2_high_bits (analog_data_ch2_high_bits msg))
    (cl:cons ':analog_data_ch1_low_byte (analog_data_ch1_low_byte msg))
    (cl:cons ':analog_data_ch1_high_bits (analog_data_ch1_high_bits msg))
))
