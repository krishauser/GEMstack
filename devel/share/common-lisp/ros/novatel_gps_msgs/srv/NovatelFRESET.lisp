; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-srv)


;//! \htmlinclude NovatelFRESET-request.msg.html

(cl:defclass <NovatelFRESET-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type cl:string
    :initform ""))
)

(cl:defclass NovatelFRESET-request (<NovatelFRESET-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NovatelFRESET-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NovatelFRESET-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-srv:<NovatelFRESET-request> is deprecated: use novatel_gps_msgs-srv:NovatelFRESET-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <NovatelFRESET-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-srv:target-val is deprecated.  Use novatel_gps_msgs-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NovatelFRESET-request>) ostream)
  "Serializes a message object of type '<NovatelFRESET-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NovatelFRESET-request>) istream)
  "Deserializes a message object of type '<NovatelFRESET-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NovatelFRESET-request>)))
  "Returns string type for a service object of type '<NovatelFRESET-request>"
  "novatel_gps_msgs/NovatelFRESETRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelFRESET-request)))
  "Returns string type for a service object of type 'NovatelFRESET-request"
  "novatel_gps_msgs/NovatelFRESETRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NovatelFRESET-request>)))
  "Returns md5sum for a message object of type '<NovatelFRESET-request>"
  "98b515b2d837c6d9bd64b53971bc09b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NovatelFRESET-request)))
  "Returns md5sum for a message object of type 'NovatelFRESET-request"
  "98b515b2d837c6d9bd64b53971bc09b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NovatelFRESET-request>)))
  "Returns full string definition for message of type '<NovatelFRESET-request>"
  (cl:format cl:nil "# Request to send a FRESET command to the Novatel unit~%#Request~%~%string target~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NovatelFRESET-request)))
  "Returns full string definition for message of type 'NovatelFRESET-request"
  (cl:format cl:nil "# Request to send a FRESET command to the Novatel unit~%#Request~%~%string target~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NovatelFRESET-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NovatelFRESET-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NovatelFRESET-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude NovatelFRESET-response.msg.html

(cl:defclass <NovatelFRESET-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass NovatelFRESET-response (<NovatelFRESET-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NovatelFRESET-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NovatelFRESET-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-srv:<NovatelFRESET-response> is deprecated: use novatel_gps_msgs-srv:NovatelFRESET-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <NovatelFRESET-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-srv:success-val is deprecated.  Use novatel_gps_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NovatelFRESET-response>) ostream)
  "Serializes a message object of type '<NovatelFRESET-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NovatelFRESET-response>) istream)
  "Deserializes a message object of type '<NovatelFRESET-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NovatelFRESET-response>)))
  "Returns string type for a service object of type '<NovatelFRESET-response>"
  "novatel_gps_msgs/NovatelFRESETResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelFRESET-response)))
  "Returns string type for a service object of type 'NovatelFRESET-response"
  "novatel_gps_msgs/NovatelFRESETResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NovatelFRESET-response>)))
  "Returns md5sum for a message object of type '<NovatelFRESET-response>"
  "98b515b2d837c6d9bd64b53971bc09b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NovatelFRESET-response)))
  "Returns md5sum for a message object of type 'NovatelFRESET-response"
  "98b515b2d837c6d9bd64b53971bc09b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NovatelFRESET-response>)))
  "Returns full string definition for message of type '<NovatelFRESET-response>"
  (cl:format cl:nil "~%#Response~%bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NovatelFRESET-response)))
  "Returns full string definition for message of type 'NovatelFRESET-response"
  (cl:format cl:nil "~%#Response~%bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NovatelFRESET-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NovatelFRESET-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NovatelFRESET-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NovatelFRESET)))
  'NovatelFRESET-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NovatelFRESET)))
  'NovatelFRESET-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelFRESET)))
  "Returns string type for a service object of type '<NovatelFRESET>"
  "novatel_gps_msgs/NovatelFRESET")