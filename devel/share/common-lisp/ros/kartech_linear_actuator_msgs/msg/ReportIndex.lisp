; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ReportIndex.msg.html

(cl:defclass <ReportIndex> (roslisp-msg-protocol:ros-message)
  ((report_index
    :reader report_index
    :initarg :report_index
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ReportIndex (<ReportIndex>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReportIndex>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReportIndex)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ReportIndex> is deprecated: use kartech_linear_actuator_msgs-msg:ReportIndex instead.")))

(cl:ensure-generic-function 'report_index-val :lambda-list '(m))
(cl:defmethod report_index-val ((m <ReportIndex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:report_index-val is deprecated.  Use kartech_linear_actuator_msgs-msg:report_index instead.")
  (report_index m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ReportIndex>)))
    "Constants for message type '<ReportIndex>"
  '((:POSITION_REPORT_INDEX . 128)
    (:MOTOR_CURRENT_REPORT_INDEX . 129)
    (:ENHANCED_POSITION_REPORT_INDEX . 152)
    (:UNIQUE_DEVICE_ID_REPORTS_INDEX . 167)
    (:SOFTWARE_REVISION_REPORT_INDEX . 229)
    (:ZEROING_MESSAGE_REPORT_INDEX . 238))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ReportIndex)))
    "Constants for message type 'ReportIndex"
  '((:POSITION_REPORT_INDEX . 128)
    (:MOTOR_CURRENT_REPORT_INDEX . 129)
    (:ENHANCED_POSITION_REPORT_INDEX . 152)
    (:UNIQUE_DEVICE_ID_REPORTS_INDEX . 167)
    (:SOFTWARE_REVISION_REPORT_INDEX . 229)
    (:ZEROING_MESSAGE_REPORT_INDEX . 238))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReportIndex>) ostream)
  "Serializes a message object of type '<ReportIndex>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'report_index)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReportIndex>) istream)
  "Deserializes a message object of type '<ReportIndex>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'report_index)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReportIndex>)))
  "Returns string type for a message object of type '<ReportIndex>"
  "kartech_linear_actuator_msgs/ReportIndex")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReportIndex)))
  "Returns string type for a message object of type 'ReportIndex"
  "kartech_linear_actuator_msgs/ReportIndex")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReportIndex>)))
  "Returns md5sum for a message object of type '<ReportIndex>"
  "05847e803066ad58819c151b2e8471e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReportIndex)))
  "Returns md5sum for a message object of type 'ReportIndex"
  "05847e803066ad58819c151b2e8471e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReportIndex>)))
  "Returns full string definition for message of type '<ReportIndex>"
  (cl:format cl:nil "uint8 POSITION_REPORT_INDEX = 128~%uint8 MOTOR_CURRENT_REPORT_INDEX = 129~%uint8 ENHANCED_POSITION_REPORT_INDEX = 152~%uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167~%uint8 SOFTWARE_REVISION_REPORT_INDEX = 229~%uint8 ZEROING_MESSAGE_REPORT_INDEX = 238~%~%uint8 report_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReportIndex)))
  "Returns full string definition for message of type 'ReportIndex"
  (cl:format cl:nil "uint8 POSITION_REPORT_INDEX = 128~%uint8 MOTOR_CURRENT_REPORT_INDEX = 129~%uint8 ENHANCED_POSITION_REPORT_INDEX = 152~%uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167~%uint8 SOFTWARE_REVISION_REPORT_INDEX = 229~%uint8 ZEROING_MESSAGE_REPORT_INDEX = 238~%~%uint8 report_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReportIndex>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReportIndex>))
  "Converts a ROS message object to a list"
  (cl:list 'ReportIndex
    (cl:cons ':report_index (report_index msg))
))
