; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ScheduledReportRatesReq.msg.html

(cl:defclass <ScheduledReportRatesReq> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (confirm
    :reader confirm
    :initarg :confirm
    :type cl:boolean
    :initform cl:nil)
   (index_1
    :reader index_1
    :initarg :index_1
    :type kartech_linear_actuator_msgs-msg:ReportIndex
    :initform (cl:make-instance 'kartech_linear_actuator_msgs-msg:ReportIndex))
   (index_1_report_time
    :reader index_1_report_time
    :initarg :index_1_report_time
    :type cl:fixnum
    :initform 0)
   (index_2
    :reader index_2
    :initarg :index_2
    :type kartech_linear_actuator_msgs-msg:ReportIndex
    :initform (cl:make-instance 'kartech_linear_actuator_msgs-msg:ReportIndex))
   (index_2_report_time
    :reader index_2_report_time
    :initarg :index_2_report_time
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ScheduledReportRatesReq (<ScheduledReportRatesReq>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ScheduledReportRatesReq>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ScheduledReportRatesReq)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ScheduledReportRatesReq> is deprecated: use kartech_linear_actuator_msgs-msg:ScheduledReportRatesReq instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ScheduledReportRatesReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ScheduledReportRatesReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'index_1-val :lambda-list '(m))
(cl:defmethod index_1-val ((m <ScheduledReportRatesReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:index_1-val is deprecated.  Use kartech_linear_actuator_msgs-msg:index_1 instead.")
  (index_1 m))

(cl:ensure-generic-function 'index_1_report_time-val :lambda-list '(m))
(cl:defmethod index_1_report_time-val ((m <ScheduledReportRatesReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:index_1_report_time-val is deprecated.  Use kartech_linear_actuator_msgs-msg:index_1_report_time instead.")
  (index_1_report_time m))

(cl:ensure-generic-function 'index_2-val :lambda-list '(m))
(cl:defmethod index_2-val ((m <ScheduledReportRatesReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:index_2-val is deprecated.  Use kartech_linear_actuator_msgs-msg:index_2 instead.")
  (index_2 m))

(cl:ensure-generic-function 'index_2_report_time-val :lambda-list '(m))
(cl:defmethod index_2_report_time-val ((m <ScheduledReportRatesReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:index_2_report_time-val is deprecated.  Use kartech_linear_actuator_msgs-msg:index_2_report_time instead.")
  (index_2_report_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ScheduledReportRatesReq>) ostream)
  "Serializes a message object of type '<ScheduledReportRatesReq>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'index_1) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_1_report_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index_1_report_time)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'index_2) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_2_report_time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index_2_report_time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ScheduledReportRatesReq>) istream)
  "Deserializes a message object of type '<ScheduledReportRatesReq>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'index_1) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_1_report_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index_1_report_time)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'index_2) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index_2_report_time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index_2_report_time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ScheduledReportRatesReq>)))
  "Returns string type for a message object of type '<ScheduledReportRatesReq>"
  "kartech_linear_actuator_msgs/ScheduledReportRatesReq")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ScheduledReportRatesReq)))
  "Returns string type for a message object of type 'ScheduledReportRatesReq"
  "kartech_linear_actuator_msgs/ScheduledReportRatesReq")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ScheduledReportRatesReq>)))
  "Returns md5sum for a message object of type '<ScheduledReportRatesReq>"
  "26225aeadc02f4f458a0546ea8c99d87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ScheduledReportRatesReq)))
  "Returns md5sum for a message object of type 'ScheduledReportRatesReq"
  "26225aeadc02f4f458a0546ea8c99d87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ScheduledReportRatesReq>)))
  "Returns full string definition for message of type '<ScheduledReportRatesReq>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%kartech_linear_actuator_msgs/ReportIndex index_1~%uint16 index_1_report_time                       # How often to publish the requested report in ms.~%kartech_linear_actuator_msgs/ReportIndex index_2 # If this is set to REPORT_NONE_INDEX then only the first index will be reported.~%uint16 index_2_report_time                       # Ignored if index_2 is set to REPORT_NONE_INDEX.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kartech_linear_actuator_msgs/ReportIndex~%uint8 POSITION_REPORT_INDEX = 128~%uint8 MOTOR_CURRENT_REPORT_INDEX = 129~%uint8 ENHANCED_POSITION_REPORT_INDEX = 152~%uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167~%uint8 SOFTWARE_REVISION_REPORT_INDEX = 229~%uint8 ZEROING_MESSAGE_REPORT_INDEX = 238~%~%uint8 report_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ScheduledReportRatesReq)))
  "Returns full string definition for message of type 'ScheduledReportRatesReq"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%kartech_linear_actuator_msgs/ReportIndex index_1~%uint16 index_1_report_time                       # How often to publish the requested report in ms.~%kartech_linear_actuator_msgs/ReportIndex index_2 # If this is set to REPORT_NONE_INDEX then only the first index will be reported.~%uint16 index_2_report_time                       # Ignored if index_2 is set to REPORT_NONE_INDEX.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kartech_linear_actuator_msgs/ReportIndex~%uint8 POSITION_REPORT_INDEX = 128~%uint8 MOTOR_CURRENT_REPORT_INDEX = 129~%uint8 ENHANCED_POSITION_REPORT_INDEX = 152~%uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167~%uint8 SOFTWARE_REVISION_REPORT_INDEX = 229~%uint8 ZEROING_MESSAGE_REPORT_INDEX = 238~%~%uint8 report_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ScheduledReportRatesReq>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'index_1))
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'index_2))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ScheduledReportRatesReq>))
  "Converts a ROS message object to a list"
  (cl:list 'ScheduledReportRatesReq
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':index_1 (index_1 msg))
    (cl:cons ':index_1_report_time (index_1_report_time msg))
    (cl:cons ':index_2 (index_2 msg))
    (cl:cons ':index_2_report_time (index_2_report_time msg))
))
