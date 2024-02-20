; Auto-generated. Do not edit!


(cl:in-package kartech_linear_actuator_msgs-msg)


;//! \htmlinclude ReportPollReq.msg.html

(cl:defclass <ReportPollReq> (roslisp-msg-protocol:ros-message)
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
   (report_indices
    :reader report_indices
    :initarg :report_indices
    :type (cl:vector kartech_linear_actuator_msgs-msg:ReportIndex)
   :initform (cl:make-array 0 :element-type 'kartech_linear_actuator_msgs-msg:ReportIndex :initial-element (cl:make-instance 'kartech_linear_actuator_msgs-msg:ReportIndex))))
)

(cl:defclass ReportPollReq (<ReportPollReq>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReportPollReq>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReportPollReq)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kartech_linear_actuator_msgs-msg:<ReportPollReq> is deprecated: use kartech_linear_actuator_msgs-msg:ReportPollReq instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ReportPollReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:header-val is deprecated.  Use kartech_linear_actuator_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'confirm-val :lambda-list '(m))
(cl:defmethod confirm-val ((m <ReportPollReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:confirm-val is deprecated.  Use kartech_linear_actuator_msgs-msg:confirm instead.")
  (confirm m))

(cl:ensure-generic-function 'report_indices-val :lambda-list '(m))
(cl:defmethod report_indices-val ((m <ReportPollReq>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kartech_linear_actuator_msgs-msg:report_indices-val is deprecated.  Use kartech_linear_actuator_msgs-msg:report_indices instead.")
  (report_indices m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReportPollReq>) ostream)
  "Serializes a message object of type '<ReportPollReq>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'confirm) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'report_indices))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'report_indices))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReportPollReq>) istream)
  "Deserializes a message object of type '<ReportPollReq>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'confirm) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'report_indices) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'report_indices)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'kartech_linear_actuator_msgs-msg:ReportIndex))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReportPollReq>)))
  "Returns string type for a message object of type '<ReportPollReq>"
  "kartech_linear_actuator_msgs/ReportPollReq")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReportPollReq)))
  "Returns string type for a message object of type 'ReportPollReq"
  "kartech_linear_actuator_msgs/ReportPollReq")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReportPollReq>)))
  "Returns md5sum for a message object of type '<ReportPollReq>"
  "f75ac448280dc0453a2f53fff2ba9c03")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReportPollReq)))
  "Returns md5sum for a message object of type 'ReportPollReq"
  "f75ac448280dc0453a2f53fff2ba9c03")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReportPollReq>)))
  "Returns full string definition for message of type '<ReportPollReq>"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%kartech_linear_actuator_msgs/ReportIndex[] report_indices     # The indicies of the reports that you would like to receive. Up to 6 may be requested at a time.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kartech_linear_actuator_msgs/ReportIndex~%uint8 POSITION_REPORT_INDEX = 128~%uint8 MOTOR_CURRENT_REPORT_INDEX = 129~%uint8 ENHANCED_POSITION_REPORT_INDEX = 152~%uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167~%uint8 SOFTWARE_REVISION_REPORT_INDEX = 229~%uint8 ZEROING_MESSAGE_REPORT_INDEX = 238~%~%uint8 report_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReportPollReq)))
  "Returns full string definition for message of type 'ReportPollReq"
  (cl:format cl:nil "std_msgs/Header header~%bool confirm~%kartech_linear_actuator_msgs/ReportIndex[] report_indices     # The indicies of the reports that you would like to receive. Up to 6 may be requested at a time.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: kartech_linear_actuator_msgs/ReportIndex~%uint8 POSITION_REPORT_INDEX = 128~%uint8 MOTOR_CURRENT_REPORT_INDEX = 129~%uint8 ENHANCED_POSITION_REPORT_INDEX = 152~%uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167~%uint8 SOFTWARE_REVISION_REPORT_INDEX = 229~%uint8 ZEROING_MESSAGE_REPORT_INDEX = 238~%~%uint8 report_index~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReportPollReq>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'report_indices) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReportPollReq>))
  "Converts a ROS message object to a list"
  (cl:list 'ReportPollReq
    (cl:cons ':header (header msg))
    (cl:cons ':confirm (confirm msg))
    (cl:cons ':report_indices (report_indices msg))
))
