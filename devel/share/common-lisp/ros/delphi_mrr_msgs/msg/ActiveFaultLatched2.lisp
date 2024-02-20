; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude ActiveFaultLatched2.msg.html

(cl:defclass <ActiveFaultLatched2> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ipma_pcan_data_range_check
    :reader ipma_pcan_data_range_check
    :initarg :ipma_pcan_data_range_check
    :type cl:boolean
    :initform cl:nil)
   (ipma_pcan_missing_msg
    :reader ipma_pcan_missing_msg
    :initarg :ipma_pcan_missing_msg
    :type cl:boolean
    :initform cl:nil)
   (vin_signal_compare_failure
    :reader vin_signal_compare_failure
    :initarg :vin_signal_compare_failure
    :type cl:boolean
    :initform cl:nil)
   (module_not_configured_error
    :reader module_not_configured_error
    :initarg :module_not_configured_error
    :type cl:boolean
    :initform cl:nil)
   (car_cfg_not_configured_error
    :reader car_cfg_not_configured_error
    :initarg :car_cfg_not_configured_error
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ActiveFaultLatched2 (<ActiveFaultLatched2>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActiveFaultLatched2>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActiveFaultLatched2)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<ActiveFaultLatched2> is deprecated: use delphi_mrr_msgs-msg:ActiveFaultLatched2 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ActiveFaultLatched2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ipma_pcan_data_range_check-val :lambda-list '(m))
(cl:defmethod ipma_pcan_data_range_check-val ((m <ActiveFaultLatched2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:ipma_pcan_data_range_check-val is deprecated.  Use delphi_mrr_msgs-msg:ipma_pcan_data_range_check instead.")
  (ipma_pcan_data_range_check m))

(cl:ensure-generic-function 'ipma_pcan_missing_msg-val :lambda-list '(m))
(cl:defmethod ipma_pcan_missing_msg-val ((m <ActiveFaultLatched2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:ipma_pcan_missing_msg-val is deprecated.  Use delphi_mrr_msgs-msg:ipma_pcan_missing_msg instead.")
  (ipma_pcan_missing_msg m))

(cl:ensure-generic-function 'vin_signal_compare_failure-val :lambda-list '(m))
(cl:defmethod vin_signal_compare_failure-val ((m <ActiveFaultLatched2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:vin_signal_compare_failure-val is deprecated.  Use delphi_mrr_msgs-msg:vin_signal_compare_failure instead.")
  (vin_signal_compare_failure m))

(cl:ensure-generic-function 'module_not_configured_error-val :lambda-list '(m))
(cl:defmethod module_not_configured_error-val ((m <ActiveFaultLatched2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:module_not_configured_error-val is deprecated.  Use delphi_mrr_msgs-msg:module_not_configured_error instead.")
  (module_not_configured_error m))

(cl:ensure-generic-function 'car_cfg_not_configured_error-val :lambda-list '(m))
(cl:defmethod car_cfg_not_configured_error-val ((m <ActiveFaultLatched2>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:car_cfg_not_configured_error-val is deprecated.  Use delphi_mrr_msgs-msg:car_cfg_not_configured_error instead.")
  (car_cfg_not_configured_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActiveFaultLatched2>) ostream)
  "Serializes a message object of type '<ActiveFaultLatched2>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipma_pcan_data_range_check) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipma_pcan_missing_msg) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vin_signal_compare_failure) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'module_not_configured_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'car_cfg_not_configured_error) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActiveFaultLatched2>) istream)
  "Deserializes a message object of type '<ActiveFaultLatched2>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'ipma_pcan_data_range_check) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ipma_pcan_missing_msg) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'vin_signal_compare_failure) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'module_not_configured_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'car_cfg_not_configured_error) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActiveFaultLatched2>)))
  "Returns string type for a message object of type '<ActiveFaultLatched2>"
  "delphi_mrr_msgs/ActiveFaultLatched2")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActiveFaultLatched2)))
  "Returns string type for a message object of type 'ActiveFaultLatched2"
  "delphi_mrr_msgs/ActiveFaultLatched2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActiveFaultLatched2>)))
  "Returns md5sum for a message object of type '<ActiveFaultLatched2>"
  "fcd978d054e184e337e27e3c35da2f6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActiveFaultLatched2)))
  "Returns md5sum for a message object of type 'ActiveFaultLatched2"
  "fcd978d054e184e337e27e3c35da2f6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActiveFaultLatched2>)))
  "Returns full string definition for message of type '<ActiveFaultLatched2>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool ipma_pcan_data_range_check~%bool ipma_pcan_missing_msg~%bool vin_signal_compare_failure~%bool module_not_configured_error~%bool car_cfg_not_configured_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActiveFaultLatched2)))
  "Returns full string definition for message of type 'ActiveFaultLatched2"
  (cl:format cl:nil "std_msgs/Header header~%~%bool ipma_pcan_data_range_check~%bool ipma_pcan_missing_msg~%bool vin_signal_compare_failure~%bool module_not_configured_error~%bool car_cfg_not_configured_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActiveFaultLatched2>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActiveFaultLatched2>))
  "Converts a ROS message object to a list"
  (cl:list 'ActiveFaultLatched2
    (cl:cons ':header (header msg))
    (cl:cons ':ipma_pcan_data_range_check (ipma_pcan_data_range_check msg))
    (cl:cons ':ipma_pcan_missing_msg (ipma_pcan_missing_msg msg))
    (cl:cons ':vin_signal_compare_failure (vin_signal_compare_failure msg))
    (cl:cons ':module_not_configured_error (module_not_configured_error msg))
    (cl:cons ':car_cfg_not_configured_error (car_cfg_not_configured_error msg))
))
