;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ReportPollReq)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ReportPollReq (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTPOLLREQ")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTPOLLREQ"))

(in-package "ROS")
;;//! \htmlinclude ReportPollReq.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ReportPollReq
  :super ros::object
  :slots (_header _confirm _report_indices ))

(defmethod kartech_linear_actuator_msgs::ReportPollReq
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:report_indices __report_indices) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _report_indices __report_indices)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:confirm
   (&optional (__confirm :null))
   (if (not (eq __confirm :null)) (setq _confirm __confirm)) _confirm)
  (:report_indices
   (&rest __report_indices)
   (if (keywordp (car __report_indices))
       (send* _report_indices __report_indices)
     (progn
       (if __report_indices (setq _report_indices (car __report_indices)))
       _report_indices)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; kartech_linear_actuator_msgs/ReportIndex[] _report_indices
    (apply #'+ (send-all _report_indices :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;; kartech_linear_actuator_msgs/ReportIndex[] _report_indices
     (write-long (length _report_indices) s)
     (dolist (elem _report_indices)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; kartech_linear_actuator_msgs/ReportIndex[] _report_indices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _report_indices (let (r) (dotimes (i n) (push (instance kartech_linear_actuator_msgs::ReportIndex :init) r)) r))
     (dolist (elem- _report_indices)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ReportPollReq :md5sum-) "f75ac448280dc0453a2f53fff2ba9c03")
(setf (get kartech_linear_actuator_msgs::ReportPollReq :datatype-) "kartech_linear_actuator_msgs/ReportPollReq")
(setf (get kartech_linear_actuator_msgs::ReportPollReq :definition-)
      "std_msgs/Header header
bool confirm
kartech_linear_actuator_msgs/ReportIndex[] report_indices     # The indicies of the reports that you would like to receive. Up to 6 may be requested at a time.

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: kartech_linear_actuator_msgs/ReportIndex
uint8 POSITION_REPORT_INDEX = 128
uint8 MOTOR_CURRENT_REPORT_INDEX = 129
uint8 ENHANCED_POSITION_REPORT_INDEX = 152
uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167
uint8 SOFTWARE_REVISION_REPORT_INDEX = 229
uint8 ZEROING_MESSAGE_REPORT_INDEX = 238

uint8 report_index

")



(provide :kartech_linear_actuator_msgs/ReportPollReq "f75ac448280dc0453a2f53fff2ba9c03")


