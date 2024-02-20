;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ScheduledReportRatesReq)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ScheduledReportRatesReq (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::SCHEDULEDREPORTRATESREQ")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::SCHEDULEDREPORTRATESREQ"))

(in-package "ROS")
;;//! \htmlinclude ScheduledReportRatesReq.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass kartech_linear_actuator_msgs::ScheduledReportRatesReq
  :super ros::object
  :slots (_header _confirm _index_1 _index_1_report_time _index_2 _index_2_report_time ))

(defmethod kartech_linear_actuator_msgs::ScheduledReportRatesReq
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:confirm __confirm) nil)
    ((:index_1 __index_1) (instance kartech_linear_actuator_msgs::ReportIndex :init))
    ((:index_1_report_time __index_1_report_time) 0)
    ((:index_2 __index_2) (instance kartech_linear_actuator_msgs::ReportIndex :init))
    ((:index_2_report_time __index_2_report_time) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _confirm __confirm)
   (setq _index_1 __index_1)
   (setq _index_1_report_time (round __index_1_report_time))
   (setq _index_2 __index_2)
   (setq _index_2_report_time (round __index_2_report_time))
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
  (:index_1
   (&rest __index_1)
   (if (keywordp (car __index_1))
       (send* _index_1 __index_1)
     (progn
       (if __index_1 (setq _index_1 (car __index_1)))
       _index_1)))
  (:index_1_report_time
   (&optional __index_1_report_time)
   (if __index_1_report_time (setq _index_1_report_time __index_1_report_time)) _index_1_report_time)
  (:index_2
   (&rest __index_2)
   (if (keywordp (car __index_2))
       (send* _index_2 __index_2)
     (progn
       (if __index_2 (setq _index_2 (car __index_2)))
       _index_2)))
  (:index_2_report_time
   (&optional __index_2_report_time)
   (if __index_2_report_time (setq _index_2_report_time __index_2_report_time)) _index_2_report_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _confirm
    1
    ;; kartech_linear_actuator_msgs/ReportIndex _index_1
    (send _index_1 :serialization-length)
    ;; uint16 _index_1_report_time
    2
    ;; kartech_linear_actuator_msgs/ReportIndex _index_2
    (send _index_2 :serialization-length)
    ;; uint16 _index_2_report_time
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _confirm
       (if _confirm (write-byte -1 s) (write-byte 0 s))
     ;; kartech_linear_actuator_msgs/ReportIndex _index_1
       (send _index_1 :serialize s)
     ;; uint16 _index_1_report_time
       (write-word _index_1_report_time s)
     ;; kartech_linear_actuator_msgs/ReportIndex _index_2
       (send _index_2 :serialize s)
     ;; uint16 _index_2_report_time
       (write-word _index_2_report_time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _confirm
     (setq _confirm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; kartech_linear_actuator_msgs/ReportIndex _index_1
     (send _index_1 :deserialize buf ptr-) (incf ptr- (send _index_1 :serialization-length))
   ;; uint16 _index_1_report_time
     (setq _index_1_report_time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; kartech_linear_actuator_msgs/ReportIndex _index_2
     (send _index_2 :deserialize buf ptr-) (incf ptr- (send _index_2 :serialization-length))
   ;; uint16 _index_2_report_time
     (setq _index_2_report_time (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ScheduledReportRatesReq :md5sum-) "26225aeadc02f4f458a0546ea8c99d87")
(setf (get kartech_linear_actuator_msgs::ScheduledReportRatesReq :datatype-) "kartech_linear_actuator_msgs/ScheduledReportRatesReq")
(setf (get kartech_linear_actuator_msgs::ScheduledReportRatesReq :definition-)
      "std_msgs/Header header
bool confirm
kartech_linear_actuator_msgs/ReportIndex index_1
uint16 index_1_report_time                       # How often to publish the requested report in ms.
kartech_linear_actuator_msgs/ReportIndex index_2 # If this is set to REPORT_NONE_INDEX then only the first index will be reported.
uint16 index_2_report_time                       # Ignored if index_2 is set to REPORT_NONE_INDEX.

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



(provide :kartech_linear_actuator_msgs/ScheduledReportRatesReq "26225aeadc02f4f458a0546ea8c99d87")


