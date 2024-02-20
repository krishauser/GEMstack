;; Auto-generated. Do not edit!


(when (boundp 'kartech_linear_actuator_msgs::ReportIndex)
  (if (not (find-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
    (make-package "KARTECH_LINEAR_ACTUATOR_MSGS"))
  (shadow 'ReportIndex (find-package "KARTECH_LINEAR_ACTUATOR_MSGS")))
(unless (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX")
  (make-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))

(in-package "ROS")
;;//! \htmlinclude ReportIndex.msg.html


(intern "*POSITION_REPORT_INDEX*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(shadow '*POSITION_REPORT_INDEX* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(defconstant kartech_linear_actuator_msgs::ReportIndex::*POSITION_REPORT_INDEX* 128)
(intern "*MOTOR_CURRENT_REPORT_INDEX*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(shadow '*MOTOR_CURRENT_REPORT_INDEX* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(defconstant kartech_linear_actuator_msgs::ReportIndex::*MOTOR_CURRENT_REPORT_INDEX* 129)
(intern "*ENHANCED_POSITION_REPORT_INDEX*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(shadow '*ENHANCED_POSITION_REPORT_INDEX* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(defconstant kartech_linear_actuator_msgs::ReportIndex::*ENHANCED_POSITION_REPORT_INDEX* 152)
(intern "*UNIQUE_DEVICE_ID_REPORTS_INDEX*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(shadow '*UNIQUE_DEVICE_ID_REPORTS_INDEX* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(defconstant kartech_linear_actuator_msgs::ReportIndex::*UNIQUE_DEVICE_ID_REPORTS_INDEX* 167)
(intern "*SOFTWARE_REVISION_REPORT_INDEX*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(shadow '*SOFTWARE_REVISION_REPORT_INDEX* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(defconstant kartech_linear_actuator_msgs::ReportIndex::*SOFTWARE_REVISION_REPORT_INDEX* 229)
(intern "*ZEROING_MESSAGE_REPORT_INDEX*" (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(shadow '*ZEROING_MESSAGE_REPORT_INDEX* (find-package "KARTECH_LINEAR_ACTUATOR_MSGS::REPORTINDEX"))
(defconstant kartech_linear_actuator_msgs::ReportIndex::*ZEROING_MESSAGE_REPORT_INDEX* 238)

(defun kartech_linear_actuator_msgs::ReportIndex-to-symbol (const)
  (cond
        ((= const 128) 'kartech_linear_actuator_msgs::ReportIndex::*POSITION_REPORT_INDEX*)
        ((= const 129) 'kartech_linear_actuator_msgs::ReportIndex::*MOTOR_CURRENT_REPORT_INDEX*)
        ((= const 152) 'kartech_linear_actuator_msgs::ReportIndex::*ENHANCED_POSITION_REPORT_INDEX*)
        ((= const 167) 'kartech_linear_actuator_msgs::ReportIndex::*UNIQUE_DEVICE_ID_REPORTS_INDEX*)
        ((= const 229) 'kartech_linear_actuator_msgs::ReportIndex::*SOFTWARE_REVISION_REPORT_INDEX*)
        ((= const 238) 'kartech_linear_actuator_msgs::ReportIndex::*ZEROING_MESSAGE_REPORT_INDEX*)
        (t nil)))

(defclass kartech_linear_actuator_msgs::ReportIndex
  :super ros::object
  :slots (_report_index ))

(defmethod kartech_linear_actuator_msgs::ReportIndex
  (:init
   (&key
    ((:report_index __report_index) 0)
    )
   (send-super :init)
   (setq _report_index (round __report_index))
   self)
  (:report_index
   (&optional __report_index)
   (if __report_index (setq _report_index __report_index)) _report_index)
  (:serialization-length
   ()
   (+
    ;; uint8 _report_index
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _report_index
       (write-byte _report_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _report_index
     (setq _report_index (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kartech_linear_actuator_msgs::ReportIndex :md5sum-) "05847e803066ad58819c151b2e8471e0")
(setf (get kartech_linear_actuator_msgs::ReportIndex :datatype-) "kartech_linear_actuator_msgs/ReportIndex")
(setf (get kartech_linear_actuator_msgs::ReportIndex :definition-)
      "uint8 POSITION_REPORT_INDEX = 128
uint8 MOTOR_CURRENT_REPORT_INDEX = 129
uint8 ENHANCED_POSITION_REPORT_INDEX = 152
uint8 UNIQUE_DEVICE_ID_REPORTS_INDEX = 167
uint8 SOFTWARE_REVISION_REPORT_INDEX = 229
uint8 ZEROING_MESSAGE_REPORT_INDEX = 238

uint8 report_index

")



(provide :kartech_linear_actuator_msgs/ReportIndex "05847e803066ad58819c151b2e8471e0")


