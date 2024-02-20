;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::ObstacleStatus)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'ObstacleStatus (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS")
  (make-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))

(in-package "ROS")
;;//! \htmlinclude ObstacleStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STOP_GO_STOP*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*STOP_GO_STOP* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_STOP* 0)
(intern "*STOP_GO_GO*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*STOP_GO_GO* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_GO* 1)
(intern "*STOP_GO_UNDECIDED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*STOP_GO_UNDECIDED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_UNDECIDED* 2)
(intern "*STOP_GO_DRIVER_DECISION_REQUIRED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*STOP_GO_DRIVER_DECISION_REQUIRED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_DRIVER_DECISION_REQUIRED* 3)
(intern "*STOP_GO_NOT_CALCULATED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*STOP_GO_NOT_CALCULATED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_NOT_CALCULATED* 15)
(intern "*FAILSAFE_NONE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*FAILSAFE_NONE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*FAILSAFE_NONE* 0)
(intern "*FAILSAFE_LOW_SUN*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*FAILSAFE_LOW_SUN* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*FAILSAFE_LOW_SUN* 1)
(intern "*FAILSAFE_BLUR_IMAGE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(shadow '*FAILSAFE_BLUR_IMAGE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLESTATUS"))
(defconstant mobileye_560_660_msgs::ObstacleStatus::*FAILSAFE_BLUR_IMAGE* 2)

(defun mobileye_560_660_msgs::ObstacleStatus-to-symbol (const)
  (cond
        ((= const 0) 'mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_STOP*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_GO*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_UNDECIDED*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_DRIVER_DECISION_REQUIRED*)
        ((= const 15) 'mobileye_560_660_msgs::ObstacleStatus::*STOP_GO_NOT_CALCULATED*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleStatus::*FAILSAFE_NONE*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleStatus::*FAILSAFE_LOW_SUN*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleStatus::*FAILSAFE_BLUR_IMAGE*)
        (t nil)))

(defclass mobileye_560_660_msgs::ObstacleStatus
  :super ros::object
  :slots (_header _num_obstacles _timestamp _application_version _active_version_number_section _left_close_range_cut_in _right_close_range_cut_in _stop_go _protocol_version _close_car _failsafe ))

(defmethod mobileye_560_660_msgs::ObstacleStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_obstacles __num_obstacles) 0)
    ((:timestamp __timestamp) 0)
    ((:application_version __application_version) 0)
    ((:active_version_number_section __active_version_number_section) 0)
    ((:left_close_range_cut_in __left_close_range_cut_in) nil)
    ((:right_close_range_cut_in __right_close_range_cut_in) nil)
    ((:stop_go __stop_go) 0)
    ((:protocol_version __protocol_version) 0)
    ((:close_car __close_car) nil)
    ((:failsafe __failsafe) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_obstacles (round __num_obstacles))
   (setq _timestamp (round __timestamp))
   (setq _application_version (round __application_version))
   (setq _active_version_number_section (round __active_version_number_section))
   (setq _left_close_range_cut_in __left_close_range_cut_in)
   (setq _right_close_range_cut_in __right_close_range_cut_in)
   (setq _stop_go (round __stop_go))
   (setq _protocol_version (round __protocol_version))
   (setq _close_car __close_car)
   (setq _failsafe (round __failsafe))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_obstacles
   (&optional __num_obstacles)
   (if __num_obstacles (setq _num_obstacles __num_obstacles)) _num_obstacles)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:application_version
   (&optional __application_version)
   (if __application_version (setq _application_version __application_version)) _application_version)
  (:active_version_number_section
   (&optional __active_version_number_section)
   (if __active_version_number_section (setq _active_version_number_section __active_version_number_section)) _active_version_number_section)
  (:left_close_range_cut_in
   (&optional (__left_close_range_cut_in :null))
   (if (not (eq __left_close_range_cut_in :null)) (setq _left_close_range_cut_in __left_close_range_cut_in)) _left_close_range_cut_in)
  (:right_close_range_cut_in
   (&optional (__right_close_range_cut_in :null))
   (if (not (eq __right_close_range_cut_in :null)) (setq _right_close_range_cut_in __right_close_range_cut_in)) _right_close_range_cut_in)
  (:stop_go
   (&optional __stop_go)
   (if __stop_go (setq _stop_go __stop_go)) _stop_go)
  (:protocol_version
   (&optional __protocol_version)
   (if __protocol_version (setq _protocol_version __protocol_version)) _protocol_version)
  (:close_car
   (&optional (__close_car :null))
   (if (not (eq __close_car :null)) (setq _close_car __close_car)) _close_car)
  (:failsafe
   (&optional __failsafe)
   (if __failsafe (setq _failsafe __failsafe)) _failsafe)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _num_obstacles
    2
    ;; uint16 _timestamp
    2
    ;; uint16 _application_version
    2
    ;; uint16 _active_version_number_section
    2
    ;; bool _left_close_range_cut_in
    1
    ;; bool _right_close_range_cut_in
    1
    ;; uint8 _stop_go
    1
    ;; uint8 _protocol_version
    1
    ;; bool _close_car
    1
    ;; uint8 _failsafe
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _num_obstacles
       (write-word _num_obstacles s)
     ;; uint16 _timestamp
       (write-word _timestamp s)
     ;; uint16 _application_version
       (write-word _application_version s)
     ;; uint16 _active_version_number_section
       (write-word _active_version_number_section s)
     ;; bool _left_close_range_cut_in
       (if _left_close_range_cut_in (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_close_range_cut_in
       (if _right_close_range_cut_in (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _stop_go
       (write-byte _stop_go s)
     ;; uint8 _protocol_version
       (write-byte _protocol_version s)
     ;; bool _close_car
       (if _close_car (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _failsafe
       (write-byte _failsafe s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _num_obstacles
     (setq _num_obstacles (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _timestamp
     (setq _timestamp (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _application_version
     (setq _application_version (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _active_version_number_section
     (setq _active_version_number_section (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _left_close_range_cut_in
     (setq _left_close_range_cut_in (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_close_range_cut_in
     (setq _right_close_range_cut_in (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _stop_go
     (setq _stop_go (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _protocol_version
     (setq _protocol_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _close_car
     (setq _close_car (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _failsafe
     (setq _failsafe (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::ObstacleStatus :md5sum-) "b963ecf49d557c90935e49005018b9ff")
(setf (get mobileye_560_660_msgs::ObstacleStatus :datatype-) "mobileye_560_660_msgs/ObstacleStatus")
(setf (get mobileye_560_660_msgs::ObstacleStatus :definition-)
      "std_msgs/Header header

uint16 num_obstacles
uint16 timestamp
uint16 application_version
uint16 active_version_number_section
bool left_close_range_cut_in
bool right_close_range_cut_in

uint8 STOP_GO_STOP = 0
uint8 STOP_GO_GO = 1
uint8 STOP_GO_UNDECIDED = 2
uint8 STOP_GO_DRIVER_DECISION_REQUIRED = 3
uint8 STOP_GO_NOT_CALCULATED = 15
uint8 stop_go

uint8 protocol_version
bool close_car

uint8 FAILSAFE_NONE = 0
uint8 FAILSAFE_LOW_SUN = 1
uint8 FAILSAFE_BLUR_IMAGE = 2
uint8 failsafe

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

")



(provide :mobileye_560_660_msgs/ObstacleStatus "b963ecf49d557c90935e49005018b9ff")


