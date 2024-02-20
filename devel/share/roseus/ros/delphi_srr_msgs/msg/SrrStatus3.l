;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrStatus3)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrStatus3 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRSTATUS3")
  (make-package "DELPHI_SRR_MSGS::SRRSTATUS3"))

(in-package "ROS")
;;//! \htmlinclude SrrStatus3.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CAN_TX_ALIGNMENT_STATE_OFF*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(shadow '*CAN_TX_ALIGNMENT_STATE_OFF* (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(defconstant delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_OFF* 0)
(intern "*CAN_TX_ALIGNMENT_STATE_INIT*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(shadow '*CAN_TX_ALIGNMENT_STATE_INIT* (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(defconstant delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_INIT* 1)
(intern "*CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(shadow '*CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT* (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(defconstant delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT* 2)
(intern "*CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(shadow '*CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT* (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(defconstant delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT* 3)
(intern "*CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(shadow '*CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT* (find-package "DELPHI_SRR_MSGS::SRRSTATUS3"))
(defconstant delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT* 4)

(defun delphi_srr_msgs::SrrStatus3-to-symbol (const)
  (cond
        ((= const 0) 'delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_OFF*)
        ((= const 1) 'delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_INIT*)
        ((= const 2) 'delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT*)
        ((= const 3) 'delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT*)
        ((= const 4) 'delphi_srr_msgs::SrrStatus3::*CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT*)
        (t nil)))

(defclass delphi_srr_msgs::SrrStatus3
  :super ros::object
  :slots (_header _can_tx_alignment_state _can_tx_interface_ver_minor _can_tx_sw_version_arm _can_tx_hw_version _can_tx_interface_version _can_tx_serial_num ))

(defmethod delphi_srr_msgs::SrrStatus3
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_tx_alignment_state __can_tx_alignment_state) 0)
    ((:can_tx_interface_ver_minor __can_tx_interface_ver_minor) 0)
    ((:can_tx_sw_version_arm __can_tx_sw_version_arm) 0)
    ((:can_tx_hw_version __can_tx_hw_version) 0)
    ((:can_tx_interface_version __can_tx_interface_version) 0)
    ((:can_tx_serial_num __can_tx_serial_num) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_tx_alignment_state (round __can_tx_alignment_state))
   (setq _can_tx_interface_ver_minor (round __can_tx_interface_ver_minor))
   (setq _can_tx_sw_version_arm (round __can_tx_sw_version_arm))
   (setq _can_tx_hw_version (round __can_tx_hw_version))
   (setq _can_tx_interface_version (round __can_tx_interface_version))
   (setq _can_tx_serial_num (round __can_tx_serial_num))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_tx_alignment_state
   (&optional __can_tx_alignment_state)
   (if __can_tx_alignment_state (setq _can_tx_alignment_state __can_tx_alignment_state)) _can_tx_alignment_state)
  (:can_tx_interface_ver_minor
   (&optional __can_tx_interface_ver_minor)
   (if __can_tx_interface_ver_minor (setq _can_tx_interface_ver_minor __can_tx_interface_ver_minor)) _can_tx_interface_ver_minor)
  (:can_tx_sw_version_arm
   (&optional __can_tx_sw_version_arm)
   (if __can_tx_sw_version_arm (setq _can_tx_sw_version_arm __can_tx_sw_version_arm)) _can_tx_sw_version_arm)
  (:can_tx_hw_version
   (&optional __can_tx_hw_version)
   (if __can_tx_hw_version (setq _can_tx_hw_version __can_tx_hw_version)) _can_tx_hw_version)
  (:can_tx_interface_version
   (&optional __can_tx_interface_version)
   (if __can_tx_interface_version (setq _can_tx_interface_version __can_tx_interface_version)) _can_tx_interface_version)
  (:can_tx_serial_num
   (&optional __can_tx_serial_num)
   (if __can_tx_serial_num (setq _can_tx_serial_num __can_tx_serial_num)) _can_tx_serial_num)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_tx_alignment_state
    1
    ;; uint8 _can_tx_interface_ver_minor
    1
    ;; uint32 _can_tx_sw_version_arm
    4
    ;; uint8 _can_tx_hw_version
    1
    ;; uint8 _can_tx_interface_version
    1
    ;; uint32 _can_tx_serial_num
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_tx_alignment_state
       (write-byte _can_tx_alignment_state s)
     ;; uint8 _can_tx_interface_ver_minor
       (write-byte _can_tx_interface_ver_minor s)
     ;; uint32 _can_tx_sw_version_arm
       (write-long _can_tx_sw_version_arm s)
     ;; uint8 _can_tx_hw_version
       (write-byte _can_tx_hw_version s)
     ;; uint8 _can_tx_interface_version
       (write-byte _can_tx_interface_version s)
     ;; uint32 _can_tx_serial_num
       (write-long _can_tx_serial_num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_tx_alignment_state
     (setq _can_tx_alignment_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_tx_interface_ver_minor
     (setq _can_tx_interface_ver_minor (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _can_tx_sw_version_arm
     (setq _can_tx_sw_version_arm (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _can_tx_hw_version
     (setq _can_tx_hw_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_tx_interface_version
     (setq _can_tx_interface_version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _can_tx_serial_num
     (setq _can_tx_serial_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrStatus3 :md5sum-) "7a40100fb28cf1c5e2bf4d3c15d6aeb8")
(setf (get delphi_srr_msgs::SrrStatus3 :datatype-) "delphi_srr_msgs/SrrStatus3")
(setf (get delphi_srr_msgs::SrrStatus3 :definition-)
      "# Message file for srr_status3

std_msgs/Header header

uint8     can_tx_alignment_state
uint8     CAN_TX_ALIGNMENT_STATE_OFF=0
uint8     CAN_TX_ALIGNMENT_STATE_INIT=1
uint8     CAN_TX_ALIGNMENT_STATE_AUTOMATIC_ALIGNMENT=2
uint8     CAN_TX_ALIGNMENT_STATE_FACTORY_ALIGNMENT=3
uint8     CAN_TX_ALIGNMENT_STATE_SERVICE_ALIGNMENT=4

uint8     can_tx_interface_ver_minor
uint32    can_tx_sw_version_arm
uint8     can_tx_hw_version
uint8     can_tx_interface_version
uint32    can_tx_serial_num

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



(provide :delphi_srr_msgs/SrrStatus3 "7a40100fb28cf1c5e2bf4d3c15d6aeb8")


