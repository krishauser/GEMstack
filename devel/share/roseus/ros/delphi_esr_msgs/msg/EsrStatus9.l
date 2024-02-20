;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrStatus9)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrStatus9 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRSTATUS9")
  (make-package "DELPHI_ESR_MSGS::ESRSTATUS9"))

(in-package "ROS")
;;//! \htmlinclude EsrStatus9.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrStatus9
  :super ros::object
  :slots (_header _canmsg _avg_pwr_cwblkg _sideslip_angle _serial_num_3rd_byte _water_spray_target_id _filtered_xohp_acc_cipv _path_id_acc_2 _path_id_acc_3 ))

(defmethod delphi_esr_msgs::EsrStatus9
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:avg_pwr_cwblkg __avg_pwr_cwblkg) 0)
    ((:sideslip_angle __sideslip_angle) 0.0)
    ((:serial_num_3rd_byte __serial_num_3rd_byte) 0)
    ((:water_spray_target_id __water_spray_target_id) 0)
    ((:filtered_xohp_acc_cipv __filtered_xohp_acc_cipv) 0.0)
    ((:path_id_acc_2 __path_id_acc_2) 0)
    ((:path_id_acc_3 __path_id_acc_3) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _avg_pwr_cwblkg (round __avg_pwr_cwblkg))
   (setq _sideslip_angle (float __sideslip_angle))
   (setq _serial_num_3rd_byte (round __serial_num_3rd_byte))
   (setq _water_spray_target_id (round __water_spray_target_id))
   (setq _filtered_xohp_acc_cipv (float __filtered_xohp_acc_cipv))
   (setq _path_id_acc_2 (round __path_id_acc_2))
   (setq _path_id_acc_3 (round __path_id_acc_3))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:canmsg
   (&optional __canmsg)
   (if __canmsg (setq _canmsg __canmsg)) _canmsg)
  (:avg_pwr_cwblkg
   (&optional __avg_pwr_cwblkg)
   (if __avg_pwr_cwblkg (setq _avg_pwr_cwblkg __avg_pwr_cwblkg)) _avg_pwr_cwblkg)
  (:sideslip_angle
   (&optional __sideslip_angle)
   (if __sideslip_angle (setq _sideslip_angle __sideslip_angle)) _sideslip_angle)
  (:serial_num_3rd_byte
   (&optional __serial_num_3rd_byte)
   (if __serial_num_3rd_byte (setq _serial_num_3rd_byte __serial_num_3rd_byte)) _serial_num_3rd_byte)
  (:water_spray_target_id
   (&optional __water_spray_target_id)
   (if __water_spray_target_id (setq _water_spray_target_id __water_spray_target_id)) _water_spray_target_id)
  (:filtered_xohp_acc_cipv
   (&optional __filtered_xohp_acc_cipv)
   (if __filtered_xohp_acc_cipv (setq _filtered_xohp_acc_cipv __filtered_xohp_acc_cipv)) _filtered_xohp_acc_cipv)
  (:path_id_acc_2
   (&optional __path_id_acc_2)
   (if __path_id_acc_2 (setq _path_id_acc_2 __path_id_acc_2)) _path_id_acc_2)
  (:path_id_acc_3
   (&optional __path_id_acc_3)
   (if __path_id_acc_3 (setq _path_id_acc_3 __path_id_acc_3)) _path_id_acc_3)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint16 _avg_pwr_cwblkg
    2
    ;; float32 _sideslip_angle
    4
    ;; uint8 _serial_num_3rd_byte
    1
    ;; uint8 _water_spray_target_id
    1
    ;; float32 _filtered_xohp_acc_cipv
    4
    ;; uint8 _path_id_acc_2
    1
    ;; uint8 _path_id_acc_3
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; uint16 _avg_pwr_cwblkg
       (write-word _avg_pwr_cwblkg s)
     ;; float32 _sideslip_angle
       (sys::poke _sideslip_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _serial_num_3rd_byte
       (write-byte _serial_num_3rd_byte s)
     ;; uint8 _water_spray_target_id
       (write-byte _water_spray_target_id s)
     ;; float32 _filtered_xohp_acc_cipv
       (sys::poke _filtered_xohp_acc_cipv (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _path_id_acc_2
       (write-byte _path_id_acc_2 s)
     ;; uint8 _path_id_acc_3
       (write-byte _path_id_acc_3 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint16 _avg_pwr_cwblkg
     (setq _avg_pwr_cwblkg (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _sideslip_angle
     (setq _sideslip_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _serial_num_3rd_byte
     (setq _serial_num_3rd_byte (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _water_spray_target_id
     (setq _water_spray_target_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _filtered_xohp_acc_cipv
     (setq _filtered_xohp_acc_cipv (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _path_id_acc_2
     (setq _path_id_acc_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _path_id_acc_3
     (setq _path_id_acc_3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrStatus9 :md5sum-) "303ebfcbdd8866a01d6c2acd5e4df496")
(setf (get delphi_esr_msgs::EsrStatus9 :datatype-) "delphi_esr_msgs/EsrStatus9")
(setf (get delphi_esr_msgs::EsrStatus9 :definition-)
      "std_msgs/Header header

# ESR Status9
string      canmsg

uint16      avg_pwr_cwblkg
float32     sideslip_angle
uint8       serial_num_3rd_byte
uint8       water_spray_target_id
float32     filtered_xohp_acc_cipv
uint8       path_id_acc_2
uint8       path_id_acc_3

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



(provide :delphi_esr_msgs/EsrStatus9 "303ebfcbdd8866a01d6c2acd5e4df496")


