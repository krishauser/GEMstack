;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrStatus1)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrStatus1 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRSTATUS1")
  (make-package "DELPHI_ESR_MSGS::ESRSTATUS1"))

(in-package "ROS")
;;//! \htmlinclude EsrStatus1.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrStatus1
  :super ros::object
  :slots (_header _canmsg _rolling_count_1 _dsp_timestamp _comm_error _radius_curvature_calc _scan_index _yaw_rate_calc _vehicle_speed_calc ))

(defmethod delphi_esr_msgs::EsrStatus1
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:rolling_count_1 __rolling_count_1) 0)
    ((:dsp_timestamp __dsp_timestamp) 0)
    ((:comm_error __comm_error) nil)
    ((:radius_curvature_calc __radius_curvature_calc) 0)
    ((:scan_index __scan_index) 0)
    ((:yaw_rate_calc __yaw_rate_calc) 0.0)
    ((:vehicle_speed_calc __vehicle_speed_calc) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _rolling_count_1 (round __rolling_count_1))
   (setq _dsp_timestamp (round __dsp_timestamp))
   (setq _comm_error __comm_error)
   (setq _radius_curvature_calc (round __radius_curvature_calc))
   (setq _scan_index (round __scan_index))
   (setq _yaw_rate_calc (float __yaw_rate_calc))
   (setq _vehicle_speed_calc (float __vehicle_speed_calc))
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
  (:rolling_count_1
   (&optional __rolling_count_1)
   (if __rolling_count_1 (setq _rolling_count_1 __rolling_count_1)) _rolling_count_1)
  (:dsp_timestamp
   (&optional __dsp_timestamp)
   (if __dsp_timestamp (setq _dsp_timestamp __dsp_timestamp)) _dsp_timestamp)
  (:comm_error
   (&optional (__comm_error :null))
   (if (not (eq __comm_error :null)) (setq _comm_error __comm_error)) _comm_error)
  (:radius_curvature_calc
   (&optional __radius_curvature_calc)
   (if __radius_curvature_calc (setq _radius_curvature_calc __radius_curvature_calc)) _radius_curvature_calc)
  (:scan_index
   (&optional __scan_index)
   (if __scan_index (setq _scan_index __scan_index)) _scan_index)
  (:yaw_rate_calc
   (&optional __yaw_rate_calc)
   (if __yaw_rate_calc (setq _yaw_rate_calc __yaw_rate_calc)) _yaw_rate_calc)
  (:vehicle_speed_calc
   (&optional __vehicle_speed_calc)
   (if __vehicle_speed_calc (setq _vehicle_speed_calc __vehicle_speed_calc)) _vehicle_speed_calc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _rolling_count_1
    1
    ;; uint8 _dsp_timestamp
    1
    ;; bool _comm_error
    1
    ;; int16 _radius_curvature_calc
    2
    ;; uint16 _scan_index
    2
    ;; float32 _yaw_rate_calc
    4
    ;; float32 _vehicle_speed_calc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; uint8 _rolling_count_1
       (write-byte _rolling_count_1 s)
     ;; uint8 _dsp_timestamp
       (write-byte _dsp_timestamp s)
     ;; bool _comm_error
       (if _comm_error (write-byte -1 s) (write-byte 0 s))
     ;; int16 _radius_curvature_calc
       (write-word _radius_curvature_calc s)
     ;; uint16 _scan_index
       (write-word _scan_index s)
     ;; float32 _yaw_rate_calc
       (sys::poke _yaw_rate_calc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vehicle_speed_calc
       (sys::poke _vehicle_speed_calc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _rolling_count_1
     (setq _rolling_count_1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _dsp_timestamp
     (setq _dsp_timestamp (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _comm_error
     (setq _comm_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _radius_curvature_calc
     (setq _radius_curvature_calc (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _scan_index
     (setq _scan_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _yaw_rate_calc
     (setq _yaw_rate_calc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vehicle_speed_calc
     (setq _vehicle_speed_calc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrStatus1 :md5sum-) "f3f440bdd87b7ce3da2d8d915a5970b7")
(setf (get delphi_esr_msgs::EsrStatus1 :datatype-) "delphi_esr_msgs/EsrStatus1")
(setf (get delphi_esr_msgs::EsrStatus1 :definition-)
      "std_msgs/Header header

# ESR Status1
string      canmsg

uint8       rolling_count_1
uint8       dsp_timestamp
bool        comm_error
int16       radius_curvature_calc
uint16      scan_index
float32     yaw_rate_calc
float32     vehicle_speed_calc

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



(provide :delphi_esr_msgs/EsrStatus1 "f3f440bdd87b7ce3da2d8d915a5970b7")


