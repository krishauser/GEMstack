;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrStatus2)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrStatus2 (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRSTATUS2")
  (make-package "DELPHI_ESR_MSGS::ESRSTATUS2"))

(in-package "ROS")
;;//! \htmlinclude EsrStatus2.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrStatus2
  :super ros::object
  :slots (_header _canmsg _maximum_tracks_ack _rolling_count_2 _overheat_error _range_perf_error _internal_error _xcvr_operational _raw_data_mode _steering_angle_ack _temperature _veh_spd_comp_factor _grouping_mode _yaw_rate_bias _sw_version_dsp ))

(defmethod delphi_esr_msgs::EsrStatus2
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:maximum_tracks_ack __maximum_tracks_ack) 0)
    ((:rolling_count_2 __rolling_count_2) 0)
    ((:overheat_error __overheat_error) nil)
    ((:range_perf_error __range_perf_error) nil)
    ((:internal_error __internal_error) nil)
    ((:xcvr_operational __xcvr_operational) nil)
    ((:raw_data_mode __raw_data_mode) nil)
    ((:steering_angle_ack __steering_angle_ack) 0)
    ((:temperature __temperature) 0)
    ((:veh_spd_comp_factor __veh_spd_comp_factor) 0.0)
    ((:grouping_mode __grouping_mode) 0)
    ((:yaw_rate_bias __yaw_rate_bias) 0.0)
    ((:sw_version_dsp __sw_version_dsp) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _maximum_tracks_ack (round __maximum_tracks_ack))
   (setq _rolling_count_2 (round __rolling_count_2))
   (setq _overheat_error __overheat_error)
   (setq _range_perf_error __range_perf_error)
   (setq _internal_error __internal_error)
   (setq _xcvr_operational __xcvr_operational)
   (setq _raw_data_mode __raw_data_mode)
   (setq _steering_angle_ack (round __steering_angle_ack))
   (setq _temperature (round __temperature))
   (setq _veh_spd_comp_factor (float __veh_spd_comp_factor))
   (setq _grouping_mode (round __grouping_mode))
   (setq _yaw_rate_bias (float __yaw_rate_bias))
   (setq _sw_version_dsp (string __sw_version_dsp))
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
  (:maximum_tracks_ack
   (&optional __maximum_tracks_ack)
   (if __maximum_tracks_ack (setq _maximum_tracks_ack __maximum_tracks_ack)) _maximum_tracks_ack)
  (:rolling_count_2
   (&optional __rolling_count_2)
   (if __rolling_count_2 (setq _rolling_count_2 __rolling_count_2)) _rolling_count_2)
  (:overheat_error
   (&optional (__overheat_error :null))
   (if (not (eq __overheat_error :null)) (setq _overheat_error __overheat_error)) _overheat_error)
  (:range_perf_error
   (&optional (__range_perf_error :null))
   (if (not (eq __range_perf_error :null)) (setq _range_perf_error __range_perf_error)) _range_perf_error)
  (:internal_error
   (&optional (__internal_error :null))
   (if (not (eq __internal_error :null)) (setq _internal_error __internal_error)) _internal_error)
  (:xcvr_operational
   (&optional (__xcvr_operational :null))
   (if (not (eq __xcvr_operational :null)) (setq _xcvr_operational __xcvr_operational)) _xcvr_operational)
  (:raw_data_mode
   (&optional (__raw_data_mode :null))
   (if (not (eq __raw_data_mode :null)) (setq _raw_data_mode __raw_data_mode)) _raw_data_mode)
  (:steering_angle_ack
   (&optional __steering_angle_ack)
   (if __steering_angle_ack (setq _steering_angle_ack __steering_angle_ack)) _steering_angle_ack)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:veh_spd_comp_factor
   (&optional __veh_spd_comp_factor)
   (if __veh_spd_comp_factor (setq _veh_spd_comp_factor __veh_spd_comp_factor)) _veh_spd_comp_factor)
  (:grouping_mode
   (&optional __grouping_mode)
   (if __grouping_mode (setq _grouping_mode __grouping_mode)) _grouping_mode)
  (:yaw_rate_bias
   (&optional __yaw_rate_bias)
   (if __yaw_rate_bias (setq _yaw_rate_bias __yaw_rate_bias)) _yaw_rate_bias)
  (:sw_version_dsp
   (&optional __sw_version_dsp)
   (if __sw_version_dsp (setq _sw_version_dsp __sw_version_dsp)) _sw_version_dsp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _maximum_tracks_ack
    1
    ;; uint8 _rolling_count_2
    1
    ;; bool _overheat_error
    1
    ;; bool _range_perf_error
    1
    ;; bool _internal_error
    1
    ;; bool _xcvr_operational
    1
    ;; bool _raw_data_mode
    1
    ;; uint16 _steering_angle_ack
    2
    ;; int8 _temperature
    1
    ;; float32 _veh_spd_comp_factor
    4
    ;; uint8 _grouping_mode
    1
    ;; float32 _yaw_rate_bias
    4
    ;; string _sw_version_dsp
    4 (length _sw_version_dsp)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; uint8 _maximum_tracks_ack
       (write-byte _maximum_tracks_ack s)
     ;; uint8 _rolling_count_2
       (write-byte _rolling_count_2 s)
     ;; bool _overheat_error
       (if _overheat_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _range_perf_error
       (if _range_perf_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _internal_error
       (if _internal_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _xcvr_operational
       (if _xcvr_operational (write-byte -1 s) (write-byte 0 s))
     ;; bool _raw_data_mode
       (if _raw_data_mode (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _steering_angle_ack
       (write-word _steering_angle_ack s)
     ;; int8 _temperature
       (write-byte _temperature s)
     ;; float32 _veh_spd_comp_factor
       (sys::poke _veh_spd_comp_factor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _grouping_mode
       (write-byte _grouping_mode s)
     ;; float32 _yaw_rate_bias
       (sys::poke _yaw_rate_bias (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _sw_version_dsp
       (write-long (length _sw_version_dsp) s) (princ _sw_version_dsp s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _maximum_tracks_ack
     (setq _maximum_tracks_ack (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rolling_count_2
     (setq _rolling_count_2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _overheat_error
     (setq _overheat_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _range_perf_error
     (setq _range_perf_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _internal_error
     (setq _internal_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _xcvr_operational
     (setq _xcvr_operational (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _raw_data_mode
     (setq _raw_data_mode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _steering_angle_ack
     (setq _steering_angle_ack (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int8 _temperature
     (setq _temperature (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _temperature 127) (setq _temperature (- _temperature 256)))
   ;; float32 _veh_spd_comp_factor
     (setq _veh_spd_comp_factor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _grouping_mode
     (setq _grouping_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _yaw_rate_bias
     (setq _yaw_rate_bias (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _sw_version_dsp
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sw_version_dsp (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrStatus2 :md5sum-) "5f62b9bdfb50f851bdab4bdfcce1c49b")
(setf (get delphi_esr_msgs::EsrStatus2 :datatype-) "delphi_esr_msgs/EsrStatus2")
(setf (get delphi_esr_msgs::EsrStatus2 :definition-)
      "std_msgs/Header header

# ESR Status2
string      canmsg

uint8       maximum_tracks_ack
uint8       rolling_count_2
bool        overheat_error
bool        range_perf_error
bool        internal_error
bool        xcvr_operational
bool        raw_data_mode
uint16      steering_angle_ack
int8        temperature
float32     veh_spd_comp_factor
uint8       grouping_mode
float32     yaw_rate_bias
string      sw_version_dsp

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



(provide :delphi_esr_msgs/EsrStatus2 "5f62b9bdfb50f851bdab4bdfcce1c49b")


