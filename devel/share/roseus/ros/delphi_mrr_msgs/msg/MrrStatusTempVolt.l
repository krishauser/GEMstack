;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrStatusTempVolt)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrStatusTempVolt (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRSTATUSTEMPVOLT")
  (make-package "DELPHI_MRR_MSGS::MRRSTATUSTEMPVOLT"))

(in-package "ROS")
;;//! \htmlinclude MrrStatusTempVolt.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrStatusTempVolt
  :super ros::object
  :slots (_header _can_batt_volts _can_1_25_v _can_5_v _can_3_3_v_raw _can_3_3_v_dac _can_mmic_temp1 _can_processor_thermistor _can_processor_temp1 ))

(defmethod delphi_mrr_msgs::MrrStatusTempVolt
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_batt_volts __can_batt_volts) 0.0)
    ((:can_1_25_v __can_1_25_v) 0.0)
    ((:can_5_v __can_5_v) 0.0)
    ((:can_3_3_v_raw __can_3_3_v_raw) 0.0)
    ((:can_3_3_v_dac __can_3_3_v_dac) 0.0)
    ((:can_mmic_temp1 __can_mmic_temp1) 0)
    ((:can_processor_thermistor __can_processor_thermistor) 0)
    ((:can_processor_temp1 __can_processor_temp1) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_batt_volts (float __can_batt_volts))
   (setq _can_1_25_v (float __can_1_25_v))
   (setq _can_5_v (float __can_5_v))
   (setq _can_3_3_v_raw (float __can_3_3_v_raw))
   (setq _can_3_3_v_dac (float __can_3_3_v_dac))
   (setq _can_mmic_temp1 (round __can_mmic_temp1))
   (setq _can_processor_thermistor (round __can_processor_thermistor))
   (setq _can_processor_temp1 (round __can_processor_temp1))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_batt_volts
   (&optional __can_batt_volts)
   (if __can_batt_volts (setq _can_batt_volts __can_batt_volts)) _can_batt_volts)
  (:can_1_25_v
   (&optional __can_1_25_v)
   (if __can_1_25_v (setq _can_1_25_v __can_1_25_v)) _can_1_25_v)
  (:can_5_v
   (&optional __can_5_v)
   (if __can_5_v (setq _can_5_v __can_5_v)) _can_5_v)
  (:can_3_3_v_raw
   (&optional __can_3_3_v_raw)
   (if __can_3_3_v_raw (setq _can_3_3_v_raw __can_3_3_v_raw)) _can_3_3_v_raw)
  (:can_3_3_v_dac
   (&optional __can_3_3_v_dac)
   (if __can_3_3_v_dac (setq _can_3_3_v_dac __can_3_3_v_dac)) _can_3_3_v_dac)
  (:can_mmic_temp1
   (&optional __can_mmic_temp1)
   (if __can_mmic_temp1 (setq _can_mmic_temp1 __can_mmic_temp1)) _can_mmic_temp1)
  (:can_processor_thermistor
   (&optional __can_processor_thermistor)
   (if __can_processor_thermistor (setq _can_processor_thermistor __can_processor_thermistor)) _can_processor_thermistor)
  (:can_processor_temp1
   (&optional __can_processor_temp1)
   (if __can_processor_temp1 (setq _can_processor_temp1 __can_processor_temp1)) _can_processor_temp1)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _can_batt_volts
    4
    ;; float32 _can_1_25_v
    4
    ;; float32 _can_5_v
    4
    ;; float32 _can_3_3_v_raw
    4
    ;; float32 _can_3_3_v_dac
    4
    ;; int8 _can_mmic_temp1
    1
    ;; int8 _can_processor_thermistor
    1
    ;; int8 _can_processor_temp1
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _can_batt_volts
       (sys::poke _can_batt_volts (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_1_25_v
       (sys::poke _can_1_25_v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_5_v
       (sys::poke _can_5_v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_3_3_v_raw
       (sys::poke _can_3_3_v_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_3_3_v_dac
       (sys::poke _can_3_3_v_dac (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _can_mmic_temp1
       (write-byte _can_mmic_temp1 s)
     ;; int8 _can_processor_thermistor
       (write-byte _can_processor_thermistor s)
     ;; int8 _can_processor_temp1
       (write-byte _can_processor_temp1 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _can_batt_volts
     (setq _can_batt_volts (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_1_25_v
     (setq _can_1_25_v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_5_v
     (setq _can_5_v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_3_3_v_raw
     (setq _can_3_3_v_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_3_3_v_dac
     (setq _can_3_3_v_dac (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _can_mmic_temp1
     (setq _can_mmic_temp1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _can_mmic_temp1 127) (setq _can_mmic_temp1 (- _can_mmic_temp1 256)))
   ;; int8 _can_processor_thermistor
     (setq _can_processor_thermistor (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _can_processor_thermistor 127) (setq _can_processor_thermistor (- _can_processor_thermistor 256)))
   ;; int8 _can_processor_temp1
     (setq _can_processor_temp1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _can_processor_temp1 127) (setq _can_processor_temp1 (- _can_processor_temp1 256)))
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrStatusTempVolt :md5sum-) "4200163646320431c4493e783122961d")
(setf (get delphi_mrr_msgs::MrrStatusTempVolt :datatype-) "delphi_mrr_msgs/MrrStatusTempVolt")
(setf (get delphi_mrr_msgs::MrrStatusTempVolt :definition-)
      "std_msgs/Header header

float32 can_batt_volts
float32 can_1_25_v
float32 can_5_v
float32 can_3_3_v_raw
float32 can_3_3_v_dac
int8    can_mmic_temp1
int8    can_processor_thermistor
int8    can_processor_temp1

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



(provide :delphi_mrr_msgs/MrrStatusTempVolt "4200163646320431c4493e783122961d")


