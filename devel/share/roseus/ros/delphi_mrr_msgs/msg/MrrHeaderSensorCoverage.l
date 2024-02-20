;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrHeaderSensorCoverage)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrHeaderSensorCoverage (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRHEADERSENSORCOVERAGE")
  (make-package "DELPHI_MRR_MSGS::MRRHEADERSENSORCOVERAGE"))

(in-package "ROS")
;;//! \htmlinclude MrrHeaderSensorCoverage.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrHeaderSensorCoverage
  :super ros::object
  :slots (_header _can_sensor_fov_hor _can_doppler_coverage _can_range_coverage ))

(defmethod delphi_mrr_msgs::MrrHeaderSensorCoverage
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_sensor_fov_hor __can_sensor_fov_hor) 0)
    ((:can_doppler_coverage __can_doppler_coverage) 0)
    ((:can_range_coverage __can_range_coverage) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_sensor_fov_hor (round __can_sensor_fov_hor))
   (setq _can_doppler_coverage (round __can_doppler_coverage))
   (setq _can_range_coverage (round __can_range_coverage))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_sensor_fov_hor
   (&optional __can_sensor_fov_hor)
   (if __can_sensor_fov_hor (setq _can_sensor_fov_hor __can_sensor_fov_hor)) _can_sensor_fov_hor)
  (:can_doppler_coverage
   (&optional __can_doppler_coverage)
   (if __can_doppler_coverage (setq _can_doppler_coverage __can_doppler_coverage)) _can_doppler_coverage)
  (:can_range_coverage
   (&optional __can_range_coverage)
   (if __can_range_coverage (setq _can_range_coverage __can_range_coverage)) _can_range_coverage)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _can_sensor_fov_hor
    1
    ;; int8 _can_doppler_coverage
    1
    ;; uint8 _can_range_coverage
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _can_sensor_fov_hor
       (write-byte _can_sensor_fov_hor s)
     ;; int8 _can_doppler_coverage
       (write-byte _can_doppler_coverage s)
     ;; uint8 _can_range_coverage
       (write-byte _can_range_coverage s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _can_sensor_fov_hor
     (setq _can_sensor_fov_hor (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int8 _can_doppler_coverage
     (setq _can_doppler_coverage (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _can_doppler_coverage 127) (setq _can_doppler_coverage (- _can_doppler_coverage 256)))
   ;; uint8 _can_range_coverage
     (setq _can_range_coverage (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrHeaderSensorCoverage :md5sum-) "138445a2498338a847056e570b23da55")
(setf (get delphi_mrr_msgs::MrrHeaderSensorCoverage :datatype-) "delphi_mrr_msgs/MrrHeaderSensorCoverage")
(setf (get delphi_mrr_msgs::MrrHeaderSensorCoverage :definition-)
      "std_msgs/Header header

uint8 can_sensor_fov_hor
int8  can_doppler_coverage
uint8 can_range_coverage

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



(provide :delphi_mrr_msgs/MrrHeaderSensorCoverage "138445a2498338a847056e570b23da55")


