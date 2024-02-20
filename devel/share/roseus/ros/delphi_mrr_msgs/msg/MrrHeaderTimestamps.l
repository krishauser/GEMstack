;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrHeaderTimestamps)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrHeaderTimestamps (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRHEADERTIMESTAMPS")
  (make-package "DELPHI_MRR_MSGS::MRRHEADERTIMESTAMPS"))

(in-package "ROS")
;;//! \htmlinclude MrrHeaderTimestamps.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrHeaderTimestamps
  :super ros::object
  :slots (_header _can_det_time_since_meas _can_sensor_time_stamp ))

(defmethod delphi_mrr_msgs::MrrHeaderTimestamps
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_det_time_since_meas __can_det_time_since_meas) 0.0)
    ((:can_sensor_time_stamp __can_sensor_time_stamp) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_det_time_since_meas (float __can_det_time_since_meas))
   (setq _can_sensor_time_stamp (float __can_sensor_time_stamp))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_det_time_since_meas
   (&optional __can_det_time_since_meas)
   (if __can_det_time_since_meas (setq _can_det_time_since_meas __can_det_time_since_meas)) _can_det_time_since_meas)
  (:can_sensor_time_stamp
   (&optional __can_sensor_time_stamp)
   (if __can_sensor_time_stamp (setq _can_sensor_time_stamp __can_sensor_time_stamp)) _can_sensor_time_stamp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _can_det_time_since_meas
    4
    ;; float32 _can_sensor_time_stamp
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _can_det_time_since_meas
       (sys::poke _can_det_time_since_meas (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_sensor_time_stamp
       (sys::poke _can_sensor_time_stamp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _can_det_time_since_meas
     (setq _can_det_time_since_meas (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_sensor_time_stamp
     (setq _can_sensor_time_stamp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrHeaderTimestamps :md5sum-) "31560a809bee8d977f1d25fd94db961e")
(setf (get delphi_mrr_msgs::MrrHeaderTimestamps :datatype-) "delphi_mrr_msgs/MrrHeaderTimestamps")
(setf (get delphi_mrr_msgs::MrrHeaderTimestamps :definition-)
      "std_msgs/Header header

float32 can_det_time_since_meas
float32 can_sensor_time_stamp

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



(provide :delphi_mrr_msgs/MrrHeaderTimestamps "31560a809bee8d977f1d25fd94db961e")


