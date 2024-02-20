;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrDetection)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrDetection (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRDETECTION")
  (make-package "DELPHI_MRR_MSGS::MRRDETECTION"))

(in-package "ROS")
;;//! \htmlinclude MrrDetection.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrDetection
  :super ros::object
  :slots (_header _detection_id _confid_azimuth _super_res_target _nd_target _host_veh_clutter _valid_level _azimuth _range _range_rate _amplitude _index_2lsb ))

(defmethod delphi_mrr_msgs::MrrDetection
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detection_id __detection_id) 0)
    ((:confid_azimuth __confid_azimuth) 0)
    ((:super_res_target __super_res_target) nil)
    ((:nd_target __nd_target) nil)
    ((:host_veh_clutter __host_veh_clutter) nil)
    ((:valid_level __valid_level) nil)
    ((:azimuth __azimuth) 0.0)
    ((:range __range) 0.0)
    ((:range_rate __range_rate) 0.0)
    ((:amplitude __amplitude) 0)
    ((:index_2lsb __index_2lsb) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _detection_id (round __detection_id))
   (setq _confid_azimuth (round __confid_azimuth))
   (setq _super_res_target __super_res_target)
   (setq _nd_target __nd_target)
   (setq _host_veh_clutter __host_veh_clutter)
   (setq _valid_level __valid_level)
   (setq _azimuth (float __azimuth))
   (setq _range (float __range))
   (setq _range_rate (float __range_rate))
   (setq _amplitude (round __amplitude))
   (setq _index_2lsb (round __index_2lsb))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detection_id
   (&optional __detection_id)
   (if __detection_id (setq _detection_id __detection_id)) _detection_id)
  (:confid_azimuth
   (&optional __confid_azimuth)
   (if __confid_azimuth (setq _confid_azimuth __confid_azimuth)) _confid_azimuth)
  (:super_res_target
   (&optional (__super_res_target :null))
   (if (not (eq __super_res_target :null)) (setq _super_res_target __super_res_target)) _super_res_target)
  (:nd_target
   (&optional (__nd_target :null))
   (if (not (eq __nd_target :null)) (setq _nd_target __nd_target)) _nd_target)
  (:host_veh_clutter
   (&optional (__host_veh_clutter :null))
   (if (not (eq __host_veh_clutter :null)) (setq _host_veh_clutter __host_veh_clutter)) _host_veh_clutter)
  (:valid_level
   (&optional (__valid_level :null))
   (if (not (eq __valid_level :null)) (setq _valid_level __valid_level)) _valid_level)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:range_rate
   (&optional __range_rate)
   (if __range_rate (setq _range_rate __range_rate)) _range_rate)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:index_2lsb
   (&optional __index_2lsb)
   (if __index_2lsb (setq _index_2lsb __index_2lsb)) _index_2lsb)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _detection_id
    1
    ;; uint8 _confid_azimuth
    1
    ;; bool _super_res_target
    1
    ;; bool _nd_target
    1
    ;; bool _host_veh_clutter
    1
    ;; bool _valid_level
    1
    ;; float32 _azimuth
    4
    ;; float32 _range
    4
    ;; float32 _range_rate
    4
    ;; int8 _amplitude
    1
    ;; uint8 _index_2lsb
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _detection_id
       (write-byte _detection_id s)
     ;; uint8 _confid_azimuth
       (write-byte _confid_azimuth s)
     ;; bool _super_res_target
       (if _super_res_target (write-byte -1 s) (write-byte 0 s))
     ;; bool _nd_target
       (if _nd_target (write-byte -1 s) (write-byte 0 s))
     ;; bool _host_veh_clutter
       (if _host_veh_clutter (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid_level
       (if _valid_level (write-byte -1 s) (write-byte 0 s))
     ;; float32 _azimuth
       (sys::poke _azimuth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range_rate
       (sys::poke _range_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _amplitude
       (write-byte _amplitude s)
     ;; uint8 _index_2lsb
       (write-byte _index_2lsb s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _detection_id
     (setq _detection_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _confid_azimuth
     (setq _confid_azimuth (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _super_res_target
     (setq _super_res_target (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _nd_target
     (setq _nd_target (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _host_veh_clutter
     (setq _host_veh_clutter (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid_level
     (setq _valid_level (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _azimuth
     (setq _azimuth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range_rate
     (setq _range_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _amplitude
     (setq _amplitude (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _amplitude 127) (setq _amplitude (- _amplitude 256)))
   ;; uint8 _index_2lsb
     (setq _index_2lsb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrDetection :md5sum-) "beed6f988400c44635b6b62be6463175")
(setf (get delphi_mrr_msgs::MrrDetection :datatype-) "delphi_mrr_msgs/MrrDetection")
(setf (get delphi_mrr_msgs::MrrDetection :definition-)
      "std_msgs/Header header

uint8   detection_id
uint8   confid_azimuth
bool    super_res_target
bool    nd_target
bool    host_veh_clutter
bool    valid_level
float32 azimuth
float32 range
float32 range_rate
int8    amplitude
uint8   index_2lsb

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



(provide :delphi_mrr_msgs/MrrDetection "beed6f988400c44635b6b62be6463175")


