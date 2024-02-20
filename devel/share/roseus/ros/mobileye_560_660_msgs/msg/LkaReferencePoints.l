;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::LkaReferencePoints)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'LkaReferencePoints (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::LKAREFERENCEPOINTS")
  (make-package "MOBILEYE_560_660_MSGS::LKAREFERENCEPOINTS"))

(in-package "ROS")
;;//! \htmlinclude LkaReferencePoints.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mobileye_560_660_msgs::LkaReferencePoints
  :super ros::object
  :slots (_header _ref_point_1_position _ref_point_1_distance _ref_point_1_validity _ref_point_2_position _ref_point_2_distance _ref_point_2_validity ))

(defmethod mobileye_560_660_msgs::LkaReferencePoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ref_point_1_position __ref_point_1_position) 0.0)
    ((:ref_point_1_distance __ref_point_1_distance) 0.0)
    ((:ref_point_1_validity __ref_point_1_validity) nil)
    ((:ref_point_2_position __ref_point_2_position) 0.0)
    ((:ref_point_2_distance __ref_point_2_distance) 0.0)
    ((:ref_point_2_validity __ref_point_2_validity) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ref_point_1_position (float __ref_point_1_position))
   (setq _ref_point_1_distance (float __ref_point_1_distance))
   (setq _ref_point_1_validity __ref_point_1_validity)
   (setq _ref_point_2_position (float __ref_point_2_position))
   (setq _ref_point_2_distance (float __ref_point_2_distance))
   (setq _ref_point_2_validity __ref_point_2_validity)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ref_point_1_position
   (&optional __ref_point_1_position)
   (if __ref_point_1_position (setq _ref_point_1_position __ref_point_1_position)) _ref_point_1_position)
  (:ref_point_1_distance
   (&optional __ref_point_1_distance)
   (if __ref_point_1_distance (setq _ref_point_1_distance __ref_point_1_distance)) _ref_point_1_distance)
  (:ref_point_1_validity
   (&optional (__ref_point_1_validity :null))
   (if (not (eq __ref_point_1_validity :null)) (setq _ref_point_1_validity __ref_point_1_validity)) _ref_point_1_validity)
  (:ref_point_2_position
   (&optional __ref_point_2_position)
   (if __ref_point_2_position (setq _ref_point_2_position __ref_point_2_position)) _ref_point_2_position)
  (:ref_point_2_distance
   (&optional __ref_point_2_distance)
   (if __ref_point_2_distance (setq _ref_point_2_distance __ref_point_2_distance)) _ref_point_2_distance)
  (:ref_point_2_validity
   (&optional (__ref_point_2_validity :null))
   (if (not (eq __ref_point_2_validity :null)) (setq _ref_point_2_validity __ref_point_2_validity)) _ref_point_2_validity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _ref_point_1_position
    8
    ;; float64 _ref_point_1_distance
    8
    ;; bool _ref_point_1_validity
    1
    ;; float64 _ref_point_2_position
    8
    ;; float64 _ref_point_2_distance
    8
    ;; bool _ref_point_2_validity
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _ref_point_1_position
       (sys::poke _ref_point_1_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ref_point_1_distance
       (sys::poke _ref_point_1_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _ref_point_1_validity
       (if _ref_point_1_validity (write-byte -1 s) (write-byte 0 s))
     ;; float64 _ref_point_2_position
       (sys::poke _ref_point_2_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ref_point_2_distance
       (sys::poke _ref_point_2_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _ref_point_2_validity
       (if _ref_point_2_validity (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _ref_point_1_position
     (setq _ref_point_1_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ref_point_1_distance
     (setq _ref_point_1_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _ref_point_1_validity
     (setq _ref_point_1_validity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _ref_point_2_position
     (setq _ref_point_2_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ref_point_2_distance
     (setq _ref_point_2_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _ref_point_2_validity
     (setq _ref_point_2_validity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::LkaReferencePoints :md5sum-) "0da833fa4330bb296afc10246a88cb60")
(setf (get mobileye_560_660_msgs::LkaReferencePoints :datatype-) "mobileye_560_660_msgs/LkaReferencePoints")
(setf (get mobileye_560_660_msgs::LkaReferencePoints :definition-)
      "std_msgs/Header header

float64 ref_point_1_position
float64 ref_point_1_distance
bool ref_point_1_validity
float64 ref_point_2_position
float64 ref_point_2_distance
bool ref_point_2_validity

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



(provide :mobileye_560_660_msgs/LkaReferencePoints "0da833fa4330bb296afc10246a88cb60")


