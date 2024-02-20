;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::Lane)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'Lane (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::LANE")
  (make-package "MOBILEYE_560_660_MSGS::LANE"))

(in-package "ROS")
;;//! \htmlinclude Lane.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mobileye_560_660_msgs::Lane
  :super ros::object
  :slots (_header _lane_curvature _lane_heading _construction_area _pitch_angle _yaw_angle _right_ldw_availability _left_ldw_availability ))

(defmethod mobileye_560_660_msgs::Lane
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lane_curvature __lane_curvature) 0.0)
    ((:lane_heading __lane_heading) 0.0)
    ((:construction_area __construction_area) nil)
    ((:pitch_angle __pitch_angle) 0.0)
    ((:yaw_angle __yaw_angle) 0.0)
    ((:right_ldw_availability __right_ldw_availability) nil)
    ((:left_ldw_availability __left_ldw_availability) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lane_curvature (float __lane_curvature))
   (setq _lane_heading (float __lane_heading))
   (setq _construction_area __construction_area)
   (setq _pitch_angle (float __pitch_angle))
   (setq _yaw_angle (float __yaw_angle))
   (setq _right_ldw_availability __right_ldw_availability)
   (setq _left_ldw_availability __left_ldw_availability)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lane_curvature
   (&optional __lane_curvature)
   (if __lane_curvature (setq _lane_curvature __lane_curvature)) _lane_curvature)
  (:lane_heading
   (&optional __lane_heading)
   (if __lane_heading (setq _lane_heading __lane_heading)) _lane_heading)
  (:construction_area
   (&optional (__construction_area :null))
   (if (not (eq __construction_area :null)) (setq _construction_area __construction_area)) _construction_area)
  (:pitch_angle
   (&optional __pitch_angle)
   (if __pitch_angle (setq _pitch_angle __pitch_angle)) _pitch_angle)
  (:yaw_angle
   (&optional __yaw_angle)
   (if __yaw_angle (setq _yaw_angle __yaw_angle)) _yaw_angle)
  (:right_ldw_availability
   (&optional (__right_ldw_availability :null))
   (if (not (eq __right_ldw_availability :null)) (setq _right_ldw_availability __right_ldw_availability)) _right_ldw_availability)
  (:left_ldw_availability
   (&optional (__left_ldw_availability :null))
   (if (not (eq __left_ldw_availability :null)) (setq _left_ldw_availability __left_ldw_availability)) _left_ldw_availability)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _lane_curvature
    8
    ;; float64 _lane_heading
    8
    ;; bool _construction_area
    1
    ;; float64 _pitch_angle
    8
    ;; float64 _yaw_angle
    8
    ;; bool _right_ldw_availability
    1
    ;; bool _left_ldw_availability
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _lane_curvature
       (sys::poke _lane_curvature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _lane_heading
       (sys::poke _lane_heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _construction_area
       (if _construction_area (write-byte -1 s) (write-byte 0 s))
     ;; float64 _pitch_angle
       (sys::poke _pitch_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_angle
       (sys::poke _yaw_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _right_ldw_availability
       (if _right_ldw_availability (write-byte -1 s) (write-byte 0 s))
     ;; bool _left_ldw_availability
       (if _left_ldw_availability (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _lane_curvature
     (setq _lane_curvature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _lane_heading
     (setq _lane_heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _construction_area
     (setq _construction_area (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _pitch_angle
     (setq _pitch_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_angle
     (setq _yaw_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _right_ldw_availability
     (setq _right_ldw_availability (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left_ldw_availability
     (setq _left_ldw_availability (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::Lane :md5sum-) "86e77b353e33571bb5143fbccbe07372")
(setf (get mobileye_560_660_msgs::Lane :datatype-) "mobileye_560_660_msgs/Lane")
(setf (get mobileye_560_660_msgs::Lane :definition-)
      "std_msgs/Header header

float64 lane_curvature
float64 lane_heading
bool construction_area
float64 pitch_angle
float64 yaw_angle
bool right_ldw_availability
bool left_ldw_availability

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



(provide :mobileye_560_660_msgs/Lane "86e77b353e33571bb5143fbccbe07372")


