;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::AhbcGradual)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'AhbcGradual (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::AHBCGRADUAL")
  (make-package "MOBILEYE_560_660_MSGS::AHBCGRADUAL"))

(in-package "ROS")
;;//! \htmlinclude AhbcGradual.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mobileye_560_660_msgs::AhbcGradual
  :super ros::object
  :slots (_header _boundary_domain_bottom_non_glare_hlb _boundary_domain_non_glare_left_hand_hlb _boundary_domain_non_glare_right_hand_hlb _object_distance_hlb _status_boundary_domain_bottom_non_glare_hlb _status_boundary_domain_non_glare_left_hand_hlb _status_boundary_domain_non_glare_right_hand_hlb _status_object_distance_hlb _left_target_change _right_target_change _too_many_cars _busy_scene ))

(defmethod mobileye_560_660_msgs::AhbcGradual
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:boundary_domain_bottom_non_glare_hlb __boundary_domain_bottom_non_glare_hlb) 0.0)
    ((:boundary_domain_non_glare_left_hand_hlb __boundary_domain_non_glare_left_hand_hlb) 0.0)
    ((:boundary_domain_non_glare_right_hand_hlb __boundary_domain_non_glare_right_hand_hlb) 0.0)
    ((:object_distance_hlb __object_distance_hlb) 0)
    ((:status_boundary_domain_bottom_non_glare_hlb __status_boundary_domain_bottom_non_glare_hlb) 0)
    ((:status_boundary_domain_non_glare_left_hand_hlb __status_boundary_domain_non_glare_left_hand_hlb) 0)
    ((:status_boundary_domain_non_glare_right_hand_hlb __status_boundary_domain_non_glare_right_hand_hlb) 0)
    ((:status_object_distance_hlb __status_object_distance_hlb) 0)
    ((:left_target_change __left_target_change) nil)
    ((:right_target_change __right_target_change) nil)
    ((:too_many_cars __too_many_cars) nil)
    ((:busy_scene __busy_scene) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _boundary_domain_bottom_non_glare_hlb (float __boundary_domain_bottom_non_glare_hlb))
   (setq _boundary_domain_non_glare_left_hand_hlb (float __boundary_domain_non_glare_left_hand_hlb))
   (setq _boundary_domain_non_glare_right_hand_hlb (float __boundary_domain_non_glare_right_hand_hlb))
   (setq _object_distance_hlb (round __object_distance_hlb))
   (setq _status_boundary_domain_bottom_non_glare_hlb (round __status_boundary_domain_bottom_non_glare_hlb))
   (setq _status_boundary_domain_non_glare_left_hand_hlb (round __status_boundary_domain_non_glare_left_hand_hlb))
   (setq _status_boundary_domain_non_glare_right_hand_hlb (round __status_boundary_domain_non_glare_right_hand_hlb))
   (setq _status_object_distance_hlb (round __status_object_distance_hlb))
   (setq _left_target_change __left_target_change)
   (setq _right_target_change __right_target_change)
   (setq _too_many_cars __too_many_cars)
   (setq _busy_scene __busy_scene)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:boundary_domain_bottom_non_glare_hlb
   (&optional __boundary_domain_bottom_non_glare_hlb)
   (if __boundary_domain_bottom_non_glare_hlb (setq _boundary_domain_bottom_non_glare_hlb __boundary_domain_bottom_non_glare_hlb)) _boundary_domain_bottom_non_glare_hlb)
  (:boundary_domain_non_glare_left_hand_hlb
   (&optional __boundary_domain_non_glare_left_hand_hlb)
   (if __boundary_domain_non_glare_left_hand_hlb (setq _boundary_domain_non_glare_left_hand_hlb __boundary_domain_non_glare_left_hand_hlb)) _boundary_domain_non_glare_left_hand_hlb)
  (:boundary_domain_non_glare_right_hand_hlb
   (&optional __boundary_domain_non_glare_right_hand_hlb)
   (if __boundary_domain_non_glare_right_hand_hlb (setq _boundary_domain_non_glare_right_hand_hlb __boundary_domain_non_glare_right_hand_hlb)) _boundary_domain_non_glare_right_hand_hlb)
  (:object_distance_hlb
   (&optional __object_distance_hlb)
   (if __object_distance_hlb (setq _object_distance_hlb __object_distance_hlb)) _object_distance_hlb)
  (:status_boundary_domain_bottom_non_glare_hlb
   (&optional __status_boundary_domain_bottom_non_glare_hlb)
   (if __status_boundary_domain_bottom_non_glare_hlb (setq _status_boundary_domain_bottom_non_glare_hlb __status_boundary_domain_bottom_non_glare_hlb)) _status_boundary_domain_bottom_non_glare_hlb)
  (:status_boundary_domain_non_glare_left_hand_hlb
   (&optional __status_boundary_domain_non_glare_left_hand_hlb)
   (if __status_boundary_domain_non_glare_left_hand_hlb (setq _status_boundary_domain_non_glare_left_hand_hlb __status_boundary_domain_non_glare_left_hand_hlb)) _status_boundary_domain_non_glare_left_hand_hlb)
  (:status_boundary_domain_non_glare_right_hand_hlb
   (&optional __status_boundary_domain_non_glare_right_hand_hlb)
   (if __status_boundary_domain_non_glare_right_hand_hlb (setq _status_boundary_domain_non_glare_right_hand_hlb __status_boundary_domain_non_glare_right_hand_hlb)) _status_boundary_domain_non_glare_right_hand_hlb)
  (:status_object_distance_hlb
   (&optional __status_object_distance_hlb)
   (if __status_object_distance_hlb (setq _status_object_distance_hlb __status_object_distance_hlb)) _status_object_distance_hlb)
  (:left_target_change
   (&optional (__left_target_change :null))
   (if (not (eq __left_target_change :null)) (setq _left_target_change __left_target_change)) _left_target_change)
  (:right_target_change
   (&optional (__right_target_change :null))
   (if (not (eq __right_target_change :null)) (setq _right_target_change __right_target_change)) _right_target_change)
  (:too_many_cars
   (&optional (__too_many_cars :null))
   (if (not (eq __too_many_cars :null)) (setq _too_many_cars __too_many_cars)) _too_many_cars)
  (:busy_scene
   (&optional (__busy_scene :null))
   (if (not (eq __busy_scene :null)) (setq _busy_scene __busy_scene)) _busy_scene)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _boundary_domain_bottom_non_glare_hlb
    4
    ;; float32 _boundary_domain_non_glare_left_hand_hlb
    4
    ;; float32 _boundary_domain_non_glare_right_hand_hlb
    4
    ;; uint16 _object_distance_hlb
    2
    ;; uint8 _status_boundary_domain_bottom_non_glare_hlb
    1
    ;; uint8 _status_boundary_domain_non_glare_left_hand_hlb
    1
    ;; uint8 _status_boundary_domain_non_glare_right_hand_hlb
    1
    ;; uint8 _status_object_distance_hlb
    1
    ;; bool _left_target_change
    1
    ;; bool _right_target_change
    1
    ;; bool _too_many_cars
    1
    ;; bool _busy_scene
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _boundary_domain_bottom_non_glare_hlb
       (sys::poke _boundary_domain_bottom_non_glare_hlb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _boundary_domain_non_glare_left_hand_hlb
       (sys::poke _boundary_domain_non_glare_left_hand_hlb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _boundary_domain_non_glare_right_hand_hlb
       (sys::poke _boundary_domain_non_glare_right_hand_hlb (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _object_distance_hlb
       (write-word _object_distance_hlb s)
     ;; uint8 _status_boundary_domain_bottom_non_glare_hlb
       (write-byte _status_boundary_domain_bottom_non_glare_hlb s)
     ;; uint8 _status_boundary_domain_non_glare_left_hand_hlb
       (write-byte _status_boundary_domain_non_glare_left_hand_hlb s)
     ;; uint8 _status_boundary_domain_non_glare_right_hand_hlb
       (write-byte _status_boundary_domain_non_glare_right_hand_hlb s)
     ;; uint8 _status_object_distance_hlb
       (write-byte _status_object_distance_hlb s)
     ;; bool _left_target_change
       (if _left_target_change (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_target_change
       (if _right_target_change (write-byte -1 s) (write-byte 0 s))
     ;; bool _too_many_cars
       (if _too_many_cars (write-byte -1 s) (write-byte 0 s))
     ;; bool _busy_scene
       (if _busy_scene (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _boundary_domain_bottom_non_glare_hlb
     (setq _boundary_domain_bottom_non_glare_hlb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _boundary_domain_non_glare_left_hand_hlb
     (setq _boundary_domain_non_glare_left_hand_hlb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _boundary_domain_non_glare_right_hand_hlb
     (setq _boundary_domain_non_glare_right_hand_hlb (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _object_distance_hlb
     (setq _object_distance_hlb (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _status_boundary_domain_bottom_non_glare_hlb
     (setq _status_boundary_domain_bottom_non_glare_hlb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _status_boundary_domain_non_glare_left_hand_hlb
     (setq _status_boundary_domain_non_glare_left_hand_hlb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _status_boundary_domain_non_glare_right_hand_hlb
     (setq _status_boundary_domain_non_glare_right_hand_hlb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _status_object_distance_hlb
     (setq _status_object_distance_hlb (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _left_target_change
     (setq _left_target_change (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_target_change
     (setq _right_target_change (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _too_many_cars
     (setq _too_many_cars (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _busy_scene
     (setq _busy_scene (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::AhbcGradual :md5sum-) "06801ea66cd7dc52de9867c12dbfa5bf")
(setf (get mobileye_560_660_msgs::AhbcGradual :datatype-) "mobileye_560_660_msgs/AhbcGradual")
(setf (get mobileye_560_660_msgs::AhbcGradual :definition-)
      "std_msgs/Header header

float32 boundary_domain_bottom_non_glare_hlb
float32 boundary_domain_non_glare_left_hand_hlb
float32 boundary_domain_non_glare_right_hand_hlb
uint16 object_distance_hlb
uint8 status_boundary_domain_bottom_non_glare_hlb
uint8 status_boundary_domain_non_glare_left_hand_hlb
uint8 status_boundary_domain_non_glare_right_hand_hlb
uint8 status_object_distance_hlb
bool left_target_change
bool right_target_change
bool too_many_cars
bool busy_scene

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



(provide :mobileye_560_660_msgs/AhbcGradual "06801ea66cd7dc52de9867c12dbfa5bf")


