;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::AftermarketLane)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'AftermarketLane (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE")
  (make-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))

(in-package "ROS")
;;//! \htmlinclude AftermarketLane.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*LANE_CONFIDENCE_NONE*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_CONFIDENCE_NONE* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_NONE* 0)
(intern "*LANE_CONFIDENCE_LOW*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_CONFIDENCE_LOW* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_LOW* 1)
(intern "*LANE_CONFIDENCE_MED*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_CONFIDENCE_MED* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_MED* 2)
(intern "*LANE_CONFIDENCE_HIGH*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_CONFIDENCE_HIGH* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_HIGH* 3)
(intern "*LANE_TYPE_DASHED*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_DASHED* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_DASHED* 0)
(intern "*LANE_TYPE_SOLID*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_SOLID* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_SOLID* 1)
(intern "*LANE_TYPE_NONE*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_NONE* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_NONE* 2)
(intern "*LANE_TYPE_ROAD_EDGE*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_ROAD_EDGE* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_ROAD_EDGE* 3)
(intern "*LANE_TYPE_DOUBLE_LANE_MARK*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_DOUBLE_LANE_MARK* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_DOUBLE_LANE_MARK* 4)
(intern "*LANE_TYPE_BOTTS_DOTS*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_BOTTS_DOTS* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_BOTTS_DOTS* 5)
(intern "*LANE_TYPE_INVALID*" (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(shadow '*LANE_TYPE_INVALID* (find-package "MOBILEYE_560_660_MSGS::AFTERMARKETLANE"))
(defconstant mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_INVALID* 6)

(defun mobileye_560_660_msgs::AftermarketLane-to-symbol (const)
  (cond
        ((= const 0) 'mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_NONE*)
        ((= const 1) 'mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_LOW*)
        ((= const 2) 'mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_MED*)
        ((= const 3) 'mobileye_560_660_msgs::AftermarketLane::*LANE_CONFIDENCE_HIGH*)
        ((= const 0) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_DASHED*)
        ((= const 1) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_SOLID*)
        ((= const 2) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_NONE*)
        ((= const 3) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_ROAD_EDGE*)
        ((= const 4) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_DOUBLE_LANE_MARK*)
        ((= const 5) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_BOTTS_DOTS*)
        ((= const 6) 'mobileye_560_660_msgs::AftermarketLane::*LANE_TYPE_INVALID*)
        (t nil)))

(defclass mobileye_560_660_msgs::AftermarketLane
  :super ros::object
  :slots (_header _lane_confidence_left _ldw_available_left _lane_type_left _distance_to_left_lane _lane_confidence_right _ldw_available_right _lane_type_right _distance_to_right_lane ))

(defmethod mobileye_560_660_msgs::AftermarketLane
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lane_confidence_left __lane_confidence_left) 0)
    ((:ldw_available_left __ldw_available_left) nil)
    ((:lane_type_left __lane_type_left) 0)
    ((:distance_to_left_lane __distance_to_left_lane) 0.0)
    ((:lane_confidence_right __lane_confidence_right) 0)
    ((:ldw_available_right __ldw_available_right) nil)
    ((:lane_type_right __lane_type_right) 0)
    ((:distance_to_right_lane __distance_to_right_lane) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lane_confidence_left (round __lane_confidence_left))
   (setq _ldw_available_left __ldw_available_left)
   (setq _lane_type_left (round __lane_type_left))
   (setq _distance_to_left_lane (float __distance_to_left_lane))
   (setq _lane_confidence_right (round __lane_confidence_right))
   (setq _ldw_available_right __ldw_available_right)
   (setq _lane_type_right (round __lane_type_right))
   (setq _distance_to_right_lane (float __distance_to_right_lane))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lane_confidence_left
   (&optional __lane_confidence_left)
   (if __lane_confidence_left (setq _lane_confidence_left __lane_confidence_left)) _lane_confidence_left)
  (:ldw_available_left
   (&optional (__ldw_available_left :null))
   (if (not (eq __ldw_available_left :null)) (setq _ldw_available_left __ldw_available_left)) _ldw_available_left)
  (:lane_type_left
   (&optional __lane_type_left)
   (if __lane_type_left (setq _lane_type_left __lane_type_left)) _lane_type_left)
  (:distance_to_left_lane
   (&optional __distance_to_left_lane)
   (if __distance_to_left_lane (setq _distance_to_left_lane __distance_to_left_lane)) _distance_to_left_lane)
  (:lane_confidence_right
   (&optional __lane_confidence_right)
   (if __lane_confidence_right (setq _lane_confidence_right __lane_confidence_right)) _lane_confidence_right)
  (:ldw_available_right
   (&optional (__ldw_available_right :null))
   (if (not (eq __ldw_available_right :null)) (setq _ldw_available_right __ldw_available_right)) _ldw_available_right)
  (:lane_type_right
   (&optional __lane_type_right)
   (if __lane_type_right (setq _lane_type_right __lane_type_right)) _lane_type_right)
  (:distance_to_right_lane
   (&optional __distance_to_right_lane)
   (if __distance_to_right_lane (setq _distance_to_right_lane __distance_to_right_lane)) _distance_to_right_lane)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _lane_confidence_left
    1
    ;; bool _ldw_available_left
    1
    ;; uint8 _lane_type_left
    1
    ;; float32 _distance_to_left_lane
    4
    ;; uint8 _lane_confidence_right
    1
    ;; bool _ldw_available_right
    1
    ;; uint8 _lane_type_right
    1
    ;; float32 _distance_to_right_lane
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _lane_confidence_left
       (write-byte _lane_confidence_left s)
     ;; bool _ldw_available_left
       (if _ldw_available_left (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _lane_type_left
       (write-byte _lane_type_left s)
     ;; float32 _distance_to_left_lane
       (sys::poke _distance_to_left_lane (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _lane_confidence_right
       (write-byte _lane_confidence_right s)
     ;; bool _ldw_available_right
       (if _ldw_available_right (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _lane_type_right
       (write-byte _lane_type_right s)
     ;; float32 _distance_to_right_lane
       (sys::poke _distance_to_right_lane (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _lane_confidence_left
     (setq _lane_confidence_left (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _ldw_available_left
     (setq _ldw_available_left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _lane_type_left
     (setq _lane_type_left (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _distance_to_left_lane
     (setq _distance_to_left_lane (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _lane_confidence_right
     (setq _lane_confidence_right (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _ldw_available_right
     (setq _ldw_available_right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _lane_type_right
     (setq _lane_type_right (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _distance_to_right_lane
     (setq _distance_to_right_lane (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::AftermarketLane :md5sum-) "8a56b7a5f0247252831a59dfc0910af7")
(setf (get mobileye_560_660_msgs::AftermarketLane :datatype-) "mobileye_560_660_msgs/AftermarketLane")
(setf (get mobileye_560_660_msgs::AftermarketLane :definition-)
      "std_msgs/Header header

uint8 LANE_CONFIDENCE_NONE = 0
uint8 LANE_CONFIDENCE_LOW = 1
uint8 LANE_CONFIDENCE_MED = 2
uint8 LANE_CONFIDENCE_HIGH = 3

uint8 LANE_TYPE_DASHED = 0
uint8 LANE_TYPE_SOLID = 1
uint8 LANE_TYPE_NONE = 2
uint8 LANE_TYPE_ROAD_EDGE = 3
uint8 LANE_TYPE_DOUBLE_LANE_MARK = 4
uint8 LANE_TYPE_BOTTS_DOTS = 5
uint8 LANE_TYPE_INVALID = 6

uint8 lane_confidence_left
bool ldw_available_left
uint8 lane_type_left
float32 distance_to_left_lane
uint8 lane_confidence_right
bool ldw_available_right
uint8 lane_type_right
float32 distance_to_right_lane

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



(provide :mobileye_560_660_msgs/AftermarketLane "8a56b7a5f0247252831a59dfc0910af7")


