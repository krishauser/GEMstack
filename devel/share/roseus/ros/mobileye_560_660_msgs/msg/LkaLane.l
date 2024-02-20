;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::LkaLane)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'LkaLane (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::LKALANE")
  (make-package "MOBILEYE_560_660_MSGS::LKALANE"))

(in-package "ROS")
;;//! \htmlinclude LkaLane.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*LANE_CONFIDENCE_NONE*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_CONFIDENCE_NONE* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_NONE* 0)
(intern "*LANE_CONFIDENCE_LOW*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_CONFIDENCE_LOW* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_LOW* 1)
(intern "*LANE_CONFIDENCE_MED*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_CONFIDENCE_MED* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_MED* 2)
(intern "*LANE_CONFIDENCE_HIGH*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_CONFIDENCE_HIGH* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_HIGH* 3)
(intern "*LANE_TYPE_DASHED*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_DASHED* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_DASHED* 0)
(intern "*LANE_TYPE_SOLID*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_SOLID* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_SOLID* 1)
(intern "*LANE_TYPE_NONE*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_NONE* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_NONE* 2)
(intern "*LANE_TYPE_ROAD_EDGE*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_ROAD_EDGE* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_ROAD_EDGE* 3)
(intern "*LANE_TYPE_DOUBLE_LANE_MARK*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_DOUBLE_LANE_MARK* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_DOUBLE_LANE_MARK* 4)
(intern "*LANE_TYPE_BOTTS_DOTS*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_BOTTS_DOTS* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_BOTTS_DOTS* 5)
(intern "*LANE_TYPE_INVALID*" (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(shadow '*LANE_TYPE_INVALID* (find-package "MOBILEYE_560_660_MSGS::LKALANE"))
(defconstant mobileye_560_660_msgs::LkaLane::*LANE_TYPE_INVALID* 6)

(defun mobileye_560_660_msgs::LkaLane-to-symbol (const)
  (cond
        ((= const 0) 'mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_NONE*)
        ((= const 1) 'mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_LOW*)
        ((= const 2) 'mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_MED*)
        ((= const 3) 'mobileye_560_660_msgs::LkaLane::*LANE_CONFIDENCE_HIGH*)
        ((= const 0) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_DASHED*)
        ((= const 1) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_SOLID*)
        ((= const 2) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_NONE*)
        ((= const 3) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_ROAD_EDGE*)
        ((= const 4) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_DOUBLE_LANE_MARK*)
        ((= const 5) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_BOTTS_DOTS*)
        ((= const 6) 'mobileye_560_660_msgs::LkaLane::*LANE_TYPE_INVALID*)
        (t nil)))

(defclass mobileye_560_660_msgs::LkaLane
  :super ros::object
  :slots (_header _lane_type _quality _model_degree _position_parameter_c0 _curvature_parameter_c2 _curvature_derivative_parameter_c3 _marking_width _heading_angle_parameter_c1 _view_range _view_range_availability ))

(defmethod mobileye_560_660_msgs::LkaLane
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lane_type __lane_type) 0)
    ((:quality __quality) 0)
    ((:model_degree __model_degree) 0)
    ((:position_parameter_c0 __position_parameter_c0) 0.0)
    ((:curvature_parameter_c2 __curvature_parameter_c2) 0.0)
    ((:curvature_derivative_parameter_c3 __curvature_derivative_parameter_c3) 0.0)
    ((:marking_width __marking_width) 0.0)
    ((:heading_angle_parameter_c1 __heading_angle_parameter_c1) 0.0)
    ((:view_range __view_range) 0.0)
    ((:view_range_availability __view_range_availability) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lane_type (round __lane_type))
   (setq _quality (round __quality))
   (setq _model_degree (round __model_degree))
   (setq _position_parameter_c0 (float __position_parameter_c0))
   (setq _curvature_parameter_c2 (float __curvature_parameter_c2))
   (setq _curvature_derivative_parameter_c3 (float __curvature_derivative_parameter_c3))
   (setq _marking_width (float __marking_width))
   (setq _heading_angle_parameter_c1 (float __heading_angle_parameter_c1))
   (setq _view_range (float __view_range))
   (setq _view_range_availability __view_range_availability)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lane_type
   (&optional __lane_type)
   (if __lane_type (setq _lane_type __lane_type)) _lane_type)
  (:quality
   (&optional __quality)
   (if __quality (setq _quality __quality)) _quality)
  (:model_degree
   (&optional __model_degree)
   (if __model_degree (setq _model_degree __model_degree)) _model_degree)
  (:position_parameter_c0
   (&optional __position_parameter_c0)
   (if __position_parameter_c0 (setq _position_parameter_c0 __position_parameter_c0)) _position_parameter_c0)
  (:curvature_parameter_c2
   (&optional __curvature_parameter_c2)
   (if __curvature_parameter_c2 (setq _curvature_parameter_c2 __curvature_parameter_c2)) _curvature_parameter_c2)
  (:curvature_derivative_parameter_c3
   (&optional __curvature_derivative_parameter_c3)
   (if __curvature_derivative_parameter_c3 (setq _curvature_derivative_parameter_c3 __curvature_derivative_parameter_c3)) _curvature_derivative_parameter_c3)
  (:marking_width
   (&optional __marking_width)
   (if __marking_width (setq _marking_width __marking_width)) _marking_width)
  (:heading_angle_parameter_c1
   (&optional __heading_angle_parameter_c1)
   (if __heading_angle_parameter_c1 (setq _heading_angle_parameter_c1 __heading_angle_parameter_c1)) _heading_angle_parameter_c1)
  (:view_range
   (&optional __view_range)
   (if __view_range (setq _view_range __view_range)) _view_range)
  (:view_range_availability
   (&optional (__view_range_availability :null))
   (if (not (eq __view_range_availability :null)) (setq _view_range_availability __view_range_availability)) _view_range_availability)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _lane_type
    1
    ;; uint8 _quality
    1
    ;; uint8 _model_degree
    1
    ;; float64 _position_parameter_c0
    8
    ;; float64 _curvature_parameter_c2
    8
    ;; float64 _curvature_derivative_parameter_c3
    8
    ;; float32 _marking_width
    4
    ;; float64 _heading_angle_parameter_c1
    8
    ;; float32 _view_range
    4
    ;; bool _view_range_availability
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _lane_type
       (write-byte _lane_type s)
     ;; uint8 _quality
       (write-byte _quality s)
     ;; uint8 _model_degree
       (write-byte _model_degree s)
     ;; float64 _position_parameter_c0
       (sys::poke _position_parameter_c0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _curvature_parameter_c2
       (sys::poke _curvature_parameter_c2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _curvature_derivative_parameter_c3
       (sys::poke _curvature_derivative_parameter_c3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _marking_width
       (sys::poke _marking_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _heading_angle_parameter_c1
       (sys::poke _heading_angle_parameter_c1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _view_range
       (sys::poke _view_range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _view_range_availability
       (if _view_range_availability (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _lane_type
     (setq _lane_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _quality
     (setq _quality (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _model_degree
     (setq _model_degree (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _position_parameter_c0
     (setq _position_parameter_c0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _curvature_parameter_c2
     (setq _curvature_parameter_c2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _curvature_derivative_parameter_c3
     (setq _curvature_derivative_parameter_c3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _marking_width
     (setq _marking_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _heading_angle_parameter_c1
     (setq _heading_angle_parameter_c1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _view_range
     (setq _view_range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _view_range_availability
     (setq _view_range_availability (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::LkaLane :md5sum-) "13c7b357c14488be92473cab7e5461ca")
(setf (get mobileye_560_660_msgs::LkaLane :datatype-) "mobileye_560_660_msgs/LkaLane")
(setf (get mobileye_560_660_msgs::LkaLane :definition-)
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

uint8 lane_type
uint8 quality
uint8 model_degree
float64 position_parameter_c0
float64 curvature_parameter_c2
float64 curvature_derivative_parameter_c3
float32 marking_width
float64 heading_angle_parameter_c1
float32 view_range
bool view_range_availability

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



(provide :mobileye_560_660_msgs/LkaLane "13c7b357c14488be92473cab7e5461ca")


