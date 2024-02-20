;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::ObstacleData)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'ObstacleData (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA")
  (make-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))

(in-package "ROS")
;;//! \htmlinclude ObstacleData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*BLINKER_INFO_UNAVAILABLE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*BLINKER_INFO_UNAVAILABLE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_UNAVAILABLE* 0)
(intern "*BLINKER_INFO_OFF*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*BLINKER_INFO_OFF* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_OFF* 1)
(intern "*BLINKER_INFO_LEFT*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*BLINKER_INFO_LEFT* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_LEFT* 2)
(intern "*BLINKER_INFO_RIGHT*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*BLINKER_INFO_RIGHT* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_RIGHT* 3)
(intern "*BLINKER_INFO_BOTH*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*BLINKER_INFO_BOTH* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_BOTH* 4)
(intern "*CUT_IN_AND_OUT_UNDEFINED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*CUT_IN_AND_OUT_UNDEFINED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_UNDEFINED* 0)
(intern "*CUT_IN_AND_OUT_IN_HOST_LANE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*CUT_IN_AND_OUT_IN_HOST_LANE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_IN_HOST_LANE* 1)
(intern "*CUT_IN_AND_OUT_OUT_HOST_LANE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*CUT_IN_AND_OUT_OUT_HOST_LANE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_OUT_HOST_LANE* 2)
(intern "*CUT_IN_AND_OUT_CUT_IN*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*CUT_IN_AND_OUT_CUT_IN* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_CUT_IN* 3)
(intern "*CUT_IN_AND_OUT_CUT_OUT*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*CUT_IN_AND_OUT_CUT_OUT* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_CUT_OUT* 4)
(intern "*OBSTACLE_TYPE_VEHICLE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_TYPE_VEHICLE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_VEHICLE* 0)
(intern "*OBSTACLE_TYPE_TRUCK*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_TYPE_TRUCK* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_TRUCK* 1)
(intern "*OBSTACLE_TYPE_BIKE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_TYPE_BIKE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_BIKE* 2)
(intern "*OBSTACLE_TYPE_PED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_TYPE_PED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_PED* 3)
(intern "*OBSTACLE_TYPE_BICYCLE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_TYPE_BICYCLE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_BICYCLE* 4)
(intern "*OBSTACLE_STATUS_UNDEFINED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_STATUS_UNDEFINED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_UNDEFINED* 0)
(intern "*OBSTACLE_STATUS_STANDING*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_STATUS_STANDING* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_STANDING* 1)
(intern "*OBSTACLE_STATUS_STOPPED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_STATUS_STOPPED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_STOPPED* 2)
(intern "*OBSTACLE_STATUS_MOVING*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_STATUS_MOVING* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_MOVING* 3)
(intern "*OBSTACLE_STATUS_ONCOMING*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_STATUS_ONCOMING* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_ONCOMING* 4)
(intern "*OBSTACLE_STATUS_PARKED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_STATUS_PARKED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_PARKED* 5)
(intern "*OBSTACLE_VALID_INVALID*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_VALID_INVALID* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_VALID_INVALID* 0)
(intern "*OBSTACLE_VALID_NEW*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_VALID_NEW* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_VALID_NEW* 1)
(intern "*OBSTACLE_VALID_OLDER*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_VALID_OLDER* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_VALID_OLDER* 2)
(intern "*OBSTACLE_LANE_NOT_ASSIGNED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_LANE_NOT_ASSIGNED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_NOT_ASSIGNED* 0)
(intern "*OBSTACLE_LANE_EGO_LANE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_LANE_EGO_LANE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_EGO_LANE* 1)
(intern "*OBSTACLE_LANE_NEXT_LANE*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_LANE_NEXT_LANE* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_NEXT_LANE* 2)
(intern "*OBSTACLE_LANE_INVALID*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*OBSTACLE_LANE_INVALID* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_INVALID* 3)
(intern "*RADAR_MATCH_CONFIDENCE_NO_MATCH*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*RADAR_MATCH_CONFIDENCE_NO_MATCH* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_NO_MATCH* 0)
(intern "*RADAR_MATCH_CONFIDENCE_MULTI_MATCH*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*RADAR_MATCH_CONFIDENCE_MULTI_MATCH* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_MULTI_MATCH* 1)
(intern "*RADAR_MATCH_CONFIDENCE_BOUNDED_LOW*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*RADAR_MATCH_CONFIDENCE_BOUNDED_LOW* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_BOUNDED_LOW* 2)
(intern "*RADAR_MATCH_CONFIDENCE_BOUNDED_MED*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*RADAR_MATCH_CONFIDENCE_BOUNDED_MED* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_BOUNDED_MED* 3)
(intern "*RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH* 4)
(intern "*RADAR_MATCH_CONFIDENCE_HIGH*" (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(shadow '*RADAR_MATCH_CONFIDENCE_HIGH* (find-package "MOBILEYE_560_660_MSGS::OBSTACLEDATA"))
(defconstant mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_HIGH* 5)

(defun mobileye_560_660_msgs::ObstacleData-to-symbol (const)
  (cond
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_UNAVAILABLE*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_OFF*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_LEFT*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_RIGHT*)
        ((= const 4) 'mobileye_560_660_msgs::ObstacleData::*BLINKER_INFO_BOTH*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_UNDEFINED*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_IN_HOST_LANE*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_OUT_HOST_LANE*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_CUT_IN*)
        ((= const 4) 'mobileye_560_660_msgs::ObstacleData::*CUT_IN_AND_OUT_CUT_OUT*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_VEHICLE*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_TRUCK*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_BIKE*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_PED*)
        ((= const 4) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_TYPE_BICYCLE*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_UNDEFINED*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_STANDING*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_STOPPED*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_MOVING*)
        ((= const 4) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_ONCOMING*)
        ((= const 5) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_STATUS_PARKED*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_VALID_INVALID*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_VALID_NEW*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_VALID_OLDER*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_NOT_ASSIGNED*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_EGO_LANE*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_NEXT_LANE*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleData::*OBSTACLE_LANE_INVALID*)
        ((= const 0) 'mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_NO_MATCH*)
        ((= const 1) 'mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_MULTI_MATCH*)
        ((= const 2) 'mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_BOUNDED_LOW*)
        ((= const 3) 'mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_BOUNDED_MED*)
        ((= const 4) 'mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH*)
        ((= const 5) 'mobileye_560_660_msgs::ObstacleData::*RADAR_MATCH_CONFIDENCE_HIGH*)
        (t nil)))

(defclass mobileye_560_660_msgs::ObstacleData
  :super ros::object
  :slots (_header _obstacle_id _obstacle_pos_x _obstacle_pos_y _blinker_info _cut_in_and_out _obstacle_rel_vel_x _obstacle_type _obstacle_status _obstacle_brake_lights _obstacle_valid _obstacle_length _obstacle_width _obstacle_age _obstacle_lane _cipv_flag _radar_pos_x _radar_vel_x _radar_match_confidence _matched_radar_id _obstacle_angle_rate _obstacle_scale_change _object_accel_x _obstacle_replaced _obstacle_angle ))

(defmethod mobileye_560_660_msgs::ObstacleData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacle_id __obstacle_id) 0)
    ((:obstacle_pos_x __obstacle_pos_x) 0.0)
    ((:obstacle_pos_y __obstacle_pos_y) 0.0)
    ((:blinker_info __blinker_info) 0)
    ((:cut_in_and_out __cut_in_and_out) 0)
    ((:obstacle_rel_vel_x __obstacle_rel_vel_x) 0.0)
    ((:obstacle_type __obstacle_type) 0)
    ((:obstacle_status __obstacle_status) 0)
    ((:obstacle_brake_lights __obstacle_brake_lights) nil)
    ((:obstacle_valid __obstacle_valid) 0)
    ((:obstacle_length __obstacle_length) 0.0)
    ((:obstacle_width __obstacle_width) 0.0)
    ((:obstacle_age __obstacle_age) 0)
    ((:obstacle_lane __obstacle_lane) 0)
    ((:cipv_flag __cipv_flag) nil)
    ((:radar_pos_x __radar_pos_x) 0.0)
    ((:radar_vel_x __radar_vel_x) 0.0)
    ((:radar_match_confidence __radar_match_confidence) 0)
    ((:matched_radar_id __matched_radar_id) 0)
    ((:obstacle_angle_rate __obstacle_angle_rate) 0.0)
    ((:obstacle_scale_change __obstacle_scale_change) 0.0)
    ((:object_accel_x __object_accel_x) 0.0)
    ((:obstacle_replaced __obstacle_replaced) nil)
    ((:obstacle_angle __obstacle_angle) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacle_id (round __obstacle_id))
   (setq _obstacle_pos_x (float __obstacle_pos_x))
   (setq _obstacle_pos_y (float __obstacle_pos_y))
   (setq _blinker_info (round __blinker_info))
   (setq _cut_in_and_out (round __cut_in_and_out))
   (setq _obstacle_rel_vel_x (float __obstacle_rel_vel_x))
   (setq _obstacle_type (round __obstacle_type))
   (setq _obstacle_status (round __obstacle_status))
   (setq _obstacle_brake_lights __obstacle_brake_lights)
   (setq _obstacle_valid (round __obstacle_valid))
   (setq _obstacle_length (float __obstacle_length))
   (setq _obstacle_width (float __obstacle_width))
   (setq _obstacle_age (round __obstacle_age))
   (setq _obstacle_lane (round __obstacle_lane))
   (setq _cipv_flag __cipv_flag)
   (setq _radar_pos_x (float __radar_pos_x))
   (setq _radar_vel_x (float __radar_vel_x))
   (setq _radar_match_confidence (round __radar_match_confidence))
   (setq _matched_radar_id (round __matched_radar_id))
   (setq _obstacle_angle_rate (float __obstacle_angle_rate))
   (setq _obstacle_scale_change (float __obstacle_scale_change))
   (setq _object_accel_x (float __object_accel_x))
   (setq _obstacle_replaced __obstacle_replaced)
   (setq _obstacle_angle (float __obstacle_angle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacle_id
   (&optional __obstacle_id)
   (if __obstacle_id (setq _obstacle_id __obstacle_id)) _obstacle_id)
  (:obstacle_pos_x
   (&optional __obstacle_pos_x)
   (if __obstacle_pos_x (setq _obstacle_pos_x __obstacle_pos_x)) _obstacle_pos_x)
  (:obstacle_pos_y
   (&optional __obstacle_pos_y)
   (if __obstacle_pos_y (setq _obstacle_pos_y __obstacle_pos_y)) _obstacle_pos_y)
  (:blinker_info
   (&optional __blinker_info)
   (if __blinker_info (setq _blinker_info __blinker_info)) _blinker_info)
  (:cut_in_and_out
   (&optional __cut_in_and_out)
   (if __cut_in_and_out (setq _cut_in_and_out __cut_in_and_out)) _cut_in_and_out)
  (:obstacle_rel_vel_x
   (&optional __obstacle_rel_vel_x)
   (if __obstacle_rel_vel_x (setq _obstacle_rel_vel_x __obstacle_rel_vel_x)) _obstacle_rel_vel_x)
  (:obstacle_type
   (&optional __obstacle_type)
   (if __obstacle_type (setq _obstacle_type __obstacle_type)) _obstacle_type)
  (:obstacle_status
   (&optional __obstacle_status)
   (if __obstacle_status (setq _obstacle_status __obstacle_status)) _obstacle_status)
  (:obstacle_brake_lights
   (&optional (__obstacle_brake_lights :null))
   (if (not (eq __obstacle_brake_lights :null)) (setq _obstacle_brake_lights __obstacle_brake_lights)) _obstacle_brake_lights)
  (:obstacle_valid
   (&optional __obstacle_valid)
   (if __obstacle_valid (setq _obstacle_valid __obstacle_valid)) _obstacle_valid)
  (:obstacle_length
   (&optional __obstacle_length)
   (if __obstacle_length (setq _obstacle_length __obstacle_length)) _obstacle_length)
  (:obstacle_width
   (&optional __obstacle_width)
   (if __obstacle_width (setq _obstacle_width __obstacle_width)) _obstacle_width)
  (:obstacle_age
   (&optional __obstacle_age)
   (if __obstacle_age (setq _obstacle_age __obstacle_age)) _obstacle_age)
  (:obstacle_lane
   (&optional __obstacle_lane)
   (if __obstacle_lane (setq _obstacle_lane __obstacle_lane)) _obstacle_lane)
  (:cipv_flag
   (&optional (__cipv_flag :null))
   (if (not (eq __cipv_flag :null)) (setq _cipv_flag __cipv_flag)) _cipv_flag)
  (:radar_pos_x
   (&optional __radar_pos_x)
   (if __radar_pos_x (setq _radar_pos_x __radar_pos_x)) _radar_pos_x)
  (:radar_vel_x
   (&optional __radar_vel_x)
   (if __radar_vel_x (setq _radar_vel_x __radar_vel_x)) _radar_vel_x)
  (:radar_match_confidence
   (&optional __radar_match_confidence)
   (if __radar_match_confidence (setq _radar_match_confidence __radar_match_confidence)) _radar_match_confidence)
  (:matched_radar_id
   (&optional __matched_radar_id)
   (if __matched_radar_id (setq _matched_radar_id __matched_radar_id)) _matched_radar_id)
  (:obstacle_angle_rate
   (&optional __obstacle_angle_rate)
   (if __obstacle_angle_rate (setq _obstacle_angle_rate __obstacle_angle_rate)) _obstacle_angle_rate)
  (:obstacle_scale_change
   (&optional __obstacle_scale_change)
   (if __obstacle_scale_change (setq _obstacle_scale_change __obstacle_scale_change)) _obstacle_scale_change)
  (:object_accel_x
   (&optional __object_accel_x)
   (if __object_accel_x (setq _object_accel_x __object_accel_x)) _object_accel_x)
  (:obstacle_replaced
   (&optional (__obstacle_replaced :null))
   (if (not (eq __obstacle_replaced :null)) (setq _obstacle_replaced __obstacle_replaced)) _obstacle_replaced)
  (:obstacle_angle
   (&optional __obstacle_angle)
   (if __obstacle_angle (setq _obstacle_angle __obstacle_angle)) _obstacle_angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _obstacle_id
    2
    ;; float64 _obstacle_pos_x
    8
    ;; float64 _obstacle_pos_y
    8
    ;; uint8 _blinker_info
    1
    ;; uint8 _cut_in_and_out
    1
    ;; float64 _obstacle_rel_vel_x
    8
    ;; uint8 _obstacle_type
    1
    ;; uint8 _obstacle_status
    1
    ;; bool _obstacle_brake_lights
    1
    ;; uint8 _obstacle_valid
    1
    ;; float32 _obstacle_length
    4
    ;; float32 _obstacle_width
    4
    ;; uint16 _obstacle_age
    2
    ;; uint8 _obstacle_lane
    1
    ;; bool _cipv_flag
    1
    ;; float32 _radar_pos_x
    4
    ;; float32 _radar_vel_x
    4
    ;; uint8 _radar_match_confidence
    1
    ;; uint16 _matched_radar_id
    2
    ;; float32 _obstacle_angle_rate
    4
    ;; float64 _obstacle_scale_change
    8
    ;; float32 _object_accel_x
    4
    ;; bool _obstacle_replaced
    1
    ;; float32 _obstacle_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _obstacle_id
       (write-word _obstacle_id s)
     ;; float64 _obstacle_pos_x
       (sys::poke _obstacle_pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _obstacle_pos_y
       (sys::poke _obstacle_pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _blinker_info
       (write-byte _blinker_info s)
     ;; uint8 _cut_in_and_out
       (write-byte _cut_in_and_out s)
     ;; float64 _obstacle_rel_vel_x
       (sys::poke _obstacle_rel_vel_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _obstacle_type
       (write-byte _obstacle_type s)
     ;; uint8 _obstacle_status
       (write-byte _obstacle_status s)
     ;; bool _obstacle_brake_lights
       (if _obstacle_brake_lights (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _obstacle_valid
       (write-byte _obstacle_valid s)
     ;; float32 _obstacle_length
       (sys::poke _obstacle_length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _obstacle_width
       (sys::poke _obstacle_width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _obstacle_age
       (write-word _obstacle_age s)
     ;; uint8 _obstacle_lane
       (write-byte _obstacle_lane s)
     ;; bool _cipv_flag
       (if _cipv_flag (write-byte -1 s) (write-byte 0 s))
     ;; float32 _radar_pos_x
       (sys::poke _radar_pos_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _radar_vel_x
       (sys::poke _radar_vel_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _radar_match_confidence
       (write-byte _radar_match_confidence s)
     ;; uint16 _matched_radar_id
       (write-word _matched_radar_id s)
     ;; float32 _obstacle_angle_rate
       (sys::poke _obstacle_angle_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _obstacle_scale_change
       (sys::poke _obstacle_scale_change (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _object_accel_x
       (sys::poke _object_accel_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _obstacle_replaced
       (if _obstacle_replaced (write-byte -1 s) (write-byte 0 s))
     ;; float32 _obstacle_angle
       (sys::poke _obstacle_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _obstacle_id
     (setq _obstacle_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _obstacle_pos_x
     (setq _obstacle_pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _obstacle_pos_y
     (setq _obstacle_pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _blinker_info
     (setq _blinker_info (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cut_in_and_out
     (setq _cut_in_and_out (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _obstacle_rel_vel_x
     (setq _obstacle_rel_vel_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _obstacle_type
     (setq _obstacle_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _obstacle_status
     (setq _obstacle_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _obstacle_brake_lights
     (setq _obstacle_brake_lights (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _obstacle_valid
     (setq _obstacle_valid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _obstacle_length
     (setq _obstacle_length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _obstacle_width
     (setq _obstacle_width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _obstacle_age
     (setq _obstacle_age (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _obstacle_lane
     (setq _obstacle_lane (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _cipv_flag
     (setq _cipv_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _radar_pos_x
     (setq _radar_pos_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _radar_vel_x
     (setq _radar_vel_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _radar_match_confidence
     (setq _radar_match_confidence (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _matched_radar_id
     (setq _matched_radar_id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _obstacle_angle_rate
     (setq _obstacle_angle_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _obstacle_scale_change
     (setq _obstacle_scale_change (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _object_accel_x
     (setq _object_accel_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _obstacle_replaced
     (setq _obstacle_replaced (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _obstacle_angle
     (setq _obstacle_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::ObstacleData :md5sum-) "ff75c75f79e1f472d5b0086caa5c286f")
(setf (get mobileye_560_660_msgs::ObstacleData :datatype-) "mobileye_560_660_msgs/ObstacleData")
(setf (get mobileye_560_660_msgs::ObstacleData :definition-)
      "std_msgs/Header header

uint16 obstacle_id
float64 obstacle_pos_x
float64 obstacle_pos_y

uint8 BLINKER_INFO_UNAVAILABLE = 0
uint8 BLINKER_INFO_OFF = 1
uint8 BLINKER_INFO_LEFT = 2
uint8 BLINKER_INFO_RIGHT = 3
uint8 BLINKER_INFO_BOTH = 4
uint8 blinker_info

uint8 CUT_IN_AND_OUT_UNDEFINED = 0
uint8 CUT_IN_AND_OUT_IN_HOST_LANE = 1
uint8 CUT_IN_AND_OUT_OUT_HOST_LANE = 2
uint8 CUT_IN_AND_OUT_CUT_IN = 3
uint8 CUT_IN_AND_OUT_CUT_OUT = 4
uint8 cut_in_and_out

float64 obstacle_rel_vel_x

uint8 OBSTACLE_TYPE_VEHICLE = 0
uint8 OBSTACLE_TYPE_TRUCK = 1
uint8 OBSTACLE_TYPE_BIKE = 2
uint8 OBSTACLE_TYPE_PED = 3
uint8 OBSTACLE_TYPE_BICYCLE = 4
uint8 obstacle_type

uint8 OBSTACLE_STATUS_UNDEFINED = 0
uint8 OBSTACLE_STATUS_STANDING = 1
uint8 OBSTACLE_STATUS_STOPPED = 2
uint8 OBSTACLE_STATUS_MOVING = 3
uint8 OBSTACLE_STATUS_ONCOMING = 4
uint8 OBSTACLE_STATUS_PARKED = 5
uint8 obstacle_status

bool obstacle_brake_lights

uint8 OBSTACLE_VALID_INVALID = 0
uint8 OBSTACLE_VALID_NEW = 1
uint8 OBSTACLE_VALID_OLDER = 2
uint8 obstacle_valid

float32 obstacle_length
float32 obstacle_width
uint16 obstacle_age

uint8 OBSTACLE_LANE_NOT_ASSIGNED = 0
uint8 OBSTACLE_LANE_EGO_LANE = 1
uint8 OBSTACLE_LANE_NEXT_LANE = 2
uint8 OBSTACLE_LANE_INVALID = 3
uint8 obstacle_lane

bool cipv_flag
float32 radar_pos_x
float32 radar_vel_x

uint8 RADAR_MATCH_CONFIDENCE_NO_MATCH = 0
uint8 RADAR_MATCH_CONFIDENCE_MULTI_MATCH = 1
uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_LOW = 2
uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_MED = 3
uint8 RADAR_MATCH_CONFIDENCE_BOUNDED_HIGH = 4
uint8 RADAR_MATCH_CONFIDENCE_HIGH = 5
uint8 radar_match_confidence

uint16 matched_radar_id
float32 obstacle_angle_rate
float64 obstacle_scale_change
float32 object_accel_x
bool obstacle_replaced
float32 obstacle_angle

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



(provide :mobileye_560_660_msgs/ObstacleData "ff75c75f79e1f472d5b0086caa5c286f")


