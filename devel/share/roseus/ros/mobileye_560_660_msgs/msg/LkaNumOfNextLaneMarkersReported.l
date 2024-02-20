;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::LkaNumOfNextLaneMarkersReported)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'LkaNumOfNextLaneMarkersReported (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::LKANUMOFNEXTLANEMARKERSREPORTED")
  (make-package "MOBILEYE_560_660_MSGS::LKANUMOFNEXTLANEMARKERSREPORTED"))

(in-package "ROS")
;;//! \htmlinclude LkaNumOfNextLaneMarkersReported.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mobileye_560_660_msgs::LkaNumOfNextLaneMarkersReported
  :super ros::object
  :slots (_header _num_of_next_lane_markers_reported ))

(defmethod mobileye_560_660_msgs::LkaNumOfNextLaneMarkersReported
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_of_next_lane_markers_reported __num_of_next_lane_markers_reported) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_of_next_lane_markers_reported (round __num_of_next_lane_markers_reported))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_of_next_lane_markers_reported
   (&optional __num_of_next_lane_markers_reported)
   (if __num_of_next_lane_markers_reported (setq _num_of_next_lane_markers_reported __num_of_next_lane_markers_reported)) _num_of_next_lane_markers_reported)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _num_of_next_lane_markers_reported
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _num_of_next_lane_markers_reported
       (write-word _num_of_next_lane_markers_reported s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _num_of_next_lane_markers_reported
     (setq _num_of_next_lane_markers_reported (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::LkaNumOfNextLaneMarkersReported :md5sum-) "0313c1cecbae25d684c324d160d9925e")
(setf (get mobileye_560_660_msgs::LkaNumOfNextLaneMarkersReported :datatype-) "mobileye_560_660_msgs/LkaNumOfNextLaneMarkersReported")
(setf (get mobileye_560_660_msgs::LkaNumOfNextLaneMarkersReported :definition-)
      "std_msgs/Header header

uint16 num_of_next_lane_markers_reported

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



(provide :mobileye_560_660_msgs/LkaNumOfNextLaneMarkersReported "0313c1cecbae25d684c324d160d9925e")


