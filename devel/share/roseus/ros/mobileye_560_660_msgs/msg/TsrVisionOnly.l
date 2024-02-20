;; Auto-generated. Do not edit!


(when (boundp 'mobileye_560_660_msgs::TsrVisionOnly)
  (if (not (find-package "MOBILEYE_560_660_MSGS"))
    (make-package "MOBILEYE_560_660_MSGS"))
  (shadow 'TsrVisionOnly (find-package "MOBILEYE_560_660_MSGS")))
(unless (find-package "MOBILEYE_560_660_MSGS::TSRVISIONONLY")
  (make-package "MOBILEYE_560_660_MSGS::TSRVISIONONLY"))

(in-package "ROS")
;;//! \htmlinclude TsrVisionOnly.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mobileye_560_660_msgs::TsrVisionOnly
  :super ros::object
  :slots (_header _vision_only_sign_type_display1 _vision_only_supplementary_sign_type_display1 _vision_only_sign_type_display2 _vision_only_supplementary_sign_type_display2 _vision_only_sign_type_display3 _vision_only_supplementary_sign_type_display3 _vision_only_sign_type_display4 _vision_only_supplementary_sign_type_display4 ))

(defmethod mobileye_560_660_msgs::TsrVisionOnly
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vision_only_sign_type_display1 __vision_only_sign_type_display1) 0)
    ((:vision_only_supplementary_sign_type_display1 __vision_only_supplementary_sign_type_display1) 0)
    ((:vision_only_sign_type_display2 __vision_only_sign_type_display2) 0)
    ((:vision_only_supplementary_sign_type_display2 __vision_only_supplementary_sign_type_display2) 0)
    ((:vision_only_sign_type_display3 __vision_only_sign_type_display3) 0)
    ((:vision_only_supplementary_sign_type_display3 __vision_only_supplementary_sign_type_display3) 0)
    ((:vision_only_sign_type_display4 __vision_only_sign_type_display4) 0)
    ((:vision_only_supplementary_sign_type_display4 __vision_only_supplementary_sign_type_display4) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vision_only_sign_type_display1 (round __vision_only_sign_type_display1))
   (setq _vision_only_supplementary_sign_type_display1 (round __vision_only_supplementary_sign_type_display1))
   (setq _vision_only_sign_type_display2 (round __vision_only_sign_type_display2))
   (setq _vision_only_supplementary_sign_type_display2 (round __vision_only_supplementary_sign_type_display2))
   (setq _vision_only_sign_type_display3 (round __vision_only_sign_type_display3))
   (setq _vision_only_supplementary_sign_type_display3 (round __vision_only_supplementary_sign_type_display3))
   (setq _vision_only_sign_type_display4 (round __vision_only_sign_type_display4))
   (setq _vision_only_supplementary_sign_type_display4 (round __vision_only_supplementary_sign_type_display4))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vision_only_sign_type_display1
   (&optional __vision_only_sign_type_display1)
   (if __vision_only_sign_type_display1 (setq _vision_only_sign_type_display1 __vision_only_sign_type_display1)) _vision_only_sign_type_display1)
  (:vision_only_supplementary_sign_type_display1
   (&optional __vision_only_supplementary_sign_type_display1)
   (if __vision_only_supplementary_sign_type_display1 (setq _vision_only_supplementary_sign_type_display1 __vision_only_supplementary_sign_type_display1)) _vision_only_supplementary_sign_type_display1)
  (:vision_only_sign_type_display2
   (&optional __vision_only_sign_type_display2)
   (if __vision_only_sign_type_display2 (setq _vision_only_sign_type_display2 __vision_only_sign_type_display2)) _vision_only_sign_type_display2)
  (:vision_only_supplementary_sign_type_display2
   (&optional __vision_only_supplementary_sign_type_display2)
   (if __vision_only_supplementary_sign_type_display2 (setq _vision_only_supplementary_sign_type_display2 __vision_only_supplementary_sign_type_display2)) _vision_only_supplementary_sign_type_display2)
  (:vision_only_sign_type_display3
   (&optional __vision_only_sign_type_display3)
   (if __vision_only_sign_type_display3 (setq _vision_only_sign_type_display3 __vision_only_sign_type_display3)) _vision_only_sign_type_display3)
  (:vision_only_supplementary_sign_type_display3
   (&optional __vision_only_supplementary_sign_type_display3)
   (if __vision_only_supplementary_sign_type_display3 (setq _vision_only_supplementary_sign_type_display3 __vision_only_supplementary_sign_type_display3)) _vision_only_supplementary_sign_type_display3)
  (:vision_only_sign_type_display4
   (&optional __vision_only_sign_type_display4)
   (if __vision_only_sign_type_display4 (setq _vision_only_sign_type_display4 __vision_only_sign_type_display4)) _vision_only_sign_type_display4)
  (:vision_only_supplementary_sign_type_display4
   (&optional __vision_only_supplementary_sign_type_display4)
   (if __vision_only_supplementary_sign_type_display4 (setq _vision_only_supplementary_sign_type_display4 __vision_only_supplementary_sign_type_display4)) _vision_only_supplementary_sign_type_display4)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _vision_only_sign_type_display1
    1
    ;; uint8 _vision_only_supplementary_sign_type_display1
    1
    ;; uint8 _vision_only_sign_type_display2
    1
    ;; uint8 _vision_only_supplementary_sign_type_display2
    1
    ;; uint8 _vision_only_sign_type_display3
    1
    ;; uint8 _vision_only_supplementary_sign_type_display3
    1
    ;; uint8 _vision_only_sign_type_display4
    1
    ;; uint8 _vision_only_supplementary_sign_type_display4
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _vision_only_sign_type_display1
       (write-byte _vision_only_sign_type_display1 s)
     ;; uint8 _vision_only_supplementary_sign_type_display1
       (write-byte _vision_only_supplementary_sign_type_display1 s)
     ;; uint8 _vision_only_sign_type_display2
       (write-byte _vision_only_sign_type_display2 s)
     ;; uint8 _vision_only_supplementary_sign_type_display2
       (write-byte _vision_only_supplementary_sign_type_display2 s)
     ;; uint8 _vision_only_sign_type_display3
       (write-byte _vision_only_sign_type_display3 s)
     ;; uint8 _vision_only_supplementary_sign_type_display3
       (write-byte _vision_only_supplementary_sign_type_display3 s)
     ;; uint8 _vision_only_sign_type_display4
       (write-byte _vision_only_sign_type_display4 s)
     ;; uint8 _vision_only_supplementary_sign_type_display4
       (write-byte _vision_only_supplementary_sign_type_display4 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _vision_only_sign_type_display1
     (setq _vision_only_sign_type_display1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_supplementary_sign_type_display1
     (setq _vision_only_supplementary_sign_type_display1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_sign_type_display2
     (setq _vision_only_sign_type_display2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_supplementary_sign_type_display2
     (setq _vision_only_supplementary_sign_type_display2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_sign_type_display3
     (setq _vision_only_sign_type_display3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_supplementary_sign_type_display3
     (setq _vision_only_supplementary_sign_type_display3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_sign_type_display4
     (setq _vision_only_sign_type_display4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _vision_only_supplementary_sign_type_display4
     (setq _vision_only_supplementary_sign_type_display4 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mobileye_560_660_msgs::TsrVisionOnly :md5sum-) "84f9582e1cda52683c53338cffe795f0")
(setf (get mobileye_560_660_msgs::TsrVisionOnly :datatype-) "mobileye_560_660_msgs/TsrVisionOnly")
(setf (get mobileye_560_660_msgs::TsrVisionOnly :definition-)
      "std_msgs/Header header

uint8 vision_only_sign_type_display1
uint8 vision_only_supplementary_sign_type_display1
uint8 vision_only_sign_type_display2
uint8 vision_only_supplementary_sign_type_display2
uint8 vision_only_sign_type_display3
uint8 vision_only_supplementary_sign_type_display3
uint8 vision_only_sign_type_display4
uint8 vision_only_supplementary_sign_type_display4

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



(provide :mobileye_560_660_msgs/TsrVisionOnly "84f9582e1cda52683c53338cffe795f0")


