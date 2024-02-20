;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelSignalMask)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelSignalMask (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELSIGNALMASK")
  (make-package "NOVATEL_GPS_MSGS::NOVATELSIGNALMASK"))

(in-package "ROS")
;;//! \htmlinclude NovatelSignalMask.msg.html


(defclass novatel_gps_msgs::NovatelSignalMask
  :super ros::object
  :slots (_original_mask _gps_L1_used_in_solution _gps_L2_used_in_solution _gps_L3_used_in_solution _glonass_L1_used_in_solution _glonass_L2_used_in_solution ))

(defmethod novatel_gps_msgs::NovatelSignalMask
  (:init
   (&key
    ((:original_mask __original_mask) 0)
    ((:gps_L1_used_in_solution __gps_L1_used_in_solution) nil)
    ((:gps_L2_used_in_solution __gps_L2_used_in_solution) nil)
    ((:gps_L3_used_in_solution __gps_L3_used_in_solution) nil)
    ((:glonass_L1_used_in_solution __glonass_L1_used_in_solution) nil)
    ((:glonass_L2_used_in_solution __glonass_L2_used_in_solution) nil)
    )
   (send-super :init)
   (setq _original_mask (round __original_mask))
   (setq _gps_L1_used_in_solution __gps_L1_used_in_solution)
   (setq _gps_L2_used_in_solution __gps_L2_used_in_solution)
   (setq _gps_L3_used_in_solution __gps_L3_used_in_solution)
   (setq _glonass_L1_used_in_solution __glonass_L1_used_in_solution)
   (setq _glonass_L2_used_in_solution __glonass_L2_used_in_solution)
   self)
  (:original_mask
   (&optional __original_mask)
   (if __original_mask (setq _original_mask __original_mask)) _original_mask)
  (:gps_L1_used_in_solution
   (&optional (__gps_L1_used_in_solution :null))
   (if (not (eq __gps_L1_used_in_solution :null)) (setq _gps_L1_used_in_solution __gps_L1_used_in_solution)) _gps_L1_used_in_solution)
  (:gps_L2_used_in_solution
   (&optional (__gps_L2_used_in_solution :null))
   (if (not (eq __gps_L2_used_in_solution :null)) (setq _gps_L2_used_in_solution __gps_L2_used_in_solution)) _gps_L2_used_in_solution)
  (:gps_L3_used_in_solution
   (&optional (__gps_L3_used_in_solution :null))
   (if (not (eq __gps_L3_used_in_solution :null)) (setq _gps_L3_used_in_solution __gps_L3_used_in_solution)) _gps_L3_used_in_solution)
  (:glonass_L1_used_in_solution
   (&optional (__glonass_L1_used_in_solution :null))
   (if (not (eq __glonass_L1_used_in_solution :null)) (setq _glonass_L1_used_in_solution __glonass_L1_used_in_solution)) _glonass_L1_used_in_solution)
  (:glonass_L2_used_in_solution
   (&optional (__glonass_L2_used_in_solution :null))
   (if (not (eq __glonass_L2_used_in_solution :null)) (setq _glonass_L2_used_in_solution __glonass_L2_used_in_solution)) _glonass_L2_used_in_solution)
  (:serialization-length
   ()
   (+
    ;; uint32 _original_mask
    4
    ;; bool _gps_L1_used_in_solution
    1
    ;; bool _gps_L2_used_in_solution
    1
    ;; bool _gps_L3_used_in_solution
    1
    ;; bool _glonass_L1_used_in_solution
    1
    ;; bool _glonass_L2_used_in_solution
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _original_mask
       (write-long _original_mask s)
     ;; bool _gps_L1_used_in_solution
       (if _gps_L1_used_in_solution (write-byte -1 s) (write-byte 0 s))
     ;; bool _gps_L2_used_in_solution
       (if _gps_L2_used_in_solution (write-byte -1 s) (write-byte 0 s))
     ;; bool _gps_L3_used_in_solution
       (if _gps_L3_used_in_solution (write-byte -1 s) (write-byte 0 s))
     ;; bool _glonass_L1_used_in_solution
       (if _glonass_L1_used_in_solution (write-byte -1 s) (write-byte 0 s))
     ;; bool _glonass_L2_used_in_solution
       (if _glonass_L2_used_in_solution (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _original_mask
     (setq _original_mask (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _gps_L1_used_in_solution
     (setq _gps_L1_used_in_solution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gps_L2_used_in_solution
     (setq _gps_L2_used_in_solution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gps_L3_used_in_solution
     (setq _gps_L3_used_in_solution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _glonass_L1_used_in_solution
     (setq _glonass_L1_used_in_solution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _glonass_L2_used_in_solution
     (setq _glonass_L2_used_in_solution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelSignalMask :md5sum-) "007d687355f8f3c12ea4e18109172710")
(setf (get novatel_gps_msgs::NovatelSignalMask :datatype-) "novatel_gps_msgs/NovatelSignalMask")
(setf (get novatel_gps_msgs::NovatelSignalMask :definition-)
      "# Bit    Mask      Description
#  0     0x01      GPS L1 used in Solution
#  1     0x02      GPS L2 used in Solution
#  2     0x04      GPS L5 used in Solution
#  3     0x08      <Reserved>
#  4     0x10      GLONASS L1 used in Solution
#  5     0x20      GLONASS L2 used in Solution
# 6-7  0x40-0x80   <Reserved>
uint32 original_mask
bool gps_L1_used_in_solution
bool gps_L2_used_in_solution
bool gps_L3_used_in_solution
bool glonass_L1_used_in_solution
bool glonass_L2_used_in_solution
")



(provide :novatel_gps_msgs/NovatelSignalMask "007d687355f8f3c12ea4e18109172710")


