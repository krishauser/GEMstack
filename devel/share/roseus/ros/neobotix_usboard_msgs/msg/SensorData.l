;; Auto-generated. Do not edit!


(when (boundp 'neobotix_usboard_msgs::SensorData)
  (if (not (find-package "NEOBOTIX_USBOARD_MSGS"))
    (make-package "NEOBOTIX_USBOARD_MSGS"))
  (shadow 'SensorData (find-package "NEOBOTIX_USBOARD_MSGS")))
(unless (find-package "NEOBOTIX_USBOARD_MSGS::SENSORDATA")
  (make-package "NEOBOTIX_USBOARD_MSGS::SENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude SensorData.msg.html


(defclass neobotix_usboard_msgs::SensorData
  :super ros::object
  :slots (_distance _warn _alarm _active ))

(defmethod neobotix_usboard_msgs::SensorData
  (:init
   (&key
    ((:distance __distance) 0)
    ((:warn __warn) nil)
    ((:alarm __alarm) nil)
    ((:active __active) nil)
    )
   (send-super :init)
   (setq _distance (round __distance))
   (setq _warn __warn)
   (setq _alarm __alarm)
   (setq _active __active)
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:warn
   (&optional (__warn :null))
   (if (not (eq __warn :null)) (setq _warn __warn)) _warn)
  (:alarm
   (&optional (__alarm :null))
   (if (not (eq __alarm :null)) (setq _alarm __alarm)) _alarm)
  (:active
   (&optional (__active :null))
   (if (not (eq __active :null)) (setq _active __active)) _active)
  (:serialization-length
   ()
   (+
    ;; uint8 _distance
    1
    ;; bool _warn
    1
    ;; bool _alarm
    1
    ;; bool _active
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _distance
       (write-byte _distance s)
     ;; bool _warn
       (if _warn (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm
       (if _alarm (write-byte -1 s) (write-byte 0 s))
     ;; bool _active
       (if _active (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _distance
     (setq _distance (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _warn
     (setq _warn (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm
     (setq _alarm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _active
     (setq _active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get neobotix_usboard_msgs::SensorData :md5sum-) "8b4451cc862e6df92992cfa6088c67e3")
(setf (get neobotix_usboard_msgs::SensorData :datatype-) "neobotix_usboard_msgs/SensorData")
(setf (get neobotix_usboard_msgs::SensorData :definition-)
      "# Message file for SensorData

uint8   distance # cm
bool    warn
bool    alarm
bool    active

")



(provide :neobotix_usboard_msgs/SensorData "8b4451cc862e6df92992cfa6088c67e3")


