;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelFRESET)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelFRESET (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELFRESET")
  (make-package "NOVATEL_GPS_MSGS::NOVATELFRESET"))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELFRESETREQUEST")
  (make-package "NOVATEL_GPS_MSGS::NOVATELFRESETREQUEST"))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELFRESETRESPONSE")
  (make-package "NOVATEL_GPS_MSGS::NOVATELFRESETRESPONSE"))

(in-package "ROS")





(defclass novatel_gps_msgs::NovatelFRESETRequest
  :super ros::object
  :slots (_target ))

(defmethod novatel_gps_msgs::NovatelFRESETRequest
  (:init
   (&key
    ((:target __target) "")
    )
   (send-super :init)
   (setq _target (string __target))
   self)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:serialization-length
   ()
   (+
    ;; string _target
    4 (length _target)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _target
       (write-long (length _target) s) (princ _target s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _target
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass novatel_gps_msgs::NovatelFRESETResponse
  :super ros::object
  :slots (_success ))

(defmethod novatel_gps_msgs::NovatelFRESETResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass novatel_gps_msgs::NovatelFRESET
  :super ros::object
  :slots ())

(setf (get novatel_gps_msgs::NovatelFRESET :md5sum-) "98b515b2d837c6d9bd64b53971bc09b4")
(setf (get novatel_gps_msgs::NovatelFRESET :datatype-) "novatel_gps_msgs/NovatelFRESET")
(setf (get novatel_gps_msgs::NovatelFRESET :request) novatel_gps_msgs::NovatelFRESETRequest)
(setf (get novatel_gps_msgs::NovatelFRESET :response) novatel_gps_msgs::NovatelFRESETResponse)

(defmethod novatel_gps_msgs::NovatelFRESETRequest
  (:response () (instance novatel_gps_msgs::NovatelFRESETResponse :init)))

(setf (get novatel_gps_msgs::NovatelFRESETRequest :md5sum-) "98b515b2d837c6d9bd64b53971bc09b4")
(setf (get novatel_gps_msgs::NovatelFRESETRequest :datatype-) "novatel_gps_msgs/NovatelFRESETRequest")
(setf (get novatel_gps_msgs::NovatelFRESETRequest :definition-)
      "# Request to send a FRESET command to the Novatel unit
#Request

string target

---

#Response
bool success


")

(setf (get novatel_gps_msgs::NovatelFRESETResponse :md5sum-) "98b515b2d837c6d9bd64b53971bc09b4")
(setf (get novatel_gps_msgs::NovatelFRESETResponse :datatype-) "novatel_gps_msgs/NovatelFRESETResponse")
(setf (get novatel_gps_msgs::NovatelFRESETResponse :definition-)
      "# Request to send a FRESET command to the Novatel unit
#Request

string target

---

#Response
bool success


")



(provide :novatel_gps_msgs/NovatelFRESET "98b515b2d837c6d9bd64b53971bc09b4")


