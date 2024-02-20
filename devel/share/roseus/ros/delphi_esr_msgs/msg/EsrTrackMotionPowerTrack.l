;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrTrackMotionPowerTrack)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrTrackMotionPowerTrack (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRTRACKMOTIONPOWERTRACK")
  (make-package "DELPHI_ESR_MSGS::ESRTRACKMOTIONPOWERTRACK"))

(in-package "ROS")
;;//! \htmlinclude EsrTrackMotionPowerTrack.msg.html


(defclass delphi_esr_msgs::EsrTrackMotionPowerTrack
  :super ros::object
  :slots (_id _movable_fast _movable_slow _moving _power ))

(defmethod delphi_esr_msgs::EsrTrackMotionPowerTrack
  (:init
   (&key
    ((:id __id) 0)
    ((:movable_fast __movable_fast) nil)
    ((:movable_slow __movable_slow) nil)
    ((:moving __moving) nil)
    ((:power __power) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _movable_fast __movable_fast)
   (setq _movable_slow __movable_slow)
   (setq _moving __moving)
   (setq _power (round __power))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:movable_fast
   (&optional (__movable_fast :null))
   (if (not (eq __movable_fast :null)) (setq _movable_fast __movable_fast)) _movable_fast)
  (:movable_slow
   (&optional (__movable_slow :null))
   (if (not (eq __movable_slow :null)) (setq _movable_slow __movable_slow)) _movable_slow)
  (:moving
   (&optional (__moving :null))
   (if (not (eq __moving :null)) (setq _moving __moving)) _moving)
  (:power
   (&optional __power)
   (if __power (setq _power __power)) _power)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; bool _movable_fast
    1
    ;; bool _movable_slow
    1
    ;; bool _moving
    1
    ;; int8 _power
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; bool _movable_fast
       (if _movable_fast (write-byte -1 s) (write-byte 0 s))
     ;; bool _movable_slow
       (if _movable_slow (write-byte -1 s) (write-byte 0 s))
     ;; bool _moving
       (if _moving (write-byte -1 s) (write-byte 0 s))
     ;; int8 _power
       (write-byte _power s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _movable_fast
     (setq _movable_fast (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _movable_slow
     (setq _movable_slow (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _moving
     (setq _moving (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int8 _power
     (setq _power (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _power 127) (setq _power (- _power 256)))
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrTrackMotionPowerTrack :md5sum-) "3cb7ee3e17f03f833bf47e59a4267646")
(setf (get delphi_esr_msgs::EsrTrackMotionPowerTrack :datatype-) "delphi_esr_msgs/EsrTrackMotionPowerTrack")
(setf (get delphi_esr_msgs::EsrTrackMotionPowerTrack :definition-)
      "# ESR TrackMotionPower, track-specific information
uint8  id
bool   movable_fast
bool   movable_slow
bool   moving
int8   power

")



(provide :delphi_esr_msgs/EsrTrackMotionPowerTrack "3cb7ee3e17f03f833bf47e59a4267646")


