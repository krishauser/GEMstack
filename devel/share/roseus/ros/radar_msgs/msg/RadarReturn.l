;; Auto-generated. Do not edit!


(when (boundp 'radar_msgs::RadarReturn)
  (if (not (find-package "RADAR_MSGS"))
    (make-package "RADAR_MSGS"))
  (shadow 'RadarReturn (find-package "RADAR_MSGS")))
(unless (find-package "RADAR_MSGS::RADARRETURN")
  (make-package "RADAR_MSGS::RADARRETURN"))

(in-package "ROS")
;;//! \htmlinclude RadarReturn.msg.html


(defclass radar_msgs::RadarReturn
  :super ros::object
  :slots (_range _azimuth _elevation _doppler_velocity _amplitude ))

(defmethod radar_msgs::RadarReturn
  (:init
   (&key
    ((:range __range) 0.0)
    ((:azimuth __azimuth) 0.0)
    ((:elevation __elevation) 0.0)
    ((:doppler_velocity __doppler_velocity) 0.0)
    ((:amplitude __amplitude) 0.0)
    )
   (send-super :init)
   (setq _range (float __range))
   (setq _azimuth (float __azimuth))
   (setq _elevation (float __elevation))
   (setq _doppler_velocity (float __doppler_velocity))
   (setq _amplitude (float __amplitude))
   self)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:azimuth
   (&optional __azimuth)
   (if __azimuth (setq _azimuth __azimuth)) _azimuth)
  (:elevation
   (&optional __elevation)
   (if __elevation (setq _elevation __elevation)) _elevation)
  (:doppler_velocity
   (&optional __doppler_velocity)
   (if __doppler_velocity (setq _doppler_velocity __doppler_velocity)) _doppler_velocity)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:serialization-length
   ()
   (+
    ;; float32 _range
    4
    ;; float32 _azimuth
    4
    ;; float32 _elevation
    4
    ;; float32 _doppler_velocity
    4
    ;; float32 _amplitude
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _azimuth
       (sys::poke _azimuth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _elevation
       (sys::poke _elevation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _doppler_velocity
       (sys::poke _doppler_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _azimuth
     (setq _azimuth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _elevation
     (setq _elevation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _doppler_velocity
     (setq _doppler_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _amplitude
     (setq _amplitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get radar_msgs::RadarReturn :md5sum-) "d2fa6f7b9af80adc27de1892e316aaf6")
(setf (get radar_msgs::RadarReturn :datatype-) "radar_msgs/RadarReturn")
(setf (get radar_msgs::RadarReturn :definition-)
      "# All variables below are relative to the radar's frame of reference.
# This message is not meant to be used alone but as part of a stamped or array message.

float32 range                            # Distance (m) from the sensor to the detected return.
float32 azimuth                          # Angle (in radians) in the azimuth plane between the sensor and the detected return.
                                         #    Positive angles are anticlockwise from the sensor and negative angles clockwise from the sensor as per REP-0103.
float32 elevation                        # Angle (in radians) in the elevation plane between the sensor and the detected return.
                                         #    Negative angles are below the sensor. For 2D radar, this will be 0.
float32 doppler_velocity                 # The doppler speeds (m/s) of the return.
float32 amplitude                        # The amplitude of the of the return (dB)

")



(provide :radar_msgs/RadarReturn "d2fa6f7b9af80adc27de1892e316aaf6")


