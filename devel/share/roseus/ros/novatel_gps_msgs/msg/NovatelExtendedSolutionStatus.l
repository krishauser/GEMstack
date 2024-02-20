;; Auto-generated. Do not edit!


(when (boundp 'novatel_gps_msgs::NovatelExtendedSolutionStatus)
  (if (not (find-package "NOVATEL_GPS_MSGS"))
    (make-package "NOVATEL_GPS_MSGS"))
  (shadow 'NovatelExtendedSolutionStatus (find-package "NOVATEL_GPS_MSGS")))
(unless (find-package "NOVATEL_GPS_MSGS::NOVATELEXTENDEDSOLUTIONSTATUS")
  (make-package "NOVATEL_GPS_MSGS::NOVATELEXTENDEDSOLUTIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude NovatelExtendedSolutionStatus.msg.html


(defclass novatel_gps_msgs::NovatelExtendedSolutionStatus
  :super ros::object
  :slots (_original_mask _advance_rtk_verified _psuedorange_iono_correction ))

(defmethod novatel_gps_msgs::NovatelExtendedSolutionStatus
  (:init
   (&key
    ((:original_mask __original_mask) 0)
    ((:advance_rtk_verified __advance_rtk_verified) nil)
    ((:psuedorange_iono_correction __psuedorange_iono_correction) "")
    )
   (send-super :init)
   (setq _original_mask (round __original_mask))
   (setq _advance_rtk_verified __advance_rtk_verified)
   (setq _psuedorange_iono_correction (string __psuedorange_iono_correction))
   self)
  (:original_mask
   (&optional __original_mask)
   (if __original_mask (setq _original_mask __original_mask)) _original_mask)
  (:advance_rtk_verified
   (&optional (__advance_rtk_verified :null))
   (if (not (eq __advance_rtk_verified :null)) (setq _advance_rtk_verified __advance_rtk_verified)) _advance_rtk_verified)
  (:psuedorange_iono_correction
   (&optional __psuedorange_iono_correction)
   (if __psuedorange_iono_correction (setq _psuedorange_iono_correction __psuedorange_iono_correction)) _psuedorange_iono_correction)
  (:serialization-length
   ()
   (+
    ;; uint32 _original_mask
    4
    ;; bool _advance_rtk_verified
    1
    ;; string _psuedorange_iono_correction
    4 (length _psuedorange_iono_correction)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _original_mask
       (write-long _original_mask s)
     ;; bool _advance_rtk_verified
       (if _advance_rtk_verified (write-byte -1 s) (write-byte 0 s))
     ;; string _psuedorange_iono_correction
       (write-long (length _psuedorange_iono_correction) s) (princ _psuedorange_iono_correction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _original_mask
     (setq _original_mask (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _advance_rtk_verified
     (setq _advance_rtk_verified (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _psuedorange_iono_correction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _psuedorange_iono_correction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get novatel_gps_msgs::NovatelExtendedSolutionStatus :md5sum-) "f0e19d53094c207c4dafdfbde750c4b1")
(setf (get novatel_gps_msgs::NovatelExtendedSolutionStatus :datatype-) "novatel_gps_msgs/NovatelExtendedSolutionStatus")
(setf (get novatel_gps_msgs::NovatelExtendedSolutionStatus :definition-)
      "# Bit    Mask      Description
#  0     0x01      Advance RTK verified (0 = not verified, 1 = verified)
# 1-3    0x0E      Pseudorange Ionosphere Correction
#                    0 = unknown
#                    1 = Klobuchar Broadcast
#                    2 = SBAS Broadcast
#                    3 = Multi-frequency Computed
#                    4 = PSRDiff Correction
#                    5 = NovaTel Blended Ionosphere Value
# 4-7  0xF0        <Reserved>
uint32 original_mask
bool advance_rtk_verified
string psuedorange_iono_correction

")



(provide :novatel_gps_msgs/NovatelExtendedSolutionStatus "f0e19d53094c207c4dafdfbde750c4b1")


