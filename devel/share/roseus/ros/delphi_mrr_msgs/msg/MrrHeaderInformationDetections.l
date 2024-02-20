;; Auto-generated. Do not edit!


(when (boundp 'delphi_mrr_msgs::MrrHeaderInformationDetections)
  (if (not (find-package "DELPHI_MRR_MSGS"))
    (make-package "DELPHI_MRR_MSGS"))
  (shadow 'MrrHeaderInformationDetections (find-package "DELPHI_MRR_MSGS")))
(unless (find-package "DELPHI_MRR_MSGS::MRRHEADERINFORMATIONDETECTIONS")
  (make-package "DELPHI_MRR_MSGS::MRRHEADERINFORMATIONDETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude MrrHeaderInformationDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_mrr_msgs::MrrHeaderInformationDetections
  :super ros::object
  :slots (_header _can_align_updates_done _can_scan_index _can_number_of_det _can_look_id _can_look_index ))

(defmethod delphi_mrr_msgs::MrrHeaderInformationDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:can_align_updates_done __can_align_updates_done) 0)
    ((:can_scan_index __can_scan_index) 0)
    ((:can_number_of_det __can_number_of_det) 0)
    ((:can_look_id __can_look_id) 0)
    ((:can_look_index __can_look_index) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _can_align_updates_done (round __can_align_updates_done))
   (setq _can_scan_index (round __can_scan_index))
   (setq _can_number_of_det (round __can_number_of_det))
   (setq _can_look_id (round __can_look_id))
   (setq _can_look_index (round __can_look_index))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:can_align_updates_done
   (&optional __can_align_updates_done)
   (if __can_align_updates_done (setq _can_align_updates_done __can_align_updates_done)) _can_align_updates_done)
  (:can_scan_index
   (&optional __can_scan_index)
   (if __can_scan_index (setq _can_scan_index __can_scan_index)) _can_scan_index)
  (:can_number_of_det
   (&optional __can_number_of_det)
   (if __can_number_of_det (setq _can_number_of_det __can_number_of_det)) _can_number_of_det)
  (:can_look_id
   (&optional __can_look_id)
   (if __can_look_id (setq _can_look_id __can_look_id)) _can_look_id)
  (:can_look_index
   (&optional __can_look_index)
   (if __can_look_index (setq _can_look_index __can_look_index)) _can_look_index)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _can_align_updates_done
    2
    ;; uint16 _can_scan_index
    2
    ;; uint8 _can_number_of_det
    1
    ;; uint8 _can_look_id
    1
    ;; uint16 _can_look_index
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _can_align_updates_done
       (write-word _can_align_updates_done s)
     ;; uint16 _can_scan_index
       (write-word _can_scan_index s)
     ;; uint8 _can_number_of_det
       (write-byte _can_number_of_det s)
     ;; uint8 _can_look_id
       (write-byte _can_look_id s)
     ;; uint16 _can_look_index
       (write-word _can_look_index s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _can_align_updates_done
     (setq _can_align_updates_done (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _can_scan_index
     (setq _can_scan_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _can_number_of_det
     (setq _can_number_of_det (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_look_id
     (setq _can_look_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _can_look_index
     (setq _can_look_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get delphi_mrr_msgs::MrrHeaderInformationDetections :md5sum-) "39494a4731101be0be38c8ac22c1c084")
(setf (get delphi_mrr_msgs::MrrHeaderInformationDetections :datatype-) "delphi_mrr_msgs/MrrHeaderInformationDetections")
(setf (get delphi_mrr_msgs::MrrHeaderInformationDetections :definition-)
      "std_msgs/Header header

uint16 can_align_updates_done
uint16 can_scan_index
uint8  can_number_of_det
uint8  can_look_id
uint16 can_look_index

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



(provide :delphi_mrr_msgs/MrrHeaderInformationDetections "39494a4731101be0be38c8ac22c1c084")


