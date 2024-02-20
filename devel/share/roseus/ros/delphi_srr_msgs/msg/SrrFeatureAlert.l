;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrFeatureAlert)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrFeatureAlert (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRFEATUREALERT")
  (make-package "DELPHI_SRR_MSGS::SRRFEATUREALERT"))

(in-package "ROS")
;;//! \htmlinclude SrrFeatureAlert.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_srr_msgs::SrrFeatureAlert
  :super ros::object
  :slots (_header _lcma_blis_ignored_track_id _lcma_blis_track_id _lcma_cvw_ttc _cta_ttc_alert _cta_selected_track_ttc _cta_selected_track _cta_alert _cta_active _lcma_cvw_cipv _lcma_cvw_alert_state _lcma_blis_alert_state _lcma_active ))

(defmethod delphi_srr_msgs::SrrFeatureAlert
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lcma_blis_ignored_track_id __lcma_blis_ignored_track_id) 0)
    ((:lcma_blis_track_id __lcma_blis_track_id) 0)
    ((:lcma_cvw_ttc __lcma_cvw_ttc) 0.0)
    ((:cta_ttc_alert __cta_ttc_alert) nil)
    ((:cta_selected_track_ttc __cta_selected_track_ttc) 0.0)
    ((:cta_selected_track __cta_selected_track) 0)
    ((:cta_alert __cta_alert) 0)
    ((:cta_active __cta_active) nil)
    ((:lcma_cvw_cipv __lcma_cvw_cipv) 0)
    ((:lcma_cvw_alert_state __lcma_cvw_alert_state) 0)
    ((:lcma_blis_alert_state __lcma_blis_alert_state) 0)
    ((:lcma_active __lcma_active) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lcma_blis_ignored_track_id (round __lcma_blis_ignored_track_id))
   (setq _lcma_blis_track_id (round __lcma_blis_track_id))
   (setq _lcma_cvw_ttc (float __lcma_cvw_ttc))
   (setq _cta_ttc_alert __cta_ttc_alert)
   (setq _cta_selected_track_ttc (float __cta_selected_track_ttc))
   (setq _cta_selected_track (round __cta_selected_track))
   (setq _cta_alert (round __cta_alert))
   (setq _cta_active __cta_active)
   (setq _lcma_cvw_cipv (round __lcma_cvw_cipv))
   (setq _lcma_cvw_alert_state (round __lcma_cvw_alert_state))
   (setq _lcma_blis_alert_state (round __lcma_blis_alert_state))
   (setq _lcma_active __lcma_active)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lcma_blis_ignored_track_id
   (&optional __lcma_blis_ignored_track_id)
   (if __lcma_blis_ignored_track_id (setq _lcma_blis_ignored_track_id __lcma_blis_ignored_track_id)) _lcma_blis_ignored_track_id)
  (:lcma_blis_track_id
   (&optional __lcma_blis_track_id)
   (if __lcma_blis_track_id (setq _lcma_blis_track_id __lcma_blis_track_id)) _lcma_blis_track_id)
  (:lcma_cvw_ttc
   (&optional __lcma_cvw_ttc)
   (if __lcma_cvw_ttc (setq _lcma_cvw_ttc __lcma_cvw_ttc)) _lcma_cvw_ttc)
  (:cta_ttc_alert
   (&optional (__cta_ttc_alert :null))
   (if (not (eq __cta_ttc_alert :null)) (setq _cta_ttc_alert __cta_ttc_alert)) _cta_ttc_alert)
  (:cta_selected_track_ttc
   (&optional __cta_selected_track_ttc)
   (if __cta_selected_track_ttc (setq _cta_selected_track_ttc __cta_selected_track_ttc)) _cta_selected_track_ttc)
  (:cta_selected_track
   (&optional __cta_selected_track)
   (if __cta_selected_track (setq _cta_selected_track __cta_selected_track)) _cta_selected_track)
  (:cta_alert
   (&optional __cta_alert)
   (if __cta_alert (setq _cta_alert __cta_alert)) _cta_alert)
  (:cta_active
   (&optional (__cta_active :null))
   (if (not (eq __cta_active :null)) (setq _cta_active __cta_active)) _cta_active)
  (:lcma_cvw_cipv
   (&optional __lcma_cvw_cipv)
   (if __lcma_cvw_cipv (setq _lcma_cvw_cipv __lcma_cvw_cipv)) _lcma_cvw_cipv)
  (:lcma_cvw_alert_state
   (&optional __lcma_cvw_alert_state)
   (if __lcma_cvw_alert_state (setq _lcma_cvw_alert_state __lcma_cvw_alert_state)) _lcma_cvw_alert_state)
  (:lcma_blis_alert_state
   (&optional __lcma_blis_alert_state)
   (if __lcma_blis_alert_state (setq _lcma_blis_alert_state __lcma_blis_alert_state)) _lcma_blis_alert_state)
  (:lcma_active
   (&optional (__lcma_active :null))
   (if (not (eq __lcma_active :null)) (setq _lcma_active __lcma_active)) _lcma_active)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _lcma_blis_ignored_track_id
    1
    ;; uint8 _lcma_blis_track_id
    1
    ;; float32 _lcma_cvw_ttc
    4
    ;; bool _cta_ttc_alert
    1
    ;; float32 _cta_selected_track_ttc
    4
    ;; uint16 _cta_selected_track
    2
    ;; uint8 _cta_alert
    1
    ;; bool _cta_active
    1
    ;; uint8 _lcma_cvw_cipv
    1
    ;; uint8 _lcma_cvw_alert_state
    1
    ;; uint8 _lcma_blis_alert_state
    1
    ;; bool _lcma_active
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _lcma_blis_ignored_track_id
       (write-byte _lcma_blis_ignored_track_id s)
     ;; uint8 _lcma_blis_track_id
       (write-byte _lcma_blis_track_id s)
     ;; float32 _lcma_cvw_ttc
       (sys::poke _lcma_cvw_ttc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _cta_ttc_alert
       (if _cta_ttc_alert (write-byte -1 s) (write-byte 0 s))
     ;; float32 _cta_selected_track_ttc
       (sys::poke _cta_selected_track_ttc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _cta_selected_track
       (write-word _cta_selected_track s)
     ;; uint8 _cta_alert
       (write-byte _cta_alert s)
     ;; bool _cta_active
       (if _cta_active (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _lcma_cvw_cipv
       (write-byte _lcma_cvw_cipv s)
     ;; uint8 _lcma_cvw_alert_state
       (write-byte _lcma_cvw_alert_state s)
     ;; uint8 _lcma_blis_alert_state
       (write-byte _lcma_blis_alert_state s)
     ;; bool _lcma_active
       (if _lcma_active (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _lcma_blis_ignored_track_id
     (setq _lcma_blis_ignored_track_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lcma_blis_track_id
     (setq _lcma_blis_track_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _lcma_cvw_ttc
     (setq _lcma_cvw_ttc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _cta_ttc_alert
     (setq _cta_ttc_alert (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _cta_selected_track_ttc
     (setq _cta_selected_track_ttc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _cta_selected_track
     (setq _cta_selected_track (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _cta_alert
     (setq _cta_alert (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _cta_active
     (setq _cta_active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _lcma_cvw_cipv
     (setq _lcma_cvw_cipv (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lcma_cvw_alert_state
     (setq _lcma_cvw_alert_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _lcma_blis_alert_state
     (setq _lcma_blis_alert_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _lcma_active
     (setq _lcma_active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrFeatureAlert :md5sum-) "721bc54767b8d837fd2e98fc870215ce")
(setf (get delphi_srr_msgs::SrrFeatureAlert :datatype-) "delphi_srr_msgs/SrrFeatureAlert")
(setf (get delphi_srr_msgs::SrrFeatureAlert :definition-)
      "# Message file for srr_feature_alert

std_msgs/Header header

uint8     lcma_blis_ignored_track_id
uint8     lcma_blis_track_id
float32   lcma_cvw_ttc                             # seconds
bool      cta_ttc_alert
float32   cta_selected_track_ttc                   # seconds
uint16    cta_selected_track
uint8     cta_alert                                # binary
bool      cta_active                               # binary
uint8     lcma_cvw_cipv
uint8     lcma_cvw_alert_state
uint8     lcma_blis_alert_state
bool      lcma_active

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



(provide :delphi_srr_msgs/SrrFeatureAlert "721bc54767b8d837fd2e98fc870215ce")


