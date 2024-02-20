;; Auto-generated. Do not edit!


(when (boundp 'delphi_esr_msgs::EsrTrack)
  (if (not (find-package "DELPHI_ESR_MSGS"))
    (make-package "DELPHI_ESR_MSGS"))
  (shadow 'EsrTrack (find-package "DELPHI_ESR_MSGS")))
(unless (find-package "DELPHI_ESR_MSGS::ESRTRACK")
  (make-package "DELPHI_ESR_MSGS::ESRTRACK"))

(in-package "ROS")
;;//! \htmlinclude EsrTrack.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_esr_msgs::EsrTrack
  :super ros::object
  :slots (_header _canmsg _id _lat_rate _grouping_changed _oncoming _status _angle _range _bridge_object _rolling_count _width _range_accel _med_range_mode _range_rate ))

(defmethod delphi_esr_msgs::EsrTrack
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:canmsg __canmsg) "")
    ((:id __id) 0)
    ((:lat_rate __lat_rate) 0.0)
    ((:grouping_changed __grouping_changed) nil)
    ((:oncoming __oncoming) nil)
    ((:status __status) 0)
    ((:angle __angle) 0.0)
    ((:range __range) 0.0)
    ((:bridge_object __bridge_object) nil)
    ((:rolling_count __rolling_count) nil)
    ((:width __width) 0.0)
    ((:range_accel __range_accel) 0.0)
    ((:med_range_mode __med_range_mode) 0)
    ((:range_rate __range_rate) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _canmsg (string __canmsg))
   (setq _id (round __id))
   (setq _lat_rate (float __lat_rate))
   (setq _grouping_changed __grouping_changed)
   (setq _oncoming __oncoming)
   (setq _status (round __status))
   (setq _angle (float __angle))
   (setq _range (float __range))
   (setq _bridge_object __bridge_object)
   (setq _rolling_count __rolling_count)
   (setq _width (float __width))
   (setq _range_accel (float __range_accel))
   (setq _med_range_mode (round __med_range_mode))
   (setq _range_rate (float __range_rate))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:canmsg
   (&optional __canmsg)
   (if __canmsg (setq _canmsg __canmsg)) _canmsg)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:lat_rate
   (&optional __lat_rate)
   (if __lat_rate (setq _lat_rate __lat_rate)) _lat_rate)
  (:grouping_changed
   (&optional (__grouping_changed :null))
   (if (not (eq __grouping_changed :null)) (setq _grouping_changed __grouping_changed)) _grouping_changed)
  (:oncoming
   (&optional (__oncoming :null))
   (if (not (eq __oncoming :null)) (setq _oncoming __oncoming)) _oncoming)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:bridge_object
   (&optional (__bridge_object :null))
   (if (not (eq __bridge_object :null)) (setq _bridge_object __bridge_object)) _bridge_object)
  (:rolling_count
   (&optional (__rolling_count :null))
   (if (not (eq __rolling_count :null)) (setq _rolling_count __rolling_count)) _rolling_count)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:range_accel
   (&optional __range_accel)
   (if __range_accel (setq _range_accel __range_accel)) _range_accel)
  (:med_range_mode
   (&optional __med_range_mode)
   (if __med_range_mode (setq _med_range_mode __med_range_mode)) _med_range_mode)
  (:range_rate
   (&optional __range_rate)
   (if __range_rate (setq _range_rate __range_rate)) _range_rate)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _canmsg
    4 (length _canmsg)
    ;; uint8 _id
    1
    ;; float32 _lat_rate
    4
    ;; bool _grouping_changed
    1
    ;; bool _oncoming
    1
    ;; uint8 _status
    1
    ;; float32 _angle
    4
    ;; float32 _range
    4
    ;; bool _bridge_object
    1
    ;; bool _rolling_count
    1
    ;; float32 _width
    4
    ;; float32 _range_accel
    4
    ;; uint8 _med_range_mode
    1
    ;; float32 _range_rate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _canmsg
       (write-long (length _canmsg) s) (princ _canmsg s)
     ;; uint8 _id
       (write-byte _id s)
     ;; float32 _lat_rate
       (sys::poke _lat_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _grouping_changed
       (if _grouping_changed (write-byte -1 s) (write-byte 0 s))
     ;; bool _oncoming
       (if _oncoming (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _status
       (write-byte _status s)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _bridge_object
       (if _bridge_object (write-byte -1 s) (write-byte 0 s))
     ;; bool _rolling_count
       (if _rolling_count (write-byte -1 s) (write-byte 0 s))
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range_accel
       (sys::poke _range_accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _med_range_mode
       (write-byte _med_range_mode s)
     ;; float32 _range_rate
       (sys::poke _range_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _canmsg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _canmsg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _lat_rate
     (setq _lat_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _grouping_changed
     (setq _grouping_changed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _oncoming
     (setq _oncoming (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _bridge_object
     (setq _bridge_object (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rolling_count
     (setq _rolling_count (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range_accel
     (setq _range_accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _med_range_mode
     (setq _med_range_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _range_rate
     (setq _range_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get delphi_esr_msgs::EsrTrack :md5sum-) "dbcd2eea001ab20b27c9a37e555910ae")
(setf (get delphi_esr_msgs::EsrTrack :datatype-) "delphi_esr_msgs/EsrTrack")
(setf (get delphi_esr_msgs::EsrTrack :definition-)
      "std_msgs/Header header

# ESR Track
string        canmsg

uint8         id
float32       lat_rate
bool          grouping_changed
bool          oncoming
uint8         status
float32       angle
float32       range
bool          bridge_object
bool          rolling_count
float32       width
float32       range_accel
uint8         med_range_mode
float32       range_rate

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



(provide :delphi_esr_msgs/EsrTrack "dbcd2eea001ab20b27c9a37e555910ae")


