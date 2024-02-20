; Auto-generated. Do not edit!


(cl:in-package novatel_gps_msgs-msg)


;//! \htmlinclude NovatelReceiverStatus.msg.html

(cl:defclass <NovatelReceiverStatus> (roslisp-msg-protocol:ros-message)
  ((original_status_code
    :reader original_status_code
    :initarg :original_status_code
    :type cl:integer
    :initform 0)
   (error_flag
    :reader error_flag
    :initarg :error_flag
    :type cl:boolean
    :initform cl:nil)
   (temperature_flag
    :reader temperature_flag
    :initarg :temperature_flag
    :type cl:boolean
    :initform cl:nil)
   (voltage_supply_flag
    :reader voltage_supply_flag
    :initarg :voltage_supply_flag
    :type cl:boolean
    :initform cl:nil)
   (antenna_powered
    :reader antenna_powered
    :initarg :antenna_powered
    :type cl:boolean
    :initform cl:nil)
   (antenna_is_open
    :reader antenna_is_open
    :initarg :antenna_is_open
    :type cl:boolean
    :initform cl:nil)
   (antenna_is_shorted
    :reader antenna_is_shorted
    :initarg :antenna_is_shorted
    :type cl:boolean
    :initform cl:nil)
   (cpu_overload_flag
    :reader cpu_overload_flag
    :initarg :cpu_overload_flag
    :type cl:boolean
    :initform cl:nil)
   (com1_buffer_overrun
    :reader com1_buffer_overrun
    :initarg :com1_buffer_overrun
    :type cl:boolean
    :initform cl:nil)
   (com2_buffer_overrun
    :reader com2_buffer_overrun
    :initarg :com2_buffer_overrun
    :type cl:boolean
    :initform cl:nil)
   (com3_buffer_overrun
    :reader com3_buffer_overrun
    :initarg :com3_buffer_overrun
    :type cl:boolean
    :initform cl:nil)
   (usb_buffer_overrun
    :reader usb_buffer_overrun
    :initarg :usb_buffer_overrun
    :type cl:boolean
    :initform cl:nil)
   (rf1_agc_flag
    :reader rf1_agc_flag
    :initarg :rf1_agc_flag
    :type cl:boolean
    :initform cl:nil)
   (rf2_agc_flag
    :reader rf2_agc_flag
    :initarg :rf2_agc_flag
    :type cl:boolean
    :initform cl:nil)
   (almanac_flag
    :reader almanac_flag
    :initarg :almanac_flag
    :type cl:boolean
    :initform cl:nil)
   (position_solution_flag
    :reader position_solution_flag
    :initarg :position_solution_flag
    :type cl:boolean
    :initform cl:nil)
   (position_fixed_flag
    :reader position_fixed_flag
    :initarg :position_fixed_flag
    :type cl:boolean
    :initform cl:nil)
   (clock_steering_status_enabled
    :reader clock_steering_status_enabled
    :initarg :clock_steering_status_enabled
    :type cl:boolean
    :initform cl:nil)
   (clock_model_flag
    :reader clock_model_flag
    :initarg :clock_model_flag
    :type cl:boolean
    :initform cl:nil)
   (oemv_external_oscillator_flag
    :reader oemv_external_oscillator_flag
    :initarg :oemv_external_oscillator_flag
    :type cl:boolean
    :initform cl:nil)
   (software_resource_flag
    :reader software_resource_flag
    :initarg :software_resource_flag
    :type cl:boolean
    :initform cl:nil)
   (aux1_status_event_flag
    :reader aux1_status_event_flag
    :initarg :aux1_status_event_flag
    :type cl:boolean
    :initform cl:nil)
   (aux2_status_event_flag
    :reader aux2_status_event_flag
    :initarg :aux2_status_event_flag
    :type cl:boolean
    :initform cl:nil)
   (aux3_status_event_flag
    :reader aux3_status_event_flag
    :initarg :aux3_status_event_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass NovatelReceiverStatus (<NovatelReceiverStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NovatelReceiverStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NovatelReceiverStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name novatel_gps_msgs-msg:<NovatelReceiverStatus> is deprecated: use novatel_gps_msgs-msg:NovatelReceiverStatus instead.")))

(cl:ensure-generic-function 'original_status_code-val :lambda-list '(m))
(cl:defmethod original_status_code-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:original_status_code-val is deprecated.  Use novatel_gps_msgs-msg:original_status_code instead.")
  (original_status_code m))

(cl:ensure-generic-function 'error_flag-val :lambda-list '(m))
(cl:defmethod error_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:error_flag-val is deprecated.  Use novatel_gps_msgs-msg:error_flag instead.")
  (error_flag m))

(cl:ensure-generic-function 'temperature_flag-val :lambda-list '(m))
(cl:defmethod temperature_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:temperature_flag-val is deprecated.  Use novatel_gps_msgs-msg:temperature_flag instead.")
  (temperature_flag m))

(cl:ensure-generic-function 'voltage_supply_flag-val :lambda-list '(m))
(cl:defmethod voltage_supply_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:voltage_supply_flag-val is deprecated.  Use novatel_gps_msgs-msg:voltage_supply_flag instead.")
  (voltage_supply_flag m))

(cl:ensure-generic-function 'antenna_powered-val :lambda-list '(m))
(cl:defmethod antenna_powered-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:antenna_powered-val is deprecated.  Use novatel_gps_msgs-msg:antenna_powered instead.")
  (antenna_powered m))

(cl:ensure-generic-function 'antenna_is_open-val :lambda-list '(m))
(cl:defmethod antenna_is_open-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:antenna_is_open-val is deprecated.  Use novatel_gps_msgs-msg:antenna_is_open instead.")
  (antenna_is_open m))

(cl:ensure-generic-function 'antenna_is_shorted-val :lambda-list '(m))
(cl:defmethod antenna_is_shorted-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:antenna_is_shorted-val is deprecated.  Use novatel_gps_msgs-msg:antenna_is_shorted instead.")
  (antenna_is_shorted m))

(cl:ensure-generic-function 'cpu_overload_flag-val :lambda-list '(m))
(cl:defmethod cpu_overload_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:cpu_overload_flag-val is deprecated.  Use novatel_gps_msgs-msg:cpu_overload_flag instead.")
  (cpu_overload_flag m))

(cl:ensure-generic-function 'com1_buffer_overrun-val :lambda-list '(m))
(cl:defmethod com1_buffer_overrun-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:com1_buffer_overrun-val is deprecated.  Use novatel_gps_msgs-msg:com1_buffer_overrun instead.")
  (com1_buffer_overrun m))

(cl:ensure-generic-function 'com2_buffer_overrun-val :lambda-list '(m))
(cl:defmethod com2_buffer_overrun-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:com2_buffer_overrun-val is deprecated.  Use novatel_gps_msgs-msg:com2_buffer_overrun instead.")
  (com2_buffer_overrun m))

(cl:ensure-generic-function 'com3_buffer_overrun-val :lambda-list '(m))
(cl:defmethod com3_buffer_overrun-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:com3_buffer_overrun-val is deprecated.  Use novatel_gps_msgs-msg:com3_buffer_overrun instead.")
  (com3_buffer_overrun m))

(cl:ensure-generic-function 'usb_buffer_overrun-val :lambda-list '(m))
(cl:defmethod usb_buffer_overrun-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:usb_buffer_overrun-val is deprecated.  Use novatel_gps_msgs-msg:usb_buffer_overrun instead.")
  (usb_buffer_overrun m))

(cl:ensure-generic-function 'rf1_agc_flag-val :lambda-list '(m))
(cl:defmethod rf1_agc_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:rf1_agc_flag-val is deprecated.  Use novatel_gps_msgs-msg:rf1_agc_flag instead.")
  (rf1_agc_flag m))

(cl:ensure-generic-function 'rf2_agc_flag-val :lambda-list '(m))
(cl:defmethod rf2_agc_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:rf2_agc_flag-val is deprecated.  Use novatel_gps_msgs-msg:rf2_agc_flag instead.")
  (rf2_agc_flag m))

(cl:ensure-generic-function 'almanac_flag-val :lambda-list '(m))
(cl:defmethod almanac_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:almanac_flag-val is deprecated.  Use novatel_gps_msgs-msg:almanac_flag instead.")
  (almanac_flag m))

(cl:ensure-generic-function 'position_solution_flag-val :lambda-list '(m))
(cl:defmethod position_solution_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:position_solution_flag-val is deprecated.  Use novatel_gps_msgs-msg:position_solution_flag instead.")
  (position_solution_flag m))

(cl:ensure-generic-function 'position_fixed_flag-val :lambda-list '(m))
(cl:defmethod position_fixed_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:position_fixed_flag-val is deprecated.  Use novatel_gps_msgs-msg:position_fixed_flag instead.")
  (position_fixed_flag m))

(cl:ensure-generic-function 'clock_steering_status_enabled-val :lambda-list '(m))
(cl:defmethod clock_steering_status_enabled-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:clock_steering_status_enabled-val is deprecated.  Use novatel_gps_msgs-msg:clock_steering_status_enabled instead.")
  (clock_steering_status_enabled m))

(cl:ensure-generic-function 'clock_model_flag-val :lambda-list '(m))
(cl:defmethod clock_model_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:clock_model_flag-val is deprecated.  Use novatel_gps_msgs-msg:clock_model_flag instead.")
  (clock_model_flag m))

(cl:ensure-generic-function 'oemv_external_oscillator_flag-val :lambda-list '(m))
(cl:defmethod oemv_external_oscillator_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:oemv_external_oscillator_flag-val is deprecated.  Use novatel_gps_msgs-msg:oemv_external_oscillator_flag instead.")
  (oemv_external_oscillator_flag m))

(cl:ensure-generic-function 'software_resource_flag-val :lambda-list '(m))
(cl:defmethod software_resource_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:software_resource_flag-val is deprecated.  Use novatel_gps_msgs-msg:software_resource_flag instead.")
  (software_resource_flag m))

(cl:ensure-generic-function 'aux1_status_event_flag-val :lambda-list '(m))
(cl:defmethod aux1_status_event_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:aux1_status_event_flag-val is deprecated.  Use novatel_gps_msgs-msg:aux1_status_event_flag instead.")
  (aux1_status_event_flag m))

(cl:ensure-generic-function 'aux2_status_event_flag-val :lambda-list '(m))
(cl:defmethod aux2_status_event_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:aux2_status_event_flag-val is deprecated.  Use novatel_gps_msgs-msg:aux2_status_event_flag instead.")
  (aux2_status_event_flag m))

(cl:ensure-generic-function 'aux3_status_event_flag-val :lambda-list '(m))
(cl:defmethod aux3_status_event_flag-val ((m <NovatelReceiverStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader novatel_gps_msgs-msg:aux3_status_event_flag-val is deprecated.  Use novatel_gps_msgs-msg:aux3_status_event_flag instead.")
  (aux3_status_event_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NovatelReceiverStatus>) ostream)
  "Serializes a message object of type '<NovatelReceiverStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'original_status_code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'original_status_code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'original_status_code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'original_status_code)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'error_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'temperature_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'voltage_supply_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'antenna_powered) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'antenna_is_open) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'antenna_is_shorted) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cpu_overload_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'com1_buffer_overrun) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'com2_buffer_overrun) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'com3_buffer_overrun) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'usb_buffer_overrun) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rf1_agc_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rf2_agc_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'almanac_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'position_solution_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'position_fixed_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clock_steering_status_enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'clock_model_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'oemv_external_oscillator_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'software_resource_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aux1_status_event_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aux2_status_event_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aux3_status_event_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NovatelReceiverStatus>) istream)
  "Deserializes a message object of type '<NovatelReceiverStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'original_status_code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'original_status_code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'original_status_code)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'original_status_code)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'temperature_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'voltage_supply_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'antenna_powered) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'antenna_is_open) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'antenna_is_shorted) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cpu_overload_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'com1_buffer_overrun) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'com2_buffer_overrun) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'com3_buffer_overrun) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'usb_buffer_overrun) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rf1_agc_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rf2_agc_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'almanac_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'position_solution_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'position_fixed_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clock_steering_status_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'clock_model_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'oemv_external_oscillator_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'software_resource_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'aux1_status_event_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'aux2_status_event_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'aux3_status_event_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NovatelReceiverStatus>)))
  "Returns string type for a message object of type '<NovatelReceiverStatus>"
  "novatel_gps_msgs/NovatelReceiverStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NovatelReceiverStatus)))
  "Returns string type for a message object of type 'NovatelReceiverStatus"
  "novatel_gps_msgs/NovatelReceiverStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NovatelReceiverStatus>)))
  "Returns md5sum for a message object of type '<NovatelReceiverStatus>"
  "cf2774401808a6dde392e2ebdb09ca15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NovatelReceiverStatus)))
  "Returns md5sum for a message object of type 'NovatelReceiverStatus"
  "cf2774401808a6dde392e2ebdb09ca15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NovatelReceiverStatus>)))
  "Returns full string definition for message of type '<NovatelReceiverStatus>"
  (cl:format cl:nil "# From the original Novatel receiver status message bitfield~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%uint32 original_status_code~%bool error_flag~%bool temperature_flag~%bool voltage_supply_flag~%bool antenna_powered~%bool antenna_is_open~%bool antenna_is_shorted~%bool cpu_overload_flag~%bool com1_buffer_overrun~%bool com2_buffer_overrun~%bool com3_buffer_overrun~%bool usb_buffer_overrun~%bool rf1_agc_flag~%bool rf2_agc_flag~%bool almanac_flag~%bool position_solution_flag~%bool position_fixed_flag~%bool clock_steering_status_enabled~%bool clock_model_flag~%bool oemv_external_oscillator_flag~%bool software_resource_flag~%bool aux1_status_event_flag~%bool aux2_status_event_flag~%bool aux3_status_event_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NovatelReceiverStatus)))
  "Returns full string definition for message of type 'NovatelReceiverStatus"
  (cl:format cl:nil "# From the original Novatel receiver status message bitfield~%#  0     0x00000001   Error flag (Bit = 0: No Error, Bit = 1: Error)~%#  1     0x00000002   Temperature Status (0: Within Spec, 1: Warning)~%#  2     0x00000004   Voltage Supply Status (0: OK, 1: Warning)~%#  3     0x00000008   Antenna Power Status (0: Powered, 1: Not Powered)~%#  4     0x00000010   <Reserved>~%#  5     0x00000020   Antenna open flag (0: OK, 1: Open)~%#  6     0x00000040   Antenna shorted flag (0: OK, 1: Shorted)~%#  7     0x00000080   CPU overload flag~%#  8     0x00000100   COM1 buffer overrun flag (0: No overrun, 1: Overrun)~%#  9     0x00000200   COM2 buffer overrun flag (0: No overrun, 1: Overrun)~%#  10    0x00000400   COM3 buffer overrun flag (0: No overrun, 1: Overrun)~%#  11    0x00000800   USB buffer overrun flag (0: No overrun, 1: Overrun)~%#  12    0x00001000   <Reserved>~%#  13    0x00002000   <Reserved>~%#  14    0x00004000   <Reserved>~%#  15    0x00008000   RF1 AGC Status (0: OK, 1: Bad)~%#  16    0x00010000   <Reserverd>~%#  17    0x00020000   RF2 AGC status (0: OK, 1: Bad)~%#  18    0x00040000   Almanac flag/UTC known (0: Valid, 1: Invalid)~%#  19    0x00080000   Position solution flag (0: Valid, 1: Invalid)~%#  20    0x00100000   Position fixed flag (0: Not fixed, 1: Fixed)~%#  21    0x00200000   Clock steering status (0: Enabled, 1: Disabled)~%#  22    0x00400000   Clock model flag (0: Valid, 1: Invalid)~%#  23    0x00800000   OEMV external oscillator flag (0: Disabled, 1: Enabled)~%#  24    0x01000000   Software resource (0: OK, 1: Warning)~%#  25    0x02000000   <Reserved>~%#  26    0x04000000   <Reserved>~%#  27    0x08000000   <Reserved>~%#  28    0x10000000   <Reserved>~%#  29    0x20000000   Auxiliary 3 status event flag (0: No event, 1: Event)~%#  30    0x40000000   Auxiliary 2 status event flag (0: No event, 1: Event)~%#  31    0x80000000   Auxiliary 1 status event flag (0: No event, 1: Event)~%uint32 original_status_code~%bool error_flag~%bool temperature_flag~%bool voltage_supply_flag~%bool antenna_powered~%bool antenna_is_open~%bool antenna_is_shorted~%bool cpu_overload_flag~%bool com1_buffer_overrun~%bool com2_buffer_overrun~%bool com3_buffer_overrun~%bool usb_buffer_overrun~%bool rf1_agc_flag~%bool rf2_agc_flag~%bool almanac_flag~%bool position_solution_flag~%bool position_fixed_flag~%bool clock_steering_status_enabled~%bool clock_model_flag~%bool oemv_external_oscillator_flag~%bool software_resource_flag~%bool aux1_status_event_flag~%bool aux2_status_event_flag~%bool aux3_status_event_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NovatelReceiverStatus>))
  (cl:+ 0
     4
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NovatelReceiverStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'NovatelReceiverStatus
    (cl:cons ':original_status_code (original_status_code msg))
    (cl:cons ':error_flag (error_flag msg))
    (cl:cons ':temperature_flag (temperature_flag msg))
    (cl:cons ':voltage_supply_flag (voltage_supply_flag msg))
    (cl:cons ':antenna_powered (antenna_powered msg))
    (cl:cons ':antenna_is_open (antenna_is_open msg))
    (cl:cons ':antenna_is_shorted (antenna_is_shorted msg))
    (cl:cons ':cpu_overload_flag (cpu_overload_flag msg))
    (cl:cons ':com1_buffer_overrun (com1_buffer_overrun msg))
    (cl:cons ':com2_buffer_overrun (com2_buffer_overrun msg))
    (cl:cons ':com3_buffer_overrun (com3_buffer_overrun msg))
    (cl:cons ':usb_buffer_overrun (usb_buffer_overrun msg))
    (cl:cons ':rf1_agc_flag (rf1_agc_flag msg))
    (cl:cons ':rf2_agc_flag (rf2_agc_flag msg))
    (cl:cons ':almanac_flag (almanac_flag msg))
    (cl:cons ':position_solution_flag (position_solution_flag msg))
    (cl:cons ':position_fixed_flag (position_fixed_flag msg))
    (cl:cons ':clock_steering_status_enabled (clock_steering_status_enabled msg))
    (cl:cons ':clock_model_flag (clock_model_flag msg))
    (cl:cons ':oemv_external_oscillator_flag (oemv_external_oscillator_flag msg))
    (cl:cons ':software_resource_flag (software_resource_flag msg))
    (cl:cons ':aux1_status_event_flag (aux1_status_event_flag msg))
    (cl:cons ':aux2_status_event_flag (aux2_status_event_flag msg))
    (cl:cons ':aux3_status_event_flag (aux3_status_event_flag msg))
))
