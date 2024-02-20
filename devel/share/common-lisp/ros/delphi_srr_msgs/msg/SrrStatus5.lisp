; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrStatus5.msg.html

(cl:defclass <SrrStatus5> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (disable_auto_align
    :reader disable_auto_align
    :initarg :disable_auto_align
    :type cl:boolean
    :initform cl:nil)
   (can_tx_yaw_rate_ref_qf
    :reader can_tx_yaw_rate_ref_qf
    :initarg :can_tx_yaw_rate_ref_qf
    :type cl:fixnum
    :initform 0)
   (can_tx_yaw_rate_raw_qf
    :reader can_tx_yaw_rate_raw_qf
    :initarg :can_tx_yaw_rate_raw_qf
    :type cl:fixnum
    :initform 0)
   (can_tx_yaw_rate_reference
    :reader can_tx_yaw_rate_reference
    :initarg :can_tx_yaw_rate_reference
    :type cl:float
    :initform 0.0)
   (can_tx_yaw_rate_raw
    :reader can_tx_yaw_rate_raw
    :initarg :can_tx_yaw_rate_raw
    :type cl:float
    :initform 0.0)
   (can_tx_system_status
    :reader can_tx_system_status
    :initarg :can_tx_system_status
    :type cl:fixnum
    :initform 0)
   (can_tx_outside_temperature
    :reader can_tx_outside_temperature
    :initarg :can_tx_outside_temperature
    :type cl:fixnum
    :initform 0)
   (can_blockage_mnr_blocked
    :reader can_blockage_mnr_blocked
    :initarg :can_blockage_mnr_blocked
    :type cl:boolean
    :initform cl:nil)
   (can_blockage_bb_blocked
    :reader can_blockage_bb_blocked
    :initarg :can_blockage_bb_blocked
    :type cl:boolean
    :initform cl:nil)
   (can_blockage_radar_blocked
    :reader can_blockage_radar_blocked
    :initarg :can_blockage_radar_blocked
    :type cl:boolean
    :initform cl:nil)
   (can_td_blocked
    :reader can_td_blocked
    :initarg :can_td_blocked
    :type cl:boolean
    :initform cl:nil)
   (radar_tx_power_error
    :reader radar_tx_power_error
    :initarg :radar_tx_power_error
    :type cl:boolean
    :initform cl:nil)
   (radar_lo_power_error
    :reader radar_lo_power_error
    :initarg :radar_lo_power_error
    :type cl:boolean
    :initform cl:nil)
   (radar_data_sync_error
    :reader radar_data_sync_error
    :initarg :radar_data_sync_error
    :type cl:boolean
    :initform cl:nil)
   (linearizer_spi_transfer_error
    :reader linearizer_spi_transfer_error
    :initarg :linearizer_spi_transfer_error
    :type cl:boolean
    :initform cl:nil)
   (saturated_tuning_freq_error
    :reader saturated_tuning_freq_error
    :initarg :saturated_tuning_freq_error
    :type cl:boolean
    :initform cl:nil)
   (rtn_spi_transfer_error
    :reader rtn_spi_transfer_error
    :initarg :rtn_spi_transfer_error
    :type cl:boolean
    :initform cl:nil)
   (rrn_spi_transfer_error
    :reader rrn_spi_transfer_error
    :initarg :rrn_spi_transfer_error
    :type cl:boolean
    :initform cl:nil)
   (video_port_capture_error
    :reader video_port_capture_error
    :initarg :video_port_capture_error
    :type cl:boolean
    :initform cl:nil)
   (vertical_misalignment_error
    :reader vertical_misalignment_error
    :initarg :vertical_misalignment_error
    :type cl:boolean
    :initform cl:nil)
   (tx_temperature_fault
    :reader tx_temperature_fault
    :initarg :tx_temperature_fault
    :type cl:boolean
    :initform cl:nil)
   (transmitter_id_error
    :reader transmitter_id_error
    :initarg :transmitter_id_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_unit_cal_checksum_error
    :reader dsp_unit_cal_checksum_error
    :initarg :dsp_unit_cal_checksum_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_unit_cal_block_chcksm_error
    :reader dsp_unit_cal_block_chcksm_error
    :initarg :dsp_unit_cal_block_chcksm_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_tuning_sensitivity_error
    :reader dsp_tuning_sensitivity_error
    :initarg :dsp_tuning_sensitivity_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_loop_overrun_error
    :reader dsp_loop_overrun_error
    :initarg :dsp_loop_overrun_error
    :type cl:boolean
    :initform cl:nil)
   (adc_spi_transfer_error
    :reader adc_spi_transfer_error
    :initarg :adc_spi_transfer_error
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrrStatus5 (<SrrStatus5>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrStatus5>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrStatus5)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrStatus5> is deprecated: use delphi_srr_msgs-msg:SrrStatus5 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'disable_auto_align-val :lambda-list '(m))
(cl:defmethod disable_auto_align-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:disable_auto_align-val is deprecated.  Use delphi_srr_msgs-msg:disable_auto_align instead.")
  (disable_auto_align m))

(cl:ensure-generic-function 'can_tx_yaw_rate_ref_qf-val :lambda-list '(m))
(cl:defmethod can_tx_yaw_rate_ref_qf-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_yaw_rate_ref_qf-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_yaw_rate_ref_qf instead.")
  (can_tx_yaw_rate_ref_qf m))

(cl:ensure-generic-function 'can_tx_yaw_rate_raw_qf-val :lambda-list '(m))
(cl:defmethod can_tx_yaw_rate_raw_qf-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_yaw_rate_raw_qf-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_yaw_rate_raw_qf instead.")
  (can_tx_yaw_rate_raw_qf m))

(cl:ensure-generic-function 'can_tx_yaw_rate_reference-val :lambda-list '(m))
(cl:defmethod can_tx_yaw_rate_reference-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_yaw_rate_reference-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_yaw_rate_reference instead.")
  (can_tx_yaw_rate_reference m))

(cl:ensure-generic-function 'can_tx_yaw_rate_raw-val :lambda-list '(m))
(cl:defmethod can_tx_yaw_rate_raw-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_yaw_rate_raw-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_yaw_rate_raw instead.")
  (can_tx_yaw_rate_raw m))

(cl:ensure-generic-function 'can_tx_system_status-val :lambda-list '(m))
(cl:defmethod can_tx_system_status-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_system_status-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_system_status instead.")
  (can_tx_system_status m))

(cl:ensure-generic-function 'can_tx_outside_temperature-val :lambda-list '(m))
(cl:defmethod can_tx_outside_temperature-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_tx_outside_temperature-val is deprecated.  Use delphi_srr_msgs-msg:can_tx_outside_temperature instead.")
  (can_tx_outside_temperature m))

(cl:ensure-generic-function 'can_blockage_mnr_blocked-val :lambda-list '(m))
(cl:defmethod can_blockage_mnr_blocked-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_blockage_mnr_blocked-val is deprecated.  Use delphi_srr_msgs-msg:can_blockage_mnr_blocked instead.")
  (can_blockage_mnr_blocked m))

(cl:ensure-generic-function 'can_blockage_bb_blocked-val :lambda-list '(m))
(cl:defmethod can_blockage_bb_blocked-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_blockage_bb_blocked-val is deprecated.  Use delphi_srr_msgs-msg:can_blockage_bb_blocked instead.")
  (can_blockage_bb_blocked m))

(cl:ensure-generic-function 'can_blockage_radar_blocked-val :lambda-list '(m))
(cl:defmethod can_blockage_radar_blocked-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_blockage_radar_blocked-val is deprecated.  Use delphi_srr_msgs-msg:can_blockage_radar_blocked instead.")
  (can_blockage_radar_blocked m))

(cl:ensure-generic-function 'can_td_blocked-val :lambda-list '(m))
(cl:defmethod can_td_blocked-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_td_blocked-val is deprecated.  Use delphi_srr_msgs-msg:can_td_blocked instead.")
  (can_td_blocked m))

(cl:ensure-generic-function 'radar_tx_power_error-val :lambda-list '(m))
(cl:defmethod radar_tx_power_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:radar_tx_power_error-val is deprecated.  Use delphi_srr_msgs-msg:radar_tx_power_error instead.")
  (radar_tx_power_error m))

(cl:ensure-generic-function 'radar_lo_power_error-val :lambda-list '(m))
(cl:defmethod radar_lo_power_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:radar_lo_power_error-val is deprecated.  Use delphi_srr_msgs-msg:radar_lo_power_error instead.")
  (radar_lo_power_error m))

(cl:ensure-generic-function 'radar_data_sync_error-val :lambda-list '(m))
(cl:defmethod radar_data_sync_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:radar_data_sync_error-val is deprecated.  Use delphi_srr_msgs-msg:radar_data_sync_error instead.")
  (radar_data_sync_error m))

(cl:ensure-generic-function 'linearizer_spi_transfer_error-val :lambda-list '(m))
(cl:defmethod linearizer_spi_transfer_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:linearizer_spi_transfer_error-val is deprecated.  Use delphi_srr_msgs-msg:linearizer_spi_transfer_error instead.")
  (linearizer_spi_transfer_error m))

(cl:ensure-generic-function 'saturated_tuning_freq_error-val :lambda-list '(m))
(cl:defmethod saturated_tuning_freq_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:saturated_tuning_freq_error-val is deprecated.  Use delphi_srr_msgs-msg:saturated_tuning_freq_error instead.")
  (saturated_tuning_freq_error m))

(cl:ensure-generic-function 'rtn_spi_transfer_error-val :lambda-list '(m))
(cl:defmethod rtn_spi_transfer_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:rtn_spi_transfer_error-val is deprecated.  Use delphi_srr_msgs-msg:rtn_spi_transfer_error instead.")
  (rtn_spi_transfer_error m))

(cl:ensure-generic-function 'rrn_spi_transfer_error-val :lambda-list '(m))
(cl:defmethod rrn_spi_transfer_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:rrn_spi_transfer_error-val is deprecated.  Use delphi_srr_msgs-msg:rrn_spi_transfer_error instead.")
  (rrn_spi_transfer_error m))

(cl:ensure-generic-function 'video_port_capture_error-val :lambda-list '(m))
(cl:defmethod video_port_capture_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:video_port_capture_error-val is deprecated.  Use delphi_srr_msgs-msg:video_port_capture_error instead.")
  (video_port_capture_error m))

(cl:ensure-generic-function 'vertical_misalignment_error-val :lambda-list '(m))
(cl:defmethod vertical_misalignment_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:vertical_misalignment_error-val is deprecated.  Use delphi_srr_msgs-msg:vertical_misalignment_error instead.")
  (vertical_misalignment_error m))

(cl:ensure-generic-function 'tx_temperature_fault-val :lambda-list '(m))
(cl:defmethod tx_temperature_fault-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:tx_temperature_fault-val is deprecated.  Use delphi_srr_msgs-msg:tx_temperature_fault instead.")
  (tx_temperature_fault m))

(cl:ensure-generic-function 'transmitter_id_error-val :lambda-list '(m))
(cl:defmethod transmitter_id_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:transmitter_id_error-val is deprecated.  Use delphi_srr_msgs-msg:transmitter_id_error instead.")
  (transmitter_id_error m))

(cl:ensure-generic-function 'dsp_unit_cal_checksum_error-val :lambda-list '(m))
(cl:defmethod dsp_unit_cal_checksum_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_unit_cal_checksum_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_unit_cal_checksum_error instead.")
  (dsp_unit_cal_checksum_error m))

(cl:ensure-generic-function 'dsp_unit_cal_block_chcksm_error-val :lambda-list '(m))
(cl:defmethod dsp_unit_cal_block_chcksm_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_unit_cal_block_chcksm_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_unit_cal_block_chcksm_error instead.")
  (dsp_unit_cal_block_chcksm_error m))

(cl:ensure-generic-function 'dsp_tuning_sensitivity_error-val :lambda-list '(m))
(cl:defmethod dsp_tuning_sensitivity_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_tuning_sensitivity_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_tuning_sensitivity_error instead.")
  (dsp_tuning_sensitivity_error m))

(cl:ensure-generic-function 'dsp_loop_overrun_error-val :lambda-list '(m))
(cl:defmethod dsp_loop_overrun_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_loop_overrun_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_loop_overrun_error instead.")
  (dsp_loop_overrun_error m))

(cl:ensure-generic-function 'adc_spi_transfer_error-val :lambda-list '(m))
(cl:defmethod adc_spi_transfer_error-val ((m <SrrStatus5>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:adc_spi_transfer_error-val is deprecated.  Use delphi_srr_msgs-msg:adc_spi_transfer_error instead.")
  (adc_spi_transfer_error m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SrrStatus5>)))
    "Constants for message type '<SrrStatus5>"
  '((:CAN_TX_YAW_RATE_REF_QF_UNDEFINED . 0)
    (:CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED . 1)
    (:CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE . 2)
    (:CAN_TX_YAW_RATE_REF_QF_ACCURATE . 3)
    (:CAN_TX_YAW_RATE_RAW_QF_UNDEFINED . 0)
    (:CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED . 1)
    (:CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE . 2)
    (:CAN_TX_YAW_RATE_RAW_QF_ACCURATE . 3)
    (:CAN_TX_SYSTEM_STATUS_CONFIGURATION . 0)
    (:CAN_TX_SYSTEM_STATUS_STARTUP . 1)
    (:CAN_TX_SYSTEM_STATUS_RUNNING . 2)
    (:CAN_TX_SYSTEM_STATUS_BLOCKED . 3)
    (:CAN_TX_SYSTEM_STATUS_FAULTY . 4)
    (:CAN_TX_SYSTEM_STATUS_SHUTDOWN . 5)
    (:CAN_TX_SYSTEM_STATUS_HOT . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SrrStatus5)))
    "Constants for message type 'SrrStatus5"
  '((:CAN_TX_YAW_RATE_REF_QF_UNDEFINED . 0)
    (:CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED . 1)
    (:CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE . 2)
    (:CAN_TX_YAW_RATE_REF_QF_ACCURATE . 3)
    (:CAN_TX_YAW_RATE_RAW_QF_UNDEFINED . 0)
    (:CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED . 1)
    (:CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE . 2)
    (:CAN_TX_YAW_RATE_RAW_QF_ACCURATE . 3)
    (:CAN_TX_SYSTEM_STATUS_CONFIGURATION . 0)
    (:CAN_TX_SYSTEM_STATUS_STARTUP . 1)
    (:CAN_TX_SYSTEM_STATUS_RUNNING . 2)
    (:CAN_TX_SYSTEM_STATUS_BLOCKED . 3)
    (:CAN_TX_SYSTEM_STATUS_FAULTY . 4)
    (:CAN_TX_SYSTEM_STATUS_SHUTDOWN . 5)
    (:CAN_TX_SYSTEM_STATUS_HOT . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrStatus5>) ostream)
  "Serializes a message object of type '<SrrStatus5>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'disable_auto_align) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_yaw_rate_ref_qf)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_yaw_rate_raw_qf)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_yaw_rate_reference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'can_tx_yaw_rate_raw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_system_status)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'can_tx_outside_temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_blockage_mnr_blocked) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_blockage_bb_blocked) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_blockage_radar_blocked) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_td_blocked) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'radar_tx_power_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'radar_lo_power_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'radar_data_sync_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'linearizer_spi_transfer_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'saturated_tuning_freq_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rtn_spi_transfer_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rrn_spi_transfer_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'video_port_capture_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vertical_misalignment_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tx_temperature_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'transmitter_id_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_unit_cal_checksum_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_unit_cal_block_chcksm_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_tuning_sensitivity_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_loop_overrun_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'adc_spi_transfer_error) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrStatus5>) istream)
  "Deserializes a message object of type '<SrrStatus5>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'disable_auto_align) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_yaw_rate_ref_qf)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_yaw_rate_raw_qf)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_yaw_rate_reference) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'can_tx_yaw_rate_raw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'can_tx_system_status)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'can_tx_outside_temperature) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'can_blockage_mnr_blocked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_blockage_bb_blocked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_blockage_radar_blocked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_td_blocked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'radar_tx_power_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'radar_lo_power_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'radar_data_sync_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'linearizer_spi_transfer_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'saturated_tuning_freq_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rtn_spi_transfer_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'rrn_spi_transfer_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'video_port_capture_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'vertical_misalignment_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tx_temperature_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'transmitter_id_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_unit_cal_checksum_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_unit_cal_block_chcksm_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_tuning_sensitivity_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_loop_overrun_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'adc_spi_transfer_error) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrStatus5>)))
  "Returns string type for a message object of type '<SrrStatus5>"
  "delphi_srr_msgs/SrrStatus5")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrStatus5)))
  "Returns string type for a message object of type 'SrrStatus5"
  "delphi_srr_msgs/SrrStatus5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrStatus5>)))
  "Returns md5sum for a message object of type '<SrrStatus5>"
  "cd86757abd1063dffe9941dbe3f4362f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrStatus5)))
  "Returns md5sum for a message object of type 'SrrStatus5"
  "cd86757abd1063dffe9941dbe3f4362f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrStatus5>)))
  "Returns full string definition for message of type '<SrrStatus5>"
  (cl:format cl:nil "# Message file for srr_status5~%~%std_msgs/Header header~%~%bool      disable_auto_align~%~%uint8     can_tx_yaw_rate_ref_qf~%uint8     CAN_TX_YAW_RATE_REF_QF_UNDEFINED=0~%uint8     CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED=1~%uint8     CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE=2~%uint8     CAN_TX_YAW_RATE_REF_QF_ACCURATE=3~%~%uint8     can_tx_yaw_rate_raw_qf~%uint8     CAN_TX_YAW_RATE_RAW_QF_UNDEFINED=0~%uint8     CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED=1~%uint8     CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE=2~%uint8     CAN_TX_YAW_RATE_RAW_QF_ACCURATE=3~%~%float32   can_tx_yaw_rate_reference                # deg/s~%float32   can_tx_yaw_rate_raw                      # deg/s~%~%uint8     can_tx_system_status~%uint8     CAN_TX_SYSTEM_STATUS_CONFIGURATION=0~%uint8     CAN_TX_SYSTEM_STATUS_STARTUP=1~%uint8     CAN_TX_SYSTEM_STATUS_RUNNING=2~%uint8     CAN_TX_SYSTEM_STATUS_BLOCKED=3~%uint8     CAN_TX_SYSTEM_STATUS_FAULTY=4~%uint8     CAN_TX_SYSTEM_STATUS_SHUTDOWN=5~%uint8     CAN_TX_SYSTEM_STATUS_HOT=6~%~%int16     can_tx_outside_temperature               # degc~%bool      can_blockage_mnr_blocked~%bool      can_blockage_bb_blocked~%bool      can_blockage_radar_blocked~%bool      can_td_blocked~%bool      radar_tx_power_error~%bool      radar_lo_power_error~%bool      radar_data_sync_error~%bool      linearizer_spi_transfer_error~%bool      saturated_tuning_freq_error~%bool      rtn_spi_transfer_error~%bool      rrn_spi_transfer_error~%bool      video_port_capture_error~%bool      vertical_misalignment_error~%bool      tx_temperature_fault~%bool      transmitter_id_error~%bool      dsp_unit_cal_checksum_error~%bool      dsp_unit_cal_block_chcksm_error~%bool      dsp_tuning_sensitivity_error~%bool      dsp_loop_overrun_error~%bool      adc_spi_transfer_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrStatus5)))
  "Returns full string definition for message of type 'SrrStatus5"
  (cl:format cl:nil "# Message file for srr_status5~%~%std_msgs/Header header~%~%bool      disable_auto_align~%~%uint8     can_tx_yaw_rate_ref_qf~%uint8     CAN_TX_YAW_RATE_REF_QF_UNDEFINED=0~%uint8     CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED=1~%uint8     CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE=2~%uint8     CAN_TX_YAW_RATE_REF_QF_ACCURATE=3~%~%uint8     can_tx_yaw_rate_raw_qf~%uint8     CAN_TX_YAW_RATE_RAW_QF_UNDEFINED=0~%uint8     CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED=1~%uint8     CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE=2~%uint8     CAN_TX_YAW_RATE_RAW_QF_ACCURATE=3~%~%float32   can_tx_yaw_rate_reference                # deg/s~%float32   can_tx_yaw_rate_raw                      # deg/s~%~%uint8     can_tx_system_status~%uint8     CAN_TX_SYSTEM_STATUS_CONFIGURATION=0~%uint8     CAN_TX_SYSTEM_STATUS_STARTUP=1~%uint8     CAN_TX_SYSTEM_STATUS_RUNNING=2~%uint8     CAN_TX_SYSTEM_STATUS_BLOCKED=3~%uint8     CAN_TX_SYSTEM_STATUS_FAULTY=4~%uint8     CAN_TX_SYSTEM_STATUS_SHUTDOWN=5~%uint8     CAN_TX_SYSTEM_STATUS_HOT=6~%~%int16     can_tx_outside_temperature               # degc~%bool      can_blockage_mnr_blocked~%bool      can_blockage_bb_blocked~%bool      can_blockage_radar_blocked~%bool      can_td_blocked~%bool      radar_tx_power_error~%bool      radar_lo_power_error~%bool      radar_data_sync_error~%bool      linearizer_spi_transfer_error~%bool      saturated_tuning_freq_error~%bool      rtn_spi_transfer_error~%bool      rrn_spi_transfer_error~%bool      video_port_capture_error~%bool      vertical_misalignment_error~%bool      tx_temperature_fault~%bool      transmitter_id_error~%bool      dsp_unit_cal_checksum_error~%bool      dsp_unit_cal_block_chcksm_error~%bool      dsp_tuning_sensitivity_error~%bool      dsp_loop_overrun_error~%bool      adc_spi_transfer_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrStatus5>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     4
     1
     2
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrStatus5>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrStatus5
    (cl:cons ':header (header msg))
    (cl:cons ':disable_auto_align (disable_auto_align msg))
    (cl:cons ':can_tx_yaw_rate_ref_qf (can_tx_yaw_rate_ref_qf msg))
    (cl:cons ':can_tx_yaw_rate_raw_qf (can_tx_yaw_rate_raw_qf msg))
    (cl:cons ':can_tx_yaw_rate_reference (can_tx_yaw_rate_reference msg))
    (cl:cons ':can_tx_yaw_rate_raw (can_tx_yaw_rate_raw msg))
    (cl:cons ':can_tx_system_status (can_tx_system_status msg))
    (cl:cons ':can_tx_outside_temperature (can_tx_outside_temperature msg))
    (cl:cons ':can_blockage_mnr_blocked (can_blockage_mnr_blocked msg))
    (cl:cons ':can_blockage_bb_blocked (can_blockage_bb_blocked msg))
    (cl:cons ':can_blockage_radar_blocked (can_blockage_radar_blocked msg))
    (cl:cons ':can_td_blocked (can_td_blocked msg))
    (cl:cons ':radar_tx_power_error (radar_tx_power_error msg))
    (cl:cons ':radar_lo_power_error (radar_lo_power_error msg))
    (cl:cons ':radar_data_sync_error (radar_data_sync_error msg))
    (cl:cons ':linearizer_spi_transfer_error (linearizer_spi_transfer_error msg))
    (cl:cons ':saturated_tuning_freq_error (saturated_tuning_freq_error msg))
    (cl:cons ':rtn_spi_transfer_error (rtn_spi_transfer_error msg))
    (cl:cons ':rrn_spi_transfer_error (rrn_spi_transfer_error msg))
    (cl:cons ':video_port_capture_error (video_port_capture_error msg))
    (cl:cons ':vertical_misalignment_error (vertical_misalignment_error msg))
    (cl:cons ':tx_temperature_fault (tx_temperature_fault msg))
    (cl:cons ':transmitter_id_error (transmitter_id_error msg))
    (cl:cons ':dsp_unit_cal_checksum_error (dsp_unit_cal_checksum_error msg))
    (cl:cons ':dsp_unit_cal_block_chcksm_error (dsp_unit_cal_block_chcksm_error msg))
    (cl:cons ':dsp_tuning_sensitivity_error (dsp_tuning_sensitivity_error msg))
    (cl:cons ':dsp_loop_overrun_error (dsp_loop_overrun_error msg))
    (cl:cons ':adc_spi_transfer_error (adc_spi_transfer_error msg))
))
