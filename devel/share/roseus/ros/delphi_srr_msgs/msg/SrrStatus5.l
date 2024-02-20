;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrStatus5)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrStatus5 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRSTATUS5")
  (make-package "DELPHI_SRR_MSGS::SRRSTATUS5"))

(in-package "ROS")
;;//! \htmlinclude SrrStatus5.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CAN_TX_YAW_RATE_REF_QF_UNDEFINED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_REF_QF_UNDEFINED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_UNDEFINED* 0)
(intern "*CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED* 1)
(intern "*CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE* 2)
(intern "*CAN_TX_YAW_RATE_REF_QF_ACCURATE*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_REF_QF_ACCURATE* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_ACCURATE* 3)
(intern "*CAN_TX_YAW_RATE_RAW_QF_UNDEFINED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_RAW_QF_UNDEFINED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_UNDEFINED* 0)
(intern "*CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED* 1)
(intern "*CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE* 2)
(intern "*CAN_TX_YAW_RATE_RAW_QF_ACCURATE*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_YAW_RATE_RAW_QF_ACCURATE* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_ACCURATE* 3)
(intern "*CAN_TX_SYSTEM_STATUS_CONFIGURATION*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_CONFIGURATION* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_CONFIGURATION* 0)
(intern "*CAN_TX_SYSTEM_STATUS_STARTUP*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_STARTUP* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_STARTUP* 1)
(intern "*CAN_TX_SYSTEM_STATUS_RUNNING*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_RUNNING* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_RUNNING* 2)
(intern "*CAN_TX_SYSTEM_STATUS_BLOCKED*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_BLOCKED* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_BLOCKED* 3)
(intern "*CAN_TX_SYSTEM_STATUS_FAULTY*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_FAULTY* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_FAULTY* 4)
(intern "*CAN_TX_SYSTEM_STATUS_SHUTDOWN*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_SHUTDOWN* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_SHUTDOWN* 5)
(intern "*CAN_TX_SYSTEM_STATUS_HOT*" (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(shadow '*CAN_TX_SYSTEM_STATUS_HOT* (find-package "DELPHI_SRR_MSGS::SRRSTATUS5"))
(defconstant delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_HOT* 6)

(defun delphi_srr_msgs::SrrStatus5-to-symbol (const)
  (cond
        ((= const 0) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_UNDEFINED*)
        ((= const 1) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED*)
        ((= const 2) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE*)
        ((= const 3) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_REF_QF_ACCURATE*)
        ((= const 0) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_UNDEFINED*)
        ((= const 1) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED*)
        ((= const 2) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE*)
        ((= const 3) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_YAW_RATE_RAW_QF_ACCURATE*)
        ((= const 0) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_CONFIGURATION*)
        ((= const 1) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_STARTUP*)
        ((= const 2) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_RUNNING*)
        ((= const 3) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_BLOCKED*)
        ((= const 4) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_FAULTY*)
        ((= const 5) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_SHUTDOWN*)
        ((= const 6) 'delphi_srr_msgs::SrrStatus5::*CAN_TX_SYSTEM_STATUS_HOT*)
        (t nil)))

(defclass delphi_srr_msgs::SrrStatus5
  :super ros::object
  :slots (_header _disable_auto_align _can_tx_yaw_rate_ref_qf _can_tx_yaw_rate_raw_qf _can_tx_yaw_rate_reference _can_tx_yaw_rate_raw _can_tx_system_status _can_tx_outside_temperature _can_blockage_mnr_blocked _can_blockage_bb_blocked _can_blockage_radar_blocked _can_td_blocked _radar_tx_power_error _radar_lo_power_error _radar_data_sync_error _linearizer_spi_transfer_error _saturated_tuning_freq_error _rtn_spi_transfer_error _rrn_spi_transfer_error _video_port_capture_error _vertical_misalignment_error _tx_temperature_fault _transmitter_id_error _dsp_unit_cal_checksum_error _dsp_unit_cal_block_chcksm_error _dsp_tuning_sensitivity_error _dsp_loop_overrun_error _adc_spi_transfer_error ))

(defmethod delphi_srr_msgs::SrrStatus5
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:disable_auto_align __disable_auto_align) nil)
    ((:can_tx_yaw_rate_ref_qf __can_tx_yaw_rate_ref_qf) 0)
    ((:can_tx_yaw_rate_raw_qf __can_tx_yaw_rate_raw_qf) 0)
    ((:can_tx_yaw_rate_reference __can_tx_yaw_rate_reference) 0.0)
    ((:can_tx_yaw_rate_raw __can_tx_yaw_rate_raw) 0.0)
    ((:can_tx_system_status __can_tx_system_status) 0)
    ((:can_tx_outside_temperature __can_tx_outside_temperature) 0)
    ((:can_blockage_mnr_blocked __can_blockage_mnr_blocked) nil)
    ((:can_blockage_bb_blocked __can_blockage_bb_blocked) nil)
    ((:can_blockage_radar_blocked __can_blockage_radar_blocked) nil)
    ((:can_td_blocked __can_td_blocked) nil)
    ((:radar_tx_power_error __radar_tx_power_error) nil)
    ((:radar_lo_power_error __radar_lo_power_error) nil)
    ((:radar_data_sync_error __radar_data_sync_error) nil)
    ((:linearizer_spi_transfer_error __linearizer_spi_transfer_error) nil)
    ((:saturated_tuning_freq_error __saturated_tuning_freq_error) nil)
    ((:rtn_spi_transfer_error __rtn_spi_transfer_error) nil)
    ((:rrn_spi_transfer_error __rrn_spi_transfer_error) nil)
    ((:video_port_capture_error __video_port_capture_error) nil)
    ((:vertical_misalignment_error __vertical_misalignment_error) nil)
    ((:tx_temperature_fault __tx_temperature_fault) nil)
    ((:transmitter_id_error __transmitter_id_error) nil)
    ((:dsp_unit_cal_checksum_error __dsp_unit_cal_checksum_error) nil)
    ((:dsp_unit_cal_block_chcksm_error __dsp_unit_cal_block_chcksm_error) nil)
    ((:dsp_tuning_sensitivity_error __dsp_tuning_sensitivity_error) nil)
    ((:dsp_loop_overrun_error __dsp_loop_overrun_error) nil)
    ((:adc_spi_transfer_error __adc_spi_transfer_error) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _disable_auto_align __disable_auto_align)
   (setq _can_tx_yaw_rate_ref_qf (round __can_tx_yaw_rate_ref_qf))
   (setq _can_tx_yaw_rate_raw_qf (round __can_tx_yaw_rate_raw_qf))
   (setq _can_tx_yaw_rate_reference (float __can_tx_yaw_rate_reference))
   (setq _can_tx_yaw_rate_raw (float __can_tx_yaw_rate_raw))
   (setq _can_tx_system_status (round __can_tx_system_status))
   (setq _can_tx_outside_temperature (round __can_tx_outside_temperature))
   (setq _can_blockage_mnr_blocked __can_blockage_mnr_blocked)
   (setq _can_blockage_bb_blocked __can_blockage_bb_blocked)
   (setq _can_blockage_radar_blocked __can_blockage_radar_blocked)
   (setq _can_td_blocked __can_td_blocked)
   (setq _radar_tx_power_error __radar_tx_power_error)
   (setq _radar_lo_power_error __radar_lo_power_error)
   (setq _radar_data_sync_error __radar_data_sync_error)
   (setq _linearizer_spi_transfer_error __linearizer_spi_transfer_error)
   (setq _saturated_tuning_freq_error __saturated_tuning_freq_error)
   (setq _rtn_spi_transfer_error __rtn_spi_transfer_error)
   (setq _rrn_spi_transfer_error __rrn_spi_transfer_error)
   (setq _video_port_capture_error __video_port_capture_error)
   (setq _vertical_misalignment_error __vertical_misalignment_error)
   (setq _tx_temperature_fault __tx_temperature_fault)
   (setq _transmitter_id_error __transmitter_id_error)
   (setq _dsp_unit_cal_checksum_error __dsp_unit_cal_checksum_error)
   (setq _dsp_unit_cal_block_chcksm_error __dsp_unit_cal_block_chcksm_error)
   (setq _dsp_tuning_sensitivity_error __dsp_tuning_sensitivity_error)
   (setq _dsp_loop_overrun_error __dsp_loop_overrun_error)
   (setq _adc_spi_transfer_error __adc_spi_transfer_error)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:disable_auto_align
   (&optional (__disable_auto_align :null))
   (if (not (eq __disable_auto_align :null)) (setq _disable_auto_align __disable_auto_align)) _disable_auto_align)
  (:can_tx_yaw_rate_ref_qf
   (&optional __can_tx_yaw_rate_ref_qf)
   (if __can_tx_yaw_rate_ref_qf (setq _can_tx_yaw_rate_ref_qf __can_tx_yaw_rate_ref_qf)) _can_tx_yaw_rate_ref_qf)
  (:can_tx_yaw_rate_raw_qf
   (&optional __can_tx_yaw_rate_raw_qf)
   (if __can_tx_yaw_rate_raw_qf (setq _can_tx_yaw_rate_raw_qf __can_tx_yaw_rate_raw_qf)) _can_tx_yaw_rate_raw_qf)
  (:can_tx_yaw_rate_reference
   (&optional __can_tx_yaw_rate_reference)
   (if __can_tx_yaw_rate_reference (setq _can_tx_yaw_rate_reference __can_tx_yaw_rate_reference)) _can_tx_yaw_rate_reference)
  (:can_tx_yaw_rate_raw
   (&optional __can_tx_yaw_rate_raw)
   (if __can_tx_yaw_rate_raw (setq _can_tx_yaw_rate_raw __can_tx_yaw_rate_raw)) _can_tx_yaw_rate_raw)
  (:can_tx_system_status
   (&optional __can_tx_system_status)
   (if __can_tx_system_status (setq _can_tx_system_status __can_tx_system_status)) _can_tx_system_status)
  (:can_tx_outside_temperature
   (&optional __can_tx_outside_temperature)
   (if __can_tx_outside_temperature (setq _can_tx_outside_temperature __can_tx_outside_temperature)) _can_tx_outside_temperature)
  (:can_blockage_mnr_blocked
   (&optional (__can_blockage_mnr_blocked :null))
   (if (not (eq __can_blockage_mnr_blocked :null)) (setq _can_blockage_mnr_blocked __can_blockage_mnr_blocked)) _can_blockage_mnr_blocked)
  (:can_blockage_bb_blocked
   (&optional (__can_blockage_bb_blocked :null))
   (if (not (eq __can_blockage_bb_blocked :null)) (setq _can_blockage_bb_blocked __can_blockage_bb_blocked)) _can_blockage_bb_blocked)
  (:can_blockage_radar_blocked
   (&optional (__can_blockage_radar_blocked :null))
   (if (not (eq __can_blockage_radar_blocked :null)) (setq _can_blockage_radar_blocked __can_blockage_radar_blocked)) _can_blockage_radar_blocked)
  (:can_td_blocked
   (&optional (__can_td_blocked :null))
   (if (not (eq __can_td_blocked :null)) (setq _can_td_blocked __can_td_blocked)) _can_td_blocked)
  (:radar_tx_power_error
   (&optional (__radar_tx_power_error :null))
   (if (not (eq __radar_tx_power_error :null)) (setq _radar_tx_power_error __radar_tx_power_error)) _radar_tx_power_error)
  (:radar_lo_power_error
   (&optional (__radar_lo_power_error :null))
   (if (not (eq __radar_lo_power_error :null)) (setq _radar_lo_power_error __radar_lo_power_error)) _radar_lo_power_error)
  (:radar_data_sync_error
   (&optional (__radar_data_sync_error :null))
   (if (not (eq __radar_data_sync_error :null)) (setq _radar_data_sync_error __radar_data_sync_error)) _radar_data_sync_error)
  (:linearizer_spi_transfer_error
   (&optional (__linearizer_spi_transfer_error :null))
   (if (not (eq __linearizer_spi_transfer_error :null)) (setq _linearizer_spi_transfer_error __linearizer_spi_transfer_error)) _linearizer_spi_transfer_error)
  (:saturated_tuning_freq_error
   (&optional (__saturated_tuning_freq_error :null))
   (if (not (eq __saturated_tuning_freq_error :null)) (setq _saturated_tuning_freq_error __saturated_tuning_freq_error)) _saturated_tuning_freq_error)
  (:rtn_spi_transfer_error
   (&optional (__rtn_spi_transfer_error :null))
   (if (not (eq __rtn_spi_transfer_error :null)) (setq _rtn_spi_transfer_error __rtn_spi_transfer_error)) _rtn_spi_transfer_error)
  (:rrn_spi_transfer_error
   (&optional (__rrn_spi_transfer_error :null))
   (if (not (eq __rrn_spi_transfer_error :null)) (setq _rrn_spi_transfer_error __rrn_spi_transfer_error)) _rrn_spi_transfer_error)
  (:video_port_capture_error
   (&optional (__video_port_capture_error :null))
   (if (not (eq __video_port_capture_error :null)) (setq _video_port_capture_error __video_port_capture_error)) _video_port_capture_error)
  (:vertical_misalignment_error
   (&optional (__vertical_misalignment_error :null))
   (if (not (eq __vertical_misalignment_error :null)) (setq _vertical_misalignment_error __vertical_misalignment_error)) _vertical_misalignment_error)
  (:tx_temperature_fault
   (&optional (__tx_temperature_fault :null))
   (if (not (eq __tx_temperature_fault :null)) (setq _tx_temperature_fault __tx_temperature_fault)) _tx_temperature_fault)
  (:transmitter_id_error
   (&optional (__transmitter_id_error :null))
   (if (not (eq __transmitter_id_error :null)) (setq _transmitter_id_error __transmitter_id_error)) _transmitter_id_error)
  (:dsp_unit_cal_checksum_error
   (&optional (__dsp_unit_cal_checksum_error :null))
   (if (not (eq __dsp_unit_cal_checksum_error :null)) (setq _dsp_unit_cal_checksum_error __dsp_unit_cal_checksum_error)) _dsp_unit_cal_checksum_error)
  (:dsp_unit_cal_block_chcksm_error
   (&optional (__dsp_unit_cal_block_chcksm_error :null))
   (if (not (eq __dsp_unit_cal_block_chcksm_error :null)) (setq _dsp_unit_cal_block_chcksm_error __dsp_unit_cal_block_chcksm_error)) _dsp_unit_cal_block_chcksm_error)
  (:dsp_tuning_sensitivity_error
   (&optional (__dsp_tuning_sensitivity_error :null))
   (if (not (eq __dsp_tuning_sensitivity_error :null)) (setq _dsp_tuning_sensitivity_error __dsp_tuning_sensitivity_error)) _dsp_tuning_sensitivity_error)
  (:dsp_loop_overrun_error
   (&optional (__dsp_loop_overrun_error :null))
   (if (not (eq __dsp_loop_overrun_error :null)) (setq _dsp_loop_overrun_error __dsp_loop_overrun_error)) _dsp_loop_overrun_error)
  (:adc_spi_transfer_error
   (&optional (__adc_spi_transfer_error :null))
   (if (not (eq __adc_spi_transfer_error :null)) (setq _adc_spi_transfer_error __adc_spi_transfer_error)) _adc_spi_transfer_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _disable_auto_align
    1
    ;; uint8 _can_tx_yaw_rate_ref_qf
    1
    ;; uint8 _can_tx_yaw_rate_raw_qf
    1
    ;; float32 _can_tx_yaw_rate_reference
    4
    ;; float32 _can_tx_yaw_rate_raw
    4
    ;; uint8 _can_tx_system_status
    1
    ;; int16 _can_tx_outside_temperature
    2
    ;; bool _can_blockage_mnr_blocked
    1
    ;; bool _can_blockage_bb_blocked
    1
    ;; bool _can_blockage_radar_blocked
    1
    ;; bool _can_td_blocked
    1
    ;; bool _radar_tx_power_error
    1
    ;; bool _radar_lo_power_error
    1
    ;; bool _radar_data_sync_error
    1
    ;; bool _linearizer_spi_transfer_error
    1
    ;; bool _saturated_tuning_freq_error
    1
    ;; bool _rtn_spi_transfer_error
    1
    ;; bool _rrn_spi_transfer_error
    1
    ;; bool _video_port_capture_error
    1
    ;; bool _vertical_misalignment_error
    1
    ;; bool _tx_temperature_fault
    1
    ;; bool _transmitter_id_error
    1
    ;; bool _dsp_unit_cal_checksum_error
    1
    ;; bool _dsp_unit_cal_block_chcksm_error
    1
    ;; bool _dsp_tuning_sensitivity_error
    1
    ;; bool _dsp_loop_overrun_error
    1
    ;; bool _adc_spi_transfer_error
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _disable_auto_align
       (if _disable_auto_align (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _can_tx_yaw_rate_ref_qf
       (write-byte _can_tx_yaw_rate_ref_qf s)
     ;; uint8 _can_tx_yaw_rate_raw_qf
       (write-byte _can_tx_yaw_rate_raw_qf s)
     ;; float32 _can_tx_yaw_rate_reference
       (sys::poke _can_tx_yaw_rate_reference (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _can_tx_yaw_rate_raw
       (sys::poke _can_tx_yaw_rate_raw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _can_tx_system_status
       (write-byte _can_tx_system_status s)
     ;; int16 _can_tx_outside_temperature
       (write-word _can_tx_outside_temperature s)
     ;; bool _can_blockage_mnr_blocked
       (if _can_blockage_mnr_blocked (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_blockage_bb_blocked
       (if _can_blockage_bb_blocked (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_blockage_radar_blocked
       (if _can_blockage_radar_blocked (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_td_blocked
       (if _can_td_blocked (write-byte -1 s) (write-byte 0 s))
     ;; bool _radar_tx_power_error
       (if _radar_tx_power_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _radar_lo_power_error
       (if _radar_lo_power_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _radar_data_sync_error
       (if _radar_data_sync_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _linearizer_spi_transfer_error
       (if _linearizer_spi_transfer_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _saturated_tuning_freq_error
       (if _saturated_tuning_freq_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _rtn_spi_transfer_error
       (if _rtn_spi_transfer_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _rrn_spi_transfer_error
       (if _rrn_spi_transfer_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _video_port_capture_error
       (if _video_port_capture_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _vertical_misalignment_error
       (if _vertical_misalignment_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _tx_temperature_fault
       (if _tx_temperature_fault (write-byte -1 s) (write-byte 0 s))
     ;; bool _transmitter_id_error
       (if _transmitter_id_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_unit_cal_checksum_error
       (if _dsp_unit_cal_checksum_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_unit_cal_block_chcksm_error
       (if _dsp_unit_cal_block_chcksm_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_tuning_sensitivity_error
       (if _dsp_tuning_sensitivity_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_loop_overrun_error
       (if _dsp_loop_overrun_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _adc_spi_transfer_error
       (if _adc_spi_transfer_error (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _disable_auto_align
     (setq _disable_auto_align (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _can_tx_yaw_rate_ref_qf
     (setq _can_tx_yaw_rate_ref_qf (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _can_tx_yaw_rate_raw_qf
     (setq _can_tx_yaw_rate_raw_qf (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _can_tx_yaw_rate_reference
     (setq _can_tx_yaw_rate_reference (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _can_tx_yaw_rate_raw
     (setq _can_tx_yaw_rate_raw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _can_tx_system_status
     (setq _can_tx_system_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _can_tx_outside_temperature
     (setq _can_tx_outside_temperature (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _can_blockage_mnr_blocked
     (setq _can_blockage_mnr_blocked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_blockage_bb_blocked
     (setq _can_blockage_bb_blocked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_blockage_radar_blocked
     (setq _can_blockage_radar_blocked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_td_blocked
     (setq _can_td_blocked (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _radar_tx_power_error
     (setq _radar_tx_power_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _radar_lo_power_error
     (setq _radar_lo_power_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _radar_data_sync_error
     (setq _radar_data_sync_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _linearizer_spi_transfer_error
     (setq _linearizer_spi_transfer_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _saturated_tuning_freq_error
     (setq _saturated_tuning_freq_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rtn_spi_transfer_error
     (setq _rtn_spi_transfer_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _rrn_spi_transfer_error
     (setq _rrn_spi_transfer_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _video_port_capture_error
     (setq _video_port_capture_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _vertical_misalignment_error
     (setq _vertical_misalignment_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tx_temperature_fault
     (setq _tx_temperature_fault (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _transmitter_id_error
     (setq _transmitter_id_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_unit_cal_checksum_error
     (setq _dsp_unit_cal_checksum_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_unit_cal_block_chcksm_error
     (setq _dsp_unit_cal_block_chcksm_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_tuning_sensitivity_error
     (setq _dsp_tuning_sensitivity_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_loop_overrun_error
     (setq _dsp_loop_overrun_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _adc_spi_transfer_error
     (setq _adc_spi_transfer_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrStatus5 :md5sum-) "cd86757abd1063dffe9941dbe3f4362f")
(setf (get delphi_srr_msgs::SrrStatus5 :datatype-) "delphi_srr_msgs/SrrStatus5")
(setf (get delphi_srr_msgs::SrrStatus5 :definition-)
      "# Message file for srr_status5

std_msgs/Header header

bool      disable_auto_align

uint8     can_tx_yaw_rate_ref_qf
uint8     CAN_TX_YAW_RATE_REF_QF_UNDEFINED=0
uint8     CAN_TX_YAW_RATE_REF_QF_TEMP_UNDEFINED=1
uint8     CAN_TX_YAW_RATE_REF_QF_NOT_ACCURATE=2
uint8     CAN_TX_YAW_RATE_REF_QF_ACCURATE=3

uint8     can_tx_yaw_rate_raw_qf
uint8     CAN_TX_YAW_RATE_RAW_QF_UNDEFINED=0
uint8     CAN_TX_YAW_RATE_RAW_QF_TEMP_UNDEFINED=1
uint8     CAN_TX_YAW_RATE_RAW_QF_NOT_ACCURATE=2
uint8     CAN_TX_YAW_RATE_RAW_QF_ACCURATE=3

float32   can_tx_yaw_rate_reference                # deg/s
float32   can_tx_yaw_rate_raw                      # deg/s

uint8     can_tx_system_status
uint8     CAN_TX_SYSTEM_STATUS_CONFIGURATION=0
uint8     CAN_TX_SYSTEM_STATUS_STARTUP=1
uint8     CAN_TX_SYSTEM_STATUS_RUNNING=2
uint8     CAN_TX_SYSTEM_STATUS_BLOCKED=3
uint8     CAN_TX_SYSTEM_STATUS_FAULTY=4
uint8     CAN_TX_SYSTEM_STATUS_SHUTDOWN=5
uint8     CAN_TX_SYSTEM_STATUS_HOT=6

int16     can_tx_outside_temperature               # degc
bool      can_blockage_mnr_blocked
bool      can_blockage_bb_blocked
bool      can_blockage_radar_blocked
bool      can_td_blocked
bool      radar_tx_power_error
bool      radar_lo_power_error
bool      radar_data_sync_error
bool      linearizer_spi_transfer_error
bool      saturated_tuning_freq_error
bool      rtn_spi_transfer_error
bool      rrn_spi_transfer_error
bool      video_port_capture_error
bool      vertical_misalignment_error
bool      tx_temperature_fault
bool      transmitter_id_error
bool      dsp_unit_cal_checksum_error
bool      dsp_unit_cal_block_chcksm_error
bool      dsp_tuning_sensitivity_error
bool      dsp_loop_overrun_error
bool      adc_spi_transfer_error

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



(provide :delphi_srr_msgs/SrrStatus5 "cd86757abd1063dffe9941dbe3f4362f")


