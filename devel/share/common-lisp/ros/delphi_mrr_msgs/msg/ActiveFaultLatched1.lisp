; Auto-generated. Do not edit!


(cl:in-package delphi_mrr_msgs-msg)


;//! \htmlinclude ActiveFaultLatched1.msg.html

(cl:defclass <ActiveFaultLatched1> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (active_flt_latched_byte7_bit7
    :reader active_flt_latched_byte7_bit7
    :initarg :active_flt_latched_byte7_bit7
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte7_bit6
    :reader active_flt_latched_byte7_bit6
    :initarg :active_flt_latched_byte7_bit6
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte7_bit5
    :reader active_flt_latched_byte7_bit5
    :initarg :active_flt_latched_byte7_bit5
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte7_bit4
    :reader active_flt_latched_byte7_bit4
    :initarg :active_flt_latched_byte7_bit4
    :type cl:boolean
    :initform cl:nil)
   (arm_to_dsp_chksum_fault
    :reader arm_to_dsp_chksum_fault
    :initarg :arm_to_dsp_chksum_fault
    :type cl:boolean
    :initform cl:nil)
   (dsp_to_arm_chksum_fault
    :reader dsp_to_arm_chksum_fault
    :initarg :dsp_to_arm_chksum_fault
    :type cl:boolean
    :initform cl:nil)
   (host_to_arm_chksum_fault
    :reader host_to_arm_chksum_fault
    :initarg :host_to_arm_chksum_fault
    :type cl:boolean
    :initform cl:nil)
   (arm_to_host_chksum_fault
    :reader arm_to_host_chksum_fault
    :initarg :arm_to_host_chksum_fault
    :type cl:boolean
    :initform cl:nil)
   (loop_bw_out_of_range
    :reader loop_bw_out_of_range
    :initarg :loop_bw_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (dsp_overrun_fault
    :reader dsp_overrun_fault
    :initarg :dsp_overrun_fault
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte6_bit5
    :reader active_flt_latched_byte6_bit5
    :initarg :active_flt_latched_byte6_bit5
    :type cl:boolean
    :initform cl:nil)
   (tuning_sensitivity_fault
    :reader tuning_sensitivity_fault
    :initarg :tuning_sensitivity_fault
    :type cl:boolean
    :initform cl:nil)
   (saturated_tuning_freq_fault
    :reader saturated_tuning_freq_fault
    :initarg :saturated_tuning_freq_fault
    :type cl:boolean
    :initform cl:nil)
   (local_osc_power_fault
    :reader local_osc_power_fault
    :initarg :local_osc_power_fault
    :type cl:boolean
    :initform cl:nil)
   (transmitter_power_fault
    :reader transmitter_power_fault
    :initarg :transmitter_power_fault
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte6_bit0
    :reader active_flt_latched_byte6_bit0
    :initarg :active_flt_latched_byte6_bit0
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte5_bit7
    :reader active_flt_latched_byte5_bit7
    :initarg :active_flt_latched_byte5_bit7
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte5_bit6
    :reader active_flt_latched_byte5_bit6
    :initarg :active_flt_latched_byte5_bit6
    :type cl:boolean
    :initform cl:nil)
   (xcvr_device_spi_fault
    :reader xcvr_device_spi_fault
    :initarg :xcvr_device_spi_fault
    :type cl:boolean
    :initform cl:nil)
   (freq_synthesizer_spi_fault
    :reader freq_synthesizer_spi_fault
    :initarg :freq_synthesizer_spi_fault
    :type cl:boolean
    :initform cl:nil)
   (analog_converter_devic_spi_fault
    :reader analog_converter_devic_spi_fault
    :initarg :analog_converter_devic_spi_fault
    :type cl:boolean
    :initform cl:nil)
   (side_lobe_blockage
    :reader side_lobe_blockage
    :initarg :side_lobe_blockage
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte5_bit1
    :reader active_flt_latched_byte5_bit1
    :initarg :active_flt_latched_byte5_bit1
    :type cl:boolean
    :initform cl:nil)
   (mnr_blocked
    :reader mnr_blocked
    :initarg :mnr_blocked
    :type cl:boolean
    :initform cl:nil)
   (ecu_temp_high_fault
    :reader ecu_temp_high_fault
    :initarg :ecu_temp_high_fault
    :type cl:boolean
    :initform cl:nil)
   (transmitter_temp_high_fault
    :reader transmitter_temp_high_fault
    :initarg :transmitter_temp_high_fault
    :type cl:boolean
    :initform cl:nil)
   (alignment_routine_failed_fault
    :reader alignment_routine_failed_fault
    :initarg :alignment_routine_failed_fault
    :type cl:boolean
    :initform cl:nil)
   (unreasonable_radar_data
    :reader unreasonable_radar_data
    :initarg :unreasonable_radar_data
    :type cl:boolean
    :initform cl:nil)
   (microprocessor_temp_high_fault
    :reader microprocessor_temp_high_fault
    :initarg :microprocessor_temp_high_fault
    :type cl:boolean
    :initform cl:nil)
   (vertical_alignment_out_of_range
    :reader vertical_alignment_out_of_range
    :initarg :vertical_alignment_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (horizontal_alignment_out_of_range
    :reader horizontal_alignment_out_of_range
    :initarg :horizontal_alignment_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (factory_alignment_mode
    :reader factory_alignment_mode
    :initarg :factory_alignment_mode
    :type cl:boolean
    :initform cl:nil)
   (battery_low_fault
    :reader battery_low_fault
    :initarg :battery_low_fault
    :type cl:boolean
    :initform cl:nil)
   (battery_high_fault
    :reader battery_high_fault
    :initarg :battery_high_fault
    :type cl:boolean
    :initform cl:nil)
   (v_1p25_supply_out_of_range
    :reader v_1p25_supply_out_of_range
    :initarg :v_1p25_supply_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte3_bit4
    :reader active_flt_latched_byte3_bit4
    :initarg :active_flt_latched_byte3_bit4
    :type cl:boolean
    :initform cl:nil)
   (thermistor_out_of_range
    :reader thermistor_out_of_range
    :initarg :thermistor_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (v_3p3_dac_supply_out_of_range
    :reader v_3p3_dac_supply_out_of_range
    :initarg :v_3p3_dac_supply_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (v_3p3_raw_supply_out_of_range
    :reader v_3p3_raw_supply_out_of_range
    :initarg :v_3p3_raw_supply_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (v_5_supply_out_of_range
    :reader v_5_supply_out_of_range
    :initarg :v_5_supply_out_of_range
    :type cl:boolean
    :initform cl:nil)
   (transmitter_id_fault
    :reader transmitter_id_fault
    :initarg :transmitter_id_fault
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte2_bit6
    :reader active_flt_latched_byte2_bit6
    :initarg :active_flt_latched_byte2_bit6
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte2_bit5
    :reader active_flt_latched_byte2_bit5
    :initarg :active_flt_latched_byte2_bit5
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte2_bit4
    :reader active_flt_latched_byte2_bit4
    :initarg :active_flt_latched_byte2_bit4
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte2_bit3
    :reader active_flt_latched_byte2_bit3
    :initarg :active_flt_latched_byte2_bit3
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte2_bit2
    :reader active_flt_latched_byte2_bit2
    :initarg :active_flt_latched_byte2_bit2
    :type cl:boolean
    :initform cl:nil)
   (pcan_missing_msg_fault
    :reader pcan_missing_msg_fault
    :initarg :pcan_missing_msg_fault
    :type cl:boolean
    :initform cl:nil)
   (pcan_bus_off
    :reader pcan_bus_off
    :initarg :pcan_bus_off
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte1_bit7
    :reader active_flt_latched_byte1_bit7
    :initarg :active_flt_latched_byte1_bit7
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte1_bit6
    :reader active_flt_latched_byte1_bit6
    :initarg :active_flt_latched_byte1_bit6
    :type cl:boolean
    :initform cl:nil)
   (instruction_set_check_fault
    :reader instruction_set_check_fault
    :initarg :instruction_set_check_fault
    :type cl:boolean
    :initform cl:nil)
   (stack_overflow_fault
    :reader stack_overflow_fault
    :initarg :stack_overflow_fault
    :type cl:boolean
    :initform cl:nil)
   (watchdog_fault
    :reader watchdog_fault
    :initarg :watchdog_fault
    :type cl:boolean
    :initform cl:nil)
   (pll_lock_fault
    :reader pll_lock_fault
    :initarg :pll_lock_fault
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte1_bit1
    :reader active_flt_latched_byte1_bit1
    :initarg :active_flt_latched_byte1_bit1
    :type cl:boolean
    :initform cl:nil)
   (ram_memory_test_fault
    :reader ram_memory_test_fault
    :initarg :ram_memory_test_fault
    :type cl:boolean
    :initform cl:nil)
   (usc_validation_fault
    :reader usc_validation_fault
    :initarg :usc_validation_fault
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte0_bit6
    :reader active_flt_latched_byte0_bit6
    :initarg :active_flt_latched_byte0_bit6
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte0_bit5
    :reader active_flt_latched_byte0_bit5
    :initarg :active_flt_latched_byte0_bit5
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte0_bit4
    :reader active_flt_latched_byte0_bit4
    :initarg :active_flt_latched_byte0_bit4
    :type cl:boolean
    :initform cl:nil)
   (active_flt_latched_byte0_bit3
    :reader active_flt_latched_byte0_bit3
    :initarg :active_flt_latched_byte0_bit3
    :type cl:boolean
    :initform cl:nil)
   (keep_alive_checksum_fault
    :reader keep_alive_checksum_fault
    :initarg :keep_alive_checksum_fault
    :type cl:boolean
    :initform cl:nil)
   (program_calibration_flash_checksum
    :reader program_calibration_flash_checksum
    :initarg :program_calibration_flash_checksum
    :type cl:boolean
    :initform cl:nil)
   (application_flash_checksum_fault
    :reader application_flash_checksum_fault
    :initarg :application_flash_checksum_fault
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ActiveFaultLatched1 (<ActiveFaultLatched1>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActiveFaultLatched1>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActiveFaultLatched1)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_mrr_msgs-msg:<ActiveFaultLatched1> is deprecated: use delphi_mrr_msgs-msg:ActiveFaultLatched1 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:header-val is deprecated.  Use delphi_mrr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'active_flt_latched_byte7_bit7-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte7_bit7-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte7_bit7-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte7_bit7 instead.")
  (active_flt_latched_byte7_bit7 m))

(cl:ensure-generic-function 'active_flt_latched_byte7_bit6-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte7_bit6-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte7_bit6-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte7_bit6 instead.")
  (active_flt_latched_byte7_bit6 m))

(cl:ensure-generic-function 'active_flt_latched_byte7_bit5-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte7_bit5-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte7_bit5-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte7_bit5 instead.")
  (active_flt_latched_byte7_bit5 m))

(cl:ensure-generic-function 'active_flt_latched_byte7_bit4-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte7_bit4-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte7_bit4-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte7_bit4 instead.")
  (active_flt_latched_byte7_bit4 m))

(cl:ensure-generic-function 'arm_to_dsp_chksum_fault-val :lambda-list '(m))
(cl:defmethod arm_to_dsp_chksum_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:arm_to_dsp_chksum_fault-val is deprecated.  Use delphi_mrr_msgs-msg:arm_to_dsp_chksum_fault instead.")
  (arm_to_dsp_chksum_fault m))

(cl:ensure-generic-function 'dsp_to_arm_chksum_fault-val :lambda-list '(m))
(cl:defmethod dsp_to_arm_chksum_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:dsp_to_arm_chksum_fault-val is deprecated.  Use delphi_mrr_msgs-msg:dsp_to_arm_chksum_fault instead.")
  (dsp_to_arm_chksum_fault m))

(cl:ensure-generic-function 'host_to_arm_chksum_fault-val :lambda-list '(m))
(cl:defmethod host_to_arm_chksum_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:host_to_arm_chksum_fault-val is deprecated.  Use delphi_mrr_msgs-msg:host_to_arm_chksum_fault instead.")
  (host_to_arm_chksum_fault m))

(cl:ensure-generic-function 'arm_to_host_chksum_fault-val :lambda-list '(m))
(cl:defmethod arm_to_host_chksum_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:arm_to_host_chksum_fault-val is deprecated.  Use delphi_mrr_msgs-msg:arm_to_host_chksum_fault instead.")
  (arm_to_host_chksum_fault m))

(cl:ensure-generic-function 'loop_bw_out_of_range-val :lambda-list '(m))
(cl:defmethod loop_bw_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:loop_bw_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:loop_bw_out_of_range instead.")
  (loop_bw_out_of_range m))

(cl:ensure-generic-function 'dsp_overrun_fault-val :lambda-list '(m))
(cl:defmethod dsp_overrun_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:dsp_overrun_fault-val is deprecated.  Use delphi_mrr_msgs-msg:dsp_overrun_fault instead.")
  (dsp_overrun_fault m))

(cl:ensure-generic-function 'active_flt_latched_byte6_bit5-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte6_bit5-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte6_bit5-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte6_bit5 instead.")
  (active_flt_latched_byte6_bit5 m))

(cl:ensure-generic-function 'tuning_sensitivity_fault-val :lambda-list '(m))
(cl:defmethod tuning_sensitivity_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:tuning_sensitivity_fault-val is deprecated.  Use delphi_mrr_msgs-msg:tuning_sensitivity_fault instead.")
  (tuning_sensitivity_fault m))

(cl:ensure-generic-function 'saturated_tuning_freq_fault-val :lambda-list '(m))
(cl:defmethod saturated_tuning_freq_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:saturated_tuning_freq_fault-val is deprecated.  Use delphi_mrr_msgs-msg:saturated_tuning_freq_fault instead.")
  (saturated_tuning_freq_fault m))

(cl:ensure-generic-function 'local_osc_power_fault-val :lambda-list '(m))
(cl:defmethod local_osc_power_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:local_osc_power_fault-val is deprecated.  Use delphi_mrr_msgs-msg:local_osc_power_fault instead.")
  (local_osc_power_fault m))

(cl:ensure-generic-function 'transmitter_power_fault-val :lambda-list '(m))
(cl:defmethod transmitter_power_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:transmitter_power_fault-val is deprecated.  Use delphi_mrr_msgs-msg:transmitter_power_fault instead.")
  (transmitter_power_fault m))

(cl:ensure-generic-function 'active_flt_latched_byte6_bit0-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte6_bit0-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte6_bit0-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte6_bit0 instead.")
  (active_flt_latched_byte6_bit0 m))

(cl:ensure-generic-function 'active_flt_latched_byte5_bit7-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte5_bit7-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte5_bit7-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte5_bit7 instead.")
  (active_flt_latched_byte5_bit7 m))

(cl:ensure-generic-function 'active_flt_latched_byte5_bit6-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte5_bit6-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte5_bit6-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte5_bit6 instead.")
  (active_flt_latched_byte5_bit6 m))

(cl:ensure-generic-function 'xcvr_device_spi_fault-val :lambda-list '(m))
(cl:defmethod xcvr_device_spi_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:xcvr_device_spi_fault-val is deprecated.  Use delphi_mrr_msgs-msg:xcvr_device_spi_fault instead.")
  (xcvr_device_spi_fault m))

(cl:ensure-generic-function 'freq_synthesizer_spi_fault-val :lambda-list '(m))
(cl:defmethod freq_synthesizer_spi_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:freq_synthesizer_spi_fault-val is deprecated.  Use delphi_mrr_msgs-msg:freq_synthesizer_spi_fault instead.")
  (freq_synthesizer_spi_fault m))

(cl:ensure-generic-function 'analog_converter_devic_spi_fault-val :lambda-list '(m))
(cl:defmethod analog_converter_devic_spi_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:analog_converter_devic_spi_fault-val is deprecated.  Use delphi_mrr_msgs-msg:analog_converter_devic_spi_fault instead.")
  (analog_converter_devic_spi_fault m))

(cl:ensure-generic-function 'side_lobe_blockage-val :lambda-list '(m))
(cl:defmethod side_lobe_blockage-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:side_lobe_blockage-val is deprecated.  Use delphi_mrr_msgs-msg:side_lobe_blockage instead.")
  (side_lobe_blockage m))

(cl:ensure-generic-function 'active_flt_latched_byte5_bit1-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte5_bit1-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte5_bit1-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte5_bit1 instead.")
  (active_flt_latched_byte5_bit1 m))

(cl:ensure-generic-function 'mnr_blocked-val :lambda-list '(m))
(cl:defmethod mnr_blocked-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:mnr_blocked-val is deprecated.  Use delphi_mrr_msgs-msg:mnr_blocked instead.")
  (mnr_blocked m))

(cl:ensure-generic-function 'ecu_temp_high_fault-val :lambda-list '(m))
(cl:defmethod ecu_temp_high_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:ecu_temp_high_fault-val is deprecated.  Use delphi_mrr_msgs-msg:ecu_temp_high_fault instead.")
  (ecu_temp_high_fault m))

(cl:ensure-generic-function 'transmitter_temp_high_fault-val :lambda-list '(m))
(cl:defmethod transmitter_temp_high_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:transmitter_temp_high_fault-val is deprecated.  Use delphi_mrr_msgs-msg:transmitter_temp_high_fault instead.")
  (transmitter_temp_high_fault m))

(cl:ensure-generic-function 'alignment_routine_failed_fault-val :lambda-list '(m))
(cl:defmethod alignment_routine_failed_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:alignment_routine_failed_fault-val is deprecated.  Use delphi_mrr_msgs-msg:alignment_routine_failed_fault instead.")
  (alignment_routine_failed_fault m))

(cl:ensure-generic-function 'unreasonable_radar_data-val :lambda-list '(m))
(cl:defmethod unreasonable_radar_data-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:unreasonable_radar_data-val is deprecated.  Use delphi_mrr_msgs-msg:unreasonable_radar_data instead.")
  (unreasonable_radar_data m))

(cl:ensure-generic-function 'microprocessor_temp_high_fault-val :lambda-list '(m))
(cl:defmethod microprocessor_temp_high_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:microprocessor_temp_high_fault-val is deprecated.  Use delphi_mrr_msgs-msg:microprocessor_temp_high_fault instead.")
  (microprocessor_temp_high_fault m))

(cl:ensure-generic-function 'vertical_alignment_out_of_range-val :lambda-list '(m))
(cl:defmethod vertical_alignment_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:vertical_alignment_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:vertical_alignment_out_of_range instead.")
  (vertical_alignment_out_of_range m))

(cl:ensure-generic-function 'horizontal_alignment_out_of_range-val :lambda-list '(m))
(cl:defmethod horizontal_alignment_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:horizontal_alignment_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:horizontal_alignment_out_of_range instead.")
  (horizontal_alignment_out_of_range m))

(cl:ensure-generic-function 'factory_alignment_mode-val :lambda-list '(m))
(cl:defmethod factory_alignment_mode-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:factory_alignment_mode-val is deprecated.  Use delphi_mrr_msgs-msg:factory_alignment_mode instead.")
  (factory_alignment_mode m))

(cl:ensure-generic-function 'battery_low_fault-val :lambda-list '(m))
(cl:defmethod battery_low_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:battery_low_fault-val is deprecated.  Use delphi_mrr_msgs-msg:battery_low_fault instead.")
  (battery_low_fault m))

(cl:ensure-generic-function 'battery_high_fault-val :lambda-list '(m))
(cl:defmethod battery_high_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:battery_high_fault-val is deprecated.  Use delphi_mrr_msgs-msg:battery_high_fault instead.")
  (battery_high_fault m))

(cl:ensure-generic-function 'v_1p25_supply_out_of_range-val :lambda-list '(m))
(cl:defmethod v_1p25_supply_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:v_1p25_supply_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:v_1p25_supply_out_of_range instead.")
  (v_1p25_supply_out_of_range m))

(cl:ensure-generic-function 'active_flt_latched_byte3_bit4-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte3_bit4-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte3_bit4-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte3_bit4 instead.")
  (active_flt_latched_byte3_bit4 m))

(cl:ensure-generic-function 'thermistor_out_of_range-val :lambda-list '(m))
(cl:defmethod thermistor_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:thermistor_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:thermistor_out_of_range instead.")
  (thermistor_out_of_range m))

(cl:ensure-generic-function 'v_3p3_dac_supply_out_of_range-val :lambda-list '(m))
(cl:defmethod v_3p3_dac_supply_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:v_3p3_dac_supply_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:v_3p3_dac_supply_out_of_range instead.")
  (v_3p3_dac_supply_out_of_range m))

(cl:ensure-generic-function 'v_3p3_raw_supply_out_of_range-val :lambda-list '(m))
(cl:defmethod v_3p3_raw_supply_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:v_3p3_raw_supply_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:v_3p3_raw_supply_out_of_range instead.")
  (v_3p3_raw_supply_out_of_range m))

(cl:ensure-generic-function 'v_5_supply_out_of_range-val :lambda-list '(m))
(cl:defmethod v_5_supply_out_of_range-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:v_5_supply_out_of_range-val is deprecated.  Use delphi_mrr_msgs-msg:v_5_supply_out_of_range instead.")
  (v_5_supply_out_of_range m))

(cl:ensure-generic-function 'transmitter_id_fault-val :lambda-list '(m))
(cl:defmethod transmitter_id_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:transmitter_id_fault-val is deprecated.  Use delphi_mrr_msgs-msg:transmitter_id_fault instead.")
  (transmitter_id_fault m))

(cl:ensure-generic-function 'active_flt_latched_byte2_bit6-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte2_bit6-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte2_bit6-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte2_bit6 instead.")
  (active_flt_latched_byte2_bit6 m))

(cl:ensure-generic-function 'active_flt_latched_byte2_bit5-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte2_bit5-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte2_bit5-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte2_bit5 instead.")
  (active_flt_latched_byte2_bit5 m))

(cl:ensure-generic-function 'active_flt_latched_byte2_bit4-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte2_bit4-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte2_bit4-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte2_bit4 instead.")
  (active_flt_latched_byte2_bit4 m))

(cl:ensure-generic-function 'active_flt_latched_byte2_bit3-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte2_bit3-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte2_bit3-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte2_bit3 instead.")
  (active_flt_latched_byte2_bit3 m))

(cl:ensure-generic-function 'active_flt_latched_byte2_bit2-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte2_bit2-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte2_bit2-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte2_bit2 instead.")
  (active_flt_latched_byte2_bit2 m))

(cl:ensure-generic-function 'pcan_missing_msg_fault-val :lambda-list '(m))
(cl:defmethod pcan_missing_msg_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:pcan_missing_msg_fault-val is deprecated.  Use delphi_mrr_msgs-msg:pcan_missing_msg_fault instead.")
  (pcan_missing_msg_fault m))

(cl:ensure-generic-function 'pcan_bus_off-val :lambda-list '(m))
(cl:defmethod pcan_bus_off-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:pcan_bus_off-val is deprecated.  Use delphi_mrr_msgs-msg:pcan_bus_off instead.")
  (pcan_bus_off m))

(cl:ensure-generic-function 'active_flt_latched_byte1_bit7-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte1_bit7-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte1_bit7-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte1_bit7 instead.")
  (active_flt_latched_byte1_bit7 m))

(cl:ensure-generic-function 'active_flt_latched_byte1_bit6-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte1_bit6-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte1_bit6-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte1_bit6 instead.")
  (active_flt_latched_byte1_bit6 m))

(cl:ensure-generic-function 'instruction_set_check_fault-val :lambda-list '(m))
(cl:defmethod instruction_set_check_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:instruction_set_check_fault-val is deprecated.  Use delphi_mrr_msgs-msg:instruction_set_check_fault instead.")
  (instruction_set_check_fault m))

(cl:ensure-generic-function 'stack_overflow_fault-val :lambda-list '(m))
(cl:defmethod stack_overflow_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:stack_overflow_fault-val is deprecated.  Use delphi_mrr_msgs-msg:stack_overflow_fault instead.")
  (stack_overflow_fault m))

(cl:ensure-generic-function 'watchdog_fault-val :lambda-list '(m))
(cl:defmethod watchdog_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:watchdog_fault-val is deprecated.  Use delphi_mrr_msgs-msg:watchdog_fault instead.")
  (watchdog_fault m))

(cl:ensure-generic-function 'pll_lock_fault-val :lambda-list '(m))
(cl:defmethod pll_lock_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:pll_lock_fault-val is deprecated.  Use delphi_mrr_msgs-msg:pll_lock_fault instead.")
  (pll_lock_fault m))

(cl:ensure-generic-function 'active_flt_latched_byte1_bit1-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte1_bit1-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte1_bit1-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte1_bit1 instead.")
  (active_flt_latched_byte1_bit1 m))

(cl:ensure-generic-function 'ram_memory_test_fault-val :lambda-list '(m))
(cl:defmethod ram_memory_test_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:ram_memory_test_fault-val is deprecated.  Use delphi_mrr_msgs-msg:ram_memory_test_fault instead.")
  (ram_memory_test_fault m))

(cl:ensure-generic-function 'usc_validation_fault-val :lambda-list '(m))
(cl:defmethod usc_validation_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:usc_validation_fault-val is deprecated.  Use delphi_mrr_msgs-msg:usc_validation_fault instead.")
  (usc_validation_fault m))

(cl:ensure-generic-function 'active_flt_latched_byte0_bit6-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte0_bit6-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte0_bit6-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte0_bit6 instead.")
  (active_flt_latched_byte0_bit6 m))

(cl:ensure-generic-function 'active_flt_latched_byte0_bit5-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte0_bit5-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte0_bit5-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte0_bit5 instead.")
  (active_flt_latched_byte0_bit5 m))

(cl:ensure-generic-function 'active_flt_latched_byte0_bit4-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte0_bit4-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte0_bit4-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte0_bit4 instead.")
  (active_flt_latched_byte0_bit4 m))

(cl:ensure-generic-function 'active_flt_latched_byte0_bit3-val :lambda-list '(m))
(cl:defmethod active_flt_latched_byte0_bit3-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:active_flt_latched_byte0_bit3-val is deprecated.  Use delphi_mrr_msgs-msg:active_flt_latched_byte0_bit3 instead.")
  (active_flt_latched_byte0_bit3 m))

(cl:ensure-generic-function 'keep_alive_checksum_fault-val :lambda-list '(m))
(cl:defmethod keep_alive_checksum_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:keep_alive_checksum_fault-val is deprecated.  Use delphi_mrr_msgs-msg:keep_alive_checksum_fault instead.")
  (keep_alive_checksum_fault m))

(cl:ensure-generic-function 'program_calibration_flash_checksum-val :lambda-list '(m))
(cl:defmethod program_calibration_flash_checksum-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:program_calibration_flash_checksum-val is deprecated.  Use delphi_mrr_msgs-msg:program_calibration_flash_checksum instead.")
  (program_calibration_flash_checksum m))

(cl:ensure-generic-function 'application_flash_checksum_fault-val :lambda-list '(m))
(cl:defmethod application_flash_checksum_fault-val ((m <ActiveFaultLatched1>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_mrr_msgs-msg:application_flash_checksum_fault-val is deprecated.  Use delphi_mrr_msgs-msg:application_flash_checksum_fault instead.")
  (application_flash_checksum_fault m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActiveFaultLatched1>) ostream)
  "Serializes a message object of type '<ActiveFaultLatched1>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte7_bit7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte7_bit6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte7_bit5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte7_bit4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arm_to_dsp_chksum_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_to_arm_chksum_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'host_to_arm_chksum_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arm_to_host_chksum_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'loop_bw_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_overrun_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte6_bit5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tuning_sensitivity_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'saturated_tuning_freq_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'local_osc_power_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'transmitter_power_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte6_bit0) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte5_bit7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte5_bit6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'xcvr_device_spi_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'freq_synthesizer_spi_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'analog_converter_devic_spi_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'side_lobe_blockage) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte5_bit1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mnr_blocked) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ecu_temp_high_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'transmitter_temp_high_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alignment_routine_failed_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'unreasonable_radar_data) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'microprocessor_temp_high_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vertical_alignment_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'horizontal_alignment_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'factory_alignment_mode) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'battery_low_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'battery_high_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'v_1p25_supply_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte3_bit4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'thermistor_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'v_3p3_dac_supply_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'v_3p3_raw_supply_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'v_5_supply_out_of_range) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'transmitter_id_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte2_bit6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte2_bit5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte2_bit4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte2_bit3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte2_bit2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pcan_missing_msg_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pcan_bus_off) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte1_bit7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte1_bit6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'instruction_set_check_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stack_overflow_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'watchdog_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pll_lock_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte1_bit1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ram_memory_test_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'usc_validation_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte0_bit6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte0_bit5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte0_bit4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active_flt_latched_byte0_bit3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'keep_alive_checksum_fault) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'program_calibration_flash_checksum) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'application_flash_checksum_fault) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActiveFaultLatched1>) istream)
  "Deserializes a message object of type '<ActiveFaultLatched1>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte7_bit7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte7_bit6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte7_bit5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte7_bit4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'arm_to_dsp_chksum_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_to_arm_chksum_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'host_to_arm_chksum_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'arm_to_host_chksum_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'loop_bw_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_overrun_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte6_bit5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tuning_sensitivity_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'saturated_tuning_freq_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'local_osc_power_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'transmitter_power_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte6_bit0) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte5_bit7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte5_bit6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'xcvr_device_spi_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'freq_synthesizer_spi_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'analog_converter_devic_spi_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'side_lobe_blockage) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte5_bit1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mnr_blocked) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ecu_temp_high_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'transmitter_temp_high_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alignment_routine_failed_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'unreasonable_radar_data) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'microprocessor_temp_high_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'vertical_alignment_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'horizontal_alignment_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'factory_alignment_mode) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'battery_low_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'battery_high_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'v_1p25_supply_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte3_bit4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'thermistor_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'v_3p3_dac_supply_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'v_3p3_raw_supply_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'v_5_supply_out_of_range) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'transmitter_id_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte2_bit6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte2_bit5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte2_bit4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte2_bit3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte2_bit2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pcan_missing_msg_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pcan_bus_off) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte1_bit7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte1_bit6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'instruction_set_check_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'stack_overflow_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'watchdog_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pll_lock_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte1_bit1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ram_memory_test_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'usc_validation_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte0_bit6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte0_bit5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte0_bit4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'active_flt_latched_byte0_bit3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'keep_alive_checksum_fault) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'program_calibration_flash_checksum) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'application_flash_checksum_fault) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActiveFaultLatched1>)))
  "Returns string type for a message object of type '<ActiveFaultLatched1>"
  "delphi_mrr_msgs/ActiveFaultLatched1")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActiveFaultLatched1)))
  "Returns string type for a message object of type 'ActiveFaultLatched1"
  "delphi_mrr_msgs/ActiveFaultLatched1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActiveFaultLatched1>)))
  "Returns md5sum for a message object of type '<ActiveFaultLatched1>"
  "0ab006656c4a10fa960d61366fb2b561")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActiveFaultLatched1)))
  "Returns md5sum for a message object of type 'ActiveFaultLatched1"
  "0ab006656c4a10fa960d61366fb2b561")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActiveFaultLatched1>)))
  "Returns full string definition for message of type '<ActiveFaultLatched1>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool active_flt_latched_byte7_bit7~%bool active_flt_latched_byte7_bit6~%bool active_flt_latched_byte7_bit5~%bool active_flt_latched_byte7_bit4~%bool arm_to_dsp_chksum_fault~%bool dsp_to_arm_chksum_fault~%bool host_to_arm_chksum_fault~%bool arm_to_host_chksum_fault~%bool loop_bw_out_of_range~%bool dsp_overrun_fault~%bool active_flt_latched_byte6_bit5~%bool tuning_sensitivity_fault~%bool saturated_tuning_freq_fault~%bool local_osc_power_fault~%bool transmitter_power_fault~%bool active_flt_latched_byte6_bit0~%bool active_flt_latched_byte5_bit7~%bool active_flt_latched_byte5_bit6~%bool xcvr_device_spi_fault~%bool freq_synthesizer_spi_fault~%bool analog_converter_devic_spi_fault~%bool side_lobe_blockage~%bool active_flt_latched_byte5_bit1~%bool mnr_blocked~%bool ecu_temp_high_fault~%bool transmitter_temp_high_fault~%bool alignment_routine_failed_fault~%bool unreasonable_radar_data~%bool microprocessor_temp_high_fault~%bool vertical_alignment_out_of_range~%bool horizontal_alignment_out_of_range~%bool factory_alignment_mode~%bool battery_low_fault~%bool battery_high_fault~%bool v_1p25_supply_out_of_range~%bool active_flt_latched_byte3_bit4~%bool thermistor_out_of_range~%bool v_3p3_dac_supply_out_of_range~%bool v_3p3_raw_supply_out_of_range~%bool v_5_supply_out_of_range~%bool transmitter_id_fault~%bool active_flt_latched_byte2_bit6~%bool active_flt_latched_byte2_bit5~%bool active_flt_latched_byte2_bit4~%bool active_flt_latched_byte2_bit3~%bool active_flt_latched_byte2_bit2~%bool pcan_missing_msg_fault~%bool pcan_bus_off~%bool active_flt_latched_byte1_bit7~%bool active_flt_latched_byte1_bit6~%bool instruction_set_check_fault~%bool stack_overflow_fault~%bool watchdog_fault~%bool pll_lock_fault~%bool active_flt_latched_byte1_bit1~%bool ram_memory_test_fault~%bool usc_validation_fault~%bool active_flt_latched_byte0_bit6~%bool active_flt_latched_byte0_bit5~%bool active_flt_latched_byte0_bit4~%bool active_flt_latched_byte0_bit3~%bool keep_alive_checksum_fault~%bool program_calibration_flash_checksum~%bool application_flash_checksum_fault~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActiveFaultLatched1)))
  "Returns full string definition for message of type 'ActiveFaultLatched1"
  (cl:format cl:nil "std_msgs/Header header~%~%bool active_flt_latched_byte7_bit7~%bool active_flt_latched_byte7_bit6~%bool active_flt_latched_byte7_bit5~%bool active_flt_latched_byte7_bit4~%bool arm_to_dsp_chksum_fault~%bool dsp_to_arm_chksum_fault~%bool host_to_arm_chksum_fault~%bool arm_to_host_chksum_fault~%bool loop_bw_out_of_range~%bool dsp_overrun_fault~%bool active_flt_latched_byte6_bit5~%bool tuning_sensitivity_fault~%bool saturated_tuning_freq_fault~%bool local_osc_power_fault~%bool transmitter_power_fault~%bool active_flt_latched_byte6_bit0~%bool active_flt_latched_byte5_bit7~%bool active_flt_latched_byte5_bit6~%bool xcvr_device_spi_fault~%bool freq_synthesizer_spi_fault~%bool analog_converter_devic_spi_fault~%bool side_lobe_blockage~%bool active_flt_latched_byte5_bit1~%bool mnr_blocked~%bool ecu_temp_high_fault~%bool transmitter_temp_high_fault~%bool alignment_routine_failed_fault~%bool unreasonable_radar_data~%bool microprocessor_temp_high_fault~%bool vertical_alignment_out_of_range~%bool horizontal_alignment_out_of_range~%bool factory_alignment_mode~%bool battery_low_fault~%bool battery_high_fault~%bool v_1p25_supply_out_of_range~%bool active_flt_latched_byte3_bit4~%bool thermistor_out_of_range~%bool v_3p3_dac_supply_out_of_range~%bool v_3p3_raw_supply_out_of_range~%bool v_5_supply_out_of_range~%bool transmitter_id_fault~%bool active_flt_latched_byte2_bit6~%bool active_flt_latched_byte2_bit5~%bool active_flt_latched_byte2_bit4~%bool active_flt_latched_byte2_bit3~%bool active_flt_latched_byte2_bit2~%bool pcan_missing_msg_fault~%bool pcan_bus_off~%bool active_flt_latched_byte1_bit7~%bool active_flt_latched_byte1_bit6~%bool instruction_set_check_fault~%bool stack_overflow_fault~%bool watchdog_fault~%bool pll_lock_fault~%bool active_flt_latched_byte1_bit1~%bool ram_memory_test_fault~%bool usc_validation_fault~%bool active_flt_latched_byte0_bit6~%bool active_flt_latched_byte0_bit5~%bool active_flt_latched_byte0_bit4~%bool active_flt_latched_byte0_bit3~%bool keep_alive_checksum_fault~%bool program_calibration_flash_checksum~%bool application_flash_checksum_fault~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActiveFaultLatched1>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActiveFaultLatched1>))
  "Converts a ROS message object to a list"
  (cl:list 'ActiveFaultLatched1
    (cl:cons ':header (header msg))
    (cl:cons ':active_flt_latched_byte7_bit7 (active_flt_latched_byte7_bit7 msg))
    (cl:cons ':active_flt_latched_byte7_bit6 (active_flt_latched_byte7_bit6 msg))
    (cl:cons ':active_flt_latched_byte7_bit5 (active_flt_latched_byte7_bit5 msg))
    (cl:cons ':active_flt_latched_byte7_bit4 (active_flt_latched_byte7_bit4 msg))
    (cl:cons ':arm_to_dsp_chksum_fault (arm_to_dsp_chksum_fault msg))
    (cl:cons ':dsp_to_arm_chksum_fault (dsp_to_arm_chksum_fault msg))
    (cl:cons ':host_to_arm_chksum_fault (host_to_arm_chksum_fault msg))
    (cl:cons ':arm_to_host_chksum_fault (arm_to_host_chksum_fault msg))
    (cl:cons ':loop_bw_out_of_range (loop_bw_out_of_range msg))
    (cl:cons ':dsp_overrun_fault (dsp_overrun_fault msg))
    (cl:cons ':active_flt_latched_byte6_bit5 (active_flt_latched_byte6_bit5 msg))
    (cl:cons ':tuning_sensitivity_fault (tuning_sensitivity_fault msg))
    (cl:cons ':saturated_tuning_freq_fault (saturated_tuning_freq_fault msg))
    (cl:cons ':local_osc_power_fault (local_osc_power_fault msg))
    (cl:cons ':transmitter_power_fault (transmitter_power_fault msg))
    (cl:cons ':active_flt_latched_byte6_bit0 (active_flt_latched_byte6_bit0 msg))
    (cl:cons ':active_flt_latched_byte5_bit7 (active_flt_latched_byte5_bit7 msg))
    (cl:cons ':active_flt_latched_byte5_bit6 (active_flt_latched_byte5_bit6 msg))
    (cl:cons ':xcvr_device_spi_fault (xcvr_device_spi_fault msg))
    (cl:cons ':freq_synthesizer_spi_fault (freq_synthesizer_spi_fault msg))
    (cl:cons ':analog_converter_devic_spi_fault (analog_converter_devic_spi_fault msg))
    (cl:cons ':side_lobe_blockage (side_lobe_blockage msg))
    (cl:cons ':active_flt_latched_byte5_bit1 (active_flt_latched_byte5_bit1 msg))
    (cl:cons ':mnr_blocked (mnr_blocked msg))
    (cl:cons ':ecu_temp_high_fault (ecu_temp_high_fault msg))
    (cl:cons ':transmitter_temp_high_fault (transmitter_temp_high_fault msg))
    (cl:cons ':alignment_routine_failed_fault (alignment_routine_failed_fault msg))
    (cl:cons ':unreasonable_radar_data (unreasonable_radar_data msg))
    (cl:cons ':microprocessor_temp_high_fault (microprocessor_temp_high_fault msg))
    (cl:cons ':vertical_alignment_out_of_range (vertical_alignment_out_of_range msg))
    (cl:cons ':horizontal_alignment_out_of_range (horizontal_alignment_out_of_range msg))
    (cl:cons ':factory_alignment_mode (factory_alignment_mode msg))
    (cl:cons ':battery_low_fault (battery_low_fault msg))
    (cl:cons ':battery_high_fault (battery_high_fault msg))
    (cl:cons ':v_1p25_supply_out_of_range (v_1p25_supply_out_of_range msg))
    (cl:cons ':active_flt_latched_byte3_bit4 (active_flt_latched_byte3_bit4 msg))
    (cl:cons ':thermistor_out_of_range (thermistor_out_of_range msg))
    (cl:cons ':v_3p3_dac_supply_out_of_range (v_3p3_dac_supply_out_of_range msg))
    (cl:cons ':v_3p3_raw_supply_out_of_range (v_3p3_raw_supply_out_of_range msg))
    (cl:cons ':v_5_supply_out_of_range (v_5_supply_out_of_range msg))
    (cl:cons ':transmitter_id_fault (transmitter_id_fault msg))
    (cl:cons ':active_flt_latched_byte2_bit6 (active_flt_latched_byte2_bit6 msg))
    (cl:cons ':active_flt_latched_byte2_bit5 (active_flt_latched_byte2_bit5 msg))
    (cl:cons ':active_flt_latched_byte2_bit4 (active_flt_latched_byte2_bit4 msg))
    (cl:cons ':active_flt_latched_byte2_bit3 (active_flt_latched_byte2_bit3 msg))
    (cl:cons ':active_flt_latched_byte2_bit2 (active_flt_latched_byte2_bit2 msg))
    (cl:cons ':pcan_missing_msg_fault (pcan_missing_msg_fault msg))
    (cl:cons ':pcan_bus_off (pcan_bus_off msg))
    (cl:cons ':active_flt_latched_byte1_bit7 (active_flt_latched_byte1_bit7 msg))
    (cl:cons ':active_flt_latched_byte1_bit6 (active_flt_latched_byte1_bit6 msg))
    (cl:cons ':instruction_set_check_fault (instruction_set_check_fault msg))
    (cl:cons ':stack_overflow_fault (stack_overflow_fault msg))
    (cl:cons ':watchdog_fault (watchdog_fault msg))
    (cl:cons ':pll_lock_fault (pll_lock_fault msg))
    (cl:cons ':active_flt_latched_byte1_bit1 (active_flt_latched_byte1_bit1 msg))
    (cl:cons ':ram_memory_test_fault (ram_memory_test_fault msg))
    (cl:cons ':usc_validation_fault (usc_validation_fault msg))
    (cl:cons ':active_flt_latched_byte0_bit6 (active_flt_latched_byte0_bit6 msg))
    (cl:cons ':active_flt_latched_byte0_bit5 (active_flt_latched_byte0_bit5 msg))
    (cl:cons ':active_flt_latched_byte0_bit4 (active_flt_latched_byte0_bit4 msg))
    (cl:cons ':active_flt_latched_byte0_bit3 (active_flt_latched_byte0_bit3 msg))
    (cl:cons ':keep_alive_checksum_fault (keep_alive_checksum_fault msg))
    (cl:cons ':program_calibration_flash_checksum (program_calibration_flash_checksum msg))
    (cl:cons ':application_flash_checksum_fault (application_flash_checksum_fault msg))
))
