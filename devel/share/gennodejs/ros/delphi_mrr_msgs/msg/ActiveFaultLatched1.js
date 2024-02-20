// Auto-generated. Do not edit!

// (in-package delphi_mrr_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ActiveFaultLatched1 {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.active_flt_latched_byte7_bit7 = null;
      this.active_flt_latched_byte7_bit6 = null;
      this.active_flt_latched_byte7_bit5 = null;
      this.active_flt_latched_byte7_bit4 = null;
      this.arm_to_dsp_chksum_fault = null;
      this.dsp_to_arm_chksum_fault = null;
      this.host_to_arm_chksum_fault = null;
      this.arm_to_host_chksum_fault = null;
      this.loop_bw_out_of_range = null;
      this.dsp_overrun_fault = null;
      this.active_flt_latched_byte6_bit5 = null;
      this.tuning_sensitivity_fault = null;
      this.saturated_tuning_freq_fault = null;
      this.local_osc_power_fault = null;
      this.transmitter_power_fault = null;
      this.active_flt_latched_byte6_bit0 = null;
      this.active_flt_latched_byte5_bit7 = null;
      this.active_flt_latched_byte5_bit6 = null;
      this.xcvr_device_spi_fault = null;
      this.freq_synthesizer_spi_fault = null;
      this.analog_converter_devic_spi_fault = null;
      this.side_lobe_blockage = null;
      this.active_flt_latched_byte5_bit1 = null;
      this.mnr_blocked = null;
      this.ecu_temp_high_fault = null;
      this.transmitter_temp_high_fault = null;
      this.alignment_routine_failed_fault = null;
      this.unreasonable_radar_data = null;
      this.microprocessor_temp_high_fault = null;
      this.vertical_alignment_out_of_range = null;
      this.horizontal_alignment_out_of_range = null;
      this.factory_alignment_mode = null;
      this.battery_low_fault = null;
      this.battery_high_fault = null;
      this.v_1p25_supply_out_of_range = null;
      this.active_flt_latched_byte3_bit4 = null;
      this.thermistor_out_of_range = null;
      this.v_3p3_dac_supply_out_of_range = null;
      this.v_3p3_raw_supply_out_of_range = null;
      this.v_5_supply_out_of_range = null;
      this.transmitter_id_fault = null;
      this.active_flt_latched_byte2_bit6 = null;
      this.active_flt_latched_byte2_bit5 = null;
      this.active_flt_latched_byte2_bit4 = null;
      this.active_flt_latched_byte2_bit3 = null;
      this.active_flt_latched_byte2_bit2 = null;
      this.pcan_missing_msg_fault = null;
      this.pcan_bus_off = null;
      this.active_flt_latched_byte1_bit7 = null;
      this.active_flt_latched_byte1_bit6 = null;
      this.instruction_set_check_fault = null;
      this.stack_overflow_fault = null;
      this.watchdog_fault = null;
      this.pll_lock_fault = null;
      this.active_flt_latched_byte1_bit1 = null;
      this.ram_memory_test_fault = null;
      this.usc_validation_fault = null;
      this.active_flt_latched_byte0_bit6 = null;
      this.active_flt_latched_byte0_bit5 = null;
      this.active_flt_latched_byte0_bit4 = null;
      this.active_flt_latched_byte0_bit3 = null;
      this.keep_alive_checksum_fault = null;
      this.program_calibration_flash_checksum = null;
      this.application_flash_checksum_fault = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte7_bit7')) {
        this.active_flt_latched_byte7_bit7 = initObj.active_flt_latched_byte7_bit7
      }
      else {
        this.active_flt_latched_byte7_bit7 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte7_bit6')) {
        this.active_flt_latched_byte7_bit6 = initObj.active_flt_latched_byte7_bit6
      }
      else {
        this.active_flt_latched_byte7_bit6 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte7_bit5')) {
        this.active_flt_latched_byte7_bit5 = initObj.active_flt_latched_byte7_bit5
      }
      else {
        this.active_flt_latched_byte7_bit5 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte7_bit4')) {
        this.active_flt_latched_byte7_bit4 = initObj.active_flt_latched_byte7_bit4
      }
      else {
        this.active_flt_latched_byte7_bit4 = false;
      }
      if (initObj.hasOwnProperty('arm_to_dsp_chksum_fault')) {
        this.arm_to_dsp_chksum_fault = initObj.arm_to_dsp_chksum_fault
      }
      else {
        this.arm_to_dsp_chksum_fault = false;
      }
      if (initObj.hasOwnProperty('dsp_to_arm_chksum_fault')) {
        this.dsp_to_arm_chksum_fault = initObj.dsp_to_arm_chksum_fault
      }
      else {
        this.dsp_to_arm_chksum_fault = false;
      }
      if (initObj.hasOwnProperty('host_to_arm_chksum_fault')) {
        this.host_to_arm_chksum_fault = initObj.host_to_arm_chksum_fault
      }
      else {
        this.host_to_arm_chksum_fault = false;
      }
      if (initObj.hasOwnProperty('arm_to_host_chksum_fault')) {
        this.arm_to_host_chksum_fault = initObj.arm_to_host_chksum_fault
      }
      else {
        this.arm_to_host_chksum_fault = false;
      }
      if (initObj.hasOwnProperty('loop_bw_out_of_range')) {
        this.loop_bw_out_of_range = initObj.loop_bw_out_of_range
      }
      else {
        this.loop_bw_out_of_range = false;
      }
      if (initObj.hasOwnProperty('dsp_overrun_fault')) {
        this.dsp_overrun_fault = initObj.dsp_overrun_fault
      }
      else {
        this.dsp_overrun_fault = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte6_bit5')) {
        this.active_flt_latched_byte6_bit5 = initObj.active_flt_latched_byte6_bit5
      }
      else {
        this.active_flt_latched_byte6_bit5 = false;
      }
      if (initObj.hasOwnProperty('tuning_sensitivity_fault')) {
        this.tuning_sensitivity_fault = initObj.tuning_sensitivity_fault
      }
      else {
        this.tuning_sensitivity_fault = false;
      }
      if (initObj.hasOwnProperty('saturated_tuning_freq_fault')) {
        this.saturated_tuning_freq_fault = initObj.saturated_tuning_freq_fault
      }
      else {
        this.saturated_tuning_freq_fault = false;
      }
      if (initObj.hasOwnProperty('local_osc_power_fault')) {
        this.local_osc_power_fault = initObj.local_osc_power_fault
      }
      else {
        this.local_osc_power_fault = false;
      }
      if (initObj.hasOwnProperty('transmitter_power_fault')) {
        this.transmitter_power_fault = initObj.transmitter_power_fault
      }
      else {
        this.transmitter_power_fault = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte6_bit0')) {
        this.active_flt_latched_byte6_bit0 = initObj.active_flt_latched_byte6_bit0
      }
      else {
        this.active_flt_latched_byte6_bit0 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte5_bit7')) {
        this.active_flt_latched_byte5_bit7 = initObj.active_flt_latched_byte5_bit7
      }
      else {
        this.active_flt_latched_byte5_bit7 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte5_bit6')) {
        this.active_flt_latched_byte5_bit6 = initObj.active_flt_latched_byte5_bit6
      }
      else {
        this.active_flt_latched_byte5_bit6 = false;
      }
      if (initObj.hasOwnProperty('xcvr_device_spi_fault')) {
        this.xcvr_device_spi_fault = initObj.xcvr_device_spi_fault
      }
      else {
        this.xcvr_device_spi_fault = false;
      }
      if (initObj.hasOwnProperty('freq_synthesizer_spi_fault')) {
        this.freq_synthesizer_spi_fault = initObj.freq_synthesizer_spi_fault
      }
      else {
        this.freq_synthesizer_spi_fault = false;
      }
      if (initObj.hasOwnProperty('analog_converter_devic_spi_fault')) {
        this.analog_converter_devic_spi_fault = initObj.analog_converter_devic_spi_fault
      }
      else {
        this.analog_converter_devic_spi_fault = false;
      }
      if (initObj.hasOwnProperty('side_lobe_blockage')) {
        this.side_lobe_blockage = initObj.side_lobe_blockage
      }
      else {
        this.side_lobe_blockage = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte5_bit1')) {
        this.active_flt_latched_byte5_bit1 = initObj.active_flt_latched_byte5_bit1
      }
      else {
        this.active_flt_latched_byte5_bit1 = false;
      }
      if (initObj.hasOwnProperty('mnr_blocked')) {
        this.mnr_blocked = initObj.mnr_blocked
      }
      else {
        this.mnr_blocked = false;
      }
      if (initObj.hasOwnProperty('ecu_temp_high_fault')) {
        this.ecu_temp_high_fault = initObj.ecu_temp_high_fault
      }
      else {
        this.ecu_temp_high_fault = false;
      }
      if (initObj.hasOwnProperty('transmitter_temp_high_fault')) {
        this.transmitter_temp_high_fault = initObj.transmitter_temp_high_fault
      }
      else {
        this.transmitter_temp_high_fault = false;
      }
      if (initObj.hasOwnProperty('alignment_routine_failed_fault')) {
        this.alignment_routine_failed_fault = initObj.alignment_routine_failed_fault
      }
      else {
        this.alignment_routine_failed_fault = false;
      }
      if (initObj.hasOwnProperty('unreasonable_radar_data')) {
        this.unreasonable_radar_data = initObj.unreasonable_radar_data
      }
      else {
        this.unreasonable_radar_data = false;
      }
      if (initObj.hasOwnProperty('microprocessor_temp_high_fault')) {
        this.microprocessor_temp_high_fault = initObj.microprocessor_temp_high_fault
      }
      else {
        this.microprocessor_temp_high_fault = false;
      }
      if (initObj.hasOwnProperty('vertical_alignment_out_of_range')) {
        this.vertical_alignment_out_of_range = initObj.vertical_alignment_out_of_range
      }
      else {
        this.vertical_alignment_out_of_range = false;
      }
      if (initObj.hasOwnProperty('horizontal_alignment_out_of_range')) {
        this.horizontal_alignment_out_of_range = initObj.horizontal_alignment_out_of_range
      }
      else {
        this.horizontal_alignment_out_of_range = false;
      }
      if (initObj.hasOwnProperty('factory_alignment_mode')) {
        this.factory_alignment_mode = initObj.factory_alignment_mode
      }
      else {
        this.factory_alignment_mode = false;
      }
      if (initObj.hasOwnProperty('battery_low_fault')) {
        this.battery_low_fault = initObj.battery_low_fault
      }
      else {
        this.battery_low_fault = false;
      }
      if (initObj.hasOwnProperty('battery_high_fault')) {
        this.battery_high_fault = initObj.battery_high_fault
      }
      else {
        this.battery_high_fault = false;
      }
      if (initObj.hasOwnProperty('v_1p25_supply_out_of_range')) {
        this.v_1p25_supply_out_of_range = initObj.v_1p25_supply_out_of_range
      }
      else {
        this.v_1p25_supply_out_of_range = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte3_bit4')) {
        this.active_flt_latched_byte3_bit4 = initObj.active_flt_latched_byte3_bit4
      }
      else {
        this.active_flt_latched_byte3_bit4 = false;
      }
      if (initObj.hasOwnProperty('thermistor_out_of_range')) {
        this.thermistor_out_of_range = initObj.thermistor_out_of_range
      }
      else {
        this.thermistor_out_of_range = false;
      }
      if (initObj.hasOwnProperty('v_3p3_dac_supply_out_of_range')) {
        this.v_3p3_dac_supply_out_of_range = initObj.v_3p3_dac_supply_out_of_range
      }
      else {
        this.v_3p3_dac_supply_out_of_range = false;
      }
      if (initObj.hasOwnProperty('v_3p3_raw_supply_out_of_range')) {
        this.v_3p3_raw_supply_out_of_range = initObj.v_3p3_raw_supply_out_of_range
      }
      else {
        this.v_3p3_raw_supply_out_of_range = false;
      }
      if (initObj.hasOwnProperty('v_5_supply_out_of_range')) {
        this.v_5_supply_out_of_range = initObj.v_5_supply_out_of_range
      }
      else {
        this.v_5_supply_out_of_range = false;
      }
      if (initObj.hasOwnProperty('transmitter_id_fault')) {
        this.transmitter_id_fault = initObj.transmitter_id_fault
      }
      else {
        this.transmitter_id_fault = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte2_bit6')) {
        this.active_flt_latched_byte2_bit6 = initObj.active_flt_latched_byte2_bit6
      }
      else {
        this.active_flt_latched_byte2_bit6 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte2_bit5')) {
        this.active_flt_latched_byte2_bit5 = initObj.active_flt_latched_byte2_bit5
      }
      else {
        this.active_flt_latched_byte2_bit5 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte2_bit4')) {
        this.active_flt_latched_byte2_bit4 = initObj.active_flt_latched_byte2_bit4
      }
      else {
        this.active_flt_latched_byte2_bit4 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte2_bit3')) {
        this.active_flt_latched_byte2_bit3 = initObj.active_flt_latched_byte2_bit3
      }
      else {
        this.active_flt_latched_byte2_bit3 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte2_bit2')) {
        this.active_flt_latched_byte2_bit2 = initObj.active_flt_latched_byte2_bit2
      }
      else {
        this.active_flt_latched_byte2_bit2 = false;
      }
      if (initObj.hasOwnProperty('pcan_missing_msg_fault')) {
        this.pcan_missing_msg_fault = initObj.pcan_missing_msg_fault
      }
      else {
        this.pcan_missing_msg_fault = false;
      }
      if (initObj.hasOwnProperty('pcan_bus_off')) {
        this.pcan_bus_off = initObj.pcan_bus_off
      }
      else {
        this.pcan_bus_off = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte1_bit7')) {
        this.active_flt_latched_byte1_bit7 = initObj.active_flt_latched_byte1_bit7
      }
      else {
        this.active_flt_latched_byte1_bit7 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte1_bit6')) {
        this.active_flt_latched_byte1_bit6 = initObj.active_flt_latched_byte1_bit6
      }
      else {
        this.active_flt_latched_byte1_bit6 = false;
      }
      if (initObj.hasOwnProperty('instruction_set_check_fault')) {
        this.instruction_set_check_fault = initObj.instruction_set_check_fault
      }
      else {
        this.instruction_set_check_fault = false;
      }
      if (initObj.hasOwnProperty('stack_overflow_fault')) {
        this.stack_overflow_fault = initObj.stack_overflow_fault
      }
      else {
        this.stack_overflow_fault = false;
      }
      if (initObj.hasOwnProperty('watchdog_fault')) {
        this.watchdog_fault = initObj.watchdog_fault
      }
      else {
        this.watchdog_fault = false;
      }
      if (initObj.hasOwnProperty('pll_lock_fault')) {
        this.pll_lock_fault = initObj.pll_lock_fault
      }
      else {
        this.pll_lock_fault = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte1_bit1')) {
        this.active_flt_latched_byte1_bit1 = initObj.active_flt_latched_byte1_bit1
      }
      else {
        this.active_flt_latched_byte1_bit1 = false;
      }
      if (initObj.hasOwnProperty('ram_memory_test_fault')) {
        this.ram_memory_test_fault = initObj.ram_memory_test_fault
      }
      else {
        this.ram_memory_test_fault = false;
      }
      if (initObj.hasOwnProperty('usc_validation_fault')) {
        this.usc_validation_fault = initObj.usc_validation_fault
      }
      else {
        this.usc_validation_fault = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte0_bit6')) {
        this.active_flt_latched_byte0_bit6 = initObj.active_flt_latched_byte0_bit6
      }
      else {
        this.active_flt_latched_byte0_bit6 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte0_bit5')) {
        this.active_flt_latched_byte0_bit5 = initObj.active_flt_latched_byte0_bit5
      }
      else {
        this.active_flt_latched_byte0_bit5 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte0_bit4')) {
        this.active_flt_latched_byte0_bit4 = initObj.active_flt_latched_byte0_bit4
      }
      else {
        this.active_flt_latched_byte0_bit4 = false;
      }
      if (initObj.hasOwnProperty('active_flt_latched_byte0_bit3')) {
        this.active_flt_latched_byte0_bit3 = initObj.active_flt_latched_byte0_bit3
      }
      else {
        this.active_flt_latched_byte0_bit3 = false;
      }
      if (initObj.hasOwnProperty('keep_alive_checksum_fault')) {
        this.keep_alive_checksum_fault = initObj.keep_alive_checksum_fault
      }
      else {
        this.keep_alive_checksum_fault = false;
      }
      if (initObj.hasOwnProperty('program_calibration_flash_checksum')) {
        this.program_calibration_flash_checksum = initObj.program_calibration_flash_checksum
      }
      else {
        this.program_calibration_flash_checksum = false;
      }
      if (initObj.hasOwnProperty('application_flash_checksum_fault')) {
        this.application_flash_checksum_fault = initObj.application_flash_checksum_fault
      }
      else {
        this.application_flash_checksum_fault = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActiveFaultLatched1
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte7_bit7]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte7_bit7, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte7_bit6]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte7_bit6, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte7_bit5]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte7_bit5, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte7_bit4]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte7_bit4, buffer, bufferOffset);
    // Serialize message field [arm_to_dsp_chksum_fault]
    bufferOffset = _serializer.bool(obj.arm_to_dsp_chksum_fault, buffer, bufferOffset);
    // Serialize message field [dsp_to_arm_chksum_fault]
    bufferOffset = _serializer.bool(obj.dsp_to_arm_chksum_fault, buffer, bufferOffset);
    // Serialize message field [host_to_arm_chksum_fault]
    bufferOffset = _serializer.bool(obj.host_to_arm_chksum_fault, buffer, bufferOffset);
    // Serialize message field [arm_to_host_chksum_fault]
    bufferOffset = _serializer.bool(obj.arm_to_host_chksum_fault, buffer, bufferOffset);
    // Serialize message field [loop_bw_out_of_range]
    bufferOffset = _serializer.bool(obj.loop_bw_out_of_range, buffer, bufferOffset);
    // Serialize message field [dsp_overrun_fault]
    bufferOffset = _serializer.bool(obj.dsp_overrun_fault, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte6_bit5]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte6_bit5, buffer, bufferOffset);
    // Serialize message field [tuning_sensitivity_fault]
    bufferOffset = _serializer.bool(obj.tuning_sensitivity_fault, buffer, bufferOffset);
    // Serialize message field [saturated_tuning_freq_fault]
    bufferOffset = _serializer.bool(obj.saturated_tuning_freq_fault, buffer, bufferOffset);
    // Serialize message field [local_osc_power_fault]
    bufferOffset = _serializer.bool(obj.local_osc_power_fault, buffer, bufferOffset);
    // Serialize message field [transmitter_power_fault]
    bufferOffset = _serializer.bool(obj.transmitter_power_fault, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte6_bit0]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte6_bit0, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte5_bit7]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte5_bit7, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte5_bit6]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte5_bit6, buffer, bufferOffset);
    // Serialize message field [xcvr_device_spi_fault]
    bufferOffset = _serializer.bool(obj.xcvr_device_spi_fault, buffer, bufferOffset);
    // Serialize message field [freq_synthesizer_spi_fault]
    bufferOffset = _serializer.bool(obj.freq_synthesizer_spi_fault, buffer, bufferOffset);
    // Serialize message field [analog_converter_devic_spi_fault]
    bufferOffset = _serializer.bool(obj.analog_converter_devic_spi_fault, buffer, bufferOffset);
    // Serialize message field [side_lobe_blockage]
    bufferOffset = _serializer.bool(obj.side_lobe_blockage, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte5_bit1]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte5_bit1, buffer, bufferOffset);
    // Serialize message field [mnr_blocked]
    bufferOffset = _serializer.bool(obj.mnr_blocked, buffer, bufferOffset);
    // Serialize message field [ecu_temp_high_fault]
    bufferOffset = _serializer.bool(obj.ecu_temp_high_fault, buffer, bufferOffset);
    // Serialize message field [transmitter_temp_high_fault]
    bufferOffset = _serializer.bool(obj.transmitter_temp_high_fault, buffer, bufferOffset);
    // Serialize message field [alignment_routine_failed_fault]
    bufferOffset = _serializer.bool(obj.alignment_routine_failed_fault, buffer, bufferOffset);
    // Serialize message field [unreasonable_radar_data]
    bufferOffset = _serializer.bool(obj.unreasonable_radar_data, buffer, bufferOffset);
    // Serialize message field [microprocessor_temp_high_fault]
    bufferOffset = _serializer.bool(obj.microprocessor_temp_high_fault, buffer, bufferOffset);
    // Serialize message field [vertical_alignment_out_of_range]
    bufferOffset = _serializer.bool(obj.vertical_alignment_out_of_range, buffer, bufferOffset);
    // Serialize message field [horizontal_alignment_out_of_range]
    bufferOffset = _serializer.bool(obj.horizontal_alignment_out_of_range, buffer, bufferOffset);
    // Serialize message field [factory_alignment_mode]
    bufferOffset = _serializer.bool(obj.factory_alignment_mode, buffer, bufferOffset);
    // Serialize message field [battery_low_fault]
    bufferOffset = _serializer.bool(obj.battery_low_fault, buffer, bufferOffset);
    // Serialize message field [battery_high_fault]
    bufferOffset = _serializer.bool(obj.battery_high_fault, buffer, bufferOffset);
    // Serialize message field [v_1p25_supply_out_of_range]
    bufferOffset = _serializer.bool(obj.v_1p25_supply_out_of_range, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte3_bit4]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte3_bit4, buffer, bufferOffset);
    // Serialize message field [thermistor_out_of_range]
    bufferOffset = _serializer.bool(obj.thermistor_out_of_range, buffer, bufferOffset);
    // Serialize message field [v_3p3_dac_supply_out_of_range]
    bufferOffset = _serializer.bool(obj.v_3p3_dac_supply_out_of_range, buffer, bufferOffset);
    // Serialize message field [v_3p3_raw_supply_out_of_range]
    bufferOffset = _serializer.bool(obj.v_3p3_raw_supply_out_of_range, buffer, bufferOffset);
    // Serialize message field [v_5_supply_out_of_range]
    bufferOffset = _serializer.bool(obj.v_5_supply_out_of_range, buffer, bufferOffset);
    // Serialize message field [transmitter_id_fault]
    bufferOffset = _serializer.bool(obj.transmitter_id_fault, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte2_bit6]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte2_bit6, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte2_bit5]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte2_bit5, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte2_bit4]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte2_bit4, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte2_bit3]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte2_bit3, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte2_bit2]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte2_bit2, buffer, bufferOffset);
    // Serialize message field [pcan_missing_msg_fault]
    bufferOffset = _serializer.bool(obj.pcan_missing_msg_fault, buffer, bufferOffset);
    // Serialize message field [pcan_bus_off]
    bufferOffset = _serializer.bool(obj.pcan_bus_off, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte1_bit7]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte1_bit7, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte1_bit6]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte1_bit6, buffer, bufferOffset);
    // Serialize message field [instruction_set_check_fault]
    bufferOffset = _serializer.bool(obj.instruction_set_check_fault, buffer, bufferOffset);
    // Serialize message field [stack_overflow_fault]
    bufferOffset = _serializer.bool(obj.stack_overflow_fault, buffer, bufferOffset);
    // Serialize message field [watchdog_fault]
    bufferOffset = _serializer.bool(obj.watchdog_fault, buffer, bufferOffset);
    // Serialize message field [pll_lock_fault]
    bufferOffset = _serializer.bool(obj.pll_lock_fault, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte1_bit1]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte1_bit1, buffer, bufferOffset);
    // Serialize message field [ram_memory_test_fault]
    bufferOffset = _serializer.bool(obj.ram_memory_test_fault, buffer, bufferOffset);
    // Serialize message field [usc_validation_fault]
    bufferOffset = _serializer.bool(obj.usc_validation_fault, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte0_bit6]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte0_bit6, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte0_bit5]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte0_bit5, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte0_bit4]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte0_bit4, buffer, bufferOffset);
    // Serialize message field [active_flt_latched_byte0_bit3]
    bufferOffset = _serializer.bool(obj.active_flt_latched_byte0_bit3, buffer, bufferOffset);
    // Serialize message field [keep_alive_checksum_fault]
    bufferOffset = _serializer.bool(obj.keep_alive_checksum_fault, buffer, bufferOffset);
    // Serialize message field [program_calibration_flash_checksum]
    bufferOffset = _serializer.bool(obj.program_calibration_flash_checksum, buffer, bufferOffset);
    // Serialize message field [application_flash_checksum_fault]
    bufferOffset = _serializer.bool(obj.application_flash_checksum_fault, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActiveFaultLatched1
    let len;
    let data = new ActiveFaultLatched1(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte7_bit7]
    data.active_flt_latched_byte7_bit7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte7_bit6]
    data.active_flt_latched_byte7_bit6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte7_bit5]
    data.active_flt_latched_byte7_bit5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte7_bit4]
    data.active_flt_latched_byte7_bit4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [arm_to_dsp_chksum_fault]
    data.arm_to_dsp_chksum_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_to_arm_chksum_fault]
    data.dsp_to_arm_chksum_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [host_to_arm_chksum_fault]
    data.host_to_arm_chksum_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [arm_to_host_chksum_fault]
    data.arm_to_host_chksum_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [loop_bw_out_of_range]
    data.loop_bw_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dsp_overrun_fault]
    data.dsp_overrun_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte6_bit5]
    data.active_flt_latched_byte6_bit5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tuning_sensitivity_fault]
    data.tuning_sensitivity_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [saturated_tuning_freq_fault]
    data.saturated_tuning_freq_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [local_osc_power_fault]
    data.local_osc_power_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [transmitter_power_fault]
    data.transmitter_power_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte6_bit0]
    data.active_flt_latched_byte6_bit0 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte5_bit7]
    data.active_flt_latched_byte5_bit7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte5_bit6]
    data.active_flt_latched_byte5_bit6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [xcvr_device_spi_fault]
    data.xcvr_device_spi_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [freq_synthesizer_spi_fault]
    data.freq_synthesizer_spi_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [analog_converter_devic_spi_fault]
    data.analog_converter_devic_spi_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [side_lobe_blockage]
    data.side_lobe_blockage = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte5_bit1]
    data.active_flt_latched_byte5_bit1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mnr_blocked]
    data.mnr_blocked = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ecu_temp_high_fault]
    data.ecu_temp_high_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [transmitter_temp_high_fault]
    data.transmitter_temp_high_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alignment_routine_failed_fault]
    data.alignment_routine_failed_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [unreasonable_radar_data]
    data.unreasonable_radar_data = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [microprocessor_temp_high_fault]
    data.microprocessor_temp_high_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [vertical_alignment_out_of_range]
    data.vertical_alignment_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [horizontal_alignment_out_of_range]
    data.horizontal_alignment_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [factory_alignment_mode]
    data.factory_alignment_mode = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [battery_low_fault]
    data.battery_low_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [battery_high_fault]
    data.battery_high_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [v_1p25_supply_out_of_range]
    data.v_1p25_supply_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte3_bit4]
    data.active_flt_latched_byte3_bit4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [thermistor_out_of_range]
    data.thermistor_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [v_3p3_dac_supply_out_of_range]
    data.v_3p3_dac_supply_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [v_3p3_raw_supply_out_of_range]
    data.v_3p3_raw_supply_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [v_5_supply_out_of_range]
    data.v_5_supply_out_of_range = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [transmitter_id_fault]
    data.transmitter_id_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte2_bit6]
    data.active_flt_latched_byte2_bit6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte2_bit5]
    data.active_flt_latched_byte2_bit5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte2_bit4]
    data.active_flt_latched_byte2_bit4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte2_bit3]
    data.active_flt_latched_byte2_bit3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte2_bit2]
    data.active_flt_latched_byte2_bit2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pcan_missing_msg_fault]
    data.pcan_missing_msg_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pcan_bus_off]
    data.pcan_bus_off = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte1_bit7]
    data.active_flt_latched_byte1_bit7 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte1_bit6]
    data.active_flt_latched_byte1_bit6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [instruction_set_check_fault]
    data.instruction_set_check_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stack_overflow_fault]
    data.stack_overflow_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [watchdog_fault]
    data.watchdog_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pll_lock_fault]
    data.pll_lock_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte1_bit1]
    data.active_flt_latched_byte1_bit1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ram_memory_test_fault]
    data.ram_memory_test_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [usc_validation_fault]
    data.usc_validation_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte0_bit6]
    data.active_flt_latched_byte0_bit6 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte0_bit5]
    data.active_flt_latched_byte0_bit5 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte0_bit4]
    data.active_flt_latched_byte0_bit4 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [active_flt_latched_byte0_bit3]
    data.active_flt_latched_byte0_bit3 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [keep_alive_checksum_fault]
    data.keep_alive_checksum_fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [program_calibration_flash_checksum]
    data.program_calibration_flash_checksum = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [application_flash_checksum_fault]
    data.application_flash_checksum_fault = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'delphi_mrr_msgs/ActiveFaultLatched1';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ab006656c4a10fa960d61366fb2b561';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool active_flt_latched_byte7_bit7
    bool active_flt_latched_byte7_bit6
    bool active_flt_latched_byte7_bit5
    bool active_flt_latched_byte7_bit4
    bool arm_to_dsp_chksum_fault
    bool dsp_to_arm_chksum_fault
    bool host_to_arm_chksum_fault
    bool arm_to_host_chksum_fault
    bool loop_bw_out_of_range
    bool dsp_overrun_fault
    bool active_flt_latched_byte6_bit5
    bool tuning_sensitivity_fault
    bool saturated_tuning_freq_fault
    bool local_osc_power_fault
    bool transmitter_power_fault
    bool active_flt_latched_byte6_bit0
    bool active_flt_latched_byte5_bit7
    bool active_flt_latched_byte5_bit6
    bool xcvr_device_spi_fault
    bool freq_synthesizer_spi_fault
    bool analog_converter_devic_spi_fault
    bool side_lobe_blockage
    bool active_flt_latched_byte5_bit1
    bool mnr_blocked
    bool ecu_temp_high_fault
    bool transmitter_temp_high_fault
    bool alignment_routine_failed_fault
    bool unreasonable_radar_data
    bool microprocessor_temp_high_fault
    bool vertical_alignment_out_of_range
    bool horizontal_alignment_out_of_range
    bool factory_alignment_mode
    bool battery_low_fault
    bool battery_high_fault
    bool v_1p25_supply_out_of_range
    bool active_flt_latched_byte3_bit4
    bool thermistor_out_of_range
    bool v_3p3_dac_supply_out_of_range
    bool v_3p3_raw_supply_out_of_range
    bool v_5_supply_out_of_range
    bool transmitter_id_fault
    bool active_flt_latched_byte2_bit6
    bool active_flt_latched_byte2_bit5
    bool active_flt_latched_byte2_bit4
    bool active_flt_latched_byte2_bit3
    bool active_flt_latched_byte2_bit2
    bool pcan_missing_msg_fault
    bool pcan_bus_off
    bool active_flt_latched_byte1_bit7
    bool active_flt_latched_byte1_bit6
    bool instruction_set_check_fault
    bool stack_overflow_fault
    bool watchdog_fault
    bool pll_lock_fault
    bool active_flt_latched_byte1_bit1
    bool ram_memory_test_fault
    bool usc_validation_fault
    bool active_flt_latched_byte0_bit6
    bool active_flt_latched_byte0_bit5
    bool active_flt_latched_byte0_bit4
    bool active_flt_latched_byte0_bit3
    bool keep_alive_checksum_fault
    bool program_calibration_flash_checksum
    bool application_flash_checksum_fault
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActiveFaultLatched1(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.active_flt_latched_byte7_bit7 !== undefined) {
      resolved.active_flt_latched_byte7_bit7 = msg.active_flt_latched_byte7_bit7;
    }
    else {
      resolved.active_flt_latched_byte7_bit7 = false
    }

    if (msg.active_flt_latched_byte7_bit6 !== undefined) {
      resolved.active_flt_latched_byte7_bit6 = msg.active_flt_latched_byte7_bit6;
    }
    else {
      resolved.active_flt_latched_byte7_bit6 = false
    }

    if (msg.active_flt_latched_byte7_bit5 !== undefined) {
      resolved.active_flt_latched_byte7_bit5 = msg.active_flt_latched_byte7_bit5;
    }
    else {
      resolved.active_flt_latched_byte7_bit5 = false
    }

    if (msg.active_flt_latched_byte7_bit4 !== undefined) {
      resolved.active_flt_latched_byte7_bit4 = msg.active_flt_latched_byte7_bit4;
    }
    else {
      resolved.active_flt_latched_byte7_bit4 = false
    }

    if (msg.arm_to_dsp_chksum_fault !== undefined) {
      resolved.arm_to_dsp_chksum_fault = msg.arm_to_dsp_chksum_fault;
    }
    else {
      resolved.arm_to_dsp_chksum_fault = false
    }

    if (msg.dsp_to_arm_chksum_fault !== undefined) {
      resolved.dsp_to_arm_chksum_fault = msg.dsp_to_arm_chksum_fault;
    }
    else {
      resolved.dsp_to_arm_chksum_fault = false
    }

    if (msg.host_to_arm_chksum_fault !== undefined) {
      resolved.host_to_arm_chksum_fault = msg.host_to_arm_chksum_fault;
    }
    else {
      resolved.host_to_arm_chksum_fault = false
    }

    if (msg.arm_to_host_chksum_fault !== undefined) {
      resolved.arm_to_host_chksum_fault = msg.arm_to_host_chksum_fault;
    }
    else {
      resolved.arm_to_host_chksum_fault = false
    }

    if (msg.loop_bw_out_of_range !== undefined) {
      resolved.loop_bw_out_of_range = msg.loop_bw_out_of_range;
    }
    else {
      resolved.loop_bw_out_of_range = false
    }

    if (msg.dsp_overrun_fault !== undefined) {
      resolved.dsp_overrun_fault = msg.dsp_overrun_fault;
    }
    else {
      resolved.dsp_overrun_fault = false
    }

    if (msg.active_flt_latched_byte6_bit5 !== undefined) {
      resolved.active_flt_latched_byte6_bit5 = msg.active_flt_latched_byte6_bit5;
    }
    else {
      resolved.active_flt_latched_byte6_bit5 = false
    }

    if (msg.tuning_sensitivity_fault !== undefined) {
      resolved.tuning_sensitivity_fault = msg.tuning_sensitivity_fault;
    }
    else {
      resolved.tuning_sensitivity_fault = false
    }

    if (msg.saturated_tuning_freq_fault !== undefined) {
      resolved.saturated_tuning_freq_fault = msg.saturated_tuning_freq_fault;
    }
    else {
      resolved.saturated_tuning_freq_fault = false
    }

    if (msg.local_osc_power_fault !== undefined) {
      resolved.local_osc_power_fault = msg.local_osc_power_fault;
    }
    else {
      resolved.local_osc_power_fault = false
    }

    if (msg.transmitter_power_fault !== undefined) {
      resolved.transmitter_power_fault = msg.transmitter_power_fault;
    }
    else {
      resolved.transmitter_power_fault = false
    }

    if (msg.active_flt_latched_byte6_bit0 !== undefined) {
      resolved.active_flt_latched_byte6_bit0 = msg.active_flt_latched_byte6_bit0;
    }
    else {
      resolved.active_flt_latched_byte6_bit0 = false
    }

    if (msg.active_flt_latched_byte5_bit7 !== undefined) {
      resolved.active_flt_latched_byte5_bit7 = msg.active_flt_latched_byte5_bit7;
    }
    else {
      resolved.active_flt_latched_byte5_bit7 = false
    }

    if (msg.active_flt_latched_byte5_bit6 !== undefined) {
      resolved.active_flt_latched_byte5_bit6 = msg.active_flt_latched_byte5_bit6;
    }
    else {
      resolved.active_flt_latched_byte5_bit6 = false
    }

    if (msg.xcvr_device_spi_fault !== undefined) {
      resolved.xcvr_device_spi_fault = msg.xcvr_device_spi_fault;
    }
    else {
      resolved.xcvr_device_spi_fault = false
    }

    if (msg.freq_synthesizer_spi_fault !== undefined) {
      resolved.freq_synthesizer_spi_fault = msg.freq_synthesizer_spi_fault;
    }
    else {
      resolved.freq_synthesizer_spi_fault = false
    }

    if (msg.analog_converter_devic_spi_fault !== undefined) {
      resolved.analog_converter_devic_spi_fault = msg.analog_converter_devic_spi_fault;
    }
    else {
      resolved.analog_converter_devic_spi_fault = false
    }

    if (msg.side_lobe_blockage !== undefined) {
      resolved.side_lobe_blockage = msg.side_lobe_blockage;
    }
    else {
      resolved.side_lobe_blockage = false
    }

    if (msg.active_flt_latched_byte5_bit1 !== undefined) {
      resolved.active_flt_latched_byte5_bit1 = msg.active_flt_latched_byte5_bit1;
    }
    else {
      resolved.active_flt_latched_byte5_bit1 = false
    }

    if (msg.mnr_blocked !== undefined) {
      resolved.mnr_blocked = msg.mnr_blocked;
    }
    else {
      resolved.mnr_blocked = false
    }

    if (msg.ecu_temp_high_fault !== undefined) {
      resolved.ecu_temp_high_fault = msg.ecu_temp_high_fault;
    }
    else {
      resolved.ecu_temp_high_fault = false
    }

    if (msg.transmitter_temp_high_fault !== undefined) {
      resolved.transmitter_temp_high_fault = msg.transmitter_temp_high_fault;
    }
    else {
      resolved.transmitter_temp_high_fault = false
    }

    if (msg.alignment_routine_failed_fault !== undefined) {
      resolved.alignment_routine_failed_fault = msg.alignment_routine_failed_fault;
    }
    else {
      resolved.alignment_routine_failed_fault = false
    }

    if (msg.unreasonable_radar_data !== undefined) {
      resolved.unreasonable_radar_data = msg.unreasonable_radar_data;
    }
    else {
      resolved.unreasonable_radar_data = false
    }

    if (msg.microprocessor_temp_high_fault !== undefined) {
      resolved.microprocessor_temp_high_fault = msg.microprocessor_temp_high_fault;
    }
    else {
      resolved.microprocessor_temp_high_fault = false
    }

    if (msg.vertical_alignment_out_of_range !== undefined) {
      resolved.vertical_alignment_out_of_range = msg.vertical_alignment_out_of_range;
    }
    else {
      resolved.vertical_alignment_out_of_range = false
    }

    if (msg.horizontal_alignment_out_of_range !== undefined) {
      resolved.horizontal_alignment_out_of_range = msg.horizontal_alignment_out_of_range;
    }
    else {
      resolved.horizontal_alignment_out_of_range = false
    }

    if (msg.factory_alignment_mode !== undefined) {
      resolved.factory_alignment_mode = msg.factory_alignment_mode;
    }
    else {
      resolved.factory_alignment_mode = false
    }

    if (msg.battery_low_fault !== undefined) {
      resolved.battery_low_fault = msg.battery_low_fault;
    }
    else {
      resolved.battery_low_fault = false
    }

    if (msg.battery_high_fault !== undefined) {
      resolved.battery_high_fault = msg.battery_high_fault;
    }
    else {
      resolved.battery_high_fault = false
    }

    if (msg.v_1p25_supply_out_of_range !== undefined) {
      resolved.v_1p25_supply_out_of_range = msg.v_1p25_supply_out_of_range;
    }
    else {
      resolved.v_1p25_supply_out_of_range = false
    }

    if (msg.active_flt_latched_byte3_bit4 !== undefined) {
      resolved.active_flt_latched_byte3_bit4 = msg.active_flt_latched_byte3_bit4;
    }
    else {
      resolved.active_flt_latched_byte3_bit4 = false
    }

    if (msg.thermistor_out_of_range !== undefined) {
      resolved.thermistor_out_of_range = msg.thermistor_out_of_range;
    }
    else {
      resolved.thermistor_out_of_range = false
    }

    if (msg.v_3p3_dac_supply_out_of_range !== undefined) {
      resolved.v_3p3_dac_supply_out_of_range = msg.v_3p3_dac_supply_out_of_range;
    }
    else {
      resolved.v_3p3_dac_supply_out_of_range = false
    }

    if (msg.v_3p3_raw_supply_out_of_range !== undefined) {
      resolved.v_3p3_raw_supply_out_of_range = msg.v_3p3_raw_supply_out_of_range;
    }
    else {
      resolved.v_3p3_raw_supply_out_of_range = false
    }

    if (msg.v_5_supply_out_of_range !== undefined) {
      resolved.v_5_supply_out_of_range = msg.v_5_supply_out_of_range;
    }
    else {
      resolved.v_5_supply_out_of_range = false
    }

    if (msg.transmitter_id_fault !== undefined) {
      resolved.transmitter_id_fault = msg.transmitter_id_fault;
    }
    else {
      resolved.transmitter_id_fault = false
    }

    if (msg.active_flt_latched_byte2_bit6 !== undefined) {
      resolved.active_flt_latched_byte2_bit6 = msg.active_flt_latched_byte2_bit6;
    }
    else {
      resolved.active_flt_latched_byte2_bit6 = false
    }

    if (msg.active_flt_latched_byte2_bit5 !== undefined) {
      resolved.active_flt_latched_byte2_bit5 = msg.active_flt_latched_byte2_bit5;
    }
    else {
      resolved.active_flt_latched_byte2_bit5 = false
    }

    if (msg.active_flt_latched_byte2_bit4 !== undefined) {
      resolved.active_flt_latched_byte2_bit4 = msg.active_flt_latched_byte2_bit4;
    }
    else {
      resolved.active_flt_latched_byte2_bit4 = false
    }

    if (msg.active_flt_latched_byte2_bit3 !== undefined) {
      resolved.active_flt_latched_byte2_bit3 = msg.active_flt_latched_byte2_bit3;
    }
    else {
      resolved.active_flt_latched_byte2_bit3 = false
    }

    if (msg.active_flt_latched_byte2_bit2 !== undefined) {
      resolved.active_flt_latched_byte2_bit2 = msg.active_flt_latched_byte2_bit2;
    }
    else {
      resolved.active_flt_latched_byte2_bit2 = false
    }

    if (msg.pcan_missing_msg_fault !== undefined) {
      resolved.pcan_missing_msg_fault = msg.pcan_missing_msg_fault;
    }
    else {
      resolved.pcan_missing_msg_fault = false
    }

    if (msg.pcan_bus_off !== undefined) {
      resolved.pcan_bus_off = msg.pcan_bus_off;
    }
    else {
      resolved.pcan_bus_off = false
    }

    if (msg.active_flt_latched_byte1_bit7 !== undefined) {
      resolved.active_flt_latched_byte1_bit7 = msg.active_flt_latched_byte1_bit7;
    }
    else {
      resolved.active_flt_latched_byte1_bit7 = false
    }

    if (msg.active_flt_latched_byte1_bit6 !== undefined) {
      resolved.active_flt_latched_byte1_bit6 = msg.active_flt_latched_byte1_bit6;
    }
    else {
      resolved.active_flt_latched_byte1_bit6 = false
    }

    if (msg.instruction_set_check_fault !== undefined) {
      resolved.instruction_set_check_fault = msg.instruction_set_check_fault;
    }
    else {
      resolved.instruction_set_check_fault = false
    }

    if (msg.stack_overflow_fault !== undefined) {
      resolved.stack_overflow_fault = msg.stack_overflow_fault;
    }
    else {
      resolved.stack_overflow_fault = false
    }

    if (msg.watchdog_fault !== undefined) {
      resolved.watchdog_fault = msg.watchdog_fault;
    }
    else {
      resolved.watchdog_fault = false
    }

    if (msg.pll_lock_fault !== undefined) {
      resolved.pll_lock_fault = msg.pll_lock_fault;
    }
    else {
      resolved.pll_lock_fault = false
    }

    if (msg.active_flt_latched_byte1_bit1 !== undefined) {
      resolved.active_flt_latched_byte1_bit1 = msg.active_flt_latched_byte1_bit1;
    }
    else {
      resolved.active_flt_latched_byte1_bit1 = false
    }

    if (msg.ram_memory_test_fault !== undefined) {
      resolved.ram_memory_test_fault = msg.ram_memory_test_fault;
    }
    else {
      resolved.ram_memory_test_fault = false
    }

    if (msg.usc_validation_fault !== undefined) {
      resolved.usc_validation_fault = msg.usc_validation_fault;
    }
    else {
      resolved.usc_validation_fault = false
    }

    if (msg.active_flt_latched_byte0_bit6 !== undefined) {
      resolved.active_flt_latched_byte0_bit6 = msg.active_flt_latched_byte0_bit6;
    }
    else {
      resolved.active_flt_latched_byte0_bit6 = false
    }

    if (msg.active_flt_latched_byte0_bit5 !== undefined) {
      resolved.active_flt_latched_byte0_bit5 = msg.active_flt_latched_byte0_bit5;
    }
    else {
      resolved.active_flt_latched_byte0_bit5 = false
    }

    if (msg.active_flt_latched_byte0_bit4 !== undefined) {
      resolved.active_flt_latched_byte0_bit4 = msg.active_flt_latched_byte0_bit4;
    }
    else {
      resolved.active_flt_latched_byte0_bit4 = false
    }

    if (msg.active_flt_latched_byte0_bit3 !== undefined) {
      resolved.active_flt_latched_byte0_bit3 = msg.active_flt_latched_byte0_bit3;
    }
    else {
      resolved.active_flt_latched_byte0_bit3 = false
    }

    if (msg.keep_alive_checksum_fault !== undefined) {
      resolved.keep_alive_checksum_fault = msg.keep_alive_checksum_fault;
    }
    else {
      resolved.keep_alive_checksum_fault = false
    }

    if (msg.program_calibration_flash_checksum !== undefined) {
      resolved.program_calibration_flash_checksum = msg.program_calibration_flash_checksum;
    }
    else {
      resolved.program_calibration_flash_checksum = false
    }

    if (msg.application_flash_checksum_fault !== undefined) {
      resolved.application_flash_checksum_fault = msg.application_flash_checksum_fault;
    }
    else {
      resolved.application_flash_checksum_fault = false
    }

    return resolved;
    }
};

module.exports = ActiveFaultLatched1;
