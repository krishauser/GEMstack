; Auto-generated. Do not edit!


(cl:in-package delphi_srr_msgs-msg)


;//! \htmlinclude SrrDebug3.msg.html

(cl:defclass <SrrDebug3> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (timer_create_error
    :reader timer_create_error
    :initarg :timer_create_error
    :type cl:boolean
    :initform cl:nil)
   (thread_create_error
    :reader thread_create_error
    :initarg :thread_create_error
    :type cl:boolean
    :initform cl:nil)
   (arm_calibration_error
    :reader arm_calibration_error
    :initarg :arm_calibration_error
    :type cl:boolean
    :initform cl:nil)
   (spi_fee_error
    :reader spi_fee_error
    :initarg :spi_fee_error
    :type cl:boolean
    :initform cl:nil)
   (spi_comm_error
    :reader spi_comm_error
    :initarg :spi_comm_error
    :type cl:boolean
    :initform cl:nil)
   (socket_write_error
    :reader socket_write_error
    :initarg :socket_write_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_cal_obsolete_62_error
    :reader dsp_cal_obsolete_62_error
    :initarg :dsp_cal_obsolete_62_error
    :type cl:boolean
    :initform cl:nil)
   (socket_read_error
    :reader socket_read_error
    :initarg :socket_read_error
    :type cl:boolean
    :initform cl:nil)
   (socket_init_error
    :reader socket_init_error
    :initarg :socket_init_error
    :type cl:boolean
    :initform cl:nil)
   (signal_wait_error
    :reader signal_wait_error
    :initarg :signal_wait_error
    :type cl:boolean
    :initform cl:nil)
   (signal_send_error
    :reader signal_send_error
    :initarg :signal_send_error
    :type cl:boolean
    :initform cl:nil)
   (signal_create_error
    :reader signal_create_error
    :initarg :signal_create_error
    :type cl:boolean
    :initform cl:nil)
   (shared_mem_write_error
    :reader shared_mem_write_error
    :initarg :shared_mem_write_error
    :type cl:boolean
    :initform cl:nil)
   (shared_mem_read_error
    :reader shared_mem_read_error
    :initarg :shared_mem_read_error
    :type cl:boolean
    :initform cl:nil)
   (shared_mem_config_error
    :reader shared_mem_config_error
    :initarg :shared_mem_config_error
    :type cl:boolean
    :initform cl:nil)
   (share_mem_init_error
    :reader share_mem_init_error
    :initarg :share_mem_init_error
    :type cl:boolean
    :initform cl:nil)
   (ram_test_error
    :reader ram_test_error
    :initarg :ram_test_error
    :type cl:boolean
    :initform cl:nil)
   (num_errors
    :reader num_errors
    :initarg :num_errors
    :type cl:boolean
    :initform cl:nil)
   (mmap_memory_error
    :reader mmap_memory_error
    :initarg :mmap_memory_error
    :type cl:boolean
    :initform cl:nil)
   (isr_attach_error
    :reader isr_attach_error
    :initarg :isr_attach_error
    :type cl:boolean
    :initform cl:nil)
   (ipc_drv_write_error
    :reader ipc_drv_write_error
    :initarg :ipc_drv_write_error
    :type cl:boolean
    :initform cl:nil)
   (ipc_drv_trigger_error
    :reader ipc_drv_trigger_error
    :initarg :ipc_drv_trigger_error
    :type cl:boolean
    :initform cl:nil)
   (ipc_drv_sync_error
    :reader ipc_drv_sync_error
    :initarg :ipc_drv_sync_error
    :type cl:boolean
    :initform cl:nil)
   (ipc_drv_read_error
    :reader ipc_drv_read_error
    :initarg :ipc_drv_read_error
    :type cl:boolean
    :initform cl:nil)
   (ipc_drv_init_error
    :reader ipc_drv_init_error
    :initarg :ipc_drv_init_error
    :type cl:boolean
    :initform cl:nil)
   (interrupt_enable_error
    :reader interrupt_enable_error
    :initarg :interrupt_enable_error
    :type cl:boolean
    :initform cl:nil)
   (hil_format_error
    :reader hil_format_error
    :initarg :hil_format_error
    :type cl:boolean
    :initform cl:nil)
   (flash_filesystem_error
    :reader flash_filesystem_error
    :initarg :flash_filesystem_error
    :type cl:boolean
    :initform cl:nil)
   (error_none
    :reader error_none
    :initarg :error_none
    :type cl:boolean
    :initform cl:nil)
   (dsp_load_read_error
    :reader dsp_load_read_error
    :initarg :dsp_load_read_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_load_open_error
    :reader dsp_load_open_error
    :initarg :dsp_load_open_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_load_address_error
    :reader dsp_load_address_error
    :initarg :dsp_load_address_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_isp_write_error
    :reader dsp_isp_write_error
    :initarg :dsp_isp_write_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_ipc_read_error
    :reader dsp_ipc_read_error
    :initarg :dsp_ipc_read_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_ipc_init
    :reader dsp_ipc_init
    :initarg :dsp_ipc_init
    :type cl:boolean
    :initform cl:nil)
   (dsp_init_error
    :reader dsp_init_error
    :initarg :dsp_init_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_drv_start_error
    :reader dsp_drv_start_error
    :initarg :dsp_drv_start_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_drv_load_error
    :reader dsp_drv_load_error
    :initarg :dsp_drv_load_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_drv_init_error
    :reader dsp_drv_init_error
    :initarg :dsp_drv_init_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_drv_init2_error
    :reader dsp_drv_init2_error
    :initarg :dsp_drv_init2_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_drv_init1_error
    :reader dsp_drv_init1_error
    :initarg :dsp_drv_init1_error
    :type cl:boolean
    :initform cl:nil)
   (dsp_calibration_error
    :reader dsp_calibration_error
    :initarg :dsp_calibration_error
    :type cl:boolean
    :initform cl:nil)
   (can_xmt_error
    :reader can_xmt_error
    :initarg :can_xmt_error
    :type cl:boolean
    :initform cl:nil)
   (can_rcv_error
    :reader can_rcv_error
    :initarg :can_rcv_error
    :type cl:boolean
    :initform cl:nil)
   (can_hardware_error
    :reader can_hardware_error
    :initarg :can_hardware_error
    :type cl:boolean
    :initform cl:nil)
   (always_true
    :reader always_true
    :initarg :always_true
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SrrDebug3 (<SrrDebug3>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrrDebug3>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrrDebug3)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name delphi_srr_msgs-msg:<SrrDebug3> is deprecated: use delphi_srr_msgs-msg:SrrDebug3 instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:header-val is deprecated.  Use delphi_srr_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'timer_create_error-val :lambda-list '(m))
(cl:defmethod timer_create_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:timer_create_error-val is deprecated.  Use delphi_srr_msgs-msg:timer_create_error instead.")
  (timer_create_error m))

(cl:ensure-generic-function 'thread_create_error-val :lambda-list '(m))
(cl:defmethod thread_create_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:thread_create_error-val is deprecated.  Use delphi_srr_msgs-msg:thread_create_error instead.")
  (thread_create_error m))

(cl:ensure-generic-function 'arm_calibration_error-val :lambda-list '(m))
(cl:defmethod arm_calibration_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:arm_calibration_error-val is deprecated.  Use delphi_srr_msgs-msg:arm_calibration_error instead.")
  (arm_calibration_error m))

(cl:ensure-generic-function 'spi_fee_error-val :lambda-list '(m))
(cl:defmethod spi_fee_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:spi_fee_error-val is deprecated.  Use delphi_srr_msgs-msg:spi_fee_error instead.")
  (spi_fee_error m))

(cl:ensure-generic-function 'spi_comm_error-val :lambda-list '(m))
(cl:defmethod spi_comm_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:spi_comm_error-val is deprecated.  Use delphi_srr_msgs-msg:spi_comm_error instead.")
  (spi_comm_error m))

(cl:ensure-generic-function 'socket_write_error-val :lambda-list '(m))
(cl:defmethod socket_write_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:socket_write_error-val is deprecated.  Use delphi_srr_msgs-msg:socket_write_error instead.")
  (socket_write_error m))

(cl:ensure-generic-function 'dsp_cal_obsolete_62_error-val :lambda-list '(m))
(cl:defmethod dsp_cal_obsolete_62_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_cal_obsolete_62_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_cal_obsolete_62_error instead.")
  (dsp_cal_obsolete_62_error m))

(cl:ensure-generic-function 'socket_read_error-val :lambda-list '(m))
(cl:defmethod socket_read_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:socket_read_error-val is deprecated.  Use delphi_srr_msgs-msg:socket_read_error instead.")
  (socket_read_error m))

(cl:ensure-generic-function 'socket_init_error-val :lambda-list '(m))
(cl:defmethod socket_init_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:socket_init_error-val is deprecated.  Use delphi_srr_msgs-msg:socket_init_error instead.")
  (socket_init_error m))

(cl:ensure-generic-function 'signal_wait_error-val :lambda-list '(m))
(cl:defmethod signal_wait_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:signal_wait_error-val is deprecated.  Use delphi_srr_msgs-msg:signal_wait_error instead.")
  (signal_wait_error m))

(cl:ensure-generic-function 'signal_send_error-val :lambda-list '(m))
(cl:defmethod signal_send_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:signal_send_error-val is deprecated.  Use delphi_srr_msgs-msg:signal_send_error instead.")
  (signal_send_error m))

(cl:ensure-generic-function 'signal_create_error-val :lambda-list '(m))
(cl:defmethod signal_create_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:signal_create_error-val is deprecated.  Use delphi_srr_msgs-msg:signal_create_error instead.")
  (signal_create_error m))

(cl:ensure-generic-function 'shared_mem_write_error-val :lambda-list '(m))
(cl:defmethod shared_mem_write_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:shared_mem_write_error-val is deprecated.  Use delphi_srr_msgs-msg:shared_mem_write_error instead.")
  (shared_mem_write_error m))

(cl:ensure-generic-function 'shared_mem_read_error-val :lambda-list '(m))
(cl:defmethod shared_mem_read_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:shared_mem_read_error-val is deprecated.  Use delphi_srr_msgs-msg:shared_mem_read_error instead.")
  (shared_mem_read_error m))

(cl:ensure-generic-function 'shared_mem_config_error-val :lambda-list '(m))
(cl:defmethod shared_mem_config_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:shared_mem_config_error-val is deprecated.  Use delphi_srr_msgs-msg:shared_mem_config_error instead.")
  (shared_mem_config_error m))

(cl:ensure-generic-function 'share_mem_init_error-val :lambda-list '(m))
(cl:defmethod share_mem_init_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:share_mem_init_error-val is deprecated.  Use delphi_srr_msgs-msg:share_mem_init_error instead.")
  (share_mem_init_error m))

(cl:ensure-generic-function 'ram_test_error-val :lambda-list '(m))
(cl:defmethod ram_test_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:ram_test_error-val is deprecated.  Use delphi_srr_msgs-msg:ram_test_error instead.")
  (ram_test_error m))

(cl:ensure-generic-function 'num_errors-val :lambda-list '(m))
(cl:defmethod num_errors-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:num_errors-val is deprecated.  Use delphi_srr_msgs-msg:num_errors instead.")
  (num_errors m))

(cl:ensure-generic-function 'mmap_memory_error-val :lambda-list '(m))
(cl:defmethod mmap_memory_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:mmap_memory_error-val is deprecated.  Use delphi_srr_msgs-msg:mmap_memory_error instead.")
  (mmap_memory_error m))

(cl:ensure-generic-function 'isr_attach_error-val :lambda-list '(m))
(cl:defmethod isr_attach_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:isr_attach_error-val is deprecated.  Use delphi_srr_msgs-msg:isr_attach_error instead.")
  (isr_attach_error m))

(cl:ensure-generic-function 'ipc_drv_write_error-val :lambda-list '(m))
(cl:defmethod ipc_drv_write_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:ipc_drv_write_error-val is deprecated.  Use delphi_srr_msgs-msg:ipc_drv_write_error instead.")
  (ipc_drv_write_error m))

(cl:ensure-generic-function 'ipc_drv_trigger_error-val :lambda-list '(m))
(cl:defmethod ipc_drv_trigger_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:ipc_drv_trigger_error-val is deprecated.  Use delphi_srr_msgs-msg:ipc_drv_trigger_error instead.")
  (ipc_drv_trigger_error m))

(cl:ensure-generic-function 'ipc_drv_sync_error-val :lambda-list '(m))
(cl:defmethod ipc_drv_sync_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:ipc_drv_sync_error-val is deprecated.  Use delphi_srr_msgs-msg:ipc_drv_sync_error instead.")
  (ipc_drv_sync_error m))

(cl:ensure-generic-function 'ipc_drv_read_error-val :lambda-list '(m))
(cl:defmethod ipc_drv_read_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:ipc_drv_read_error-val is deprecated.  Use delphi_srr_msgs-msg:ipc_drv_read_error instead.")
  (ipc_drv_read_error m))

(cl:ensure-generic-function 'ipc_drv_init_error-val :lambda-list '(m))
(cl:defmethod ipc_drv_init_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:ipc_drv_init_error-val is deprecated.  Use delphi_srr_msgs-msg:ipc_drv_init_error instead.")
  (ipc_drv_init_error m))

(cl:ensure-generic-function 'interrupt_enable_error-val :lambda-list '(m))
(cl:defmethod interrupt_enable_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:interrupt_enable_error-val is deprecated.  Use delphi_srr_msgs-msg:interrupt_enable_error instead.")
  (interrupt_enable_error m))

(cl:ensure-generic-function 'hil_format_error-val :lambda-list '(m))
(cl:defmethod hil_format_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:hil_format_error-val is deprecated.  Use delphi_srr_msgs-msg:hil_format_error instead.")
  (hil_format_error m))

(cl:ensure-generic-function 'flash_filesystem_error-val :lambda-list '(m))
(cl:defmethod flash_filesystem_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:flash_filesystem_error-val is deprecated.  Use delphi_srr_msgs-msg:flash_filesystem_error instead.")
  (flash_filesystem_error m))

(cl:ensure-generic-function 'error_none-val :lambda-list '(m))
(cl:defmethod error_none-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:error_none-val is deprecated.  Use delphi_srr_msgs-msg:error_none instead.")
  (error_none m))

(cl:ensure-generic-function 'dsp_load_read_error-val :lambda-list '(m))
(cl:defmethod dsp_load_read_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_load_read_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_load_read_error instead.")
  (dsp_load_read_error m))

(cl:ensure-generic-function 'dsp_load_open_error-val :lambda-list '(m))
(cl:defmethod dsp_load_open_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_load_open_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_load_open_error instead.")
  (dsp_load_open_error m))

(cl:ensure-generic-function 'dsp_load_address_error-val :lambda-list '(m))
(cl:defmethod dsp_load_address_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_load_address_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_load_address_error instead.")
  (dsp_load_address_error m))

(cl:ensure-generic-function 'dsp_isp_write_error-val :lambda-list '(m))
(cl:defmethod dsp_isp_write_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_isp_write_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_isp_write_error instead.")
  (dsp_isp_write_error m))

(cl:ensure-generic-function 'dsp_ipc_read_error-val :lambda-list '(m))
(cl:defmethod dsp_ipc_read_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_ipc_read_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_ipc_read_error instead.")
  (dsp_ipc_read_error m))

(cl:ensure-generic-function 'dsp_ipc_init-val :lambda-list '(m))
(cl:defmethod dsp_ipc_init-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_ipc_init-val is deprecated.  Use delphi_srr_msgs-msg:dsp_ipc_init instead.")
  (dsp_ipc_init m))

(cl:ensure-generic-function 'dsp_init_error-val :lambda-list '(m))
(cl:defmethod dsp_init_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_init_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_init_error instead.")
  (dsp_init_error m))

(cl:ensure-generic-function 'dsp_drv_start_error-val :lambda-list '(m))
(cl:defmethod dsp_drv_start_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_drv_start_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_drv_start_error instead.")
  (dsp_drv_start_error m))

(cl:ensure-generic-function 'dsp_drv_load_error-val :lambda-list '(m))
(cl:defmethod dsp_drv_load_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_drv_load_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_drv_load_error instead.")
  (dsp_drv_load_error m))

(cl:ensure-generic-function 'dsp_drv_init_error-val :lambda-list '(m))
(cl:defmethod dsp_drv_init_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_drv_init_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_drv_init_error instead.")
  (dsp_drv_init_error m))

(cl:ensure-generic-function 'dsp_drv_init2_error-val :lambda-list '(m))
(cl:defmethod dsp_drv_init2_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_drv_init2_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_drv_init2_error instead.")
  (dsp_drv_init2_error m))

(cl:ensure-generic-function 'dsp_drv_init1_error-val :lambda-list '(m))
(cl:defmethod dsp_drv_init1_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_drv_init1_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_drv_init1_error instead.")
  (dsp_drv_init1_error m))

(cl:ensure-generic-function 'dsp_calibration_error-val :lambda-list '(m))
(cl:defmethod dsp_calibration_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:dsp_calibration_error-val is deprecated.  Use delphi_srr_msgs-msg:dsp_calibration_error instead.")
  (dsp_calibration_error m))

(cl:ensure-generic-function 'can_xmt_error-val :lambda-list '(m))
(cl:defmethod can_xmt_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_xmt_error-val is deprecated.  Use delphi_srr_msgs-msg:can_xmt_error instead.")
  (can_xmt_error m))

(cl:ensure-generic-function 'can_rcv_error-val :lambda-list '(m))
(cl:defmethod can_rcv_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_rcv_error-val is deprecated.  Use delphi_srr_msgs-msg:can_rcv_error instead.")
  (can_rcv_error m))

(cl:ensure-generic-function 'can_hardware_error-val :lambda-list '(m))
(cl:defmethod can_hardware_error-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:can_hardware_error-val is deprecated.  Use delphi_srr_msgs-msg:can_hardware_error instead.")
  (can_hardware_error m))

(cl:ensure-generic-function 'always_true-val :lambda-list '(m))
(cl:defmethod always_true-val ((m <SrrDebug3>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader delphi_srr_msgs-msg:always_true-val is deprecated.  Use delphi_srr_msgs-msg:always_true instead.")
  (always_true m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrrDebug3>) ostream)
  "Serializes a message object of type '<SrrDebug3>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'timer_create_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'thread_create_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arm_calibration_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'spi_fee_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'spi_comm_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'socket_write_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_cal_obsolete_62_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'socket_read_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'socket_init_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'signal_wait_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'signal_send_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'signal_create_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'shared_mem_write_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'shared_mem_read_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'shared_mem_config_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'share_mem_init_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ram_test_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'num_errors) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mmap_memory_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isr_attach_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipc_drv_write_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipc_drv_trigger_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipc_drv_sync_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipc_drv_read_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ipc_drv_init_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'interrupt_enable_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'hil_format_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flash_filesystem_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'error_none) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_load_read_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_load_open_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_load_address_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_isp_write_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_ipc_read_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_ipc_init) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_init_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_drv_start_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_drv_load_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_drv_init_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_drv_init2_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_drv_init1_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'dsp_calibration_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_xmt_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_rcv_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'can_hardware_error) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'always_true) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrrDebug3>) istream)
  "Deserializes a message object of type '<SrrDebug3>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'timer_create_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'thread_create_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'arm_calibration_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'spi_fee_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'spi_comm_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'socket_write_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_cal_obsolete_62_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'socket_read_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'socket_init_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'signal_wait_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'signal_send_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'signal_create_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'shared_mem_write_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'shared_mem_read_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'shared_mem_config_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'share_mem_init_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ram_test_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'num_errors) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mmap_memory_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isr_attach_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ipc_drv_write_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ipc_drv_trigger_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ipc_drv_sync_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ipc_drv_read_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ipc_drv_init_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'interrupt_enable_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'hil_format_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flash_filesystem_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'error_none) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_load_read_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_load_open_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_load_address_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_isp_write_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_ipc_read_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_ipc_init) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_init_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_drv_start_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_drv_load_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_drv_init_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_drv_init2_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_drv_init1_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'dsp_calibration_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_xmt_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_rcv_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'can_hardware_error) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'always_true) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrrDebug3>)))
  "Returns string type for a message object of type '<SrrDebug3>"
  "delphi_srr_msgs/SrrDebug3")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrrDebug3)))
  "Returns string type for a message object of type 'SrrDebug3"
  "delphi_srr_msgs/SrrDebug3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrrDebug3>)))
  "Returns md5sum for a message object of type '<SrrDebug3>"
  "c0ece44351bdc580e837fa3403929592")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrrDebug3)))
  "Returns md5sum for a message object of type 'SrrDebug3"
  "c0ece44351bdc580e837fa3403929592")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrrDebug3>)))
  "Returns full string definition for message of type '<SrrDebug3>"
  (cl:format cl:nil "# Message file for srr_debug3~%~%std_msgs/Header header~%~%bool      timer_create_error~%bool      thread_create_error~%bool      arm_calibration_error~%bool      spi_fee_error~%bool      spi_comm_error~%bool      socket_write_error~%bool      dsp_cal_obsolete_62_error~%bool      socket_read_error~%bool      socket_init_error~%bool      signal_wait_error~%bool      signal_send_error~%bool      signal_create_error~%bool      shared_mem_write_error~%bool      shared_mem_read_error~%bool      shared_mem_config_error~%bool      share_mem_init_error~%bool      ram_test_error~%bool      num_errors~%bool      mmap_memory_error~%bool      isr_attach_error~%bool      ipc_drv_write_error~%bool      ipc_drv_trigger_error~%bool      ipc_drv_sync_error~%bool      ipc_drv_read_error~%bool      ipc_drv_init_error~%bool      interrupt_enable_error~%bool      hil_format_error~%bool      flash_filesystem_error~%bool      error_none~%bool      dsp_load_read_error~%bool      dsp_load_open_error~%bool      dsp_load_address_error~%bool      dsp_isp_write_error~%bool      dsp_ipc_read_error~%bool      dsp_ipc_init~%bool      dsp_init_error~%bool      dsp_drv_start_error~%bool      dsp_drv_load_error~%bool      dsp_drv_init_error~%bool      dsp_drv_init2_error~%bool      dsp_drv_init1_error~%bool      dsp_calibration_error~%bool      can_xmt_error~%bool      can_rcv_error~%bool      can_hardware_error~%bool      always_true~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrrDebug3)))
  "Returns full string definition for message of type 'SrrDebug3"
  (cl:format cl:nil "# Message file for srr_debug3~%~%std_msgs/Header header~%~%bool      timer_create_error~%bool      thread_create_error~%bool      arm_calibration_error~%bool      spi_fee_error~%bool      spi_comm_error~%bool      socket_write_error~%bool      dsp_cal_obsolete_62_error~%bool      socket_read_error~%bool      socket_init_error~%bool      signal_wait_error~%bool      signal_send_error~%bool      signal_create_error~%bool      shared_mem_write_error~%bool      shared_mem_read_error~%bool      shared_mem_config_error~%bool      share_mem_init_error~%bool      ram_test_error~%bool      num_errors~%bool      mmap_memory_error~%bool      isr_attach_error~%bool      ipc_drv_write_error~%bool      ipc_drv_trigger_error~%bool      ipc_drv_sync_error~%bool      ipc_drv_read_error~%bool      ipc_drv_init_error~%bool      interrupt_enable_error~%bool      hil_format_error~%bool      flash_filesystem_error~%bool      error_none~%bool      dsp_load_read_error~%bool      dsp_load_open_error~%bool      dsp_load_address_error~%bool      dsp_isp_write_error~%bool      dsp_ipc_read_error~%bool      dsp_ipc_init~%bool      dsp_init_error~%bool      dsp_drv_start_error~%bool      dsp_drv_load_error~%bool      dsp_drv_init_error~%bool      dsp_drv_init2_error~%bool      dsp_drv_init1_error~%bool      dsp_calibration_error~%bool      can_xmt_error~%bool      can_rcv_error~%bool      can_hardware_error~%bool      always_true~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrrDebug3>))
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
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrrDebug3>))
  "Converts a ROS message object to a list"
  (cl:list 'SrrDebug3
    (cl:cons ':header (header msg))
    (cl:cons ':timer_create_error (timer_create_error msg))
    (cl:cons ':thread_create_error (thread_create_error msg))
    (cl:cons ':arm_calibration_error (arm_calibration_error msg))
    (cl:cons ':spi_fee_error (spi_fee_error msg))
    (cl:cons ':spi_comm_error (spi_comm_error msg))
    (cl:cons ':socket_write_error (socket_write_error msg))
    (cl:cons ':dsp_cal_obsolete_62_error (dsp_cal_obsolete_62_error msg))
    (cl:cons ':socket_read_error (socket_read_error msg))
    (cl:cons ':socket_init_error (socket_init_error msg))
    (cl:cons ':signal_wait_error (signal_wait_error msg))
    (cl:cons ':signal_send_error (signal_send_error msg))
    (cl:cons ':signal_create_error (signal_create_error msg))
    (cl:cons ':shared_mem_write_error (shared_mem_write_error msg))
    (cl:cons ':shared_mem_read_error (shared_mem_read_error msg))
    (cl:cons ':shared_mem_config_error (shared_mem_config_error msg))
    (cl:cons ':share_mem_init_error (share_mem_init_error msg))
    (cl:cons ':ram_test_error (ram_test_error msg))
    (cl:cons ':num_errors (num_errors msg))
    (cl:cons ':mmap_memory_error (mmap_memory_error msg))
    (cl:cons ':isr_attach_error (isr_attach_error msg))
    (cl:cons ':ipc_drv_write_error (ipc_drv_write_error msg))
    (cl:cons ':ipc_drv_trigger_error (ipc_drv_trigger_error msg))
    (cl:cons ':ipc_drv_sync_error (ipc_drv_sync_error msg))
    (cl:cons ':ipc_drv_read_error (ipc_drv_read_error msg))
    (cl:cons ':ipc_drv_init_error (ipc_drv_init_error msg))
    (cl:cons ':interrupt_enable_error (interrupt_enable_error msg))
    (cl:cons ':hil_format_error (hil_format_error msg))
    (cl:cons ':flash_filesystem_error (flash_filesystem_error msg))
    (cl:cons ':error_none (error_none msg))
    (cl:cons ':dsp_load_read_error (dsp_load_read_error msg))
    (cl:cons ':dsp_load_open_error (dsp_load_open_error msg))
    (cl:cons ':dsp_load_address_error (dsp_load_address_error msg))
    (cl:cons ':dsp_isp_write_error (dsp_isp_write_error msg))
    (cl:cons ':dsp_ipc_read_error (dsp_ipc_read_error msg))
    (cl:cons ':dsp_ipc_init (dsp_ipc_init msg))
    (cl:cons ':dsp_init_error (dsp_init_error msg))
    (cl:cons ':dsp_drv_start_error (dsp_drv_start_error msg))
    (cl:cons ':dsp_drv_load_error (dsp_drv_load_error msg))
    (cl:cons ':dsp_drv_init_error (dsp_drv_init_error msg))
    (cl:cons ':dsp_drv_init2_error (dsp_drv_init2_error msg))
    (cl:cons ':dsp_drv_init1_error (dsp_drv_init1_error msg))
    (cl:cons ':dsp_calibration_error (dsp_calibration_error msg))
    (cl:cons ':can_xmt_error (can_xmt_error msg))
    (cl:cons ':can_rcv_error (can_rcv_error msg))
    (cl:cons ':can_hardware_error (can_hardware_error msg))
    (cl:cons ':always_true (always_true msg))
))
