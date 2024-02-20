;; Auto-generated. Do not edit!


(when (boundp 'delphi_srr_msgs::SrrDebug3)
  (if (not (find-package "DELPHI_SRR_MSGS"))
    (make-package "DELPHI_SRR_MSGS"))
  (shadow 'SrrDebug3 (find-package "DELPHI_SRR_MSGS")))
(unless (find-package "DELPHI_SRR_MSGS::SRRDEBUG3")
  (make-package "DELPHI_SRR_MSGS::SRRDEBUG3"))

(in-package "ROS")
;;//! \htmlinclude SrrDebug3.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass delphi_srr_msgs::SrrDebug3
  :super ros::object
  :slots (_header _timer_create_error _thread_create_error _arm_calibration_error _spi_fee_error _spi_comm_error _socket_write_error _dsp_cal_obsolete_62_error _socket_read_error _socket_init_error _signal_wait_error _signal_send_error _signal_create_error _shared_mem_write_error _shared_mem_read_error _shared_mem_config_error _share_mem_init_error _ram_test_error _num_errors _mmap_memory_error _isr_attach_error _ipc_drv_write_error _ipc_drv_trigger_error _ipc_drv_sync_error _ipc_drv_read_error _ipc_drv_init_error _interrupt_enable_error _hil_format_error _flash_filesystem_error _error_none _dsp_load_read_error _dsp_load_open_error _dsp_load_address_error _dsp_isp_write_error _dsp_ipc_read_error _dsp_ipc_init _dsp_init_error _dsp_drv_start_error _dsp_drv_load_error _dsp_drv_init_error _dsp_drv_init2_error _dsp_drv_init1_error _dsp_calibration_error _can_xmt_error _can_rcv_error _can_hardware_error _always_true ))

(defmethod delphi_srr_msgs::SrrDebug3
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:timer_create_error __timer_create_error) nil)
    ((:thread_create_error __thread_create_error) nil)
    ((:arm_calibration_error __arm_calibration_error) nil)
    ((:spi_fee_error __spi_fee_error) nil)
    ((:spi_comm_error __spi_comm_error) nil)
    ((:socket_write_error __socket_write_error) nil)
    ((:dsp_cal_obsolete_62_error __dsp_cal_obsolete_62_error) nil)
    ((:socket_read_error __socket_read_error) nil)
    ((:socket_init_error __socket_init_error) nil)
    ((:signal_wait_error __signal_wait_error) nil)
    ((:signal_send_error __signal_send_error) nil)
    ((:signal_create_error __signal_create_error) nil)
    ((:shared_mem_write_error __shared_mem_write_error) nil)
    ((:shared_mem_read_error __shared_mem_read_error) nil)
    ((:shared_mem_config_error __shared_mem_config_error) nil)
    ((:share_mem_init_error __share_mem_init_error) nil)
    ((:ram_test_error __ram_test_error) nil)
    ((:num_errors __num_errors) nil)
    ((:mmap_memory_error __mmap_memory_error) nil)
    ((:isr_attach_error __isr_attach_error) nil)
    ((:ipc_drv_write_error __ipc_drv_write_error) nil)
    ((:ipc_drv_trigger_error __ipc_drv_trigger_error) nil)
    ((:ipc_drv_sync_error __ipc_drv_sync_error) nil)
    ((:ipc_drv_read_error __ipc_drv_read_error) nil)
    ((:ipc_drv_init_error __ipc_drv_init_error) nil)
    ((:interrupt_enable_error __interrupt_enable_error) nil)
    ((:hil_format_error __hil_format_error) nil)
    ((:flash_filesystem_error __flash_filesystem_error) nil)
    ((:error_none __error_none) nil)
    ((:dsp_load_read_error __dsp_load_read_error) nil)
    ((:dsp_load_open_error __dsp_load_open_error) nil)
    ((:dsp_load_address_error __dsp_load_address_error) nil)
    ((:dsp_isp_write_error __dsp_isp_write_error) nil)
    ((:dsp_ipc_read_error __dsp_ipc_read_error) nil)
    ((:dsp_ipc_init __dsp_ipc_init) nil)
    ((:dsp_init_error __dsp_init_error) nil)
    ((:dsp_drv_start_error __dsp_drv_start_error) nil)
    ((:dsp_drv_load_error __dsp_drv_load_error) nil)
    ((:dsp_drv_init_error __dsp_drv_init_error) nil)
    ((:dsp_drv_init2_error __dsp_drv_init2_error) nil)
    ((:dsp_drv_init1_error __dsp_drv_init1_error) nil)
    ((:dsp_calibration_error __dsp_calibration_error) nil)
    ((:can_xmt_error __can_xmt_error) nil)
    ((:can_rcv_error __can_rcv_error) nil)
    ((:can_hardware_error __can_hardware_error) nil)
    ((:always_true __always_true) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _timer_create_error __timer_create_error)
   (setq _thread_create_error __thread_create_error)
   (setq _arm_calibration_error __arm_calibration_error)
   (setq _spi_fee_error __spi_fee_error)
   (setq _spi_comm_error __spi_comm_error)
   (setq _socket_write_error __socket_write_error)
   (setq _dsp_cal_obsolete_62_error __dsp_cal_obsolete_62_error)
   (setq _socket_read_error __socket_read_error)
   (setq _socket_init_error __socket_init_error)
   (setq _signal_wait_error __signal_wait_error)
   (setq _signal_send_error __signal_send_error)
   (setq _signal_create_error __signal_create_error)
   (setq _shared_mem_write_error __shared_mem_write_error)
   (setq _shared_mem_read_error __shared_mem_read_error)
   (setq _shared_mem_config_error __shared_mem_config_error)
   (setq _share_mem_init_error __share_mem_init_error)
   (setq _ram_test_error __ram_test_error)
   (setq _num_errors __num_errors)
   (setq _mmap_memory_error __mmap_memory_error)
   (setq _isr_attach_error __isr_attach_error)
   (setq _ipc_drv_write_error __ipc_drv_write_error)
   (setq _ipc_drv_trigger_error __ipc_drv_trigger_error)
   (setq _ipc_drv_sync_error __ipc_drv_sync_error)
   (setq _ipc_drv_read_error __ipc_drv_read_error)
   (setq _ipc_drv_init_error __ipc_drv_init_error)
   (setq _interrupt_enable_error __interrupt_enable_error)
   (setq _hil_format_error __hil_format_error)
   (setq _flash_filesystem_error __flash_filesystem_error)
   (setq _error_none __error_none)
   (setq _dsp_load_read_error __dsp_load_read_error)
   (setq _dsp_load_open_error __dsp_load_open_error)
   (setq _dsp_load_address_error __dsp_load_address_error)
   (setq _dsp_isp_write_error __dsp_isp_write_error)
   (setq _dsp_ipc_read_error __dsp_ipc_read_error)
   (setq _dsp_ipc_init __dsp_ipc_init)
   (setq _dsp_init_error __dsp_init_error)
   (setq _dsp_drv_start_error __dsp_drv_start_error)
   (setq _dsp_drv_load_error __dsp_drv_load_error)
   (setq _dsp_drv_init_error __dsp_drv_init_error)
   (setq _dsp_drv_init2_error __dsp_drv_init2_error)
   (setq _dsp_drv_init1_error __dsp_drv_init1_error)
   (setq _dsp_calibration_error __dsp_calibration_error)
   (setq _can_xmt_error __can_xmt_error)
   (setq _can_rcv_error __can_rcv_error)
   (setq _can_hardware_error __can_hardware_error)
   (setq _always_true __always_true)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:timer_create_error
   (&optional (__timer_create_error :null))
   (if (not (eq __timer_create_error :null)) (setq _timer_create_error __timer_create_error)) _timer_create_error)
  (:thread_create_error
   (&optional (__thread_create_error :null))
   (if (not (eq __thread_create_error :null)) (setq _thread_create_error __thread_create_error)) _thread_create_error)
  (:arm_calibration_error
   (&optional (__arm_calibration_error :null))
   (if (not (eq __arm_calibration_error :null)) (setq _arm_calibration_error __arm_calibration_error)) _arm_calibration_error)
  (:spi_fee_error
   (&optional (__spi_fee_error :null))
   (if (not (eq __spi_fee_error :null)) (setq _spi_fee_error __spi_fee_error)) _spi_fee_error)
  (:spi_comm_error
   (&optional (__spi_comm_error :null))
   (if (not (eq __spi_comm_error :null)) (setq _spi_comm_error __spi_comm_error)) _spi_comm_error)
  (:socket_write_error
   (&optional (__socket_write_error :null))
   (if (not (eq __socket_write_error :null)) (setq _socket_write_error __socket_write_error)) _socket_write_error)
  (:dsp_cal_obsolete_62_error
   (&optional (__dsp_cal_obsolete_62_error :null))
   (if (not (eq __dsp_cal_obsolete_62_error :null)) (setq _dsp_cal_obsolete_62_error __dsp_cal_obsolete_62_error)) _dsp_cal_obsolete_62_error)
  (:socket_read_error
   (&optional (__socket_read_error :null))
   (if (not (eq __socket_read_error :null)) (setq _socket_read_error __socket_read_error)) _socket_read_error)
  (:socket_init_error
   (&optional (__socket_init_error :null))
   (if (not (eq __socket_init_error :null)) (setq _socket_init_error __socket_init_error)) _socket_init_error)
  (:signal_wait_error
   (&optional (__signal_wait_error :null))
   (if (not (eq __signal_wait_error :null)) (setq _signal_wait_error __signal_wait_error)) _signal_wait_error)
  (:signal_send_error
   (&optional (__signal_send_error :null))
   (if (not (eq __signal_send_error :null)) (setq _signal_send_error __signal_send_error)) _signal_send_error)
  (:signal_create_error
   (&optional (__signal_create_error :null))
   (if (not (eq __signal_create_error :null)) (setq _signal_create_error __signal_create_error)) _signal_create_error)
  (:shared_mem_write_error
   (&optional (__shared_mem_write_error :null))
   (if (not (eq __shared_mem_write_error :null)) (setq _shared_mem_write_error __shared_mem_write_error)) _shared_mem_write_error)
  (:shared_mem_read_error
   (&optional (__shared_mem_read_error :null))
   (if (not (eq __shared_mem_read_error :null)) (setq _shared_mem_read_error __shared_mem_read_error)) _shared_mem_read_error)
  (:shared_mem_config_error
   (&optional (__shared_mem_config_error :null))
   (if (not (eq __shared_mem_config_error :null)) (setq _shared_mem_config_error __shared_mem_config_error)) _shared_mem_config_error)
  (:share_mem_init_error
   (&optional (__share_mem_init_error :null))
   (if (not (eq __share_mem_init_error :null)) (setq _share_mem_init_error __share_mem_init_error)) _share_mem_init_error)
  (:ram_test_error
   (&optional (__ram_test_error :null))
   (if (not (eq __ram_test_error :null)) (setq _ram_test_error __ram_test_error)) _ram_test_error)
  (:num_errors
   (&optional (__num_errors :null))
   (if (not (eq __num_errors :null)) (setq _num_errors __num_errors)) _num_errors)
  (:mmap_memory_error
   (&optional (__mmap_memory_error :null))
   (if (not (eq __mmap_memory_error :null)) (setq _mmap_memory_error __mmap_memory_error)) _mmap_memory_error)
  (:isr_attach_error
   (&optional (__isr_attach_error :null))
   (if (not (eq __isr_attach_error :null)) (setq _isr_attach_error __isr_attach_error)) _isr_attach_error)
  (:ipc_drv_write_error
   (&optional (__ipc_drv_write_error :null))
   (if (not (eq __ipc_drv_write_error :null)) (setq _ipc_drv_write_error __ipc_drv_write_error)) _ipc_drv_write_error)
  (:ipc_drv_trigger_error
   (&optional (__ipc_drv_trigger_error :null))
   (if (not (eq __ipc_drv_trigger_error :null)) (setq _ipc_drv_trigger_error __ipc_drv_trigger_error)) _ipc_drv_trigger_error)
  (:ipc_drv_sync_error
   (&optional (__ipc_drv_sync_error :null))
   (if (not (eq __ipc_drv_sync_error :null)) (setq _ipc_drv_sync_error __ipc_drv_sync_error)) _ipc_drv_sync_error)
  (:ipc_drv_read_error
   (&optional (__ipc_drv_read_error :null))
   (if (not (eq __ipc_drv_read_error :null)) (setq _ipc_drv_read_error __ipc_drv_read_error)) _ipc_drv_read_error)
  (:ipc_drv_init_error
   (&optional (__ipc_drv_init_error :null))
   (if (not (eq __ipc_drv_init_error :null)) (setq _ipc_drv_init_error __ipc_drv_init_error)) _ipc_drv_init_error)
  (:interrupt_enable_error
   (&optional (__interrupt_enable_error :null))
   (if (not (eq __interrupt_enable_error :null)) (setq _interrupt_enable_error __interrupt_enable_error)) _interrupt_enable_error)
  (:hil_format_error
   (&optional (__hil_format_error :null))
   (if (not (eq __hil_format_error :null)) (setq _hil_format_error __hil_format_error)) _hil_format_error)
  (:flash_filesystem_error
   (&optional (__flash_filesystem_error :null))
   (if (not (eq __flash_filesystem_error :null)) (setq _flash_filesystem_error __flash_filesystem_error)) _flash_filesystem_error)
  (:error_none
   (&optional (__error_none :null))
   (if (not (eq __error_none :null)) (setq _error_none __error_none)) _error_none)
  (:dsp_load_read_error
   (&optional (__dsp_load_read_error :null))
   (if (not (eq __dsp_load_read_error :null)) (setq _dsp_load_read_error __dsp_load_read_error)) _dsp_load_read_error)
  (:dsp_load_open_error
   (&optional (__dsp_load_open_error :null))
   (if (not (eq __dsp_load_open_error :null)) (setq _dsp_load_open_error __dsp_load_open_error)) _dsp_load_open_error)
  (:dsp_load_address_error
   (&optional (__dsp_load_address_error :null))
   (if (not (eq __dsp_load_address_error :null)) (setq _dsp_load_address_error __dsp_load_address_error)) _dsp_load_address_error)
  (:dsp_isp_write_error
   (&optional (__dsp_isp_write_error :null))
   (if (not (eq __dsp_isp_write_error :null)) (setq _dsp_isp_write_error __dsp_isp_write_error)) _dsp_isp_write_error)
  (:dsp_ipc_read_error
   (&optional (__dsp_ipc_read_error :null))
   (if (not (eq __dsp_ipc_read_error :null)) (setq _dsp_ipc_read_error __dsp_ipc_read_error)) _dsp_ipc_read_error)
  (:dsp_ipc_init
   (&optional (__dsp_ipc_init :null))
   (if (not (eq __dsp_ipc_init :null)) (setq _dsp_ipc_init __dsp_ipc_init)) _dsp_ipc_init)
  (:dsp_init_error
   (&optional (__dsp_init_error :null))
   (if (not (eq __dsp_init_error :null)) (setq _dsp_init_error __dsp_init_error)) _dsp_init_error)
  (:dsp_drv_start_error
   (&optional (__dsp_drv_start_error :null))
   (if (not (eq __dsp_drv_start_error :null)) (setq _dsp_drv_start_error __dsp_drv_start_error)) _dsp_drv_start_error)
  (:dsp_drv_load_error
   (&optional (__dsp_drv_load_error :null))
   (if (not (eq __dsp_drv_load_error :null)) (setq _dsp_drv_load_error __dsp_drv_load_error)) _dsp_drv_load_error)
  (:dsp_drv_init_error
   (&optional (__dsp_drv_init_error :null))
   (if (not (eq __dsp_drv_init_error :null)) (setq _dsp_drv_init_error __dsp_drv_init_error)) _dsp_drv_init_error)
  (:dsp_drv_init2_error
   (&optional (__dsp_drv_init2_error :null))
   (if (not (eq __dsp_drv_init2_error :null)) (setq _dsp_drv_init2_error __dsp_drv_init2_error)) _dsp_drv_init2_error)
  (:dsp_drv_init1_error
   (&optional (__dsp_drv_init1_error :null))
   (if (not (eq __dsp_drv_init1_error :null)) (setq _dsp_drv_init1_error __dsp_drv_init1_error)) _dsp_drv_init1_error)
  (:dsp_calibration_error
   (&optional (__dsp_calibration_error :null))
   (if (not (eq __dsp_calibration_error :null)) (setq _dsp_calibration_error __dsp_calibration_error)) _dsp_calibration_error)
  (:can_xmt_error
   (&optional (__can_xmt_error :null))
   (if (not (eq __can_xmt_error :null)) (setq _can_xmt_error __can_xmt_error)) _can_xmt_error)
  (:can_rcv_error
   (&optional (__can_rcv_error :null))
   (if (not (eq __can_rcv_error :null)) (setq _can_rcv_error __can_rcv_error)) _can_rcv_error)
  (:can_hardware_error
   (&optional (__can_hardware_error :null))
   (if (not (eq __can_hardware_error :null)) (setq _can_hardware_error __can_hardware_error)) _can_hardware_error)
  (:always_true
   (&optional (__always_true :null))
   (if (not (eq __always_true :null)) (setq _always_true __always_true)) _always_true)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _timer_create_error
    1
    ;; bool _thread_create_error
    1
    ;; bool _arm_calibration_error
    1
    ;; bool _spi_fee_error
    1
    ;; bool _spi_comm_error
    1
    ;; bool _socket_write_error
    1
    ;; bool _dsp_cal_obsolete_62_error
    1
    ;; bool _socket_read_error
    1
    ;; bool _socket_init_error
    1
    ;; bool _signal_wait_error
    1
    ;; bool _signal_send_error
    1
    ;; bool _signal_create_error
    1
    ;; bool _shared_mem_write_error
    1
    ;; bool _shared_mem_read_error
    1
    ;; bool _shared_mem_config_error
    1
    ;; bool _share_mem_init_error
    1
    ;; bool _ram_test_error
    1
    ;; bool _num_errors
    1
    ;; bool _mmap_memory_error
    1
    ;; bool _isr_attach_error
    1
    ;; bool _ipc_drv_write_error
    1
    ;; bool _ipc_drv_trigger_error
    1
    ;; bool _ipc_drv_sync_error
    1
    ;; bool _ipc_drv_read_error
    1
    ;; bool _ipc_drv_init_error
    1
    ;; bool _interrupt_enable_error
    1
    ;; bool _hil_format_error
    1
    ;; bool _flash_filesystem_error
    1
    ;; bool _error_none
    1
    ;; bool _dsp_load_read_error
    1
    ;; bool _dsp_load_open_error
    1
    ;; bool _dsp_load_address_error
    1
    ;; bool _dsp_isp_write_error
    1
    ;; bool _dsp_ipc_read_error
    1
    ;; bool _dsp_ipc_init
    1
    ;; bool _dsp_init_error
    1
    ;; bool _dsp_drv_start_error
    1
    ;; bool _dsp_drv_load_error
    1
    ;; bool _dsp_drv_init_error
    1
    ;; bool _dsp_drv_init2_error
    1
    ;; bool _dsp_drv_init1_error
    1
    ;; bool _dsp_calibration_error
    1
    ;; bool _can_xmt_error
    1
    ;; bool _can_rcv_error
    1
    ;; bool _can_hardware_error
    1
    ;; bool _always_true
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _timer_create_error
       (if _timer_create_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _thread_create_error
       (if _thread_create_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _arm_calibration_error
       (if _arm_calibration_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _spi_fee_error
       (if _spi_fee_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _spi_comm_error
       (if _spi_comm_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _socket_write_error
       (if _socket_write_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_cal_obsolete_62_error
       (if _dsp_cal_obsolete_62_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _socket_read_error
       (if _socket_read_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _socket_init_error
       (if _socket_init_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _signal_wait_error
       (if _signal_wait_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _signal_send_error
       (if _signal_send_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _signal_create_error
       (if _signal_create_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _shared_mem_write_error
       (if _shared_mem_write_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _shared_mem_read_error
       (if _shared_mem_read_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _shared_mem_config_error
       (if _shared_mem_config_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _share_mem_init_error
       (if _share_mem_init_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _ram_test_error
       (if _ram_test_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _num_errors
       (if _num_errors (write-byte -1 s) (write-byte 0 s))
     ;; bool _mmap_memory_error
       (if _mmap_memory_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _isr_attach_error
       (if _isr_attach_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _ipc_drv_write_error
       (if _ipc_drv_write_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _ipc_drv_trigger_error
       (if _ipc_drv_trigger_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _ipc_drv_sync_error
       (if _ipc_drv_sync_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _ipc_drv_read_error
       (if _ipc_drv_read_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _ipc_drv_init_error
       (if _ipc_drv_init_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _interrupt_enable_error
       (if _interrupt_enable_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _hil_format_error
       (if _hil_format_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _flash_filesystem_error
       (if _flash_filesystem_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _error_none
       (if _error_none (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_load_read_error
       (if _dsp_load_read_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_load_open_error
       (if _dsp_load_open_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_load_address_error
       (if _dsp_load_address_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_isp_write_error
       (if _dsp_isp_write_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_ipc_read_error
       (if _dsp_ipc_read_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_ipc_init
       (if _dsp_ipc_init (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_init_error
       (if _dsp_init_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_drv_start_error
       (if _dsp_drv_start_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_drv_load_error
       (if _dsp_drv_load_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_drv_init_error
       (if _dsp_drv_init_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_drv_init2_error
       (if _dsp_drv_init2_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_drv_init1_error
       (if _dsp_drv_init1_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _dsp_calibration_error
       (if _dsp_calibration_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_xmt_error
       (if _can_xmt_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_rcv_error
       (if _can_rcv_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _can_hardware_error
       (if _can_hardware_error (write-byte -1 s) (write-byte 0 s))
     ;; bool _always_true
       (if _always_true (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _timer_create_error
     (setq _timer_create_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _thread_create_error
     (setq _thread_create_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _arm_calibration_error
     (setq _arm_calibration_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _spi_fee_error
     (setq _spi_fee_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _spi_comm_error
     (setq _spi_comm_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _socket_write_error
     (setq _socket_write_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_cal_obsolete_62_error
     (setq _dsp_cal_obsolete_62_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _socket_read_error
     (setq _socket_read_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _socket_init_error
     (setq _socket_init_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _signal_wait_error
     (setq _signal_wait_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _signal_send_error
     (setq _signal_send_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _signal_create_error
     (setq _signal_create_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _shared_mem_write_error
     (setq _shared_mem_write_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _shared_mem_read_error
     (setq _shared_mem_read_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _shared_mem_config_error
     (setq _shared_mem_config_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _share_mem_init_error
     (setq _share_mem_init_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ram_test_error
     (setq _ram_test_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _num_errors
     (setq _num_errors (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _mmap_memory_error
     (setq _mmap_memory_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _isr_attach_error
     (setq _isr_attach_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ipc_drv_write_error
     (setq _ipc_drv_write_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ipc_drv_trigger_error
     (setq _ipc_drv_trigger_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ipc_drv_sync_error
     (setq _ipc_drv_sync_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ipc_drv_read_error
     (setq _ipc_drv_read_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ipc_drv_init_error
     (setq _ipc_drv_init_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _interrupt_enable_error
     (setq _interrupt_enable_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _hil_format_error
     (setq _hil_format_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flash_filesystem_error
     (setq _flash_filesystem_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _error_none
     (setq _error_none (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_load_read_error
     (setq _dsp_load_read_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_load_open_error
     (setq _dsp_load_open_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_load_address_error
     (setq _dsp_load_address_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_isp_write_error
     (setq _dsp_isp_write_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_ipc_read_error
     (setq _dsp_ipc_read_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_ipc_init
     (setq _dsp_ipc_init (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_init_error
     (setq _dsp_init_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_drv_start_error
     (setq _dsp_drv_start_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_drv_load_error
     (setq _dsp_drv_load_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_drv_init_error
     (setq _dsp_drv_init_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_drv_init2_error
     (setq _dsp_drv_init2_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_drv_init1_error
     (setq _dsp_drv_init1_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _dsp_calibration_error
     (setq _dsp_calibration_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_xmt_error
     (setq _can_xmt_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_rcv_error
     (setq _can_rcv_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _can_hardware_error
     (setq _can_hardware_error (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _always_true
     (setq _always_true (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get delphi_srr_msgs::SrrDebug3 :md5sum-) "c0ece44351bdc580e837fa3403929592")
(setf (get delphi_srr_msgs::SrrDebug3 :datatype-) "delphi_srr_msgs/SrrDebug3")
(setf (get delphi_srr_msgs::SrrDebug3 :definition-)
      "# Message file for srr_debug3

std_msgs/Header header

bool      timer_create_error
bool      thread_create_error
bool      arm_calibration_error
bool      spi_fee_error
bool      spi_comm_error
bool      socket_write_error
bool      dsp_cal_obsolete_62_error
bool      socket_read_error
bool      socket_init_error
bool      signal_wait_error
bool      signal_send_error
bool      signal_create_error
bool      shared_mem_write_error
bool      shared_mem_read_error
bool      shared_mem_config_error
bool      share_mem_init_error
bool      ram_test_error
bool      num_errors
bool      mmap_memory_error
bool      isr_attach_error
bool      ipc_drv_write_error
bool      ipc_drv_trigger_error
bool      ipc_drv_sync_error
bool      ipc_drv_read_error
bool      ipc_drv_init_error
bool      interrupt_enable_error
bool      hil_format_error
bool      flash_filesystem_error
bool      error_none
bool      dsp_load_read_error
bool      dsp_load_open_error
bool      dsp_load_address_error
bool      dsp_isp_write_error
bool      dsp_ipc_read_error
bool      dsp_ipc_init
bool      dsp_init_error
bool      dsp_drv_start_error
bool      dsp_drv_load_error
bool      dsp_drv_init_error
bool      dsp_drv_init2_error
bool      dsp_drv_init1_error
bool      dsp_calibration_error
bool      can_xmt_error
bool      can_rcv_error
bool      can_hardware_error
bool      always_true

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



(provide :delphi_srr_msgs/SrrDebug3 "c0ece44351bdc580e837fa3403929592")


