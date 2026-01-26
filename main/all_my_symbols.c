// Generated using:
//   python managed_components/espressif__elf_loader/tool/symbols.py -i build/esp32dos.elf -t e
// Should stay in:
//   managed_components/espressif__elf_loader/src/esp_all_symbol.c
// Copied here just for reference.
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include "private/elf_symbol.h"

/* Extern declarations from ELF symbol table */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wbuiltin-declaration-mismatch"
extern int stpcpy;
extern int gzopen;
extern int gzread;
extern int gzwrite;
extern int gzclose;
extern int gzerror;
extern int deflateInit2_;
extern int deflate;
extern int deflateEnd;
extern int crc32;
extern int ferror;
extern int feof;
extern int breezy_http_download;
extern int esp_crt_bundle_attach;
extern int esp_http_client_cleanup;
extern int esp_http_client_get_status_code;
extern int esp_http_client_init;
extern int esp_http_client_perform;
extern int esp_http_client_set_header;
extern int elf_get_cycle_count;
extern int vTaskDelayUntil;
extern int xTaskGetTickCount;
extern int my_display_refresh_palette;
extern int vterm_set_palette;
extern int vterm_get_palette;
extern int srand;
extern int rand;
extern int sin;
extern int cos;
extern int sqrt;
extern int sinf;
extern int cosf;
extern int sqrtf;
extern int vterm_get_size;
extern int clock;
extern int isalnum;
extern int isalpha;
extern int isdigit;
extern int isspace;
extern int lseek;
extern int unlink;
extern int heap_caps_realloc;
extern int heap_caps_get_free_size;
extern int heap_caps_check_integrity_all;
extern int mspi_timing_psram_config_clear_tuning_regs;
extern int __multadd;
extern int _xt_coproc_sa_offset;
extern int pthread_detach;
extern int spi_device_polling_transmit;
extern int rtc_clk_cpu_freq_set_xtal;
extern int printf;
extern int heap_caps_calloc;
extern int netconn_prepare_delete;
extern int pxPortInitialiseStack;
extern int memspi_host_read_id_hs;
extern int __atomic_fetch_or_4;
extern int esp_psram_chip_init;
extern int rtc_spinlock;
extern int exit;
extern int esp_time_impl_set_boot_time;
extern int __cxa_guard_dummy;
extern int xPortEnterCriticalTimeout;
extern int esp_mspi_pin_reserve;
extern int spi_flash_hal_supports_direct_write;
extern int sys_mutex_new;
extern int __ratio;
extern int esp_vfs_opendir;
extern int esp_backtrace_print_from_frame;
extern int esp_vfs_fstat;
extern int gdma_hal_connect_peri;
extern int esp_vfs_uart_get_vfs;
extern int pvalloc;
extern int igmp_leavegroup_netif;
extern int esp_vfs_open;
extern int spi_bus_lock_wait_bg_done;
extern int esp_intr_ptr_in_isr_region;
extern int spi_flash_chip_winbond_erase_sector;
extern int tcp_recv_null;
extern int gpio_hal_intr_disable;
extern int xQueueGenericCreateStatic;
extern int vListInitialiseItem;
extern int __atomic_fetch_or_8;
extern int spi_flash_chip_generic_get_write_protect;
extern int rewind;
extern int raw_recv;
extern int realloc;
extern int g_exc_frames;
extern int uart_hal_get_baudrate;
extern int _getpid_r;
extern int pbuf_realloc;
extern int esp_backtrace_get_start;
extern int spi_flash_chip_generic_wait_idle;
extern int esp_clk_tree_rc_fast_d256_get_freq_hz;
extern int spi_bus_register_destroy_func;
extern int strerror_l;
extern int spicommon_dma_chan_alloc;
extern int systimer_hal_connect_alarm_counter;
extern int mld6_leavegroup_netif;
extern int esp_time_impl_get_boot_time;
extern int rtc_dig_clk8m_enable;
extern int g_panic_abort;
extern int memmove;
extern int gpio_set_intr_type;
extern int rtc_clk_cpu_freq_set_config;
extern int pbuf_chain;
extern int esp_clk_tree_lp_slow_get_freq_hz;
extern int ip4_route_src;
extern int udp_bind_netif;
extern int esp_elf_print_phdr;
extern int esp_mprot_get_split_addr_lock;
extern int esp_libc_init;
extern int memp_FRAG_PBUF;
extern int uart_hal_set_data_bit_num;
extern int __atomic_s32c1i_compare_exchange_4;
extern int heap_caps_free;
extern int console_fcntl;
extern int esp_flash_encryption_enabled;
extern int esp_task_wdt_init;
extern int labs;
extern int xTaskResumeAll;
extern int tcp_send_empty_ack;
extern int port_IntStack;
extern int esp_flash_app_init;
extern int get_vfs_for_index;
extern int registered_heaps;
extern int nvs_sec_provider_include_impl;
extern int igmp_input;
extern int localeconv;
extern int default_router_list;
extern int spi_hal_push_tx_buffer;
extern int esp_libc_include_init_funcs;
extern int _kill_r;
extern int tzset;
extern int esp_system_include_startup_funcs;
extern int setlocale;
extern int xt_ints_on;
extern int ip6_frag;
extern int vListInsert;
extern int ip4_route_src_hook;
extern int spi_flash_chip_issi_get_io_mode;
extern int esp_set_time_from_rtc;
extern int netif_get_by_index;
extern int rtc_sleep_pu;
extern int __sprint_r;
extern int s_register_config_driver;
extern int _ledc_set_pin;
extern int _invalid_pc_placeholder;
extern int igmp_lookfor_group;
extern int strcspn;
extern int esp_mspi_pin_init;
extern int xTaskGenericNotifyWait;
extern int spi_flash_chip_generic_detect_size;
extern int esp_intr_disable;
extern int spi_flash_chip_mxic_opi_erase_block;
extern int gpio_reset_pin;
extern int strstr;
extern int nan;
extern int clock_gettime;
extern int vApplicationGetIdleTaskMemory;
extern int get_vfs_for_path;
extern int gdma_config_transfer;
extern int mmu_hal_vaddr_to_paddr;
extern int xRingbufferSendFromISR;
extern int systimer_hal_enable_counter;
extern int fread;
extern int spi_flash_common_write_status_16b_wrsr;
extern int vRingbufferDelete;
extern int gpio_sleep_sel_dis;
extern int __mprec_tinytens;
extern int spi_flash_hal_suspend;
extern int tcp_pcb_lists;
extern int netif_get_ip6_addr_match;
extern int raw_bind;
extern int sar_periph_ctrl_init;
extern int gpio_od_enable;
extern int mem_malloc;
extern int esp_libc_init_global_stdio;
extern int tcp_bind_netif;
extern int __atexit_recursive_mutex;
extern int spicommon_dma_desc_alloc;
extern int memspi_host_flush_cache;
extern int pbuf_copy_partial_pbuf;
extern int spi_flash_disable_interrupts_caches_and_other_cpu;
extern int _calloc_r;
extern int esp_psram_is_initialized;
extern int heap_caps_check_add_region_allowed;
extern int pthread_attr_setstacksize;
extern int gdma_ahb_hal_get_eof_desc_addr;
extern int xQueueCreateMutex;
extern int esp_ptr_external_ram;
extern int _free_r;
extern int console_fsync;
extern int __retarget_lock_release;
extern int strtoull;
extern int esp_opiflash_set_required_regs;
extern int spi_flash_common_write_status_8b_wrsr;
extern int fgets;
extern int efuse_hal_get_major_chip_version;
extern int __wrap_mkdir;
extern int systimer_ticks_to_us;
extern int esp_cache_err_get_panic_info;
extern int _lock_acquire;
extern int close;
extern int gpio_config;
extern int mld6_input;
extern int gpio_func_sel;
extern int g_startup_time;
extern int spicommon_periph_free;
extern int esp_psram_impl_get_available_size;
extern int tlsf_create;
extern int nd6_find_route;
extern int sys_arch_sem_wait;
extern int esp_flash_chip_winbond;
extern int __action_table;
extern int gdma_new_ahb_channel;
extern int __ssvfiscanf_r;
extern int strchr;
extern int esp_vfs_rewinddir;
extern int spi_flash_mmap;
extern int ip4addr_ntoa;
extern int ipaddr_addr;
extern int bootloader_flash_reset_chip;
extern int xQueueCreateCountingSemaphoreStatic;
extern int netif_list;
extern int __match;
extern int esp_isr_names;
extern int _ZN17__eh_globals_init7_S_initE;
extern int tcp_netif_ip_addr_changed;
extern int xTaskGetCurrentTaskHandle;
extern int esp_log_default_level;
extern int memp_TCP_PCB_LISTEN;
extern int spi_flash_hal_resume;
extern int gpio_iomux_input;
extern int mspi_timing_flash_config_clear_tuning_regs;
extern int esp_flash_chip_generic;
extern int spi_flash_chip_generic_probe;
extern int __chclass;
extern int rtc_isr_register;
extern int lwip_netconn_do_listen;
extern int vQueueDelete;
extern int esp_partition_next;
extern int calloc;
extern int spi_flash_chip_generic_timeout;
extern int esp_mprot_set_prot;
extern int esp_psram_check_ptr_addr;
extern int memspi_host_erase_chip;
extern int panic_soc_check_pseudo_cause;
extern int esp_vfs_register_fs;
extern int __atomic_s32c1i_fetch_and_4;
extern int __sfp_lock_release;
extern int esp_elf_deinit;
extern int ip_addr_any_type;
extern int icmp6_input;
extern int tcp_active_pcbs_changed;
extern int cache_hal_invalidate_addr;
extern int xQueueGenericSendFromISR;
extern int esp_clk_cpu_freq;
extern int rtc_time_us_to_slowclk;
extern int strncmp;
extern int spicommon_cs_free_io;
extern int heap_caps_get_largest_free_block;
extern int nd6_input;
extern int esp_mmu_map_get_max_consecutive_free_block_size;
extern int xRingbufferGetCurFreeSize;
extern int memp_TCP_PCB;
extern int atexit;
extern int pbuf_copy;
extern int gdma_hal_disconnect_peri;
extern int __wrap_fopen;
extern int esp_psram_extram_add_to_heap_allocator;
extern int xTaskCreatePinnedToCore;
extern int _rename_r;
extern int lwip_netconn_do_bind;
extern int tcp_shutdown;
extern int esp_crosscore_int_send_yield;
extern int multi_heap_get_info;
extern int _fclose_r;
extern int fputc;
extern int tlsf_malloc;
extern int rtc_clk_fast_src_set;
extern int strrchr;
extern int tcp_tmr;
extern int rom_flash_chip_dummy_hpm;
extern int stat;
extern int _vprintf_r;
extern int esp_ipc_isr_port_init;
extern int __pow5mult;
extern int gdma_ahb_hal_init;
extern int gpio_pullup_dis;
extern int uart_vfs_dev_register;
extern int scalbn;
extern int spi_flash_common_read_status_8b_rdsr2;
extern int __lo0bits;
extern int netif_loop_output;
extern int valloc;
extern int uart_set_baudrate;
extern int _findenv;
extern int multi_heap_set_lock;
extern int netconn_recv_udp_raw_netbuf_flags;
extern int tcp_accept;
extern int spi_bus_lock_touch;
extern int esp_libc_include_getentropy_impl;
extern int spi_flash_enable_high_performance_mode;
extern int netconn_alloc;
extern int _printf_r;
extern int strerror;
extern int lwip_netconn_do_connect;
extern int free;
extern int lwip_setsockopt;
extern int spi_bus_lock_unregister_dev;
extern int _vfiprintf_r;
extern int esp_rom_efuse_get_opiconfig;
extern int tcpip_send_msg_wait_sem;
extern int __kernel_tan;
extern int xSemaphoreCreateGenericWithCaps;
extern int spi_device_acquire_bus;
extern int esp_sync_timekeeping_timers;
extern int __swhatbuf_r;
extern int esp_flash_app_enable_os_functions;
extern int cache_hal_freeze;
extern int mem_free;
extern int tcp_connect;
extern int esp_intr_disable_source;
extern int pbuf_add_header_force;
extern int netconn_free;
extern int my_display_init;
extern int panic_print_registers;
extern int prefix_list;
extern int ip4_output_if_opt;
extern int strlcat;
extern int esp_chip_info;
extern int console_tcdrain;
extern int vPortYieldOtherCore;
extern int pthread_internal_local_storage_destructor_callback;
extern int memp_NETBUF;
extern int esp_vfs_console_register;
extern int netconn_recv_tcp_pbuf_flags;
extern int panic_get_cause;
extern int _open_r;
extern int spi_flash_op_block_func;
extern int spi_flash_hal_erase_block;
extern int esp_psram_impl_get_cs_io;
extern int mspi_timing_enter_high_speed_mode;
extern int periph_rcc_exit;
extern int spi_flash_chip_gd_get_io_mode;
extern int icmp6_dest_unreach;
extern int _tzset_unlocked;
extern int esp_register_freertos_tick_hook_for_cpu;
extern int _close_r;
extern int __wrap_getcwd;
extern int xQueueCreateMutexStatic;
extern int __lock___sinit_recursive_mutex;
extern int fputs;
extern int lwip_bind;
extern int pvTaskGetThreadLocalStoragePointer;
extern int vprintf;
extern int mspi_timing_psram_tuning;
extern int console_open;
extern int brownout_hal_config;
extern int _Level2Vector;
extern int esp_vfs_write;
extern int sys_now;
extern int __atomic_fetch_and_8;
extern int esp_log_level_get_timeout;
extern int panic_print_dec;
extern int lwip_connect;
extern int include_esp_phy_override;
extern int raw_netif_ip_addr_changed;
extern int xPortcheckValidStackMem;
extern int efuse_hal_chip_revision;
extern int esp_flash_noos_functions;
extern int __sfvwrite_r;
extern int abort;
extern int igmp_leavegroup;
extern int esp_task_wdt_print_triggered_tasks;
extern int memspi_host_init_pointers;
extern int _xt_coproc_owner_sa_lock;
extern int rtc_isr_noniram_enable;
extern int tcp_err;
extern int esp_flash_chip_gd;
extern int __adddf3;
extern int esp_mmu_map_init;
extern int gpio_sleep_set_pull_mode;
extern int uart_hal_rxfifo_rst;
extern int cache_hal_resume;
extern int _system_r;
extern int __locale_mb_cur_max;
extern int gdma_hal_start_with_desc;
extern int __divdi3;
extern int efuse_hal_get_disable_wafer_version_major;
extern int esp_vfs_fsync;
extern int esp_vfs_stat;
extern int strtoll;
extern int esp_restart;
extern int tcp_new_ip_type;
extern int _unlink_r;
extern int spi_hal_cal_clock_conf;
extern int __atomic_fetch_and_4;
extern int __umoddi3;
extern int vPortSetupTimer;
extern int icmp_dest_unreach;
extern int vTaskInternalSetTimeOutState;
extern int uxListRemove;
extern int esp_psram_init;
extern int spi_flash_chip_gd_detect_size;
extern int usb_serial_jtag_write_bytes;
extern int xt_set_interrupt_handler;
extern int spi_flash_chip_th_probe;
extern int __hi0bits;
extern int __sglue;
extern int esp_clk_slowclk_cal_get;
extern int esp_newlib_time_init;
extern int esp_newlib_locks_init;
extern int memspi_host_set_write_protect;
extern int gdma_hal_deinit;
extern int getchar;
extern int esp_mprot_set_split_addr;
extern int esp_elf_malloc;
extern int opterr;
extern int pbuf_add_header;
extern int rtc_clk_32k_enable_external;
extern int g_dig_dbias_pvt_non_240m;
extern int esp_flash_init_default_chip;
extern int s_spinlock;
extern int esp_intr_get_cpu;
extern int heap_caps_aligned_calloc;
extern int spi_flash_chip_gd_set_io_mode;
extern int g_spi_lock_main_flash_dev;
extern int spi_timing_get_flash_timing_param;
extern int pthread_exit;
extern int lwip_htons;
extern int spi_flash_chip_generic_set_write_protect;
extern int spi_hal_deinit;
extern int usb_serial_jtag_write_ready;
extern int fflush;
extern int pthread_include_pthread_cond_var_impl;
extern int __sfp;
extern int __sinit;
extern int retrans_timer;
extern int netif_invoke_ext_callback;
extern int _esp_error_check_failed;
extern int xTaskGetTickCountFromISR;
extern int netconn_disconnect;
extern int uart_hal_write_txfifo;
extern int _lock_try_acquire_recursive;
extern int div;
extern int ledc_set_duty;
extern int __atexit;
extern int tlsf_alloc_overhead;
extern int __fixdfsi;
extern int rtc_io_num_map;
extern int sys_arch_mbox_fetch;
extern int clk_hal_cpu_get_freq_hz;
extern int soc_memory_types;
extern int heap_caps_calloc_base;
extern int cache_hal_disable;
extern int __eqdf2;
extern int esp_apb_backup_dma_lock_init;
extern int _xt_medint2_exit;
extern int strtof_l;
extern int spi_flash_set_rom_required_regs;
extern int esp_cpu_intr_get_desc;
extern int esp_lcd_panel_invert_color;
extern int _write_r;
extern int _fprintf_r;
extern int _exit;
extern int esp_cpu_wait_for_intr;
extern int fprintf;
extern int tlsf_size;
extern int __retarget_lock_try_acquire_recursive;
extern int spicommon_dma_chan_free;
extern int __multiply;
extern int esp_cache_get_alignment;
extern int _frxt_dispatch;
extern int tcp_listen_with_backlog_and_err;
extern int _rewind_r;
extern int vTaskGetSnapshot;
extern int spi_hal_hw_prepare_tx;
extern int xRingbufferCreate;
extern int esp_deep_sleep_wakeup_io_reset;
extern int udp_new;
extern int netconn_listen_with_backlog;
extern int pthread_include_pthread_rwlock_impl;
extern int memp_SYS_TIMEOUT;
extern int esp_cpu_configure_region_protection;
extern int esp_startup_start_app_other_cores;
extern int esp_mprot_ll_err_to_esp_err;
extern int __atomic_s32c1i_fetch_or_4;
extern int xTaskGetCoreID;
extern int lwip_recvfrom;
extern int pthread_getspecific;
extern int memcpy;
extern int uart_hal_get_data_bit_num;
extern int vTaskPrioritySet;
extern int ip_input;
extern int vRingbufferReturnItem;
extern int __mcmp;
extern int multi_heap_get_info_impl;
extern int esp_psram_impl_enable;
extern int xQueueGetMutexHolder;
extern int _vfprintf_r;
extern int vSystimerSetup;
extern int ip4_frag;
extern int esp_mmu_vaddr_to_paddr;
extern int vPortFree;
extern int spi_flash_chip_generic_page_program;
extern int tcp_output;
extern int esp_intr_alloc_intrstatus;
extern int xthal_window_spill;
extern int __swbuf;
extern int esp_libc_locks_init;
extern int __floatdidf;
extern int g_startup_fn;
extern int _user_strerror;
extern int frexp;
extern int uart_get_parity;
extern int netif_default;
extern int lwip_netconn_do_close;
extern int spi_hal_enable_data_line;
extern int __assert_func;
extern int esp_crosscore_int_send_print_backtrace;
extern int usleep;
extern int pbuf_alloc;
extern int __retarget_lock_acquire;
extern int esp_vfs_close;
extern int udp_disconnect;
extern int mmu_hal_check_valid_ext_vaddr_region;
extern int inet_chksum_pbuf;
extern int rtc_io_number_get;
extern int call_start_cpu0;
extern int multi_heap_malloc_impl;
extern int multi_heap_aligned_alloc_impl_offs;
extern int esp_ota_get_running_partition;
extern int esp_sleep_config_gpio_isolate;
extern int esp_clk_xtal_freq;
extern int spi_flash_hal_gpspi_check_status;
extern int esp_ipc_isr_end_fl;
extern int pthread_include_pthread_impl;
extern int gdma_hal_enable_intr;
extern int netconn_connect;
extern int bootloader_flash_execute_command_common;
extern int __retarget_lock_init_recursive;
extern int tcp_zero_window_probe;
extern int tcp_rexmit_rto_commit;
extern int _gettimeofday_r;
extern int xPortInterruptedFromISRContext;
extern int spi_flash_chip_gd_get_caps;
extern int esp_libc_include_heap_impl;
extern int memp_NETDB;
extern int __fp_unlock_all;
extern int udp_input;
extern int usb_serial_jtag_set_select_notif_callback;
extern int esp_gpio_revoke;
extern int console_tcgetattr;
extern int gdma_new_channel;
extern int g_flash_guard_default_ops;
extern int esp_ipc_isr_start_fl;
extern int esp_ipc_call_blocking;
extern int gpio_hal_iomux_in;
extern int esp_lcd_panel_swap_xy;
extern int xTaskGenericNotify;
extern int cache_hal_get_cache_line_size;
extern int _xt_coproc_init;
extern int esp_clk_tree_src_get_freq_hz;
extern int _tzname;
extern int mktime;
extern int spi_flash_encryption_hal_destroy;
extern int esp_log_impl_lock_timeout;
extern int esp_panic_handler_disable_timg_wdts;
extern int g_dig_dbias_pvt_240m;
extern int esp_vfs_include_console_register;
extern int esp_partition_find;
extern int esp_mprot_set_split_addr_lock;
extern int _xt_user_exit;
extern int qsort;
extern int _sungetc_r;
extern int _xt_context_save;
extern int esp_vfs_dev_usb_serial_jtag_register;
extern int tcp_seg_free;
extern int netconn_accept;
extern int esp_setup_newlib_syscalls;
extern int spi_flash_encryption_hal_done;
extern int _ftell_r;
extern int panicHandler;
extern int rtc_time_get;
extern int esp_partition_main_flash_region_safe;
extern int sys_mutex_lock;
extern int xt_highint4;
extern int rtc_clk_slow_src_get;
extern int lwip_sendto;
extern int utoa;
extern int rtc_io_desc;
extern int rtc_clk_cpu_freq_mhz_to_config;
extern int ledc_hal_set_fade_param;
extern int _tzset_r;
extern int esp_libc_include_pthread_impl;
extern int tcp_input;
extern int sys_mbox_new;
extern int spi_flash_needs_reset_check;
extern int mpu_hal_set_region_access;
extern int gpio_output_enable;
extern int fstat;
extern int strtoll_l;
extern int gpio_output_disable;
extern int _getc_r;
extern int ip4_route;
extern int vRingbufferGetInfo;
extern int uart_hal_set_stop_bits;
extern int __ltdf2;
extern int systimer_hal_counter_value_advance;
extern int __wrap_chdir;
extern int __udivdi3;
extern int lwip_send;
extern int spi_flash_chip_generic_get_caps;
extern int _xt_coproc_owner_sa;
extern int esp_psram_impl_get_physical_size;
extern int esp_system_abort;
extern int spi_flash_chip_mxic_opi_erase_sector;
extern int esp_vfs_access;
extern int esp_vApplicationTickHook;
extern int _ftello_r;
extern int spi_flash_chip_winbond_probe;
extern int ip4_addr_isbroadcast_u32;
extern int esp_mprot_get_pms_lock;
extern int soc_get_available_memory_region_max_count;
extern int netconn_new_with_proto_and_callback;
extern int inet_chksum_pseudo;
extern int _sniprintf_r;
extern int __register_exitproc;
extern int console_close;
extern int panic_restart;
extern int esp_register_shutdown_handler;
extern int neighbor_cache;
extern int esp_panic_handler;
extern int netbuf_free;
extern int xQueueGiveFromISR;
extern int tcp_pcb_remove;
extern int xTaskRemoveFromEventList;
extern int __bswapsi2;
extern int __gedf2;
extern int _fstat_r;
extern int abs;
extern int spi_bus_deinit_lock;
extern int strtok;
extern int rewinddir;
extern int closedir;
extern int udp_pcbs;
extern int fileno;
extern int environ;
extern int startup_resume_other_cores;
extern int esp_intr_free;
extern int tcp_rexmit_fast;
extern int esp_vApplicationIdleHook;
extern int _mprec_log10;
extern int esp_newlib_init;
extern int multi_heap_free;
extern int _sbrk_r;
extern int esp_vfs_dev_uart_register;
extern int tcp_poll;
extern int putchar;
extern int ip4addr_ntoa_r;
extern int __getopt_long_only_r;
extern int netconn_write_partly;
extern int destination_cache;
extern int g_panic_abort_details;
extern int igmp_tmr;
extern int __gettzinfo;
extern int _puts_r;
extern int mspi_timing_config_psram_write_data;
extern int spi_flash_common_read_status_16b_rdsr_rdsr2;
extern int esp_log_impl_lock;
extern int tcp_fasttmr;
extern int xPortStartScheduler;
extern int gpio_sleep_sel_en;
extern int spi_bus_lock_bg_clear_req;
extern int heap_caps_enable_nonos_stack_heaps;
extern int __copybits;
extern int spi_flash_chip_mxic_opi_get_io_mode;
extern int esp_efuse_check_errors;
extern int __ssprint_r;
extern int __s2b;
extern int memset;
extern int spi_flash_chip_winbond_erase_block;
extern int esp_timer_impl_init_system_time;
extern int xQueueGenericReset;
extern int spi_bus_lock_acquire_end;
extern int rtc_clk_slow_src_set;
extern int mspi_timing_get_psram_tuning_configs;
extern int clk_hal_xtal_get_freq_mhz;
extern int __kernel_sin;
extern int ip_data;
extern int gdma_ahb_hal_enable_burst;
extern int sys_sem_signal;
extern int udp_connect;
extern int vTaskPriorityDisinheritAfterTimeout;
extern int icmp6_param_problem;
extern int getenv;
extern int ledc_bind_channel_timer;
extern int lwip_netconn_do_disconnect;
extern int vfs_include_syscalls_impl;
extern int memspi_host_write_data_slicer;
extern int _localeconv_r;
extern int multi_heap_internal_lock;
extern int port_switch_flag;
extern int spi_flash_hal_gpspi_common_command;
extern int __lock___tz_mutex;
extern int gpio_od_disable;
extern int regi2c_ctrl_write_reg_mask;
extern int __lock___env_recursive_mutex;
extern int app_main;
extern int esp_mmu_unmap;
extern int esp_elf_request;
extern int _frxt_setup_switch;
extern int strtok_r;
extern int spi_flash_chip_mxic_opi_set_write_protect;
extern int mmu_hal_pages_to_bytes;
extern int spi_device_polling_end;
extern int uart_set_stop_bits;
extern int xTaskCreateStaticPinnedToCore;
extern int pbuf_ref;
extern int esp_vfs_mkdir;
extern int netconn_delete;
extern int netconn_send;
extern int _frxt_int_enter;
extern int mspi_timing_psram_set_best_tuning_config;
extern int __mdiff;
extern int spi_flash_hal_poll_cmd_done;
extern int rtc_get_xtal;
extern int GPIO_PIN_MUX_REG;
extern int igmp_joingroup;
extern int multi_heap_register_impl;
extern int getcwd;
extern int lwip_netconn_do_getaddr;
extern int panic_get_address;
extern int systimer_us_to_ticks;
extern int mspi_timing_config_set_psram_clock;
extern int esp_cpu_stall;
extern int esp_elf_arch_relocate;
extern int mld6_lookfor_group;
extern int esp_flash_chip_issi;
extern int spi_bus_get_attr;
extern int esp_opiflash_init;
extern int esp_vfs_rename;
extern int esp_task_wdt_impl_timer_feed;
extern int uart_get_word_length;
extern int heap_caps_malloc;
extern int esp_psram_io_get_cs_io;
extern int esp_mprot_monitor_clear_intr;
extern int uart_wait_tx_done;
extern int spi_bus_init_lock;
extern int _NMIExceptionVector;
extern int systimer_hal_enable_alarm_int;
extern int ledc_timer_rst;
extern int spi_flash_chip_mxic_opi_get_write_protect;
extern int spi_flash_chip_generic_set_io_mode;
extern int _svfiprintf_r;
extern int gdma_ahb_hal_set_priority;
extern int spi_hal_clear_intr_mask;
extern int tcp_close;
extern int __atomic_compare_exchange_4;
extern int __hexnan;
extern int __moddi3;
extern int __i2b;
extern int _fgets_r;
extern int fabs;
extern int rtc_isr_noniram_disable;
extern int _link_r;
extern int spi_flash_chip_mxic_opi_detect_size;
extern int bootloader_flash_update_id;
extern int _lock_release;
extern int tcp_bind;
extern int strtod_l;
extern int esp_clk_slowclk_cal_set;
extern int gdma_ahb_hal_disconnect_peri;
extern int gpio_input_enable;
extern int _lock_close;
extern int port_interruptNesting;
extern int esp_mmu_paddr_find_caps;
extern int __getopt_r;
extern int gdma_ahb_hal_clear_intr;
extern int esp_elf_print_ehdr;
extern int systimer_hal_set_alarm_period;
extern int pbuf_free;
extern int spi_flash_hal_supports_direct_read;
extern int multi_heap_free_impl;
extern int __sread;
extern int spi_flash_restore_cache;
extern int __bswapdi2;
extern int tcp_rst;
extern int esp_vfs_select_triggered_isr;
extern int periph_rcc_acquire_exit;
extern int vTaskStartScheduler;
extern int spi_flash_chip_generic_yield;
extern int strtol_l;
extern int esp_elf_init;
extern int esp_timer_impl_get_time;
extern int _xt_coproc_restorecs;
extern int esp_ipc_isr_init;
extern int pvPortMalloc;
extern int __ascii_wctomb;
extern int _perror_r;
extern int __retarget_lock_close;
extern int esp_cache_err_int_init;
extern int spi_flash_chip_gd_probe;
extern int esp_task_wdt_impl_timer_allocate;
extern int __lock___sfp_recursive_mutex;
extern int tcp_recved;
extern int __lshift;
extern int _lock_init;
extern int __ieee754_sqrt;
extern int panic_print_char;
extern int pthread_key_create;
extern int tlsf_block_size;
extern int spi_flash_op_unlock;
extern int __wrap_opendir;
extern int mmu_hal_paddr_to_vaddr;
extern int tcp_active_pcbs;
extern int lwip_socket;
extern int gdma_ahb_hal_reset;
extern int nd6_reachability_hint;
extern int uart_hal_set_parity;
extern int esp_startup_start_app;
extern int spi_flash_hal_set_write_protect;
extern int netif_poll;
extern int spi_flash_chip_generic_write;
extern int spi_flash_hal_erase_sector;
extern int esp_log_early_timestamp;
extern int xTaskGetTickCount;
extern int esp_task_wdt_impl_timeout_triggered;
extern int spi_flash_hal_disable_auto_suspend_mode;
extern int esp_vfs_seekdir;
extern int uxTaskPriorityGet;
extern int heap_caps_match;
extern int xTaskIncrementTickOtherCores;
extern int perror;
extern int periph_rtc_dig_clk8m_get_freq;
extern int systimer_hal_init;
extern int sin;
extern int spi_hal_cal_timing;
extern int multi_heap_aligned_free;
extern int _malloc_r;
extern int esp_task_wdt_add;
extern int _xt_coproc_release;
extern int esp_timer_get_time;
extern int spi_flash_hal_program_page;
extern int port_uxCriticalNesting;
extern int esp_partition_get;
extern int __global_locale_ptr;
extern int ip4_output_if_opt_src;
extern int pbuf_copy_partial;
extern int longjmp;
extern int gdma_ahb_hal_set_strategy;
extern int esp_lcd_panel_mirror;
extern int esp_vfs_readdir;
extern int ledc_update_duty;
extern int gpio_sleep_set_direction;
extern int spi_hal_setup_device;
extern int spicommon_bus_initialize_io;
extern int esp_libc_init_funcs;
extern int xQueueGenericCreate;
extern int __d2b;
extern int mmu_hal_map_region;
extern int pbuf_remove_header;
extern int spi_flash_hal_setup_auto_resume_mode;
extern int rtc_dig_clk8m_disable;
extern int ip6_output_if_src;
extern int xQueueGenericGetStaticBuffers;
extern int esp_cache_unfreeze_caches_enable_interrupts;
extern int gdma_reset;
extern int _getchar_r;
extern int tcp_arg;
extern int __sflags;
extern int lwip_standard_chksum;
extern int nanf;
extern int sys_thread_sem_get;
extern int tcp_abort;
extern int strncasecmp;
extern int strcpy;
extern int spi_flash_hal_erase_chip;
extern int lwip_netconn_do_accepted;
extern int spi_flash_chip_boya_get_caps;
extern int ip_addr_broadcast;
extern int esp_ipc_isr_stall_abort;
extern int console_access;
extern int read;
extern int esp_reent_init;
extern int _wctomb_r;
extern int tlsf_realloc;
extern int uart_hal_get_sclk;
extern int _snprintf_r;
extern int _siscanf_r;
extern int spi_flash_chip_mxic_opi_erase_chip;
extern int spi_flash_common_set_io_mode;
extern int __mprec_bigtens;
extern int spi_flash_hal_setup_read_suspend;
extern int mspi_timing_flash_config_set_tuning_regs;
extern int spi_bus_lock_bg_exit;
extern int spicommon_periph_claim;
extern int udp_netif_ip_addr_changed;
extern int _putchar_r;
extern int regi2c_ctrl_read_reg_mask;
extern int esp_restart_noos;
extern int rtc_clk_cal;
extern int __ieee754_rem_pio2;
extern int raw_bind_netif;
extern int esp_vfs_fcntl_r;
extern int clk_hal_soc_root_get_freq_mhz;
extern int esp_task_wdt_reset;
extern int pbuf_free_ooseq_pending;
extern int _Bfree;
extern int lwip_listen;
extern int tcp_process_refused_data;
extern int _sfread_r;
extern int lwip_accept;
extern int vListInitialise;
extern int lwip_setsockopt_impl_ext;
extern int __tzcalc_limits;
extern int heap_caps_malloc_default;
extern int g_twdt_isr;
extern int call_start_cpu1;
extern int raw_sendto_if_src;
extern int ip6_select_source_address;
extern int usb_serial_jtag_vfs_include_dev_init;
extern int esp_cache_suspend_ext_mem_cache;
extern int __default_global_locale;
extern int spi_hal_setup_trans;
extern int esp_mprot_set_pms_area;
extern int esp_intr_alloc;
extern int raw_new;
extern int esp_mmu_map_reserve_block_with_caps;
extern int esp_mprot_set_monitor_lock;
extern int pxCurrentTCBs;
extern int ip4_output_if_src;
extern int uart_flush_input;
extern int uart_vfs_include_dev_init;
extern int spi_bus_get_max_transaction_len;
extern int esp_time_impl_get_time;
extern int _C_time_locale;
extern int xRingbufferReceiveUpToFromISR;
extern int xTaskGetIdleTaskHandleForCore;
extern int esp_flash_default_chip;
extern int bootloader_read_flash_id;
extern int gdma_ahb_hal_connect_peri;
extern int spi_flash_enable_cache;
extern int gdma_del_channel;
extern int ledc_hal_ls_channel_update;
extern int esp_intr_enable;
extern int efuse_hal_flash_encryption_enabled;
extern int panic_print_hex;
extern int SysTickIsrHandler;
extern int strcat;
extern int __lock___arc4random_mutex;
extern int __atomic_s32c1i_compare_exchange_1;
extern int spi_flash_op_lock;
extern int heap_caps_get_info;
extern int nd6_get_destination_mtu;
extern int vPortYieldFromInt;
extern int __ssrefill_r;
extern int spi_flash_guard_set;
extern int rtc_clk_cal_internal;
extern int esp_flash_chip_mxic;
extern int mmu_hal_unmap_region;
extern int lwip_netconn_do_delconn;
extern int rtc_clk_freq_to_period;
extern int rtc_clk_8md256_enabled;
extern int spi_flash_chip_winbond_read;
extern int __muldf3;
extern int mspi_timing_psram_select_best_tuning_config;
extern int heap_caps_malloc_prefer;
extern int esp_system_get_time;
extern int sys_mutex_unlock;
extern int xRingbufferReceive;
extern int udp_bind;
extern int uart_set_select_notif_callback;
extern int _Level4Vector;
extern int esp_flash_registered_chips;
extern int strtoul_l;
extern int xt_ints_off;
extern int tcp_backlog_accepted;
extern int spi_bus_lock_bg_entry;
extern int _lock_release_recursive;
extern int _daylight;
extern int spi_flash_common_write_status_8b_wrsr2;
extern int multi_heap_aligned_alloc_offs;
extern int offset_pxEndOfStack;
extern int esp_mspi_32bit_address_flash_feature_check;
extern int optind;
extern int esp_timer_init_include_func;
extern int ledc_periph_signal;
extern int __env_unlock;
extern int rtc_clk_32k_enable;
extern int __subdf3;
extern int ledc_timer_resume;
extern int panic_print_str;
extern int __sclose;
extern int tcp_free;
extern int rtc_clk_bbpll_remove_consumer;
extern int spi_device_release_bus;
extern int spi_bus_lock_bg_check_dev_acq;
extern int bzero;
extern int spi_hal_sct_set_conf_bits_len;
extern int _timezone;
extern int __mprec_tens;
extern int netconn_err;
extern int tcp_input_pcb;
extern int sys_thread_sem_init;
extern int esp_log_util_set_cache_enabled_cb;
extern int raw_send;
extern int esp_vfs_select_triggered;
extern int __truncdfsf2;
extern int tcsetattr;
extern int esp_log_timestamp;
extern int esp_clk_tree_rc_fast_get_freq_hz;
extern int esp_efuse_startup_include_func;
extern int spi_flash_enable_interrupts_caches_and_other_cpu;
extern int lwip_netconn_do_write;
extern int regi2c_ctrl_write_reg;
extern int esp_cache_resume_ext_mem_cache;
extern int pbuf_cat;
extern int ulTaskGenericNotifyTake;
extern int multi_heap_realloc_impl;
extern int mspi_timing_config_psram_read_data;
extern int tcp_free_ooseq;
extern int spi_flash_chip_issi_set_io_mode;
extern int pbuf_clen;
extern int __kernel_cos;
extern int esp_lcd_panel_reset;
extern int gdma_ahb_hal_set_burst_size;
extern int esp_vfs_usb_serial_jtag_use_driver;
extern int vPortTCBPreDeleteHook;
extern int spi_flash_chip_generic_config_host_io_mode;
extern int atoi;
extern int gpio_intr_enable;
extern int _fputs_r;
extern int gdma_hal_enable_access_encrypt_mem;
extern int spicommon_irqsource_for_host;
extern int gdma_ahb_hal_read_intr_status;
extern int rtc_clk_32k_disable_external;
extern int spi_flash_munmap;
extern int usb_serial_jtag_driver_install;
extern int soc_get_available_memory_regions;
extern int esp_pthread_set_cfg;
extern int tcp_pcb_purge;
extern int mspi_timing_psram_init;
extern int __fixunsdfsi;
extern int esp_intr_alloc_intrstatus_bind;
extern int prvTaskCreateDynamicPinnedToCoreWithCaps;
extern int strftime;
extern int spi_flash_hal_check_status;
extern int igmp_report_groups;
extern int ets_isr_mask;
extern int _sprintf_r;
extern int esp_deregister_freertos_idle_hook_for_cpu;
extern int spi_hal_get_intr_mask;
extern int usb_serial_jtag_vfs_register;
extern int spi_flash_chip_mxic_opi_write;
extern int opendir;
extern int spi_flash_hal_gpspi_configure_host_io_mode;
extern int esp_rtc_get_time_us;
extern int optopt;
extern int __cxx_init_dummy;
extern int tcp_eff_send_mss_netif;
extern int tcdrain;
extern int pthread_setspecific;
extern int clk_hal_lp_slow_get_freq_hz;
extern int __wrap__Unwind_DeleteException;
extern int spi_flash_hal_gpspi_supports_direct_write;
extern int spi_flash_hal_device_config;
extern int vTaskSuspend;
extern int xthal_set_intclear;
extern int fwrite;
extern int esp_cache_err_get_cpuid;
extern int esp_libc_include_reent_syscalls_impl;
extern int _svfprintf_r;
extern int esp_rom_install_uart_printf;
extern int spi_flash_hal_configure_host_io_mode;
extern int heap_caps_init;
extern int spicommon_cs_initialize;
extern int raw_new_ip_type;
extern int esp_cache_freeze_caches_disable_interrupts;
extern int tlsf_get_pool;
extern int puts;
extern int pbuf_clone;
extern int tlsf_pool_overhead;
extern int udp_new_ip_type;
extern int vsnprintf;
extern int rtc_init;
extern int spi_flash_chip_xmic_opi_config_host_io_mode;
extern int spi_flash_chip_xmic_opi_set_io_mode;
extern int __retarget_lock_acquire_recursive;
extern int vPortExitCritical;
extern int strlcpy;
extern int getopt_long;
extern int fopen;
extern int port_xSchedulerRunning;
extern int _fread_r;
extern int systimer_hal_counter_can_stall_by_cpu;
extern int __errno;
extern int multi_heap_get_allocated_size;
extern int spi_flash_chip_generic_erase_block;
extern int tcp_rst_netif;
extern int _strtoul_r;
extern int _sscanf_r;
extern int do_multicore_settings;
extern int gdma_ahb_hal_enable_intr;
extern int spi_flash_timing_is_tuned;
extern int lwip_netconn_do_send;
extern int getc;
extern int Cache_Freeze_DCache_Enable;
extern int chdir;
extern int esp_ipc_call_nonblocking;
extern int strtoull_l;
extern int multi_heap_get_allocated_size_impl;
extern int tcp_segs_free;
extern int spi_flash_chip_generic_read;
extern int esp_partition_is_flash_region_writable;
extern int spi_bus_lock_get_acquiring_dev;
extern int xRingbufferReceiveUpTo;
extern int esp_lcd_panel_draw_bitmap;
extern int mkdir;
extern int io_mux_force_disable_lp_io_clock;
extern int gdma_hal_set_burst_size;
extern int __divdf3;
extern int getopt_long_only;
extern int netif_ip6_addr_set_state;
extern int igmp_joingroup_netif;
extern int tcp_send_fin;
extern int siscanf;
extern int esp_security_init_include_impl;
extern int tcp_rexmit_rto_prepare;
extern int spi_device_polling_start;
extern int putc;
extern int esp_log;
extern int s_microseconds_offset;
extern int _strtoull_r;
extern int tcp_bound_pcbs;
extern int esp_lcd_panel_disp_on_off;
extern int esp_clk_tree_lp_fast_get_freq_hz;
extern int memp_IGMP_GROUP;
extern int gdma_ahb_hal_stop;
extern int gdma_ahb_hal_get_intr_status_reg;
extern int tcp_ticks;
extern int tcp_trigger_input_pcb_close;
extern int __getopt_long_r;
extern int systimer_hal_set_tick_rate_ops;
extern int snprintf;
extern int mspi_timing_config_get_flash_extra_dummy;
extern int cache_hal_unfreeze;
extern int tcp_close_ext;
extern int tan;
extern int esp_mprot_get_monitor_en;
extern int spi_flash_common_read_status_8b_rdsr;
extern int mspi_timing_set_pin_drive_strength;
extern int raw_connect;
extern int spi_hal_user_start;
extern int tcp_recv;
extern int gdma_ahb_hal_start_with_desc;
extern int __month_lengths;
extern int clk_hal_apb_get_freq_hz;
extern int panic_set_address;
extern int __lock___atexit_recursive_mutex;
extern int esp_log_cache_add;
extern int esp_timer_impl_early_init;
extern int fcntl;
extern int multi_heap_internal_unlock;
extern int _lseek_r;
extern int gpio_pullup_en;
extern int _strtoll_r;
extern int spi_flash_chip_list_check;
extern int memp_ND6_QUEUE;
extern int access;
extern int spi_flash_chip_generic_read_unique_id;
extern int spi_hal_config_io_default_level;
extern int esp_panic_handler_increment_entry_count;
extern int _findenv_r;
extern int xTaskCheckForTimeOut;
extern int __retarget_lock_try_acquire;
extern int esp_cpu_set_breakpoint;
extern int _xt_context_restore;
extern int pthread_attr_init;
extern int vTaskGenericNotifyGiveFromISR;
extern int usb_serial_jtag_connection_monitor_include;
extern int tcpip_api_call;
extern int vRingbufferReturnItemFromISR;
extern int xTaskIncrementTick;
extern int cache_hal_suspend;
extern int tcp_enqueue_flags;
extern int _raise_r;
extern int spi_flash_chip_mxic_opi_get_caps;
extern int vSemaphoreDeleteWithCaps;
extern int __ledf2;
extern int gpio_set_level;
extern int esp_heap_adjust_alignment_to_hw;
extern int esp_gpio_reserve;
extern int usb_serial_jtag_is_driver_installed;
extern int _getenv_r;
extern int esp_backtrace_get_next_frame;
extern int mspi_timing_config_psram_set_tuning_regs;
extern int spi_flash_hal_gpspi_supports_direct_read;
extern int _putc1;
extern int __floatsidf;
extern int _fopen_r;
extern int esp_system_reset_modules_on_exit;
extern int mspi_timing_change_speed_mode_cache_safe;
extern int Xthal_intlevel;
extern int cache_hal_enable;
extern int esp_mmu_map;
extern int spi_flash_chip_mxic_opi_page_program;
extern int heap_caps_aligned_alloc_base;
extern int _xt_panic;
extern int __submore;
extern int panic_arch_fill_info;
extern int int_wdt_cpu1_ticked;
extern int spi_bus_lock_get_dev_id;
extern int heap_caps_aligned_alloc;
extern int gpio_iomux_output;
extern int rtc_clk_slow_freq_get_hz;
extern int netbuf_delete;
extern int gdma_connect;
extern int mld6_tmr;
extern int __gethex;
extern int esp_mprot_get_default_main_split_addr;
extern int offset_cpsa;
extern int memchr;
extern int xQueueGenericSend;
extern int __b2d;
extern int rtc_gpio_is_valid_gpio;
extern int _ungetc_r;
extern int port_uxOldInterruptState;
extern int _fflush_r;
extern int sys_timeout;
extern int esp_vfs_lseek;
extern int panic_print_backtrace;
extern int spi_flash_chip_mxic_probe;
extern int write;
extern int esp_mprot_set_monitor_en;
extern int strcasecmp;
extern int __ascii_mbtowc;
extern int vfprintf;
extern int esp_flash_chip_boya;
extern int spi_flash_chip_issi_get_caps;
extern int __swbuf_r;
extern int vPortYield;
extern int netconn_write_vectors_partly;
extern int gpio_pulldown_en;
extern int esp_ipc_func;
extern int esp_vfs_link;
extern int pthread_join;
extern int usb_serial_jtag_vfs_use_driver;
extern int __env_lock;
extern int usb_serial_jtag_driver_uninstall;
extern int esp_vfs_closedir;
extern int __hexdig_fun;
extern int esp_log_is_tag_loggable;
extern int esp_cpu_compare_and_set;
extern int spi_bus_lock_bg_request;
extern int ungetc;
extern int cos;
extern int _Level5Vector;
extern int xQueueReceive;
extern int __nedf2;
extern int __ssvfscanf_r;
extern int __swsetup_r;
extern int lwip_netconn_do_recv;
extern int Cache_Freeze_ICache_Enable;
extern int ip6_output_if;
extern int udp_recv;
extern int ip6_route;
extern int uart_get_buffered_data_len;
extern int _fseeko_r;
extern int ilg_int_st_bits;
extern int memspi_host_read_data_slicer;
extern int mspi_timing_flash_tuning;
extern int cache_hal_vaddr_to_cache_level_id;
extern int esp_intr_noniram_disable;
extern int tlsf_free;
extern int spi_flash_hal_common_command;
extern int usb_serial_jtag_wait_tx_done;
extern int _impure_ptr;
extern int __sfp_lock_acquire;
extern int esp_ipc_isr_handler;
extern int heap_caps_malloc_base;
extern int tcpip_callback;
extern int uart_set_word_length;
extern int ip_addr_any;
extern int _mbrtowc_r;
extern int esp_mprot_set_pms_lock;
extern int g_rtc_dbias_pvt_240m;
extern int pthread_create;
extern int _setlocale_r;
extern int mspi_timing_config_get_flash_clock_reg;
extern int pbuf_get_at;
extern int pthread_include_pthread_semaphore_impl;
extern int pvTaskIncrementMutexHeldCount;
extern int lwip_netconn_do_newconn;
extern int gettimeofday;
extern int periph_module_enable;
extern int esp_ptr_executable;
extern int tcp_timer_needed;
extern int spi_bus_lock_set_bg_control;
extern int rtc_clk_8m_enable;
extern int lwip_htonl;
extern int uart_get_selectlock;
extern int time;
extern int mspi_timing_config_psram_prepare_reference_data;
extern int cache_hal_init;
extern int fseek;
extern int esp_cache_unfreeze_ext_mem_cache;
extern int __wrap_stat;
extern int spi_flash_chip_mxic_detect_size;
extern int __srget_r;
extern int vTaskDelay;
extern int spi_flash_chip_mxic_opi_probe;
extern int __atexit0;
extern int spi_flash_hal_read;
extern int gdma_start;
extern int strncpy;
extern int memcmp;
extern int __floatundidf;
extern int fclose;
extern int esp_cache_freeze_ext_mem_cache;
extern int memp_NETCONN;
extern int esp_flash_init_main;
extern int __getreent;
extern int s_cache_hal_init_ctx;
extern int gdma_hal_enable_burst;
extern int netconn_getaddr;
extern int spi_bus_initialize;
extern int spi_flash_set_vendor_required_regs;
extern int esp_rtc_init;
extern int uart_get_stop_bits;
extern int mspi_timing_enter_low_speed_mode;
extern int spi_flash_brownout_need_reset;
extern int heap_caps_realloc_base;
extern int _Balloc;
extern int esp_clk_tree_enable_src;
extern int esp_brownout_init;
extern int esp_app_get_elf_sha256;
extern int spi_flash_chip_mxic_opi_read_id;
extern int strdup;
extern int spi_flash_chip_mxic_opi_read_reg;
extern int uxQueueMessagesWaiting;
extern int xQueueSemaphoreTake;
extern int tlsf_fit_size;
extern int ip6_chksum_pseudo;
extern int xRingbufferSend;
extern int ledc_hal_set_duty_int_part;
extern int esp_time_impl_get_time_since_boot;
extern int __lock___dd_hash_mutex;
extern int _strtod_l;
extern int esp_sleep_gpio_include;
extern int tlsf_add_pool;
extern int esp_vfs_unlink;
extern int rtc_clk_cpu_set_to_default_config;
extern int heap_caps_add_region_with_caps;
extern int __sflush_r;
extern int esp_mprot_get_monitor_lock;
extern int _fseek_r;
extern int memp_malloc;
extern int ip4_output_if;
extern int __retarget_lock_release_recursive;
extern int esp_intr_enable_source;
extern int __fp_lock_all;
extern int systimer_hal_select_alarm_mode;
extern int ip4_input;
extern int ip6_options_add_hbh_ra;
extern int __srget;
extern int ledc_hal_set_duty_start;
extern int memp_free;
extern int esp_mprot_get_pms_area;
extern int memp_TCP_SEG;
extern int spi_flash_chip_gd_suspend_cmd_conf;
extern int lwip_recv;
extern int _strtol_r;
extern int s_keys;
extern int memp_MLD6_GROUP;
extern int _mbtowc_r;
extern int tcpip_try_callback;
extern int panic_abort;
extern int tcp_slowtmr;
extern int efuse_hal_get_minor_chip_version;
extern int multi_heap_malloc;
extern int _Level3Vector;
extern int ets_isr_unmask;
extern int console_tcflush;
extern int gpio_hal_iomux_out;
extern int sys_arch_unprotect;
extern int udp_send;
extern int vTaskSuspendAll;
extern int esp_elf_relocate;
extern int esp_register_freertos_tick_hook;
extern int sys_mbox_post;
extern int xPortCheckValidTCBMem;
extern int spi_flash_chip_winbond_get_caps;
extern int gpio_intr_disable;
extern int tcp_tw_pcbs;
extern int _lock_close_recursive;
extern int _vsnprintf_r;
extern int _realloc_r;
extern int vTaskSetThreadLocalStoragePointerAndDelCallback;
extern int esp_task_wdt_impl_timer_restart;
extern int spi_flash_chip_generic_suspend_cmd_conf;
extern int Cache_Suspend_ICache;
extern int uxTopUsedPriority;
extern int esp_libc_include_syscalls_impl;
extern int tcp_seg_copy;
extern int offset_xCoreID;
extern int esp_int_wdt_cpu_init;
extern int _lock_acquire_recursive;
extern int __strtok_r;
extern int spi_flash_chip_th_get_caps;
extern int spi_flash_encryption_hal_enable;
extern int spi_periph_signal;
extern int rtc_clk_apb_freq_update;
extern int console_write;
extern int spi_bus_get_dma_ctx;
extern int esp_cache_err_acs_save_and_clr;
extern int tlsf_create_with_pool;
extern int __smakebuf_r;
extern int esp_log_impl_unlock;
extern int ledc_set_duty_with_hpoint;
extern int _putc_r;
extern int __stdio_exit_handler;
extern int esp_sleep_enable_gpio_switch;
extern int spi_flash_hal_gpspi_device_config;
extern int g_mmu_mem_regions;
extern int ledc_channel_config;
extern int memp_ARP_QUEUE;
extern int sys_arch_mbox_tryfetch;
extern int _frxt_int_exit;
extern int reachable_time;
extern int xt_unhandled_exception;
extern int netconn_bind;
extern int tcp_write;
extern int spi_flash_chip_generic_get_io_mode;
extern int esp_flash_chip_th;
extern int raw_sendto;
extern int tcp_listen_pcbs;
extern int esp_vfs_include_nullfs_register;
extern int strcasestr;
extern int esp_vfs_null_register;
extern int _xt_coproc_savecs;
extern int udp_sendto_if_src;
extern int cfree;
extern int _strtod_r;
extern int acs_int_st_bits;
extern int sys_mbox_trypost;
extern int pthread_include_pthread_local_storage_impl;
extern int tcp_next_iss;
extern int spi_flash_chip_generic_read_reg;
extern int ip_chksum_pseudo;
extern int ledc_timer_config;
extern int sqrt;
extern int xQueueTakeMutexRecursive;
extern int vTaskSetTimeOutState;
extern int tlsf_walk_pool;
extern int bootloader_execute_flash_command;
extern int rom_flash_chip_dummy;
extern int pcTaskGetName;
extern int xQueueReceiveFromISR;
extern int udp_sendto_if;
extern int mld6_joingroup_netif;
extern int __fixdfdi;
extern int _times_r;
extern int spi_flash_chip_generic_erase_sector;
extern int sys_mbox_free;
extern int fsync;
extern int tcp_backlog_delayed;
extern int strtof;
extern int uart_is_driver_installed;
extern int spi_flash_chip_mxic_get_caps;
extern int memp_PBUF;
extern int tlsf_memalign_offs;
extern int _ctype_;
extern int memspi_host_erase_sector;
extern int spi_bus_lock_bg_check_dev_req;
extern int multi_heap_register;
extern int __retarget_lock_init;
extern int esp_ipc_func_arg;
extern int spi_flash_hal_setup_auto_suspend_mode;
extern int getopt;
extern int strtoul;
extern int esp_log_cache_get_level;
extern int memp_pools;
extern int console_read;
extern int spi_flash_hal_init;
extern int uart_flush;
extern int memspi_host_read_status_hs;
extern int pbuf_free_header;
extern int iswspace;
extern int _frxt_coproc_exc_hook;
extern int esp_stack_ptr_in_extram;
extern int open;
extern int spi_flash_chip_winbond_suspend_cmd_conf;
extern int spi_flash_hal_gpspi_read;
extern int gdma_hal_clear_intr;
extern int esp_flash_get_physical_size;
extern int esp_timer_early_init;
extern int spi_flash_init_lock;
extern int sprintf;
extern int spi_flash_set_erasing_flag;
extern int __localeconv_l;
extern int ip6_input;
extern int iswspace_l;
extern int esp_mspi_get_io;
extern int __wrap_longjmp;
extern int __lock___malloc_recursive_mutex;
extern int memp_UDP_PCB;
extern int spi_flash_chip_issi_probe;
extern int uart_set_parity;
extern int tcp_keepalive;
extern int __kernel_rem_pio2;
extern int sleep;
extern int xPortSysTickHandler;
extern int esp_mprot_is_conf_locked_any;
extern int Cache_Suspend_DCache;
extern int mspi_timing_config_set_flash_clock;
extern int gpio_hal_intr_enable_on_core;
extern int uart_hal_get_parity;
extern int esp_intr_noniram_enable;
extern int gpio_set_direction;
extern int sys_arch_protect;
extern int esp_libc_time_init;
extern int _dtoa_r;
extern int _frxt_task_coproc_state;
extern int spi_device_get_trans_result;
extern int __sseek;
extern int spi_flash_chip_boya_probe;
extern int mspi_timing_config_get_cs_timing;
extern int __any_on;
extern int fseeko;
extern int xTaskGetSchedulerState;
extern int spi_flash_cache2phys;
extern int __state_table;
extern int esp_int_wdt_init;
extern int _stat_r;
extern int pbuf_alloc_reference;
extern int esp_elf_map_sym;
extern int __retarget_lock_close_recursive;
extern int uart_get_baudrate;
extern int rom_spiflash_api_funcs;
extern int esp_err_to_name;
extern int tcp_update_rcv_ann_wnd;
extern int __gtdf2;
extern int soc_memory_region_count;
extern int gdma_ahb_hal_append;
extern int _impure_data;
extern int sscanf;
extern int esp_panic_handler_feed_wdts;
extern int pbuf_header_force;
extern int strlen;
extern int memspi_host_program_page;
extern int strtod;
extern int panic_soc_fill_info;
extern int esp_rom_opiflash_cache_mode_config;
extern int _xt_medint3_exit;
extern int __popcountsi2;
extern int tcp_split_unsent_seg;
extern int __tz_unlock;
extern int spi_bus_add_device;
extern int __seofread;
extern int strtol;
extern int spi_flash_chip_generic_reset;
extern int spi_flash_encryption_hal_check;
extern int mspi_timing_psram_config_set_tuning_regs;
extern int gdma_hal_reset;
extern int esp_vfs_read;
extern int rtc_gpio_deinit;
extern int vTaskPlaceOnEventList;
extern int strncat;
extern int memp_TCPIP_MSG_API;
extern int usb_serial_jtag_read_bytes;
extern int memp_RAW_PCB;
extern int uart_hal_get_stop_bits;
extern int vTaskSwitchContext;
extern int xTaskGetCurrentTaskHandleForCore;
extern int itoa;
extern int tcp_alloc;
extern int tcp_sent;
extern int __ulp;
extern int _fputc_r;
extern int _read_r;
extern int spi_flash_init_chip_state;
extern int __srefill_r;
extern int spi_flash_chip_generic_erase_chip;
extern int spi_flash_cache_enabled;
extern int esp_libc_include_assert_impl;
extern int _UserExceptionVector;
extern int __ubsan_include;
extern int netconn_tcp_recvd;
extern int xTaskPriorityDisinherit;
extern int udp_sendto;
extern int spi_hal_fetch_result;
extern int esp_clk_apb_freq;
extern int sniprintf;
extern int sys_untimeout;
extern int icmp_input;
extern int vfiprintf;
extern int esp_cpu_unstall;
extern int spi_flash_chip_winbond_page_program;
extern int _fwalk_sglue;
extern int FreeRTOS_openocd_params;
extern int usb_serial_jtag_read_ready;
extern int setjmp;
extern int bootloader_flash_is_octal_mode_enabled;
extern int ip4addr_aton;
extern int esp_crosscore_int_init;
extern int soc_memory_regions;
extern int mem_trim;
extern int rtc_clk_cpu_freq_get_config;
extern int clearerr;
extern int spi_flash_hal_gpspi_poll_cmd_done;
extern int periph_rtc_dig_clk8m_enable;
extern int esp_log_linked_list_get_level;
extern int __atomic_compare_exchange_1;
extern int _lock_try_acquire;
extern int spi_hal_usr_is_done;
extern int seekdir;
extern int netif_find;
extern int heap_caps_realloc_default;
extern int spi_bus_lock_register_dev;
extern int g_rtc_dbias_pvt_non_240m;
extern int esp_partition_iterator_release;
extern int multi_heap_realloc;
extern int inet_chksum;
extern int rtc_clk_freq_cal;
extern int xPortInIsrContext;
extern int __floatunsidf;
extern int tcgetattr;
extern int _reclaim_reent;
extern int _tzset_unlocked_r;
extern int netbuf_alloc;
extern int __swrite;
extern int vListInsertEnd;
extern int esp_register_freertos_idle_hook_for_cpu;
extern int tcflush;
extern int esp_lcd_panel_init;
extern int localtime_r;
extern int esp_psram_extram_test;
extern int gpio_pulldown_dis;
extern int udp_remove;
extern int bootloader_init_mem;
extern int esp_log_vprint_func;
extern int spi_bus_lock_get_by_id;
extern int esp_crosscore_int_send_twdt_abort;
extern int strftime_l;
extern int xt_int_has_handler;
extern int xTaskPriorityInherit;
extern int ledc_hal_set_hpoint;
extern int spi_hal_init;
extern int esp_flash_chip_mxic_opi;
extern int console_tcsetattr;
extern int esp_panic_handler_enable_rtc_wdt;
extern int ftello;
extern int rtc_clk_cpu_freq_to_xtal;
extern int spi_flash_encryption_hal_prepare;
extern int __wrap___gxx_personality_v0;
extern int __lock___at_quick_exit_mutex;
extern int memp_PBUF_POOL;
extern int console_end_select;
extern int _fwrite_r;
extern int vTaskDelete;
extern int gmtime_r;
extern int _fcntl_r;
extern int __unorddf2;
extern int periph_rcc_acquire_enter;
extern int tcp_rexmit;
extern int tcp_rexmit_rto;
extern int lwip_netconn_is_deallocated_msg;
extern int mbrtowc;
extern int esp_elf_free;
extern int __sf;
extern int floor;
extern int __tz_lock;
extern int _strerror_r;
extern int spi_device_queue_trans;
extern int __call_exitprocs;
extern int ip6_addr_any;
extern int spi_flash_encryption_hal_disable;
extern int pbuf_try_get_at;
extern int raw_input;
extern int console_fstat;
extern int esp_ptr_byte_accessible;
extern int lwip_netconn_is_err_msg;
extern int rtc_clk_xtal_freq_get;
extern int gdma_hal_set_priority;
extern int raw_remove;
extern int memspi_host_erase_block;
extern int optarg;
extern int esp_efuse_utility_check_errors;
extern int esp_random;
extern int systimer_hal_get_counter_value;
extern int esp_flash_read_chip_id;
extern int ledc_hal_init;
extern int spi_flash_hal_disable_auto_resume_mode;
extern int esp_flash_chip_driver_initialized;
extern int spi_flash_chip_generic_read_unique_id_none;
extern int memp_TCPIP_MSG_INPKT;
extern int esp_clk_tree_xtal32k_get_freq_hz;
extern int gdma_disconnect;
extern int __sccl;
extern int malloc;
extern int nd6_adjust_mld_membership;
extern int cache_hal_is_cache_enabled;
extern int mld6_report_groups;
extern int lwip_hook_tcp_isn;
extern int task_wdt_timeout_abort;
extern int strcmp;
extern int xQueueGiveMutexRecursive;
extern int spicommon_dma_desc_setup_link;
extern int spi_flash_chip_generic_write_encrypted;
extern int spi_flash_check_and_flush_cache;
extern int ftell;
extern int esp_sleep_sub_mode_config;
extern int xt_unhandled_interrupt;
extern int spi_flash_disable_cache;
extern int app_elf_sha256_str;
extern int spi_bus_lock_acquire_start;
extern int settimeofday;
extern int esp_backtrace_print;
extern int spi_bus_remove_device;
extern int rtc_clk_bbpll_add_consumer;
extern int esp_vfs_usb_serial_jtag_get_vfs;
extern int periph_rcc_enter;
extern int err_to_errno;
extern int spi_hal_hw_prepare_rx;
extern int tcp_abandon;
extern int readdir;
#pragma GCC diagnostic pop

/* Available ELF symbols table: g_customer_elfsyms */

const struct esp_elfsym g_customer_elfsyms[] = {
    ESP_ELFSYM_EXPORT(stpcpy),
    ESP_ELFSYM_EXPORT(gzopen),
    ESP_ELFSYM_EXPORT(gzread),
    ESP_ELFSYM_EXPORT(gzwrite),
    ESP_ELFSYM_EXPORT(gzclose),
    ESP_ELFSYM_EXPORT(gzerror),
    ESP_ELFSYM_EXPORT(deflateInit2_),
    ESP_ELFSYM_EXPORT(deflate),
    ESP_ELFSYM_EXPORT(deflateEnd),
    ESP_ELFSYM_EXPORT(crc32),
    ESP_ELFSYM_EXPORT(ferror),
    ESP_ELFSYM_EXPORT(feof),
    ESP_ELFSYM_EXPORT(breezy_http_download),
    ESP_ELFSYM_EXPORT(esp_http_client_cleanup),
    ESP_ELFSYM_EXPORT(esp_http_client_get_status_code),
    ESP_ELFSYM_EXPORT(esp_http_client_init),
    ESP_ELFSYM_EXPORT(esp_http_client_perform),
    ESP_ELFSYM_EXPORT(esp_http_client_set_header),
    ESP_ELFSYM_EXPORT(esp_crt_bundle_attach),
    ESP_ELFSYM_EXPORT(elf_get_cycle_count),
    ESP_ELFSYM_EXPORT(vTaskDelayUntil),
    ESP_ELFSYM_EXPORT(xTaskGetTickCount),
    ESP_ELFSYM_EXPORT(my_display_refresh_palette),
    ESP_ELFSYM_EXPORT(vterm_set_palette),
    ESP_ELFSYM_EXPORT(vterm_get_palette),
    ESP_ELFSYM_EXPORT(srand),
    ESP_ELFSYM_EXPORT(rand),
    ESP_ELFSYM_EXPORT(sin),
    ESP_ELFSYM_EXPORT(cos),
    ESP_ELFSYM_EXPORT(sqrt),
    ESP_ELFSYM_EXPORT(sinf),
    ESP_ELFSYM_EXPORT(cosf),
    ESP_ELFSYM_EXPORT(sqrtf),
    ESP_ELFSYM_EXPORT(vterm_get_size),
    ESP_ELFSYM_EXPORT(clock),
    ESP_ELFSYM_EXPORT(isalnum),
    ESP_ELFSYM_EXPORT(isalpha),
    ESP_ELFSYM_EXPORT(isdigit),
    ESP_ELFSYM_EXPORT(isspace),
    ESP_ELFSYM_EXPORT(lseek),
    ESP_ELFSYM_EXPORT(unlink),
    ESP_ELFSYM_EXPORT(heap_caps_realloc),
    ESP_ELFSYM_EXPORT(heap_caps_get_free_size),
    ESP_ELFSYM_EXPORT(heap_caps_check_integrity_all),
    ESP_ELFSYM_EXPORT(mspi_timing_psram_config_clear_tuning_regs),
    ESP_ELFSYM_EXPORT(__multadd),
    ESP_ELFSYM_EXPORT(_xt_coproc_sa_offset),
    ESP_ELFSYM_EXPORT(pthread_detach),
    ESP_ELFSYM_EXPORT(spi_device_polling_transmit),
    ESP_ELFSYM_EXPORT(rtc_clk_cpu_freq_set_xtal),
    ESP_ELFSYM_EXPORT(printf),
    ESP_ELFSYM_EXPORT(heap_caps_calloc),
    ESP_ELFSYM_EXPORT(netconn_prepare_delete),
    ESP_ELFSYM_EXPORT(pxPortInitialiseStack),
    ESP_ELFSYM_EXPORT(memspi_host_read_id_hs),
    ESP_ELFSYM_EXPORT(__atomic_fetch_or_4),
    ESP_ELFSYM_EXPORT(esp_psram_chip_init),
    ESP_ELFSYM_EXPORT(rtc_spinlock),
    ESP_ELFSYM_EXPORT(exit),
    ESP_ELFSYM_EXPORT(esp_time_impl_set_boot_time),
    ESP_ELFSYM_EXPORT(__cxa_guard_dummy),
    ESP_ELFSYM_EXPORT(xPortEnterCriticalTimeout),
    ESP_ELFSYM_EXPORT(esp_mspi_pin_reserve),
    ESP_ELFSYM_EXPORT(spi_flash_hal_supports_direct_write),
    ESP_ELFSYM_EXPORT(sys_mutex_new),
    ESP_ELFSYM_EXPORT(__ratio),
    ESP_ELFSYM_EXPORT(esp_vfs_opendir),
    ESP_ELFSYM_EXPORT(esp_backtrace_print_from_frame),
    ESP_ELFSYM_EXPORT(esp_vfs_fstat),
    ESP_ELFSYM_EXPORT(gdma_hal_connect_peri),
    ESP_ELFSYM_EXPORT(esp_vfs_uart_get_vfs),
    ESP_ELFSYM_EXPORT(pvalloc),
    ESP_ELFSYM_EXPORT(igmp_leavegroup_netif),
    ESP_ELFSYM_EXPORT(esp_vfs_open),
    ESP_ELFSYM_EXPORT(spi_bus_lock_wait_bg_done),
    ESP_ELFSYM_EXPORT(esp_intr_ptr_in_isr_region),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_erase_sector),
    ESP_ELFSYM_EXPORT(tcp_recv_null),
    ESP_ELFSYM_EXPORT(gpio_hal_intr_disable),
    ESP_ELFSYM_EXPORT(xQueueGenericCreateStatic),
    ESP_ELFSYM_EXPORT(vListInitialiseItem),
    ESP_ELFSYM_EXPORT(__atomic_fetch_or_8),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_get_write_protect),
    ESP_ELFSYM_EXPORT(rewind),
    ESP_ELFSYM_EXPORT(raw_recv),
    ESP_ELFSYM_EXPORT(realloc),
    ESP_ELFSYM_EXPORT(g_exc_frames),
    ESP_ELFSYM_EXPORT(uart_hal_get_baudrate),
    ESP_ELFSYM_EXPORT(_getpid_r),
    ESP_ELFSYM_EXPORT(pbuf_realloc),
    ESP_ELFSYM_EXPORT(esp_backtrace_get_start),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_wait_idle),
    ESP_ELFSYM_EXPORT(esp_clk_tree_rc_fast_d256_get_freq_hz),
    ESP_ELFSYM_EXPORT(spi_bus_register_destroy_func),
    ESP_ELFSYM_EXPORT(strerror_l),
    ESP_ELFSYM_EXPORT(spicommon_dma_chan_alloc),
    ESP_ELFSYM_EXPORT(systimer_hal_connect_alarm_counter),
    ESP_ELFSYM_EXPORT(mld6_leavegroup_netif),
    ESP_ELFSYM_EXPORT(esp_time_impl_get_boot_time),
    ESP_ELFSYM_EXPORT(rtc_dig_clk8m_enable),
    ESP_ELFSYM_EXPORT(g_panic_abort),
    ESP_ELFSYM_EXPORT(memmove),
    ESP_ELFSYM_EXPORT(gpio_set_intr_type),
    ESP_ELFSYM_EXPORT(rtc_clk_cpu_freq_set_config),
    ESP_ELFSYM_EXPORT(pbuf_chain),
    ESP_ELFSYM_EXPORT(esp_clk_tree_lp_slow_get_freq_hz),
    ESP_ELFSYM_EXPORT(ip4_route_src),
    ESP_ELFSYM_EXPORT(udp_bind_netif),
    ESP_ELFSYM_EXPORT(esp_elf_print_phdr),
    ESP_ELFSYM_EXPORT(esp_mprot_get_split_addr_lock),
    ESP_ELFSYM_EXPORT(esp_libc_init),
    ESP_ELFSYM_EXPORT(memp_FRAG_PBUF),
    ESP_ELFSYM_EXPORT(uart_hal_set_data_bit_num),
    ESP_ELFSYM_EXPORT(__atomic_s32c1i_compare_exchange_4),
    ESP_ELFSYM_EXPORT(heap_caps_free),
    ESP_ELFSYM_EXPORT(console_fcntl),
    ESP_ELFSYM_EXPORT(esp_flash_encryption_enabled),
    ESP_ELFSYM_EXPORT(esp_task_wdt_init),
    ESP_ELFSYM_EXPORT(labs),
    ESP_ELFSYM_EXPORT(xTaskResumeAll),
    ESP_ELFSYM_EXPORT(tcp_send_empty_ack),
    ESP_ELFSYM_EXPORT(port_IntStack),
    ESP_ELFSYM_EXPORT(esp_flash_app_init),
    ESP_ELFSYM_EXPORT(get_vfs_for_index),
    ESP_ELFSYM_EXPORT(registered_heaps),
    ESP_ELFSYM_EXPORT(nvs_sec_provider_include_impl),
    ESP_ELFSYM_EXPORT(igmp_input),
    ESP_ELFSYM_EXPORT(localeconv),
    ESP_ELFSYM_EXPORT(default_router_list),
    ESP_ELFSYM_EXPORT(spi_hal_push_tx_buffer),
    ESP_ELFSYM_EXPORT(esp_libc_include_init_funcs),
    ESP_ELFSYM_EXPORT(_kill_r),
    ESP_ELFSYM_EXPORT(tzset),
    ESP_ELFSYM_EXPORT(esp_system_include_startup_funcs),
    ESP_ELFSYM_EXPORT(setlocale),
    ESP_ELFSYM_EXPORT(xt_ints_on),
    ESP_ELFSYM_EXPORT(ip6_frag),
    ESP_ELFSYM_EXPORT(vListInsert),
    ESP_ELFSYM_EXPORT(ip4_route_src_hook),
    ESP_ELFSYM_EXPORT(spi_flash_chip_issi_get_io_mode),
    ESP_ELFSYM_EXPORT(esp_set_time_from_rtc),
    ESP_ELFSYM_EXPORT(netif_get_by_index),
    ESP_ELFSYM_EXPORT(rtc_sleep_pu),
    ESP_ELFSYM_EXPORT(__sprint_r),
    ESP_ELFSYM_EXPORT(s_register_config_driver),
    ESP_ELFSYM_EXPORT(_ledc_set_pin),
    ESP_ELFSYM_EXPORT(_invalid_pc_placeholder),
    ESP_ELFSYM_EXPORT(igmp_lookfor_group),
    ESP_ELFSYM_EXPORT(strcspn),
    ESP_ELFSYM_EXPORT(esp_mspi_pin_init),
    ESP_ELFSYM_EXPORT(xTaskGenericNotifyWait),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_detect_size),
    ESP_ELFSYM_EXPORT(esp_intr_disable),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_erase_block),
    ESP_ELFSYM_EXPORT(gpio_reset_pin),
    ESP_ELFSYM_EXPORT(strstr),
    ESP_ELFSYM_EXPORT(nan),
    ESP_ELFSYM_EXPORT(clock_gettime),
    ESP_ELFSYM_EXPORT(vApplicationGetIdleTaskMemory),
    ESP_ELFSYM_EXPORT(get_vfs_for_path),
    ESP_ELFSYM_EXPORT(gdma_config_transfer),
    ESP_ELFSYM_EXPORT(mmu_hal_vaddr_to_paddr),
    ESP_ELFSYM_EXPORT(xRingbufferSendFromISR),
    ESP_ELFSYM_EXPORT(systimer_hal_enable_counter),
    ESP_ELFSYM_EXPORT(fread),
    ESP_ELFSYM_EXPORT(spi_flash_common_write_status_16b_wrsr),
    ESP_ELFSYM_EXPORT(vRingbufferDelete),
    ESP_ELFSYM_EXPORT(gpio_sleep_sel_dis),
    ESP_ELFSYM_EXPORT(__mprec_tinytens),
    ESP_ELFSYM_EXPORT(spi_flash_hal_suspend),
    ESP_ELFSYM_EXPORT(tcp_pcb_lists),
    ESP_ELFSYM_EXPORT(netif_get_ip6_addr_match),
    ESP_ELFSYM_EXPORT(raw_bind),
    ESP_ELFSYM_EXPORT(sar_periph_ctrl_init),
    ESP_ELFSYM_EXPORT(gpio_od_enable),
    ESP_ELFSYM_EXPORT(mem_malloc),
    ESP_ELFSYM_EXPORT(esp_libc_init_global_stdio),
    ESP_ELFSYM_EXPORT(tcp_bind_netif),
    ESP_ELFSYM_EXPORT(__atexit_recursive_mutex),
    ESP_ELFSYM_EXPORT(spicommon_dma_desc_alloc),
    ESP_ELFSYM_EXPORT(memspi_host_flush_cache),
    ESP_ELFSYM_EXPORT(pbuf_copy_partial_pbuf),
    ESP_ELFSYM_EXPORT(spi_flash_disable_interrupts_caches_and_other_cpu),
    ESP_ELFSYM_EXPORT(_calloc_r),
    ESP_ELFSYM_EXPORT(esp_psram_is_initialized),
    ESP_ELFSYM_EXPORT(heap_caps_check_add_region_allowed),
    ESP_ELFSYM_EXPORT(pthread_attr_setstacksize),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_get_eof_desc_addr),
    ESP_ELFSYM_EXPORT(xQueueCreateMutex),
    ESP_ELFSYM_EXPORT(esp_ptr_external_ram),
    ESP_ELFSYM_EXPORT(_free_r),
    ESP_ELFSYM_EXPORT(console_fsync),
    ESP_ELFSYM_EXPORT(__retarget_lock_release),
    ESP_ELFSYM_EXPORT(strtoull),
    ESP_ELFSYM_EXPORT(esp_opiflash_set_required_regs),
    ESP_ELFSYM_EXPORT(spi_flash_common_write_status_8b_wrsr),
    ESP_ELFSYM_EXPORT(fgets),
    ESP_ELFSYM_EXPORT(efuse_hal_get_major_chip_version),
    ESP_ELFSYM_EXPORT(__wrap_mkdir),
    ESP_ELFSYM_EXPORT(systimer_ticks_to_us),
    ESP_ELFSYM_EXPORT(esp_cache_err_get_panic_info),
    ESP_ELFSYM_EXPORT(_lock_acquire),
    ESP_ELFSYM_EXPORT(close),
    ESP_ELFSYM_EXPORT(gpio_config),
    ESP_ELFSYM_EXPORT(mld6_input),
    ESP_ELFSYM_EXPORT(gpio_func_sel),
    ESP_ELFSYM_EXPORT(g_startup_time),
    ESP_ELFSYM_EXPORT(spicommon_periph_free),
    ESP_ELFSYM_EXPORT(esp_psram_impl_get_available_size),
    ESP_ELFSYM_EXPORT(tlsf_create),
    ESP_ELFSYM_EXPORT(nd6_find_route),
    ESP_ELFSYM_EXPORT(sys_arch_sem_wait),
    ESP_ELFSYM_EXPORT(esp_flash_chip_winbond),
    ESP_ELFSYM_EXPORT(__action_table),
    ESP_ELFSYM_EXPORT(gdma_new_ahb_channel),
    ESP_ELFSYM_EXPORT(__ssvfiscanf_r),
    ESP_ELFSYM_EXPORT(strchr),
    ESP_ELFSYM_EXPORT(esp_vfs_rewinddir),
    ESP_ELFSYM_EXPORT(spi_flash_mmap),
    ESP_ELFSYM_EXPORT(ip4addr_ntoa),
    ESP_ELFSYM_EXPORT(ipaddr_addr),
    ESP_ELFSYM_EXPORT(bootloader_flash_reset_chip),
    ESP_ELFSYM_EXPORT(xQueueCreateCountingSemaphoreStatic),
    ESP_ELFSYM_EXPORT(netif_list),
    ESP_ELFSYM_EXPORT(__match),
    ESP_ELFSYM_EXPORT(esp_isr_names),
    ESP_ELFSYM_EXPORT(_ZN17__eh_globals_init7_S_initE),
    ESP_ELFSYM_EXPORT(tcp_netif_ip_addr_changed),
    ESP_ELFSYM_EXPORT(xTaskGetCurrentTaskHandle),
    ESP_ELFSYM_EXPORT(esp_log_default_level),
    ESP_ELFSYM_EXPORT(memp_TCP_PCB_LISTEN),
    ESP_ELFSYM_EXPORT(spi_flash_hal_resume),
    ESP_ELFSYM_EXPORT(gpio_iomux_input),
    ESP_ELFSYM_EXPORT(mspi_timing_flash_config_clear_tuning_regs),
    ESP_ELFSYM_EXPORT(esp_flash_chip_generic),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_probe),
    ESP_ELFSYM_EXPORT(__chclass),
    ESP_ELFSYM_EXPORT(rtc_isr_register),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_listen),
    ESP_ELFSYM_EXPORT(vQueueDelete),
    ESP_ELFSYM_EXPORT(esp_partition_next),
    ESP_ELFSYM_EXPORT(calloc),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_timeout),
    ESP_ELFSYM_EXPORT(esp_mprot_set_prot),
    ESP_ELFSYM_EXPORT(esp_psram_check_ptr_addr),
    ESP_ELFSYM_EXPORT(memspi_host_erase_chip),
    ESP_ELFSYM_EXPORT(panic_soc_check_pseudo_cause),
    ESP_ELFSYM_EXPORT(esp_vfs_register_fs),
    ESP_ELFSYM_EXPORT(__atomic_s32c1i_fetch_and_4),
    ESP_ELFSYM_EXPORT(__sfp_lock_release),
    ESP_ELFSYM_EXPORT(esp_elf_deinit),
    ESP_ELFSYM_EXPORT(ip_addr_any_type),
    ESP_ELFSYM_EXPORT(icmp6_input),
    ESP_ELFSYM_EXPORT(tcp_active_pcbs_changed),
    ESP_ELFSYM_EXPORT(cache_hal_invalidate_addr),
    ESP_ELFSYM_EXPORT(xQueueGenericSendFromISR),
    ESP_ELFSYM_EXPORT(esp_clk_cpu_freq),
    ESP_ELFSYM_EXPORT(rtc_time_us_to_slowclk),
    ESP_ELFSYM_EXPORT(strncmp),
    ESP_ELFSYM_EXPORT(spicommon_cs_free_io),
    ESP_ELFSYM_EXPORT(heap_caps_get_largest_free_block),
    ESP_ELFSYM_EXPORT(nd6_input),
    ESP_ELFSYM_EXPORT(esp_mmu_map_get_max_consecutive_free_block_size),
    ESP_ELFSYM_EXPORT(xRingbufferGetCurFreeSize),
    ESP_ELFSYM_EXPORT(memp_TCP_PCB),
    ESP_ELFSYM_EXPORT(atexit),
    ESP_ELFSYM_EXPORT(pbuf_copy),
    ESP_ELFSYM_EXPORT(gdma_hal_disconnect_peri),
    ESP_ELFSYM_EXPORT(__wrap_fopen),
    ESP_ELFSYM_EXPORT(esp_psram_extram_add_to_heap_allocator),
    ESP_ELFSYM_EXPORT(xTaskCreatePinnedToCore),
    ESP_ELFSYM_EXPORT(_rename_r),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_bind),
    ESP_ELFSYM_EXPORT(tcp_shutdown),
    ESP_ELFSYM_EXPORT(esp_crosscore_int_send_yield),
    ESP_ELFSYM_EXPORT(multi_heap_get_info),
    ESP_ELFSYM_EXPORT(_fclose_r),
    ESP_ELFSYM_EXPORT(fputc),
    ESP_ELFSYM_EXPORT(tlsf_malloc),
    ESP_ELFSYM_EXPORT(rtc_clk_fast_src_set),
    ESP_ELFSYM_EXPORT(strrchr),
    ESP_ELFSYM_EXPORT(tcp_tmr),
    ESP_ELFSYM_EXPORT(rom_flash_chip_dummy_hpm),
    ESP_ELFSYM_EXPORT(stat),
    ESP_ELFSYM_EXPORT(_vprintf_r),
    ESP_ELFSYM_EXPORT(esp_ipc_isr_port_init),
    ESP_ELFSYM_EXPORT(__pow5mult),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_init),
    ESP_ELFSYM_EXPORT(gpio_pullup_dis),
    ESP_ELFSYM_EXPORT(uart_vfs_dev_register),
    ESP_ELFSYM_EXPORT(scalbn),
    ESP_ELFSYM_EXPORT(spi_flash_common_read_status_8b_rdsr2),
    ESP_ELFSYM_EXPORT(__lo0bits),
    ESP_ELFSYM_EXPORT(netif_loop_output),
    ESP_ELFSYM_EXPORT(valloc),
    ESP_ELFSYM_EXPORT(uart_set_baudrate),
    ESP_ELFSYM_EXPORT(_findenv),
    ESP_ELFSYM_EXPORT(multi_heap_set_lock),
    ESP_ELFSYM_EXPORT(netconn_recv_udp_raw_netbuf_flags),
    ESP_ELFSYM_EXPORT(tcp_accept),
    ESP_ELFSYM_EXPORT(spi_bus_lock_touch),
    ESP_ELFSYM_EXPORT(esp_libc_include_getentropy_impl),
    ESP_ELFSYM_EXPORT(spi_flash_enable_high_performance_mode),
    ESP_ELFSYM_EXPORT(netconn_alloc),
    ESP_ELFSYM_EXPORT(_printf_r),
    ESP_ELFSYM_EXPORT(strerror),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_connect),
    ESP_ELFSYM_EXPORT(free),
    ESP_ELFSYM_EXPORT(lwip_setsockopt),
    ESP_ELFSYM_EXPORT(spi_bus_lock_unregister_dev),
    ESP_ELFSYM_EXPORT(_vfiprintf_r),
    ESP_ELFSYM_EXPORT(esp_rom_efuse_get_opiconfig),
    ESP_ELFSYM_EXPORT(tcpip_send_msg_wait_sem),
    ESP_ELFSYM_EXPORT(__kernel_tan),
    ESP_ELFSYM_EXPORT(xSemaphoreCreateGenericWithCaps),
    ESP_ELFSYM_EXPORT(spi_device_acquire_bus),
    ESP_ELFSYM_EXPORT(esp_sync_timekeeping_timers),
    ESP_ELFSYM_EXPORT(__swhatbuf_r),
    ESP_ELFSYM_EXPORT(esp_flash_app_enable_os_functions),
    ESP_ELFSYM_EXPORT(cache_hal_freeze),
    ESP_ELFSYM_EXPORT(mem_free),
    ESP_ELFSYM_EXPORT(tcp_connect),
    ESP_ELFSYM_EXPORT(esp_intr_disable_source),
    ESP_ELFSYM_EXPORT(pbuf_add_header_force),
    ESP_ELFSYM_EXPORT(netconn_free),
    ESP_ELFSYM_EXPORT(my_display_init),
    ESP_ELFSYM_EXPORT(panic_print_registers),
    ESP_ELFSYM_EXPORT(prefix_list),
    ESP_ELFSYM_EXPORT(ip4_output_if_opt),
    ESP_ELFSYM_EXPORT(strlcat),
    ESP_ELFSYM_EXPORT(esp_chip_info),
    ESP_ELFSYM_EXPORT(console_tcdrain),
    ESP_ELFSYM_EXPORT(vPortYieldOtherCore),
    ESP_ELFSYM_EXPORT(pthread_internal_local_storage_destructor_callback),
    ESP_ELFSYM_EXPORT(memp_NETBUF),
    ESP_ELFSYM_EXPORT(esp_vfs_console_register),
    ESP_ELFSYM_EXPORT(netconn_recv_tcp_pbuf_flags),
    ESP_ELFSYM_EXPORT(panic_get_cause),
    ESP_ELFSYM_EXPORT(_open_r),
    ESP_ELFSYM_EXPORT(spi_flash_op_block_func),
    ESP_ELFSYM_EXPORT(spi_flash_hal_erase_block),
    ESP_ELFSYM_EXPORT(esp_psram_impl_get_cs_io),
    ESP_ELFSYM_EXPORT(mspi_timing_enter_high_speed_mode),
    ESP_ELFSYM_EXPORT(periph_rcc_exit),
    ESP_ELFSYM_EXPORT(spi_flash_chip_gd_get_io_mode),
    ESP_ELFSYM_EXPORT(icmp6_dest_unreach),
    ESP_ELFSYM_EXPORT(_tzset_unlocked),
    ESP_ELFSYM_EXPORT(esp_register_freertos_tick_hook_for_cpu),
    ESP_ELFSYM_EXPORT(_close_r),
    ESP_ELFSYM_EXPORT(__wrap_getcwd),
    ESP_ELFSYM_EXPORT(xQueueCreateMutexStatic),
    ESP_ELFSYM_EXPORT(__lock___sinit_recursive_mutex),
    ESP_ELFSYM_EXPORT(fputs),
    ESP_ELFSYM_EXPORT(lwip_bind),
    ESP_ELFSYM_EXPORT(pvTaskGetThreadLocalStoragePointer),
    ESP_ELFSYM_EXPORT(vprintf),
    ESP_ELFSYM_EXPORT(mspi_timing_psram_tuning),
    ESP_ELFSYM_EXPORT(console_open),
    ESP_ELFSYM_EXPORT(brownout_hal_config),
    ESP_ELFSYM_EXPORT(_Level2Vector),
    ESP_ELFSYM_EXPORT(esp_vfs_write),
    ESP_ELFSYM_EXPORT(sys_now),
    ESP_ELFSYM_EXPORT(__atomic_fetch_and_8),
    ESP_ELFSYM_EXPORT(esp_log_level_get_timeout),
    ESP_ELFSYM_EXPORT(panic_print_dec),
    ESP_ELFSYM_EXPORT(lwip_connect),
    ESP_ELFSYM_EXPORT(include_esp_phy_override),
    ESP_ELFSYM_EXPORT(raw_netif_ip_addr_changed),
    ESP_ELFSYM_EXPORT(xPortcheckValidStackMem),
    ESP_ELFSYM_EXPORT(efuse_hal_chip_revision),
    ESP_ELFSYM_EXPORT(esp_flash_noos_functions),
    ESP_ELFSYM_EXPORT(__sfvwrite_r),
    ESP_ELFSYM_EXPORT(abort),
    ESP_ELFSYM_EXPORT(igmp_leavegroup),
    ESP_ELFSYM_EXPORT(esp_task_wdt_print_triggered_tasks),
    ESP_ELFSYM_EXPORT(memspi_host_init_pointers),
    ESP_ELFSYM_EXPORT(_xt_coproc_owner_sa_lock),
    ESP_ELFSYM_EXPORT(rtc_isr_noniram_enable),
    ESP_ELFSYM_EXPORT(tcp_err),
    ESP_ELFSYM_EXPORT(esp_flash_chip_gd),
    ESP_ELFSYM_EXPORT(__adddf3),
    ESP_ELFSYM_EXPORT(esp_mmu_map_init),
    ESP_ELFSYM_EXPORT(gpio_sleep_set_pull_mode),
    ESP_ELFSYM_EXPORT(uart_hal_rxfifo_rst),
    ESP_ELFSYM_EXPORT(cache_hal_resume),
    ESP_ELFSYM_EXPORT(_system_r),
    ESP_ELFSYM_EXPORT(__locale_mb_cur_max),
    ESP_ELFSYM_EXPORT(gdma_hal_start_with_desc),
    ESP_ELFSYM_EXPORT(__divdi3),
    ESP_ELFSYM_EXPORT(efuse_hal_get_disable_wafer_version_major),
    ESP_ELFSYM_EXPORT(esp_vfs_fsync),
    ESP_ELFSYM_EXPORT(esp_vfs_stat),
    ESP_ELFSYM_EXPORT(strtoll),
    ESP_ELFSYM_EXPORT(esp_restart),
    ESP_ELFSYM_EXPORT(tcp_new_ip_type),
    ESP_ELFSYM_EXPORT(_unlink_r),
    ESP_ELFSYM_EXPORT(spi_hal_cal_clock_conf),
    ESP_ELFSYM_EXPORT(__atomic_fetch_and_4),
    ESP_ELFSYM_EXPORT(__umoddi3),
    ESP_ELFSYM_EXPORT(vPortSetupTimer),
    ESP_ELFSYM_EXPORT(icmp_dest_unreach),
    ESP_ELFSYM_EXPORT(vTaskInternalSetTimeOutState),
    ESP_ELFSYM_EXPORT(uxListRemove),
    ESP_ELFSYM_EXPORT(esp_psram_init),
    ESP_ELFSYM_EXPORT(spi_flash_chip_gd_detect_size),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_write_bytes),
    ESP_ELFSYM_EXPORT(xt_set_interrupt_handler),
    ESP_ELFSYM_EXPORT(spi_flash_chip_th_probe),
    ESP_ELFSYM_EXPORT(__hi0bits),
    ESP_ELFSYM_EXPORT(__sglue),
    ESP_ELFSYM_EXPORT(esp_clk_slowclk_cal_get),
    ESP_ELFSYM_EXPORT(esp_newlib_time_init),
    ESP_ELFSYM_EXPORT(esp_newlib_locks_init),
    ESP_ELFSYM_EXPORT(memspi_host_set_write_protect),
    ESP_ELFSYM_EXPORT(gdma_hal_deinit),
    ESP_ELFSYM_EXPORT(getchar),
    ESP_ELFSYM_EXPORT(esp_mprot_set_split_addr),
    ESP_ELFSYM_EXPORT(esp_elf_malloc),
    ESP_ELFSYM_EXPORT(opterr),
    ESP_ELFSYM_EXPORT(pbuf_add_header),
    ESP_ELFSYM_EXPORT(rtc_clk_32k_enable_external),
    ESP_ELFSYM_EXPORT(g_dig_dbias_pvt_non_240m),
    ESP_ELFSYM_EXPORT(esp_flash_init_default_chip),
    ESP_ELFSYM_EXPORT(s_spinlock),
    ESP_ELFSYM_EXPORT(esp_intr_get_cpu),
    ESP_ELFSYM_EXPORT(heap_caps_aligned_calloc),
    ESP_ELFSYM_EXPORT(spi_flash_chip_gd_set_io_mode),
    ESP_ELFSYM_EXPORT(g_spi_lock_main_flash_dev),
    ESP_ELFSYM_EXPORT(spi_timing_get_flash_timing_param),
    ESP_ELFSYM_EXPORT(pthread_exit),
    ESP_ELFSYM_EXPORT(lwip_htons),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_set_write_protect),
    ESP_ELFSYM_EXPORT(spi_hal_deinit),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_write_ready),
    ESP_ELFSYM_EXPORT(fflush),
    ESP_ELFSYM_EXPORT(pthread_include_pthread_cond_var_impl),
    ESP_ELFSYM_EXPORT(__sfp),
    ESP_ELFSYM_EXPORT(__sinit),
    ESP_ELFSYM_EXPORT(retrans_timer),
    ESP_ELFSYM_EXPORT(netif_invoke_ext_callback),
    ESP_ELFSYM_EXPORT(_esp_error_check_failed),
    ESP_ELFSYM_EXPORT(xTaskGetTickCountFromISR),
    ESP_ELFSYM_EXPORT(netconn_disconnect),
    ESP_ELFSYM_EXPORT(uart_hal_write_txfifo),
    ESP_ELFSYM_EXPORT(_lock_try_acquire_recursive),
    ESP_ELFSYM_EXPORT(div),
    ESP_ELFSYM_EXPORT(ledc_set_duty),
    ESP_ELFSYM_EXPORT(__atexit),
    ESP_ELFSYM_EXPORT(tlsf_alloc_overhead),
    ESP_ELFSYM_EXPORT(__fixdfsi),
    ESP_ELFSYM_EXPORT(rtc_io_num_map),
    ESP_ELFSYM_EXPORT(sys_arch_mbox_fetch),
    ESP_ELFSYM_EXPORT(clk_hal_cpu_get_freq_hz),
    ESP_ELFSYM_EXPORT(soc_memory_types),
    ESP_ELFSYM_EXPORT(heap_caps_calloc_base),
    ESP_ELFSYM_EXPORT(cache_hal_disable),
    ESP_ELFSYM_EXPORT(__eqdf2),
    ESP_ELFSYM_EXPORT(esp_apb_backup_dma_lock_init),
    ESP_ELFSYM_EXPORT(_xt_medint2_exit),
    ESP_ELFSYM_EXPORT(strtof_l),
    ESP_ELFSYM_EXPORT(spi_flash_set_rom_required_regs),
    ESP_ELFSYM_EXPORT(esp_cpu_intr_get_desc),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_invert_color),
    ESP_ELFSYM_EXPORT(_write_r),
    ESP_ELFSYM_EXPORT(_fprintf_r),
    ESP_ELFSYM_EXPORT(_exit),
    ESP_ELFSYM_EXPORT(esp_cpu_wait_for_intr),
    ESP_ELFSYM_EXPORT(fprintf),
    ESP_ELFSYM_EXPORT(tlsf_size),
    ESP_ELFSYM_EXPORT(__retarget_lock_try_acquire_recursive),
    ESP_ELFSYM_EXPORT(spicommon_dma_chan_free),
    ESP_ELFSYM_EXPORT(__multiply),
    ESP_ELFSYM_EXPORT(esp_cache_get_alignment),
    ESP_ELFSYM_EXPORT(_frxt_dispatch),
    ESP_ELFSYM_EXPORT(tcp_listen_with_backlog_and_err),
    ESP_ELFSYM_EXPORT(_rewind_r),
    ESP_ELFSYM_EXPORT(vTaskGetSnapshot),
    ESP_ELFSYM_EXPORT(spi_hal_hw_prepare_tx),
    ESP_ELFSYM_EXPORT(xRingbufferCreate),
    ESP_ELFSYM_EXPORT(esp_deep_sleep_wakeup_io_reset),
    ESP_ELFSYM_EXPORT(udp_new),
    ESP_ELFSYM_EXPORT(netconn_listen_with_backlog),
    ESP_ELFSYM_EXPORT(pthread_include_pthread_rwlock_impl),
    ESP_ELFSYM_EXPORT(memp_SYS_TIMEOUT),
    ESP_ELFSYM_EXPORT(esp_cpu_configure_region_protection),
    ESP_ELFSYM_EXPORT(esp_startup_start_app_other_cores),
    ESP_ELFSYM_EXPORT(esp_mprot_ll_err_to_esp_err),
    ESP_ELFSYM_EXPORT(__atomic_s32c1i_fetch_or_4),
    ESP_ELFSYM_EXPORT(xTaskGetCoreID),
    ESP_ELFSYM_EXPORT(lwip_recvfrom),
    ESP_ELFSYM_EXPORT(pthread_getspecific),
    ESP_ELFSYM_EXPORT(memcpy),
    ESP_ELFSYM_EXPORT(uart_hal_get_data_bit_num),
    ESP_ELFSYM_EXPORT(vTaskPrioritySet),
    ESP_ELFSYM_EXPORT(ip_input),
    ESP_ELFSYM_EXPORT(vRingbufferReturnItem),
    ESP_ELFSYM_EXPORT(__mcmp),
    ESP_ELFSYM_EXPORT(multi_heap_get_info_impl),
    ESP_ELFSYM_EXPORT(esp_psram_impl_enable),
    ESP_ELFSYM_EXPORT(xQueueGetMutexHolder),
    ESP_ELFSYM_EXPORT(_vfprintf_r),
    ESP_ELFSYM_EXPORT(vSystimerSetup),
    ESP_ELFSYM_EXPORT(ip4_frag),
    ESP_ELFSYM_EXPORT(esp_mmu_vaddr_to_paddr),
    ESP_ELFSYM_EXPORT(vPortFree),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_page_program),
    ESP_ELFSYM_EXPORT(tcp_output),
    ESP_ELFSYM_EXPORT(esp_intr_alloc_intrstatus),
    ESP_ELFSYM_EXPORT(xthal_window_spill),
    ESP_ELFSYM_EXPORT(__swbuf),
    ESP_ELFSYM_EXPORT(esp_libc_locks_init),
    ESP_ELFSYM_EXPORT(__floatdidf),
    ESP_ELFSYM_EXPORT(g_startup_fn),
    ESP_ELFSYM_EXPORT(_user_strerror),
    ESP_ELFSYM_EXPORT(frexp),
    ESP_ELFSYM_EXPORT(uart_get_parity),
    ESP_ELFSYM_EXPORT(netif_default),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_close),
    ESP_ELFSYM_EXPORT(spi_hal_enable_data_line),
    ESP_ELFSYM_EXPORT(__assert_func),
    ESP_ELFSYM_EXPORT(esp_crosscore_int_send_print_backtrace),
    ESP_ELFSYM_EXPORT(usleep),
    ESP_ELFSYM_EXPORT(pbuf_alloc),
    ESP_ELFSYM_EXPORT(__retarget_lock_acquire),
    ESP_ELFSYM_EXPORT(esp_vfs_close),
    ESP_ELFSYM_EXPORT(udp_disconnect),
    ESP_ELFSYM_EXPORT(mmu_hal_check_valid_ext_vaddr_region),
    ESP_ELFSYM_EXPORT(inet_chksum_pbuf),
    ESP_ELFSYM_EXPORT(rtc_io_number_get),
    ESP_ELFSYM_EXPORT(call_start_cpu0),
    ESP_ELFSYM_EXPORT(multi_heap_malloc_impl),
    ESP_ELFSYM_EXPORT(multi_heap_aligned_alloc_impl_offs),
    ESP_ELFSYM_EXPORT(esp_ota_get_running_partition),
    ESP_ELFSYM_EXPORT(esp_sleep_config_gpio_isolate),
    ESP_ELFSYM_EXPORT(esp_clk_xtal_freq),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_check_status),
    ESP_ELFSYM_EXPORT(esp_ipc_isr_end_fl),
    ESP_ELFSYM_EXPORT(pthread_include_pthread_impl),
    ESP_ELFSYM_EXPORT(gdma_hal_enable_intr),
    ESP_ELFSYM_EXPORT(netconn_connect),
    ESP_ELFSYM_EXPORT(bootloader_flash_execute_command_common),
    ESP_ELFSYM_EXPORT(__retarget_lock_init_recursive),
    ESP_ELFSYM_EXPORT(tcp_zero_window_probe),
    ESP_ELFSYM_EXPORT(tcp_rexmit_rto_commit),
    ESP_ELFSYM_EXPORT(_gettimeofday_r),
    ESP_ELFSYM_EXPORT(xPortInterruptedFromISRContext),
    ESP_ELFSYM_EXPORT(spi_flash_chip_gd_get_caps),
    ESP_ELFSYM_EXPORT(esp_libc_include_heap_impl),
    ESP_ELFSYM_EXPORT(memp_NETDB),
    ESP_ELFSYM_EXPORT(__fp_unlock_all),
    ESP_ELFSYM_EXPORT(udp_input),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_set_select_notif_callback),
    ESP_ELFSYM_EXPORT(esp_gpio_revoke),
    ESP_ELFSYM_EXPORT(console_tcgetattr),
    ESP_ELFSYM_EXPORT(gdma_new_channel),
    ESP_ELFSYM_EXPORT(g_flash_guard_default_ops),
    ESP_ELFSYM_EXPORT(esp_ipc_isr_start_fl),
    ESP_ELFSYM_EXPORT(esp_ipc_call_blocking),
    ESP_ELFSYM_EXPORT(gpio_hal_iomux_in),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_swap_xy),
    ESP_ELFSYM_EXPORT(xTaskGenericNotify),
    ESP_ELFSYM_EXPORT(cache_hal_get_cache_line_size),
    ESP_ELFSYM_EXPORT(_xt_coproc_init),
    ESP_ELFSYM_EXPORT(esp_clk_tree_src_get_freq_hz),
    ESP_ELFSYM_EXPORT(_tzname),
    ESP_ELFSYM_EXPORT(mktime),
    ESP_ELFSYM_EXPORT(spi_flash_encryption_hal_destroy),
    ESP_ELFSYM_EXPORT(esp_log_impl_lock_timeout),
    ESP_ELFSYM_EXPORT(esp_panic_handler_disable_timg_wdts),
    ESP_ELFSYM_EXPORT(g_dig_dbias_pvt_240m),
    ESP_ELFSYM_EXPORT(esp_vfs_include_console_register),
    ESP_ELFSYM_EXPORT(esp_partition_find),
    ESP_ELFSYM_EXPORT(esp_mprot_set_split_addr_lock),
    ESP_ELFSYM_EXPORT(_xt_user_exit),
    ESP_ELFSYM_EXPORT(qsort),
    ESP_ELFSYM_EXPORT(_sungetc_r),
    ESP_ELFSYM_EXPORT(_xt_context_save),
    ESP_ELFSYM_EXPORT(esp_vfs_dev_usb_serial_jtag_register),
    ESP_ELFSYM_EXPORT(tcp_seg_free),
    ESP_ELFSYM_EXPORT(netconn_accept),
    ESP_ELFSYM_EXPORT(esp_setup_newlib_syscalls),
    ESP_ELFSYM_EXPORT(spi_flash_encryption_hal_done),
    ESP_ELFSYM_EXPORT(_ftell_r),
    ESP_ELFSYM_EXPORT(panicHandler),
    ESP_ELFSYM_EXPORT(rtc_time_get),
    ESP_ELFSYM_EXPORT(esp_partition_main_flash_region_safe),
    ESP_ELFSYM_EXPORT(sys_mutex_lock),
    ESP_ELFSYM_EXPORT(xt_highint4),
    ESP_ELFSYM_EXPORT(rtc_clk_slow_src_get),
    ESP_ELFSYM_EXPORT(lwip_sendto),
    ESP_ELFSYM_EXPORT(utoa),
    ESP_ELFSYM_EXPORT(rtc_io_desc),
    ESP_ELFSYM_EXPORT(rtc_clk_cpu_freq_mhz_to_config),
    ESP_ELFSYM_EXPORT(ledc_hal_set_fade_param),
    ESP_ELFSYM_EXPORT(_tzset_r),
    ESP_ELFSYM_EXPORT(esp_libc_include_pthread_impl),
    ESP_ELFSYM_EXPORT(tcp_input),
    ESP_ELFSYM_EXPORT(sys_mbox_new),
    ESP_ELFSYM_EXPORT(spi_flash_needs_reset_check),
    ESP_ELFSYM_EXPORT(mpu_hal_set_region_access),
    ESP_ELFSYM_EXPORT(gpio_output_enable),
    ESP_ELFSYM_EXPORT(fstat),
    ESP_ELFSYM_EXPORT(strtoll_l),
    ESP_ELFSYM_EXPORT(gpio_output_disable),
    ESP_ELFSYM_EXPORT(_getc_r),
    ESP_ELFSYM_EXPORT(ip4_route),
    ESP_ELFSYM_EXPORT(vRingbufferGetInfo),
    ESP_ELFSYM_EXPORT(uart_hal_set_stop_bits),
    ESP_ELFSYM_EXPORT(__ltdf2),
    ESP_ELFSYM_EXPORT(systimer_hal_counter_value_advance),
    ESP_ELFSYM_EXPORT(__wrap_chdir),
    ESP_ELFSYM_EXPORT(__udivdi3),
    ESP_ELFSYM_EXPORT(lwip_send),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_get_caps),
    ESP_ELFSYM_EXPORT(_xt_coproc_owner_sa),
    ESP_ELFSYM_EXPORT(esp_psram_impl_get_physical_size),
    ESP_ELFSYM_EXPORT(esp_system_abort),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_erase_sector),
    ESP_ELFSYM_EXPORT(esp_vfs_access),
    ESP_ELFSYM_EXPORT(esp_vApplicationTickHook),
    ESP_ELFSYM_EXPORT(_ftello_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_probe),
    ESP_ELFSYM_EXPORT(ip4_addr_isbroadcast_u32),
    ESP_ELFSYM_EXPORT(esp_mprot_get_pms_lock),
    ESP_ELFSYM_EXPORT(soc_get_available_memory_region_max_count),
    ESP_ELFSYM_EXPORT(netconn_new_with_proto_and_callback),
    ESP_ELFSYM_EXPORT(inet_chksum_pseudo),
    ESP_ELFSYM_EXPORT(_sniprintf_r),
    ESP_ELFSYM_EXPORT(__register_exitproc),
    ESP_ELFSYM_EXPORT(console_close),
    ESP_ELFSYM_EXPORT(panic_restart),
    ESP_ELFSYM_EXPORT(esp_register_shutdown_handler),
    ESP_ELFSYM_EXPORT(neighbor_cache),
    ESP_ELFSYM_EXPORT(esp_panic_handler),
    ESP_ELFSYM_EXPORT(netbuf_free),
    ESP_ELFSYM_EXPORT(xQueueGiveFromISR),
    ESP_ELFSYM_EXPORT(tcp_pcb_remove),
    ESP_ELFSYM_EXPORT(xTaskRemoveFromEventList),
    ESP_ELFSYM_EXPORT(__bswapsi2),
    ESP_ELFSYM_EXPORT(__gedf2),
    ESP_ELFSYM_EXPORT(_fstat_r),
    ESP_ELFSYM_EXPORT(abs),
    ESP_ELFSYM_EXPORT(spi_bus_deinit_lock),
    ESP_ELFSYM_EXPORT(strtok),
    ESP_ELFSYM_EXPORT(rewinddir),
    ESP_ELFSYM_EXPORT(closedir),
    ESP_ELFSYM_EXPORT(udp_pcbs),
    ESP_ELFSYM_EXPORT(fileno),
    ESP_ELFSYM_EXPORT(environ),
    ESP_ELFSYM_EXPORT(startup_resume_other_cores),
    ESP_ELFSYM_EXPORT(esp_intr_free),
    ESP_ELFSYM_EXPORT(tcp_rexmit_fast),
    ESP_ELFSYM_EXPORT(esp_vApplicationIdleHook),
    ESP_ELFSYM_EXPORT(_mprec_log10),
    ESP_ELFSYM_EXPORT(esp_newlib_init),
    ESP_ELFSYM_EXPORT(multi_heap_free),
    ESP_ELFSYM_EXPORT(_sbrk_r),
    ESP_ELFSYM_EXPORT(esp_vfs_dev_uart_register),
    ESP_ELFSYM_EXPORT(tcp_poll),
    ESP_ELFSYM_EXPORT(putchar),
    ESP_ELFSYM_EXPORT(ip4addr_ntoa_r),
    ESP_ELFSYM_EXPORT(__getopt_long_only_r),
    ESP_ELFSYM_EXPORT(netconn_write_partly),
    ESP_ELFSYM_EXPORT(destination_cache),
    ESP_ELFSYM_EXPORT(g_panic_abort_details),
    ESP_ELFSYM_EXPORT(igmp_tmr),
    ESP_ELFSYM_EXPORT(__gettzinfo),
    ESP_ELFSYM_EXPORT(_puts_r),
    ESP_ELFSYM_EXPORT(mspi_timing_config_psram_write_data),
    ESP_ELFSYM_EXPORT(spi_flash_common_read_status_16b_rdsr_rdsr2),
    ESP_ELFSYM_EXPORT(esp_log_impl_lock),
    ESP_ELFSYM_EXPORT(tcp_fasttmr),
    ESP_ELFSYM_EXPORT(xPortStartScheduler),
    ESP_ELFSYM_EXPORT(gpio_sleep_sel_en),
    ESP_ELFSYM_EXPORT(spi_bus_lock_bg_clear_req),
    ESP_ELFSYM_EXPORT(heap_caps_enable_nonos_stack_heaps),
    ESP_ELFSYM_EXPORT(__copybits),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_get_io_mode),
    ESP_ELFSYM_EXPORT(esp_efuse_check_errors),
    ESP_ELFSYM_EXPORT(__ssprint_r),
    ESP_ELFSYM_EXPORT(__s2b),
    ESP_ELFSYM_EXPORT(memset),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_erase_block),
    ESP_ELFSYM_EXPORT(esp_timer_impl_init_system_time),
    ESP_ELFSYM_EXPORT(xQueueGenericReset),
    ESP_ELFSYM_EXPORT(spi_bus_lock_acquire_end),
    ESP_ELFSYM_EXPORT(rtc_clk_slow_src_set),
    ESP_ELFSYM_EXPORT(mspi_timing_get_psram_tuning_configs),
    ESP_ELFSYM_EXPORT(clk_hal_xtal_get_freq_mhz),
    ESP_ELFSYM_EXPORT(__kernel_sin),
    ESP_ELFSYM_EXPORT(ip_data),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_enable_burst),
    ESP_ELFSYM_EXPORT(sys_sem_signal),
    ESP_ELFSYM_EXPORT(udp_connect),
    ESP_ELFSYM_EXPORT(vTaskPriorityDisinheritAfterTimeout),
    ESP_ELFSYM_EXPORT(icmp6_param_problem),
    ESP_ELFSYM_EXPORT(getenv),
    ESP_ELFSYM_EXPORT(ledc_bind_channel_timer),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_disconnect),
    ESP_ELFSYM_EXPORT(vfs_include_syscalls_impl),
    ESP_ELFSYM_EXPORT(memspi_host_write_data_slicer),
    ESP_ELFSYM_EXPORT(_localeconv_r),
    ESP_ELFSYM_EXPORT(multi_heap_internal_lock),
    ESP_ELFSYM_EXPORT(port_switch_flag),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_common_command),
    ESP_ELFSYM_EXPORT(__lock___tz_mutex),
    ESP_ELFSYM_EXPORT(gpio_od_disable),
    ESP_ELFSYM_EXPORT(regi2c_ctrl_write_reg_mask),
    ESP_ELFSYM_EXPORT(__lock___env_recursive_mutex),
    ESP_ELFSYM_EXPORT(app_main),
    ESP_ELFSYM_EXPORT(esp_mmu_unmap),
    ESP_ELFSYM_EXPORT(esp_elf_request),
    ESP_ELFSYM_EXPORT(_frxt_setup_switch),
    ESP_ELFSYM_EXPORT(strtok_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_set_write_protect),
    ESP_ELFSYM_EXPORT(mmu_hal_pages_to_bytes),
    ESP_ELFSYM_EXPORT(spi_device_polling_end),
    ESP_ELFSYM_EXPORT(uart_set_stop_bits),
    ESP_ELFSYM_EXPORT(xTaskCreateStaticPinnedToCore),
    ESP_ELFSYM_EXPORT(pbuf_ref),
    ESP_ELFSYM_EXPORT(esp_vfs_mkdir),
    ESP_ELFSYM_EXPORT(netconn_delete),
    ESP_ELFSYM_EXPORT(netconn_send),
    ESP_ELFSYM_EXPORT(_frxt_int_enter),
    ESP_ELFSYM_EXPORT(mspi_timing_psram_set_best_tuning_config),
    ESP_ELFSYM_EXPORT(__mdiff),
    ESP_ELFSYM_EXPORT(spi_flash_hal_poll_cmd_done),
    ESP_ELFSYM_EXPORT(rtc_get_xtal),
    ESP_ELFSYM_EXPORT(GPIO_PIN_MUX_REG),
    ESP_ELFSYM_EXPORT(igmp_joingroup),
    ESP_ELFSYM_EXPORT(multi_heap_register_impl),
    ESP_ELFSYM_EXPORT(getcwd),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_getaddr),
    ESP_ELFSYM_EXPORT(panic_get_address),
    ESP_ELFSYM_EXPORT(systimer_us_to_ticks),
    ESP_ELFSYM_EXPORT(mspi_timing_config_set_psram_clock),
    ESP_ELFSYM_EXPORT(esp_cpu_stall),
    ESP_ELFSYM_EXPORT(esp_elf_arch_relocate),
    ESP_ELFSYM_EXPORT(mld6_lookfor_group),
    ESP_ELFSYM_EXPORT(esp_flash_chip_issi),
    ESP_ELFSYM_EXPORT(spi_bus_get_attr),
    ESP_ELFSYM_EXPORT(esp_opiflash_init),
    ESP_ELFSYM_EXPORT(esp_vfs_rename),
    ESP_ELFSYM_EXPORT(esp_task_wdt_impl_timer_feed),
    ESP_ELFSYM_EXPORT(uart_get_word_length),
    ESP_ELFSYM_EXPORT(heap_caps_malloc),
    ESP_ELFSYM_EXPORT(esp_psram_io_get_cs_io),
    ESP_ELFSYM_EXPORT(esp_mprot_monitor_clear_intr),
    ESP_ELFSYM_EXPORT(uart_wait_tx_done),
    ESP_ELFSYM_EXPORT(spi_bus_init_lock),
    ESP_ELFSYM_EXPORT(_NMIExceptionVector),
    ESP_ELFSYM_EXPORT(systimer_hal_enable_alarm_int),
    ESP_ELFSYM_EXPORT(ledc_timer_rst),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_get_write_protect),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_set_io_mode),
    ESP_ELFSYM_EXPORT(_svfiprintf_r),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_set_priority),
    ESP_ELFSYM_EXPORT(spi_hal_clear_intr_mask),
    ESP_ELFSYM_EXPORT(tcp_close),
    ESP_ELFSYM_EXPORT(__atomic_compare_exchange_4),
    ESP_ELFSYM_EXPORT(__hexnan),
    ESP_ELFSYM_EXPORT(__moddi3),
    ESP_ELFSYM_EXPORT(__i2b),
    ESP_ELFSYM_EXPORT(_fgets_r),
    ESP_ELFSYM_EXPORT(fabs),
    ESP_ELFSYM_EXPORT(rtc_isr_noniram_disable),
    ESP_ELFSYM_EXPORT(_link_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_detect_size),
    ESP_ELFSYM_EXPORT(bootloader_flash_update_id),
    ESP_ELFSYM_EXPORT(_lock_release),
    ESP_ELFSYM_EXPORT(tcp_bind),
    ESP_ELFSYM_EXPORT(strtod_l),
    ESP_ELFSYM_EXPORT(esp_clk_slowclk_cal_set),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_disconnect_peri),
    ESP_ELFSYM_EXPORT(gpio_input_enable),
    ESP_ELFSYM_EXPORT(_lock_close),
    ESP_ELFSYM_EXPORT(port_interruptNesting),
    ESP_ELFSYM_EXPORT(esp_mmu_paddr_find_caps),
    ESP_ELFSYM_EXPORT(__getopt_r),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_clear_intr),
    ESP_ELFSYM_EXPORT(esp_elf_print_ehdr),
    ESP_ELFSYM_EXPORT(systimer_hal_set_alarm_period),
    ESP_ELFSYM_EXPORT(pbuf_free),
    ESP_ELFSYM_EXPORT(spi_flash_hal_supports_direct_read),
    ESP_ELFSYM_EXPORT(multi_heap_free_impl),
    ESP_ELFSYM_EXPORT(__sread),
    ESP_ELFSYM_EXPORT(spi_flash_restore_cache),
    ESP_ELFSYM_EXPORT(__bswapdi2),
    ESP_ELFSYM_EXPORT(tcp_rst),
    ESP_ELFSYM_EXPORT(esp_vfs_select_triggered_isr),
    ESP_ELFSYM_EXPORT(periph_rcc_acquire_exit),
    ESP_ELFSYM_EXPORT(vTaskStartScheduler),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_yield),
    ESP_ELFSYM_EXPORT(strtol_l),
    ESP_ELFSYM_EXPORT(esp_elf_init),
    ESP_ELFSYM_EXPORT(esp_timer_impl_get_time),
    ESP_ELFSYM_EXPORT(_xt_coproc_restorecs),
    ESP_ELFSYM_EXPORT(esp_ipc_isr_init),
    ESP_ELFSYM_EXPORT(pvPortMalloc),
    ESP_ELFSYM_EXPORT(__ascii_wctomb),
    ESP_ELFSYM_EXPORT(_perror_r),
    ESP_ELFSYM_EXPORT(__retarget_lock_close),
    ESP_ELFSYM_EXPORT(esp_cache_err_int_init),
    ESP_ELFSYM_EXPORT(spi_flash_chip_gd_probe),
    ESP_ELFSYM_EXPORT(esp_task_wdt_impl_timer_allocate),
    ESP_ELFSYM_EXPORT(__lock___sfp_recursive_mutex),
    ESP_ELFSYM_EXPORT(tcp_recved),
    ESP_ELFSYM_EXPORT(__lshift),
    ESP_ELFSYM_EXPORT(_lock_init),
    ESP_ELFSYM_EXPORT(__ieee754_sqrt),
    ESP_ELFSYM_EXPORT(panic_print_char),
    ESP_ELFSYM_EXPORT(pthread_key_create),
    ESP_ELFSYM_EXPORT(tlsf_block_size),
    ESP_ELFSYM_EXPORT(spi_flash_op_unlock),
    ESP_ELFSYM_EXPORT(__wrap_opendir),
    ESP_ELFSYM_EXPORT(mmu_hal_paddr_to_vaddr),
    ESP_ELFSYM_EXPORT(tcp_active_pcbs),
    ESP_ELFSYM_EXPORT(lwip_socket),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_reset),
    ESP_ELFSYM_EXPORT(nd6_reachability_hint),
    ESP_ELFSYM_EXPORT(uart_hal_set_parity),
    ESP_ELFSYM_EXPORT(esp_startup_start_app),
    ESP_ELFSYM_EXPORT(spi_flash_hal_set_write_protect),
    ESP_ELFSYM_EXPORT(netif_poll),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_write),
    ESP_ELFSYM_EXPORT(spi_flash_hal_erase_sector),
    ESP_ELFSYM_EXPORT(esp_log_early_timestamp),
    ESP_ELFSYM_EXPORT(xTaskGetTickCount),
    ESP_ELFSYM_EXPORT(esp_task_wdt_impl_timeout_triggered),
    ESP_ELFSYM_EXPORT(spi_flash_hal_disable_auto_suspend_mode),
    ESP_ELFSYM_EXPORT(esp_vfs_seekdir),
    ESP_ELFSYM_EXPORT(uxTaskPriorityGet),
    ESP_ELFSYM_EXPORT(heap_caps_match),
    ESP_ELFSYM_EXPORT(xTaskIncrementTickOtherCores),
    ESP_ELFSYM_EXPORT(perror),
    ESP_ELFSYM_EXPORT(periph_rtc_dig_clk8m_get_freq),
    ESP_ELFSYM_EXPORT(systimer_hal_init),
    ESP_ELFSYM_EXPORT(sin),
    ESP_ELFSYM_EXPORT(spi_hal_cal_timing),
    ESP_ELFSYM_EXPORT(multi_heap_aligned_free),
    ESP_ELFSYM_EXPORT(_malloc_r),
    ESP_ELFSYM_EXPORT(esp_task_wdt_add),
    ESP_ELFSYM_EXPORT(_xt_coproc_release),
    ESP_ELFSYM_EXPORT(esp_timer_get_time),
    ESP_ELFSYM_EXPORT(spi_flash_hal_program_page),
    ESP_ELFSYM_EXPORT(port_uxCriticalNesting),
    ESP_ELFSYM_EXPORT(esp_partition_get),
    ESP_ELFSYM_EXPORT(__global_locale_ptr),
    ESP_ELFSYM_EXPORT(ip4_output_if_opt_src),
    ESP_ELFSYM_EXPORT(pbuf_copy_partial),
    ESP_ELFSYM_EXPORT(longjmp),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_set_strategy),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_mirror),
    ESP_ELFSYM_EXPORT(esp_vfs_readdir),
    ESP_ELFSYM_EXPORT(ledc_update_duty),
    ESP_ELFSYM_EXPORT(gpio_sleep_set_direction),
    ESP_ELFSYM_EXPORT(spi_hal_setup_device),
    ESP_ELFSYM_EXPORT(spicommon_bus_initialize_io),
    ESP_ELFSYM_EXPORT(esp_libc_init_funcs),
    ESP_ELFSYM_EXPORT(xQueueGenericCreate),
    ESP_ELFSYM_EXPORT(__d2b),
    ESP_ELFSYM_EXPORT(mmu_hal_map_region),
    ESP_ELFSYM_EXPORT(pbuf_remove_header),
    ESP_ELFSYM_EXPORT(spi_flash_hal_setup_auto_resume_mode),
    ESP_ELFSYM_EXPORT(rtc_dig_clk8m_disable),
    ESP_ELFSYM_EXPORT(ip6_output_if_src),
    ESP_ELFSYM_EXPORT(xQueueGenericGetStaticBuffers),
    ESP_ELFSYM_EXPORT(esp_cache_unfreeze_caches_enable_interrupts),
    ESP_ELFSYM_EXPORT(gdma_reset),
    ESP_ELFSYM_EXPORT(_getchar_r),
    ESP_ELFSYM_EXPORT(tcp_arg),
    ESP_ELFSYM_EXPORT(__sflags),
    ESP_ELFSYM_EXPORT(lwip_standard_chksum),
    ESP_ELFSYM_EXPORT(nanf),
    ESP_ELFSYM_EXPORT(sys_thread_sem_get),
    ESP_ELFSYM_EXPORT(tcp_abort),
    ESP_ELFSYM_EXPORT(strncasecmp),
    ESP_ELFSYM_EXPORT(strcpy),
    ESP_ELFSYM_EXPORT(spi_flash_hal_erase_chip),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_accepted),
    ESP_ELFSYM_EXPORT(spi_flash_chip_boya_get_caps),
    ESP_ELFSYM_EXPORT(ip_addr_broadcast),
    ESP_ELFSYM_EXPORT(esp_ipc_isr_stall_abort),
    ESP_ELFSYM_EXPORT(console_access),
    ESP_ELFSYM_EXPORT(read),
    ESP_ELFSYM_EXPORT(esp_reent_init),
    ESP_ELFSYM_EXPORT(_wctomb_r),
    ESP_ELFSYM_EXPORT(tlsf_realloc),
    ESP_ELFSYM_EXPORT(uart_hal_get_sclk),
    ESP_ELFSYM_EXPORT(_snprintf_r),
    ESP_ELFSYM_EXPORT(_siscanf_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_erase_chip),
    ESP_ELFSYM_EXPORT(spi_flash_common_set_io_mode),
    ESP_ELFSYM_EXPORT(__mprec_bigtens),
    ESP_ELFSYM_EXPORT(spi_flash_hal_setup_read_suspend),
    ESP_ELFSYM_EXPORT(mspi_timing_flash_config_set_tuning_regs),
    ESP_ELFSYM_EXPORT(spi_bus_lock_bg_exit),
    ESP_ELFSYM_EXPORT(spicommon_periph_claim),
    ESP_ELFSYM_EXPORT(udp_netif_ip_addr_changed),
    ESP_ELFSYM_EXPORT(_putchar_r),
    ESP_ELFSYM_EXPORT(regi2c_ctrl_read_reg_mask),
    ESP_ELFSYM_EXPORT(esp_restart_noos),
    ESP_ELFSYM_EXPORT(rtc_clk_cal),
    ESP_ELFSYM_EXPORT(__ieee754_rem_pio2),
    ESP_ELFSYM_EXPORT(raw_bind_netif),
    ESP_ELFSYM_EXPORT(esp_vfs_fcntl_r),
    ESP_ELFSYM_EXPORT(clk_hal_soc_root_get_freq_mhz),
    ESP_ELFSYM_EXPORT(esp_task_wdt_reset),
    ESP_ELFSYM_EXPORT(pbuf_free_ooseq_pending),
    ESP_ELFSYM_EXPORT(_Bfree),
    ESP_ELFSYM_EXPORT(lwip_listen),
    ESP_ELFSYM_EXPORT(tcp_process_refused_data),
    ESP_ELFSYM_EXPORT(_sfread_r),
    ESP_ELFSYM_EXPORT(lwip_accept),
    ESP_ELFSYM_EXPORT(vListInitialise),
    ESP_ELFSYM_EXPORT(lwip_setsockopt_impl_ext),
    ESP_ELFSYM_EXPORT(__tzcalc_limits),
    ESP_ELFSYM_EXPORT(heap_caps_malloc_default),
    ESP_ELFSYM_EXPORT(g_twdt_isr),
    ESP_ELFSYM_EXPORT(call_start_cpu1),
    ESP_ELFSYM_EXPORT(raw_sendto_if_src),
    ESP_ELFSYM_EXPORT(ip6_select_source_address),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_vfs_include_dev_init),
    ESP_ELFSYM_EXPORT(esp_cache_suspend_ext_mem_cache),
    ESP_ELFSYM_EXPORT(__default_global_locale),
    ESP_ELFSYM_EXPORT(spi_hal_setup_trans),
    ESP_ELFSYM_EXPORT(esp_mprot_set_pms_area),
    ESP_ELFSYM_EXPORT(esp_intr_alloc),
    ESP_ELFSYM_EXPORT(raw_new),
    ESP_ELFSYM_EXPORT(esp_mmu_map_reserve_block_with_caps),
    ESP_ELFSYM_EXPORT(esp_mprot_set_monitor_lock),
    ESP_ELFSYM_EXPORT(pxCurrentTCBs),
    ESP_ELFSYM_EXPORT(ip4_output_if_src),
    ESP_ELFSYM_EXPORT(uart_flush_input),
    ESP_ELFSYM_EXPORT(uart_vfs_include_dev_init),
    ESP_ELFSYM_EXPORT(spi_bus_get_max_transaction_len),
    ESP_ELFSYM_EXPORT(esp_time_impl_get_time),
    ESP_ELFSYM_EXPORT(_C_time_locale),
    ESP_ELFSYM_EXPORT(xRingbufferReceiveUpToFromISR),
    ESP_ELFSYM_EXPORT(xTaskGetIdleTaskHandleForCore),
    ESP_ELFSYM_EXPORT(esp_flash_default_chip),
    ESP_ELFSYM_EXPORT(bootloader_read_flash_id),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_connect_peri),
    ESP_ELFSYM_EXPORT(spi_flash_enable_cache),
    ESP_ELFSYM_EXPORT(gdma_del_channel),
    ESP_ELFSYM_EXPORT(ledc_hal_ls_channel_update),
    ESP_ELFSYM_EXPORT(esp_intr_enable),
    ESP_ELFSYM_EXPORT(efuse_hal_flash_encryption_enabled),
    ESP_ELFSYM_EXPORT(panic_print_hex),
    ESP_ELFSYM_EXPORT(SysTickIsrHandler),
    ESP_ELFSYM_EXPORT(strcat),
    ESP_ELFSYM_EXPORT(__lock___arc4random_mutex),
    ESP_ELFSYM_EXPORT(__atomic_s32c1i_compare_exchange_1),
    ESP_ELFSYM_EXPORT(spi_flash_op_lock),
    ESP_ELFSYM_EXPORT(heap_caps_get_info),
    ESP_ELFSYM_EXPORT(nd6_get_destination_mtu),
    ESP_ELFSYM_EXPORT(vPortYieldFromInt),
    ESP_ELFSYM_EXPORT(__ssrefill_r),
    ESP_ELFSYM_EXPORT(spi_flash_guard_set),
    ESP_ELFSYM_EXPORT(rtc_clk_cal_internal),
    ESP_ELFSYM_EXPORT(esp_flash_chip_mxic),
    ESP_ELFSYM_EXPORT(mmu_hal_unmap_region),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_delconn),
    ESP_ELFSYM_EXPORT(rtc_clk_freq_to_period),
    ESP_ELFSYM_EXPORT(rtc_clk_8md256_enabled),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_read),
    ESP_ELFSYM_EXPORT(__muldf3),
    ESP_ELFSYM_EXPORT(mspi_timing_psram_select_best_tuning_config),
    ESP_ELFSYM_EXPORT(heap_caps_malloc_prefer),
    ESP_ELFSYM_EXPORT(esp_system_get_time),
    ESP_ELFSYM_EXPORT(sys_mutex_unlock),
    ESP_ELFSYM_EXPORT(xRingbufferReceive),
    ESP_ELFSYM_EXPORT(udp_bind),
    ESP_ELFSYM_EXPORT(uart_set_select_notif_callback),
    ESP_ELFSYM_EXPORT(_Level4Vector),
    ESP_ELFSYM_EXPORT(esp_flash_registered_chips),
    ESP_ELFSYM_EXPORT(strtoul_l),
    ESP_ELFSYM_EXPORT(xt_ints_off),
    ESP_ELFSYM_EXPORT(tcp_backlog_accepted),
    ESP_ELFSYM_EXPORT(spi_bus_lock_bg_entry),
    ESP_ELFSYM_EXPORT(_lock_release_recursive),
    ESP_ELFSYM_EXPORT(_daylight),
    ESP_ELFSYM_EXPORT(spi_flash_common_write_status_8b_wrsr2),
    ESP_ELFSYM_EXPORT(multi_heap_aligned_alloc_offs),
    ESP_ELFSYM_EXPORT(offset_pxEndOfStack),
    ESP_ELFSYM_EXPORT(esp_mspi_32bit_address_flash_feature_check),
    ESP_ELFSYM_EXPORT(optind),
    ESP_ELFSYM_EXPORT(esp_timer_init_include_func),
    ESP_ELFSYM_EXPORT(ledc_periph_signal),
    ESP_ELFSYM_EXPORT(__env_unlock),
    ESP_ELFSYM_EXPORT(rtc_clk_32k_enable),
    ESP_ELFSYM_EXPORT(__subdf3),
    ESP_ELFSYM_EXPORT(ledc_timer_resume),
    ESP_ELFSYM_EXPORT(panic_print_str),
    ESP_ELFSYM_EXPORT(__sclose),
    ESP_ELFSYM_EXPORT(tcp_free),
    ESP_ELFSYM_EXPORT(rtc_clk_bbpll_remove_consumer),
    ESP_ELFSYM_EXPORT(spi_device_release_bus),
    ESP_ELFSYM_EXPORT(spi_bus_lock_bg_check_dev_acq),
    ESP_ELFSYM_EXPORT(bzero),
    ESP_ELFSYM_EXPORT(spi_hal_sct_set_conf_bits_len),
    ESP_ELFSYM_EXPORT(_timezone),
    ESP_ELFSYM_EXPORT(__mprec_tens),
    ESP_ELFSYM_EXPORT(netconn_err),
    ESP_ELFSYM_EXPORT(tcp_input_pcb),
    ESP_ELFSYM_EXPORT(sys_thread_sem_init),
    ESP_ELFSYM_EXPORT(esp_log_util_set_cache_enabled_cb),
    ESP_ELFSYM_EXPORT(raw_send),
    ESP_ELFSYM_EXPORT(esp_vfs_select_triggered),
    ESP_ELFSYM_EXPORT(__truncdfsf2),
    ESP_ELFSYM_EXPORT(tcsetattr),
    ESP_ELFSYM_EXPORT(esp_log_timestamp),
    ESP_ELFSYM_EXPORT(esp_clk_tree_rc_fast_get_freq_hz),
    ESP_ELFSYM_EXPORT(esp_efuse_startup_include_func),
    ESP_ELFSYM_EXPORT(spi_flash_enable_interrupts_caches_and_other_cpu),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_write),
    ESP_ELFSYM_EXPORT(regi2c_ctrl_write_reg),
    ESP_ELFSYM_EXPORT(esp_cache_resume_ext_mem_cache),
    ESP_ELFSYM_EXPORT(pbuf_cat),
    ESP_ELFSYM_EXPORT(ulTaskGenericNotifyTake),
    ESP_ELFSYM_EXPORT(multi_heap_realloc_impl),
    ESP_ELFSYM_EXPORT(mspi_timing_config_psram_read_data),
    ESP_ELFSYM_EXPORT(tcp_free_ooseq),
    ESP_ELFSYM_EXPORT(spi_flash_chip_issi_set_io_mode),
    ESP_ELFSYM_EXPORT(pbuf_clen),
    ESP_ELFSYM_EXPORT(__kernel_cos),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_reset),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_set_burst_size),
    ESP_ELFSYM_EXPORT(esp_vfs_usb_serial_jtag_use_driver),
    ESP_ELFSYM_EXPORT(vPortTCBPreDeleteHook),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_config_host_io_mode),
    ESP_ELFSYM_EXPORT(atoi),
    ESP_ELFSYM_EXPORT(gpio_intr_enable),
    ESP_ELFSYM_EXPORT(_fputs_r),
    ESP_ELFSYM_EXPORT(gdma_hal_enable_access_encrypt_mem),
    ESP_ELFSYM_EXPORT(spicommon_irqsource_for_host),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_read_intr_status),
    ESP_ELFSYM_EXPORT(rtc_clk_32k_disable_external),
    ESP_ELFSYM_EXPORT(spi_flash_munmap),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_driver_install),
    ESP_ELFSYM_EXPORT(soc_get_available_memory_regions),
    ESP_ELFSYM_EXPORT(esp_pthread_set_cfg),
    ESP_ELFSYM_EXPORT(tcp_pcb_purge),
    ESP_ELFSYM_EXPORT(mspi_timing_psram_init),
    ESP_ELFSYM_EXPORT(__fixunsdfsi),
    ESP_ELFSYM_EXPORT(esp_intr_alloc_intrstatus_bind),
    ESP_ELFSYM_EXPORT(prvTaskCreateDynamicPinnedToCoreWithCaps),
    ESP_ELFSYM_EXPORT(strftime),
    ESP_ELFSYM_EXPORT(spi_flash_hal_check_status),
    ESP_ELFSYM_EXPORT(igmp_report_groups),
    ESP_ELFSYM_EXPORT(ets_isr_mask),
    ESP_ELFSYM_EXPORT(_sprintf_r),
    ESP_ELFSYM_EXPORT(esp_deregister_freertos_idle_hook_for_cpu),
    ESP_ELFSYM_EXPORT(spi_hal_get_intr_mask),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_vfs_register),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_write),
    ESP_ELFSYM_EXPORT(opendir),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_configure_host_io_mode),
    ESP_ELFSYM_EXPORT(esp_rtc_get_time_us),
    ESP_ELFSYM_EXPORT(optopt),
    ESP_ELFSYM_EXPORT(__cxx_init_dummy),
    ESP_ELFSYM_EXPORT(tcp_eff_send_mss_netif),
    ESP_ELFSYM_EXPORT(tcdrain),
    ESP_ELFSYM_EXPORT(pthread_setspecific),
    ESP_ELFSYM_EXPORT(clk_hal_lp_slow_get_freq_hz),
    ESP_ELFSYM_EXPORT(__wrap__Unwind_DeleteException),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_supports_direct_write),
    ESP_ELFSYM_EXPORT(spi_flash_hal_device_config),
    ESP_ELFSYM_EXPORT(vTaskSuspend),
    ESP_ELFSYM_EXPORT(xthal_set_intclear),
    ESP_ELFSYM_EXPORT(fwrite),
    ESP_ELFSYM_EXPORT(esp_cache_err_get_cpuid),
    ESP_ELFSYM_EXPORT(esp_libc_include_reent_syscalls_impl),
    ESP_ELFSYM_EXPORT(_svfprintf_r),
    ESP_ELFSYM_EXPORT(esp_rom_install_uart_printf),
    ESP_ELFSYM_EXPORT(spi_flash_hal_configure_host_io_mode),
    ESP_ELFSYM_EXPORT(heap_caps_init),
    ESP_ELFSYM_EXPORT(spicommon_cs_initialize),
    ESP_ELFSYM_EXPORT(raw_new_ip_type),
    ESP_ELFSYM_EXPORT(esp_cache_freeze_caches_disable_interrupts),
    ESP_ELFSYM_EXPORT(tlsf_get_pool),
    ESP_ELFSYM_EXPORT(puts),
    ESP_ELFSYM_EXPORT(pbuf_clone),
    ESP_ELFSYM_EXPORT(tlsf_pool_overhead),
    ESP_ELFSYM_EXPORT(udp_new_ip_type),
    ESP_ELFSYM_EXPORT(vsnprintf),
    ESP_ELFSYM_EXPORT(rtc_init),
    ESP_ELFSYM_EXPORT(spi_flash_chip_xmic_opi_config_host_io_mode),
    ESP_ELFSYM_EXPORT(spi_flash_chip_xmic_opi_set_io_mode),
    ESP_ELFSYM_EXPORT(__retarget_lock_acquire_recursive),
    ESP_ELFSYM_EXPORT(vPortExitCritical),
    ESP_ELFSYM_EXPORT(strlcpy),
    ESP_ELFSYM_EXPORT(getopt_long),
    ESP_ELFSYM_EXPORT(fopen),
    ESP_ELFSYM_EXPORT(port_xSchedulerRunning),
    ESP_ELFSYM_EXPORT(_fread_r),
    ESP_ELFSYM_EXPORT(systimer_hal_counter_can_stall_by_cpu),
    ESP_ELFSYM_EXPORT(__errno),
    ESP_ELFSYM_EXPORT(multi_heap_get_allocated_size),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_erase_block),
    ESP_ELFSYM_EXPORT(tcp_rst_netif),
    ESP_ELFSYM_EXPORT(_strtoul_r),
    ESP_ELFSYM_EXPORT(_sscanf_r),
    ESP_ELFSYM_EXPORT(do_multicore_settings),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_enable_intr),
    ESP_ELFSYM_EXPORT(spi_flash_timing_is_tuned),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_send),
    ESP_ELFSYM_EXPORT(getc),
    ESP_ELFSYM_EXPORT(Cache_Freeze_DCache_Enable),
    ESP_ELFSYM_EXPORT(chdir),
    ESP_ELFSYM_EXPORT(esp_ipc_call_nonblocking),
    ESP_ELFSYM_EXPORT(strtoull_l),
    ESP_ELFSYM_EXPORT(multi_heap_get_allocated_size_impl),
    ESP_ELFSYM_EXPORT(tcp_segs_free),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_read),
    ESP_ELFSYM_EXPORT(esp_partition_is_flash_region_writable),
    ESP_ELFSYM_EXPORT(spi_bus_lock_get_acquiring_dev),
    ESP_ELFSYM_EXPORT(xRingbufferReceiveUpTo),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_draw_bitmap),
    ESP_ELFSYM_EXPORT(mkdir),
    ESP_ELFSYM_EXPORT(io_mux_force_disable_lp_io_clock),
    ESP_ELFSYM_EXPORT(gdma_hal_set_burst_size),
    ESP_ELFSYM_EXPORT(__divdf3),
    ESP_ELFSYM_EXPORT(getopt_long_only),
    ESP_ELFSYM_EXPORT(netif_ip6_addr_set_state),
    ESP_ELFSYM_EXPORT(igmp_joingroup_netif),
    ESP_ELFSYM_EXPORT(tcp_send_fin),
    ESP_ELFSYM_EXPORT(siscanf),
    ESP_ELFSYM_EXPORT(esp_security_init_include_impl),
    ESP_ELFSYM_EXPORT(tcp_rexmit_rto_prepare),
    ESP_ELFSYM_EXPORT(spi_device_polling_start),
    ESP_ELFSYM_EXPORT(putc),
    ESP_ELFSYM_EXPORT(esp_log),
    ESP_ELFSYM_EXPORT(s_microseconds_offset),
    ESP_ELFSYM_EXPORT(_strtoull_r),
    ESP_ELFSYM_EXPORT(tcp_bound_pcbs),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_disp_on_off),
    ESP_ELFSYM_EXPORT(esp_clk_tree_lp_fast_get_freq_hz),
    ESP_ELFSYM_EXPORT(memp_IGMP_GROUP),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_stop),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_get_intr_status_reg),
    ESP_ELFSYM_EXPORT(tcp_ticks),
    ESP_ELFSYM_EXPORT(tcp_trigger_input_pcb_close),
    ESP_ELFSYM_EXPORT(__getopt_long_r),
    ESP_ELFSYM_EXPORT(systimer_hal_set_tick_rate_ops),
    ESP_ELFSYM_EXPORT(snprintf),
    ESP_ELFSYM_EXPORT(mspi_timing_config_get_flash_extra_dummy),
    ESP_ELFSYM_EXPORT(cache_hal_unfreeze),
    ESP_ELFSYM_EXPORT(tcp_close_ext),
    ESP_ELFSYM_EXPORT(tan),
    ESP_ELFSYM_EXPORT(esp_mprot_get_monitor_en),
    ESP_ELFSYM_EXPORT(spi_flash_common_read_status_8b_rdsr),
    ESP_ELFSYM_EXPORT(mspi_timing_set_pin_drive_strength),
    ESP_ELFSYM_EXPORT(raw_connect),
    ESP_ELFSYM_EXPORT(spi_hal_user_start),
    ESP_ELFSYM_EXPORT(tcp_recv),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_start_with_desc),
    ESP_ELFSYM_EXPORT(__month_lengths),
    ESP_ELFSYM_EXPORT(clk_hal_apb_get_freq_hz),
    ESP_ELFSYM_EXPORT(panic_set_address),
    ESP_ELFSYM_EXPORT(__lock___atexit_recursive_mutex),
    ESP_ELFSYM_EXPORT(esp_log_cache_add),
    ESP_ELFSYM_EXPORT(esp_timer_impl_early_init),
    ESP_ELFSYM_EXPORT(fcntl),
    ESP_ELFSYM_EXPORT(multi_heap_internal_unlock),
    ESP_ELFSYM_EXPORT(_lseek_r),
    ESP_ELFSYM_EXPORT(gpio_pullup_en),
    ESP_ELFSYM_EXPORT(_strtoll_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_list_check),
    ESP_ELFSYM_EXPORT(memp_ND6_QUEUE),
    ESP_ELFSYM_EXPORT(access),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_read_unique_id),
    ESP_ELFSYM_EXPORT(spi_hal_config_io_default_level),
    ESP_ELFSYM_EXPORT(esp_panic_handler_increment_entry_count),
    ESP_ELFSYM_EXPORT(_findenv_r),
    ESP_ELFSYM_EXPORT(xTaskCheckForTimeOut),
    ESP_ELFSYM_EXPORT(__retarget_lock_try_acquire),
    ESP_ELFSYM_EXPORT(esp_cpu_set_breakpoint),
    ESP_ELFSYM_EXPORT(_xt_context_restore),
    ESP_ELFSYM_EXPORT(pthread_attr_init),
    ESP_ELFSYM_EXPORT(vTaskGenericNotifyGiveFromISR),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_connection_monitor_include),
    ESP_ELFSYM_EXPORT(tcpip_api_call),
    ESP_ELFSYM_EXPORT(vRingbufferReturnItemFromISR),
    ESP_ELFSYM_EXPORT(xTaskIncrementTick),
    ESP_ELFSYM_EXPORT(cache_hal_suspend),
    ESP_ELFSYM_EXPORT(tcp_enqueue_flags),
    ESP_ELFSYM_EXPORT(_raise_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_get_caps),
    ESP_ELFSYM_EXPORT(vSemaphoreDeleteWithCaps),
    ESP_ELFSYM_EXPORT(__ledf2),
    ESP_ELFSYM_EXPORT(gpio_set_level),
    ESP_ELFSYM_EXPORT(esp_heap_adjust_alignment_to_hw),
    ESP_ELFSYM_EXPORT(esp_gpio_reserve),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_is_driver_installed),
    ESP_ELFSYM_EXPORT(_getenv_r),
    ESP_ELFSYM_EXPORT(esp_backtrace_get_next_frame),
    ESP_ELFSYM_EXPORT(mspi_timing_config_psram_set_tuning_regs),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_supports_direct_read),
    ESP_ELFSYM_EXPORT(_putc1),
    ESP_ELFSYM_EXPORT(__floatsidf),
    ESP_ELFSYM_EXPORT(_fopen_r),
    ESP_ELFSYM_EXPORT(esp_system_reset_modules_on_exit),
    ESP_ELFSYM_EXPORT(mspi_timing_change_speed_mode_cache_safe),
    ESP_ELFSYM_EXPORT(Xthal_intlevel),
    ESP_ELFSYM_EXPORT(cache_hal_enable),
    ESP_ELFSYM_EXPORT(esp_mmu_map),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_page_program),
    ESP_ELFSYM_EXPORT(heap_caps_aligned_alloc_base),
    ESP_ELFSYM_EXPORT(_xt_panic),
    ESP_ELFSYM_EXPORT(__submore),
    ESP_ELFSYM_EXPORT(panic_arch_fill_info),
    ESP_ELFSYM_EXPORT(int_wdt_cpu1_ticked),
    ESP_ELFSYM_EXPORT(spi_bus_lock_get_dev_id),
    ESP_ELFSYM_EXPORT(heap_caps_aligned_alloc),
    ESP_ELFSYM_EXPORT(gpio_iomux_output),
    ESP_ELFSYM_EXPORT(rtc_clk_slow_freq_get_hz),
    ESP_ELFSYM_EXPORT(netbuf_delete),
    ESP_ELFSYM_EXPORT(gdma_connect),
    ESP_ELFSYM_EXPORT(mld6_tmr),
    ESP_ELFSYM_EXPORT(__gethex),
    ESP_ELFSYM_EXPORT(esp_mprot_get_default_main_split_addr),
    ESP_ELFSYM_EXPORT(offset_cpsa),
    ESP_ELFSYM_EXPORT(memchr),
    ESP_ELFSYM_EXPORT(xQueueGenericSend),
    ESP_ELFSYM_EXPORT(__b2d),
    ESP_ELFSYM_EXPORT(rtc_gpio_is_valid_gpio),
    ESP_ELFSYM_EXPORT(_ungetc_r),
    ESP_ELFSYM_EXPORT(port_uxOldInterruptState),
    ESP_ELFSYM_EXPORT(_fflush_r),
    ESP_ELFSYM_EXPORT(sys_timeout),
    ESP_ELFSYM_EXPORT(esp_vfs_lseek),
    ESP_ELFSYM_EXPORT(panic_print_backtrace),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_probe),
    ESP_ELFSYM_EXPORT(write),
    ESP_ELFSYM_EXPORT(esp_mprot_set_monitor_en),
    ESP_ELFSYM_EXPORT(strcasecmp),
    ESP_ELFSYM_EXPORT(__ascii_mbtowc),
    ESP_ELFSYM_EXPORT(vfprintf),
    ESP_ELFSYM_EXPORT(esp_flash_chip_boya),
    ESP_ELFSYM_EXPORT(spi_flash_chip_issi_get_caps),
    ESP_ELFSYM_EXPORT(__swbuf_r),
    ESP_ELFSYM_EXPORT(vPortYield),
    ESP_ELFSYM_EXPORT(netconn_write_vectors_partly),
    ESP_ELFSYM_EXPORT(gpio_pulldown_en),
    ESP_ELFSYM_EXPORT(esp_ipc_func),
    ESP_ELFSYM_EXPORT(esp_vfs_link),
    ESP_ELFSYM_EXPORT(pthread_join),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_vfs_use_driver),
    ESP_ELFSYM_EXPORT(__env_lock),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_driver_uninstall),
    ESP_ELFSYM_EXPORT(esp_vfs_closedir),
    ESP_ELFSYM_EXPORT(__hexdig_fun),
    ESP_ELFSYM_EXPORT(esp_log_is_tag_loggable),
    ESP_ELFSYM_EXPORT(esp_cpu_compare_and_set),
    ESP_ELFSYM_EXPORT(spi_bus_lock_bg_request),
    ESP_ELFSYM_EXPORT(ungetc),
    ESP_ELFSYM_EXPORT(cos),
    ESP_ELFSYM_EXPORT(_Level5Vector),
    ESP_ELFSYM_EXPORT(xQueueReceive),
    ESP_ELFSYM_EXPORT(__nedf2),
    ESP_ELFSYM_EXPORT(__ssvfscanf_r),
    ESP_ELFSYM_EXPORT(__swsetup_r),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_recv),
    ESP_ELFSYM_EXPORT(Cache_Freeze_ICache_Enable),
    ESP_ELFSYM_EXPORT(ip6_output_if),
    ESP_ELFSYM_EXPORT(udp_recv),
    ESP_ELFSYM_EXPORT(ip6_route),
    ESP_ELFSYM_EXPORT(uart_get_buffered_data_len),
    ESP_ELFSYM_EXPORT(_fseeko_r),
    ESP_ELFSYM_EXPORT(ilg_int_st_bits),
    ESP_ELFSYM_EXPORT(memspi_host_read_data_slicer),
    ESP_ELFSYM_EXPORT(mspi_timing_flash_tuning),
    ESP_ELFSYM_EXPORT(cache_hal_vaddr_to_cache_level_id),
    ESP_ELFSYM_EXPORT(esp_intr_noniram_disable),
    ESP_ELFSYM_EXPORT(tlsf_free),
    ESP_ELFSYM_EXPORT(spi_flash_hal_common_command),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_wait_tx_done),
    ESP_ELFSYM_EXPORT(_impure_ptr),
    ESP_ELFSYM_EXPORT(__sfp_lock_acquire),
    ESP_ELFSYM_EXPORT(esp_ipc_isr_handler),
    ESP_ELFSYM_EXPORT(heap_caps_malloc_base),
    ESP_ELFSYM_EXPORT(tcpip_callback),
    ESP_ELFSYM_EXPORT(uart_set_word_length),
    ESP_ELFSYM_EXPORT(ip_addr_any),
    ESP_ELFSYM_EXPORT(_mbrtowc_r),
    ESP_ELFSYM_EXPORT(esp_mprot_set_pms_lock),
    ESP_ELFSYM_EXPORT(g_rtc_dbias_pvt_240m),
    ESP_ELFSYM_EXPORT(pthread_create),
    ESP_ELFSYM_EXPORT(_setlocale_r),
    ESP_ELFSYM_EXPORT(mspi_timing_config_get_flash_clock_reg),
    ESP_ELFSYM_EXPORT(pbuf_get_at),
    ESP_ELFSYM_EXPORT(pthread_include_pthread_semaphore_impl),
    ESP_ELFSYM_EXPORT(pvTaskIncrementMutexHeldCount),
    ESP_ELFSYM_EXPORT(lwip_netconn_do_newconn),
    ESP_ELFSYM_EXPORT(gettimeofday),
    ESP_ELFSYM_EXPORT(periph_module_enable),
    ESP_ELFSYM_EXPORT(esp_ptr_executable),
    ESP_ELFSYM_EXPORT(tcp_timer_needed),
    ESP_ELFSYM_EXPORT(spi_bus_lock_set_bg_control),
    ESP_ELFSYM_EXPORT(rtc_clk_8m_enable),
    ESP_ELFSYM_EXPORT(lwip_htonl),
    ESP_ELFSYM_EXPORT(uart_get_selectlock),
    ESP_ELFSYM_EXPORT(time),
    ESP_ELFSYM_EXPORT(mspi_timing_config_psram_prepare_reference_data),
    ESP_ELFSYM_EXPORT(cache_hal_init),
    ESP_ELFSYM_EXPORT(fseek),
    ESP_ELFSYM_EXPORT(esp_cache_unfreeze_ext_mem_cache),
    ESP_ELFSYM_EXPORT(__wrap_stat),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_detect_size),
    ESP_ELFSYM_EXPORT(__srget_r),
    ESP_ELFSYM_EXPORT(vTaskDelay),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_probe),
    ESP_ELFSYM_EXPORT(__atexit0),
    ESP_ELFSYM_EXPORT(spi_flash_hal_read),
    ESP_ELFSYM_EXPORT(gdma_start),
    ESP_ELFSYM_EXPORT(strncpy),
    ESP_ELFSYM_EXPORT(memcmp),
    ESP_ELFSYM_EXPORT(__floatundidf),
    ESP_ELFSYM_EXPORT(fclose),
    ESP_ELFSYM_EXPORT(esp_cache_freeze_ext_mem_cache),
    ESP_ELFSYM_EXPORT(memp_NETCONN),
    ESP_ELFSYM_EXPORT(esp_flash_init_main),
    ESP_ELFSYM_EXPORT(__getreent),
    ESP_ELFSYM_EXPORT(s_cache_hal_init_ctx),
    ESP_ELFSYM_EXPORT(gdma_hal_enable_burst),
    ESP_ELFSYM_EXPORT(netconn_getaddr),
    ESP_ELFSYM_EXPORT(spi_bus_initialize),
    ESP_ELFSYM_EXPORT(spi_flash_set_vendor_required_regs),
    ESP_ELFSYM_EXPORT(esp_rtc_init),
    ESP_ELFSYM_EXPORT(uart_get_stop_bits),
    ESP_ELFSYM_EXPORT(mspi_timing_enter_low_speed_mode),
    ESP_ELFSYM_EXPORT(spi_flash_brownout_need_reset),
    ESP_ELFSYM_EXPORT(heap_caps_realloc_base),
    ESP_ELFSYM_EXPORT(_Balloc),
    ESP_ELFSYM_EXPORT(esp_clk_tree_enable_src),
    ESP_ELFSYM_EXPORT(esp_brownout_init),
    ESP_ELFSYM_EXPORT(esp_app_get_elf_sha256),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_read_id),
    ESP_ELFSYM_EXPORT(strdup),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_opi_read_reg),
    ESP_ELFSYM_EXPORT(uxQueueMessagesWaiting),
    ESP_ELFSYM_EXPORT(xQueueSemaphoreTake),
    ESP_ELFSYM_EXPORT(tlsf_fit_size),
    ESP_ELFSYM_EXPORT(ip6_chksum_pseudo),
    ESP_ELFSYM_EXPORT(xRingbufferSend),
    ESP_ELFSYM_EXPORT(ledc_hal_set_duty_int_part),
    ESP_ELFSYM_EXPORT(esp_time_impl_get_time_since_boot),
    ESP_ELFSYM_EXPORT(__lock___dd_hash_mutex),
    ESP_ELFSYM_EXPORT(_strtod_l),
    ESP_ELFSYM_EXPORT(esp_sleep_gpio_include),
    ESP_ELFSYM_EXPORT(tlsf_add_pool),
    ESP_ELFSYM_EXPORT(esp_vfs_unlink),
    ESP_ELFSYM_EXPORT(rtc_clk_cpu_set_to_default_config),
    ESP_ELFSYM_EXPORT(heap_caps_add_region_with_caps),
    ESP_ELFSYM_EXPORT(__sflush_r),
    ESP_ELFSYM_EXPORT(esp_mprot_get_monitor_lock),
    ESP_ELFSYM_EXPORT(_fseek_r),
    ESP_ELFSYM_EXPORT(memp_malloc),
    ESP_ELFSYM_EXPORT(ip4_output_if),
    ESP_ELFSYM_EXPORT(__retarget_lock_release_recursive),
    ESP_ELFSYM_EXPORT(esp_intr_enable_source),
    ESP_ELFSYM_EXPORT(__fp_lock_all),
    ESP_ELFSYM_EXPORT(systimer_hal_select_alarm_mode),
    ESP_ELFSYM_EXPORT(ip4_input),
    ESP_ELFSYM_EXPORT(ip6_options_add_hbh_ra),
    ESP_ELFSYM_EXPORT(__srget),
    ESP_ELFSYM_EXPORT(ledc_hal_set_duty_start),
    ESP_ELFSYM_EXPORT(memp_free),
    ESP_ELFSYM_EXPORT(esp_mprot_get_pms_area),
    ESP_ELFSYM_EXPORT(memp_TCP_SEG),
    ESP_ELFSYM_EXPORT(spi_flash_chip_gd_suspend_cmd_conf),
    ESP_ELFSYM_EXPORT(lwip_recv),
    ESP_ELFSYM_EXPORT(_strtol_r),
    ESP_ELFSYM_EXPORT(s_keys),
    ESP_ELFSYM_EXPORT(memp_MLD6_GROUP),
    ESP_ELFSYM_EXPORT(_mbtowc_r),
    ESP_ELFSYM_EXPORT(tcpip_try_callback),
    ESP_ELFSYM_EXPORT(panic_abort),
    ESP_ELFSYM_EXPORT(tcp_slowtmr),
    ESP_ELFSYM_EXPORT(efuse_hal_get_minor_chip_version),
    ESP_ELFSYM_EXPORT(multi_heap_malloc),
    ESP_ELFSYM_EXPORT(_Level3Vector),
    ESP_ELFSYM_EXPORT(ets_isr_unmask),
    ESP_ELFSYM_EXPORT(console_tcflush),
    ESP_ELFSYM_EXPORT(gpio_hal_iomux_out),
    ESP_ELFSYM_EXPORT(sys_arch_unprotect),
    ESP_ELFSYM_EXPORT(udp_send),
    ESP_ELFSYM_EXPORT(vTaskSuspendAll),
    ESP_ELFSYM_EXPORT(esp_elf_relocate),
    ESP_ELFSYM_EXPORT(esp_register_freertos_tick_hook),
    ESP_ELFSYM_EXPORT(sys_mbox_post),
    ESP_ELFSYM_EXPORT(xPortCheckValidTCBMem),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_get_caps),
    ESP_ELFSYM_EXPORT(gpio_intr_disable),
    ESP_ELFSYM_EXPORT(tcp_tw_pcbs),
    ESP_ELFSYM_EXPORT(_lock_close_recursive),
    ESP_ELFSYM_EXPORT(_vsnprintf_r),
    ESP_ELFSYM_EXPORT(_realloc_r),
    ESP_ELFSYM_EXPORT(vTaskSetThreadLocalStoragePointerAndDelCallback),
    ESP_ELFSYM_EXPORT(esp_task_wdt_impl_timer_restart),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_suspend_cmd_conf),
    ESP_ELFSYM_EXPORT(Cache_Suspend_ICache),
    ESP_ELFSYM_EXPORT(uxTopUsedPriority),
    ESP_ELFSYM_EXPORT(esp_libc_include_syscalls_impl),
    ESP_ELFSYM_EXPORT(tcp_seg_copy),
    ESP_ELFSYM_EXPORT(offset_xCoreID),
    ESP_ELFSYM_EXPORT(esp_int_wdt_cpu_init),
    ESP_ELFSYM_EXPORT(_lock_acquire_recursive),
    ESP_ELFSYM_EXPORT(__strtok_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_th_get_caps),
    ESP_ELFSYM_EXPORT(spi_flash_encryption_hal_enable),
    ESP_ELFSYM_EXPORT(spi_periph_signal),
    ESP_ELFSYM_EXPORT(rtc_clk_apb_freq_update),
    ESP_ELFSYM_EXPORT(console_write),
    ESP_ELFSYM_EXPORT(spi_bus_get_dma_ctx),
    ESP_ELFSYM_EXPORT(esp_cache_err_acs_save_and_clr),
    ESP_ELFSYM_EXPORT(tlsf_create_with_pool),
    ESP_ELFSYM_EXPORT(__smakebuf_r),
    ESP_ELFSYM_EXPORT(esp_log_impl_unlock),
    ESP_ELFSYM_EXPORT(ledc_set_duty_with_hpoint),
    ESP_ELFSYM_EXPORT(_putc_r),
    ESP_ELFSYM_EXPORT(__stdio_exit_handler),
    ESP_ELFSYM_EXPORT(esp_sleep_enable_gpio_switch),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_device_config),
    ESP_ELFSYM_EXPORT(g_mmu_mem_regions),
    ESP_ELFSYM_EXPORT(ledc_channel_config),
    ESP_ELFSYM_EXPORT(memp_ARP_QUEUE),
    ESP_ELFSYM_EXPORT(sys_arch_mbox_tryfetch),
    ESP_ELFSYM_EXPORT(_frxt_int_exit),
    ESP_ELFSYM_EXPORT(reachable_time),
    ESP_ELFSYM_EXPORT(xt_unhandled_exception),
    ESP_ELFSYM_EXPORT(netconn_bind),
    ESP_ELFSYM_EXPORT(tcp_write),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_get_io_mode),
    ESP_ELFSYM_EXPORT(esp_flash_chip_th),
    ESP_ELFSYM_EXPORT(raw_sendto),
    ESP_ELFSYM_EXPORT(tcp_listen_pcbs),
    ESP_ELFSYM_EXPORT(esp_vfs_include_nullfs_register),
    ESP_ELFSYM_EXPORT(strcasestr),
    ESP_ELFSYM_EXPORT(esp_vfs_null_register),
    ESP_ELFSYM_EXPORT(_xt_coproc_savecs),
    ESP_ELFSYM_EXPORT(udp_sendto_if_src),
    ESP_ELFSYM_EXPORT(cfree),
    ESP_ELFSYM_EXPORT(_strtod_r),
    ESP_ELFSYM_EXPORT(acs_int_st_bits),
    ESP_ELFSYM_EXPORT(sys_mbox_trypost),
    ESP_ELFSYM_EXPORT(pthread_include_pthread_local_storage_impl),
    ESP_ELFSYM_EXPORT(tcp_next_iss),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_read_reg),
    ESP_ELFSYM_EXPORT(ip_chksum_pseudo),
    ESP_ELFSYM_EXPORT(ledc_timer_config),
    ESP_ELFSYM_EXPORT(sqrt),
    ESP_ELFSYM_EXPORT(xQueueTakeMutexRecursive),
    ESP_ELFSYM_EXPORT(vTaskSetTimeOutState),
    ESP_ELFSYM_EXPORT(tlsf_walk_pool),
    ESP_ELFSYM_EXPORT(bootloader_execute_flash_command),
    ESP_ELFSYM_EXPORT(rom_flash_chip_dummy),
    ESP_ELFSYM_EXPORT(pcTaskGetName),
    ESP_ELFSYM_EXPORT(xQueueReceiveFromISR),
    ESP_ELFSYM_EXPORT(udp_sendto_if),
    ESP_ELFSYM_EXPORT(mld6_joingroup_netif),
    ESP_ELFSYM_EXPORT(__fixdfdi),
    ESP_ELFSYM_EXPORT(_times_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_erase_sector),
    ESP_ELFSYM_EXPORT(sys_mbox_free),
    ESP_ELFSYM_EXPORT(fsync),
    ESP_ELFSYM_EXPORT(tcp_backlog_delayed),
    ESP_ELFSYM_EXPORT(strtof),
    ESP_ELFSYM_EXPORT(uart_is_driver_installed),
    ESP_ELFSYM_EXPORT(spi_flash_chip_mxic_get_caps),
    ESP_ELFSYM_EXPORT(memp_PBUF),
    ESP_ELFSYM_EXPORT(tlsf_memalign_offs),
    ESP_ELFSYM_EXPORT(_ctype_),
    ESP_ELFSYM_EXPORT(memspi_host_erase_sector),
    ESP_ELFSYM_EXPORT(spi_bus_lock_bg_check_dev_req),
    ESP_ELFSYM_EXPORT(multi_heap_register),
    ESP_ELFSYM_EXPORT(__retarget_lock_init),
    ESP_ELFSYM_EXPORT(esp_ipc_func_arg),
    ESP_ELFSYM_EXPORT(spi_flash_hal_setup_auto_suspend_mode),
    ESP_ELFSYM_EXPORT(getopt),
    ESP_ELFSYM_EXPORT(strtoul),
    ESP_ELFSYM_EXPORT(esp_log_cache_get_level),
    ESP_ELFSYM_EXPORT(memp_pools),
    ESP_ELFSYM_EXPORT(console_read),
    ESP_ELFSYM_EXPORT(spi_flash_hal_init),
    ESP_ELFSYM_EXPORT(uart_flush),
    ESP_ELFSYM_EXPORT(memspi_host_read_status_hs),
    ESP_ELFSYM_EXPORT(pbuf_free_header),
    ESP_ELFSYM_EXPORT(iswspace),
    ESP_ELFSYM_EXPORT(_frxt_coproc_exc_hook),
    ESP_ELFSYM_EXPORT(esp_stack_ptr_in_extram),
    ESP_ELFSYM_EXPORT(open),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_suspend_cmd_conf),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_read),
    ESP_ELFSYM_EXPORT(gdma_hal_clear_intr),
    ESP_ELFSYM_EXPORT(esp_flash_get_physical_size),
    ESP_ELFSYM_EXPORT(esp_timer_early_init),
    ESP_ELFSYM_EXPORT(spi_flash_init_lock),
    ESP_ELFSYM_EXPORT(sprintf),
    ESP_ELFSYM_EXPORT(spi_flash_set_erasing_flag),
    ESP_ELFSYM_EXPORT(__localeconv_l),
    ESP_ELFSYM_EXPORT(ip6_input),
    ESP_ELFSYM_EXPORT(iswspace_l),
    ESP_ELFSYM_EXPORT(esp_mspi_get_io),
    ESP_ELFSYM_EXPORT(__wrap_longjmp),
    ESP_ELFSYM_EXPORT(__lock___malloc_recursive_mutex),
    ESP_ELFSYM_EXPORT(memp_UDP_PCB),
    ESP_ELFSYM_EXPORT(spi_flash_chip_issi_probe),
    ESP_ELFSYM_EXPORT(uart_set_parity),
    ESP_ELFSYM_EXPORT(tcp_keepalive),
    ESP_ELFSYM_EXPORT(__kernel_rem_pio2),
    ESP_ELFSYM_EXPORT(sleep),
    ESP_ELFSYM_EXPORT(xPortSysTickHandler),
    ESP_ELFSYM_EXPORT(esp_mprot_is_conf_locked_any),
    ESP_ELFSYM_EXPORT(Cache_Suspend_DCache),
    ESP_ELFSYM_EXPORT(mspi_timing_config_set_flash_clock),
    ESP_ELFSYM_EXPORT(gpio_hal_intr_enable_on_core),
    ESP_ELFSYM_EXPORT(uart_hal_get_parity),
    ESP_ELFSYM_EXPORT(esp_intr_noniram_enable),
    ESP_ELFSYM_EXPORT(gpio_set_direction),
    ESP_ELFSYM_EXPORT(sys_arch_protect),
    ESP_ELFSYM_EXPORT(esp_libc_time_init),
    ESP_ELFSYM_EXPORT(_dtoa_r),
    ESP_ELFSYM_EXPORT(_frxt_task_coproc_state),
    ESP_ELFSYM_EXPORT(spi_device_get_trans_result),
    ESP_ELFSYM_EXPORT(__sseek),
    ESP_ELFSYM_EXPORT(spi_flash_chip_boya_probe),
    ESP_ELFSYM_EXPORT(mspi_timing_config_get_cs_timing),
    ESP_ELFSYM_EXPORT(__any_on),
    ESP_ELFSYM_EXPORT(fseeko),
    ESP_ELFSYM_EXPORT(xTaskGetSchedulerState),
    ESP_ELFSYM_EXPORT(spi_flash_cache2phys),
    ESP_ELFSYM_EXPORT(__state_table),
    ESP_ELFSYM_EXPORT(esp_int_wdt_init),
    ESP_ELFSYM_EXPORT(_stat_r),
    ESP_ELFSYM_EXPORT(pbuf_alloc_reference),
    ESP_ELFSYM_EXPORT(esp_elf_map_sym),
    ESP_ELFSYM_EXPORT(__retarget_lock_close_recursive),
    ESP_ELFSYM_EXPORT(uart_get_baudrate),
    ESP_ELFSYM_EXPORT(rom_spiflash_api_funcs),
    ESP_ELFSYM_EXPORT(esp_err_to_name),
    ESP_ELFSYM_EXPORT(tcp_update_rcv_ann_wnd),
    ESP_ELFSYM_EXPORT(__gtdf2),
    ESP_ELFSYM_EXPORT(soc_memory_region_count),
    ESP_ELFSYM_EXPORT(gdma_ahb_hal_append),
    ESP_ELFSYM_EXPORT(_impure_data),
    ESP_ELFSYM_EXPORT(sscanf),
    ESP_ELFSYM_EXPORT(esp_panic_handler_feed_wdts),
    ESP_ELFSYM_EXPORT(pbuf_header_force),
    ESP_ELFSYM_EXPORT(strlen),
    ESP_ELFSYM_EXPORT(memspi_host_program_page),
    ESP_ELFSYM_EXPORT(strtod),
    ESP_ELFSYM_EXPORT(panic_soc_fill_info),
    ESP_ELFSYM_EXPORT(esp_rom_opiflash_cache_mode_config),
    ESP_ELFSYM_EXPORT(_xt_medint3_exit),
    ESP_ELFSYM_EXPORT(__popcountsi2),
    ESP_ELFSYM_EXPORT(tcp_split_unsent_seg),
    ESP_ELFSYM_EXPORT(__tz_unlock),
    ESP_ELFSYM_EXPORT(spi_bus_add_device),
    ESP_ELFSYM_EXPORT(__seofread),
    ESP_ELFSYM_EXPORT(strtol),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_reset),
    ESP_ELFSYM_EXPORT(spi_flash_encryption_hal_check),
    ESP_ELFSYM_EXPORT(mspi_timing_psram_config_set_tuning_regs),
    ESP_ELFSYM_EXPORT(gdma_hal_reset),
    ESP_ELFSYM_EXPORT(esp_vfs_read),
    ESP_ELFSYM_EXPORT(rtc_gpio_deinit),
    ESP_ELFSYM_EXPORT(vTaskPlaceOnEventList),
    ESP_ELFSYM_EXPORT(strncat),
    ESP_ELFSYM_EXPORT(memp_TCPIP_MSG_API),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_read_bytes),
    ESP_ELFSYM_EXPORT(memp_RAW_PCB),
    ESP_ELFSYM_EXPORT(uart_hal_get_stop_bits),
    ESP_ELFSYM_EXPORT(vTaskSwitchContext),
    ESP_ELFSYM_EXPORT(xTaskGetCurrentTaskHandleForCore),
    ESP_ELFSYM_EXPORT(itoa),
    ESP_ELFSYM_EXPORT(tcp_alloc),
    ESP_ELFSYM_EXPORT(tcp_sent),
    ESP_ELFSYM_EXPORT(__ulp),
    ESP_ELFSYM_EXPORT(_fputc_r),
    ESP_ELFSYM_EXPORT(_read_r),
    ESP_ELFSYM_EXPORT(spi_flash_init_chip_state),
    ESP_ELFSYM_EXPORT(__srefill_r),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_erase_chip),
    ESP_ELFSYM_EXPORT(spi_flash_cache_enabled),
    ESP_ELFSYM_EXPORT(esp_libc_include_assert_impl),
    ESP_ELFSYM_EXPORT(_UserExceptionVector),
    ESP_ELFSYM_EXPORT(__ubsan_include),
    ESP_ELFSYM_EXPORT(netconn_tcp_recvd),
    ESP_ELFSYM_EXPORT(xTaskPriorityDisinherit),
    ESP_ELFSYM_EXPORT(udp_sendto),
    ESP_ELFSYM_EXPORT(spi_hal_fetch_result),
    ESP_ELFSYM_EXPORT(esp_clk_apb_freq),
    ESP_ELFSYM_EXPORT(sniprintf),
    ESP_ELFSYM_EXPORT(sys_untimeout),
    ESP_ELFSYM_EXPORT(icmp_input),
    ESP_ELFSYM_EXPORT(vfiprintf),
    ESP_ELFSYM_EXPORT(esp_cpu_unstall),
    ESP_ELFSYM_EXPORT(spi_flash_chip_winbond_page_program),
    ESP_ELFSYM_EXPORT(_fwalk_sglue),
    ESP_ELFSYM_EXPORT(FreeRTOS_openocd_params),
    ESP_ELFSYM_EXPORT(usb_serial_jtag_read_ready),
    ESP_ELFSYM_EXPORT(setjmp),
    ESP_ELFSYM_EXPORT(bootloader_flash_is_octal_mode_enabled),
    ESP_ELFSYM_EXPORT(ip4addr_aton),
    ESP_ELFSYM_EXPORT(esp_crosscore_int_init),
    ESP_ELFSYM_EXPORT(soc_memory_regions),
    ESP_ELFSYM_EXPORT(mem_trim),
    ESP_ELFSYM_EXPORT(rtc_clk_cpu_freq_get_config),
    ESP_ELFSYM_EXPORT(clearerr),
    ESP_ELFSYM_EXPORT(spi_flash_hal_gpspi_poll_cmd_done),
    ESP_ELFSYM_EXPORT(periph_rtc_dig_clk8m_enable),
    ESP_ELFSYM_EXPORT(esp_log_linked_list_get_level),
    ESP_ELFSYM_EXPORT(__atomic_compare_exchange_1),
    ESP_ELFSYM_EXPORT(_lock_try_acquire),
    ESP_ELFSYM_EXPORT(spi_hal_usr_is_done),
    ESP_ELFSYM_EXPORT(seekdir),
    ESP_ELFSYM_EXPORT(netif_find),
    ESP_ELFSYM_EXPORT(heap_caps_realloc_default),
    ESP_ELFSYM_EXPORT(spi_bus_lock_register_dev),
    ESP_ELFSYM_EXPORT(g_rtc_dbias_pvt_non_240m),
    ESP_ELFSYM_EXPORT(esp_partition_iterator_release),
    ESP_ELFSYM_EXPORT(multi_heap_realloc),
    ESP_ELFSYM_EXPORT(inet_chksum),
    ESP_ELFSYM_EXPORT(rtc_clk_freq_cal),
    ESP_ELFSYM_EXPORT(xPortInIsrContext),
    ESP_ELFSYM_EXPORT(__floatunsidf),
    ESP_ELFSYM_EXPORT(tcgetattr),
    ESP_ELFSYM_EXPORT(_reclaim_reent),
    ESP_ELFSYM_EXPORT(_tzset_unlocked_r),
    ESP_ELFSYM_EXPORT(netbuf_alloc),
    ESP_ELFSYM_EXPORT(__swrite),
    ESP_ELFSYM_EXPORT(vListInsertEnd),
    ESP_ELFSYM_EXPORT(esp_register_freertos_idle_hook_for_cpu),
    ESP_ELFSYM_EXPORT(tcflush),
    ESP_ELFSYM_EXPORT(esp_lcd_panel_init),
    ESP_ELFSYM_EXPORT(localtime_r),
    ESP_ELFSYM_EXPORT(esp_psram_extram_test),
    ESP_ELFSYM_EXPORT(gpio_pulldown_dis),
    ESP_ELFSYM_EXPORT(udp_remove),
    ESP_ELFSYM_EXPORT(bootloader_init_mem),
    ESP_ELFSYM_EXPORT(esp_log_vprint_func),
    ESP_ELFSYM_EXPORT(spi_bus_lock_get_by_id),
    ESP_ELFSYM_EXPORT(esp_crosscore_int_send_twdt_abort),
    ESP_ELFSYM_EXPORT(strftime_l),
    ESP_ELFSYM_EXPORT(xt_int_has_handler),
    ESP_ELFSYM_EXPORT(xTaskPriorityInherit),
    ESP_ELFSYM_EXPORT(ledc_hal_set_hpoint),
    ESP_ELFSYM_EXPORT(spi_hal_init),
    ESP_ELFSYM_EXPORT(esp_flash_chip_mxic_opi),
    ESP_ELFSYM_EXPORT(console_tcsetattr),
    ESP_ELFSYM_EXPORT(esp_panic_handler_enable_rtc_wdt),
    ESP_ELFSYM_EXPORT(ftello),
    ESP_ELFSYM_EXPORT(rtc_clk_cpu_freq_to_xtal),
    ESP_ELFSYM_EXPORT(spi_flash_encryption_hal_prepare),
    ESP_ELFSYM_EXPORT(__wrap___gxx_personality_v0),
    ESP_ELFSYM_EXPORT(__lock___at_quick_exit_mutex),
    ESP_ELFSYM_EXPORT(memp_PBUF_POOL),
    ESP_ELFSYM_EXPORT(console_end_select),
    ESP_ELFSYM_EXPORT(_fwrite_r),
    ESP_ELFSYM_EXPORT(vTaskDelete),
    ESP_ELFSYM_EXPORT(gmtime_r),
    ESP_ELFSYM_EXPORT(_fcntl_r),
    ESP_ELFSYM_EXPORT(__unorddf2),
    ESP_ELFSYM_EXPORT(periph_rcc_acquire_enter),
    ESP_ELFSYM_EXPORT(tcp_rexmit),
    ESP_ELFSYM_EXPORT(tcp_rexmit_rto),
    ESP_ELFSYM_EXPORT(lwip_netconn_is_deallocated_msg),
    ESP_ELFSYM_EXPORT(mbrtowc),
    ESP_ELFSYM_EXPORT(esp_elf_free),
    ESP_ELFSYM_EXPORT(__sf),
    ESP_ELFSYM_EXPORT(floor),
    ESP_ELFSYM_EXPORT(__tz_lock),
    ESP_ELFSYM_EXPORT(_strerror_r),
    ESP_ELFSYM_EXPORT(spi_device_queue_trans),
    ESP_ELFSYM_EXPORT(__call_exitprocs),
    ESP_ELFSYM_EXPORT(ip6_addr_any),
    ESP_ELFSYM_EXPORT(spi_flash_encryption_hal_disable),
    ESP_ELFSYM_EXPORT(pbuf_try_get_at),
    ESP_ELFSYM_EXPORT(raw_input),
    ESP_ELFSYM_EXPORT(console_fstat),
    ESP_ELFSYM_EXPORT(esp_ptr_byte_accessible),
    ESP_ELFSYM_EXPORT(lwip_netconn_is_err_msg),
    ESP_ELFSYM_EXPORT(rtc_clk_xtal_freq_get),
    ESP_ELFSYM_EXPORT(gdma_hal_set_priority),
    ESP_ELFSYM_EXPORT(raw_remove),
    ESP_ELFSYM_EXPORT(memspi_host_erase_block),
    ESP_ELFSYM_EXPORT(optarg),
    ESP_ELFSYM_EXPORT(esp_efuse_utility_check_errors),
    ESP_ELFSYM_EXPORT(esp_random),
    ESP_ELFSYM_EXPORT(systimer_hal_get_counter_value),
    ESP_ELFSYM_EXPORT(esp_flash_read_chip_id),
    ESP_ELFSYM_EXPORT(ledc_hal_init),
    ESP_ELFSYM_EXPORT(spi_flash_hal_disable_auto_resume_mode),
    ESP_ELFSYM_EXPORT(esp_flash_chip_driver_initialized),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_read_unique_id_none),
    ESP_ELFSYM_EXPORT(memp_TCPIP_MSG_INPKT),
    ESP_ELFSYM_EXPORT(esp_clk_tree_xtal32k_get_freq_hz),
    ESP_ELFSYM_EXPORT(gdma_disconnect),
    ESP_ELFSYM_EXPORT(__sccl),
    ESP_ELFSYM_EXPORT(malloc),
    ESP_ELFSYM_EXPORT(nd6_adjust_mld_membership),
    ESP_ELFSYM_EXPORT(cache_hal_is_cache_enabled),
    ESP_ELFSYM_EXPORT(mld6_report_groups),
    ESP_ELFSYM_EXPORT(lwip_hook_tcp_isn),
    ESP_ELFSYM_EXPORT(task_wdt_timeout_abort),
    ESP_ELFSYM_EXPORT(strcmp),
    ESP_ELFSYM_EXPORT(xQueueGiveMutexRecursive),
    ESP_ELFSYM_EXPORT(spicommon_dma_desc_setup_link),
    ESP_ELFSYM_EXPORT(spi_flash_chip_generic_write_encrypted),
    ESP_ELFSYM_EXPORT(spi_flash_check_and_flush_cache),
    ESP_ELFSYM_EXPORT(ftell),
    ESP_ELFSYM_EXPORT(esp_sleep_sub_mode_config),
    ESP_ELFSYM_EXPORT(xt_unhandled_interrupt),
    ESP_ELFSYM_EXPORT(spi_flash_disable_cache),
    ESP_ELFSYM_EXPORT(app_elf_sha256_str),
    ESP_ELFSYM_EXPORT(spi_bus_lock_acquire_start),
    ESP_ELFSYM_EXPORT(settimeofday),
    ESP_ELFSYM_EXPORT(esp_backtrace_print),
    ESP_ELFSYM_EXPORT(spi_bus_remove_device),
    ESP_ELFSYM_EXPORT(rtc_clk_bbpll_add_consumer),
    ESP_ELFSYM_EXPORT(esp_vfs_usb_serial_jtag_get_vfs),
    ESP_ELFSYM_EXPORT(periph_rcc_enter),
    ESP_ELFSYM_EXPORT(err_to_errno),
    ESP_ELFSYM_EXPORT(spi_hal_hw_prepare_rx),
    ESP_ELFSYM_EXPORT(tcp_abandon),
    ESP_ELFSYM_EXPORT(readdir),
    ESP_ELFSYM_END
};
