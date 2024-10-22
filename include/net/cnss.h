/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _NET_CNSS_H_
#define _NET_CNSS_H_

#include <linux/device.h>
#include <linux/pci.h>
#include <linux/skbuff.h>

/* max 20mhz channel count */
#define CNSS_MAX_CH_NUM       45

#define CNSS_MAX_FILE_NAME	  20

#define MAX_FIRMWARE_SIZE (1 * 1024 * 1024)

enum cnss_bus_width_type {
	CNSS_BUS_WIDTH_NONE,
	CNSS_BUS_WIDTH_LOW,
	CNSS_BUS_WIDTH_MEDIUM,
	CNSS_BUS_WIDTH_HIGH
};

/* FW image files */
struct cnss_fw_files {
	char image_file[CNSS_MAX_FILE_NAME];
	char board_data[CNSS_MAX_FILE_NAME];
	char otp_data[CNSS_MAX_FILE_NAME];
	char utf_file[CNSS_MAX_FILE_NAME];
	char utf_board_data[CNSS_MAX_FILE_NAME];
	char epping_file[CNSS_MAX_FILE_NAME];
	char evicted_data[CNSS_MAX_FILE_NAME];
};

struct cnss_wlan_runtime_ops {
	int (*runtime_suspend)(struct pci_dev *pdev);
	int (*runtime_resume)(struct pci_dev *pdev);
};

struct cnss_wlan_driver {
	char *name;
	int  (*probe)(struct pci_dev *pdev, const struct pci_device_id *id);
	void (*remove)(struct pci_dev *pdev);
	int  (*reinit)(struct pci_dev *pdev, const struct pci_device_id *id);
	void (*shutdown)(struct pci_dev *pdev);
	void (*crash_shutdown)(struct pci_dev *pdev);
	int  (*suspend)(struct pci_dev *pdev, pm_message_t state);
	int  (*resume)(struct pci_dev *pdev);
	void (*modem_status)(struct pci_dev *, int state);
	struct cnss_wlan_runtime_ops *runtime_ops;
	const struct pci_device_id *id_table;
};

/*
 * codeseg_total_bytes: Total bytes across all the codesegment blocks
 * num_codesegs: No of Pages used
 * codeseg_size: Size of each segment. Should be power of 2 and multiple of 4K
 * codeseg_size_log2: log2(codeseg_size)
 * codeseg_busaddr: Physical address of the DMAble memory;4K aligned
 */

#define CODESWAP_MAX_CODESEGS 16
struct codeswap_codeseg_info {
	u32   codeseg_total_bytes;
	u32   num_codesegs;
	u32   codeseg_size;
	u32   codeseg_size_log2;
	void *codeseg_busaddr[CODESWAP_MAX_CODESEGS];
};

struct image_desc_info {
	dma_addr_t fw_addr;
	u32 fw_size;
	dma_addr_t bdata_addr;
	u32 bdata_size;
};

/* platform capabilities */
enum cnss_platform_cap_flag {
	CNSS_HAS_EXTERNAL_SWREG = 0x01,
	CNSS_HAS_UART_ACCESS = 0x02,
};

struct cnss_platform_cap {
	u32 cap_flag;
};

/* WLAN driver status */
enum cnss_driver_status {
	CNSS_UNINITIALIZED,
	CNSS_INITIALIZED,
	CNSS_LOAD_UNLOAD
};

enum cnss_runtime_request {
	CNSS_PM_RUNTIME_GET,
	CNSS_PM_RUNTIME_PUT,
	CNSS_PM_RUNTIME_MARK_LAST_BUSY,
	CNSS_PM_RUNTIME_RESUME,
	CNSS_PM_RUNTIME_PUT_NOIDLE,
	CNSS_PM_REQUEST_RESUME,
	CNSS_PM_RUNTIME_PUT_AUTO,
};
/* QMI related */
struct cnss_ce_tgt_pipe_cfg {
	u32 pipe_num;
	u32 pipe_dir;
	u32 nentries;
	u32 nbytes_max;
	u32 flags;
	u32 reserved;
};

struct cnss_ce_svc_pipe_cfg {
	u32 service_id;
	u32 pipe_dir;
	u32 pipe_num;
};

struct cnss_shadow_reg_cfg {
	u16 ce_id;
	u16 reg_offset;
};

/* CE configuration to target */
struct cnss_wlan_enable_cfg {
	u32 num_ce_tgt_cfg;
	struct cnss_ce_tgt_pipe_cfg *ce_tgt_cfg;
	u32 num_ce_svc_pipe_cfg;
	struct cnss_ce_svc_pipe_cfg *ce_svc_cfg;
	u32 num_shadow_reg_cfg;
	struct cnss_shadow_reg_cfg *shadow_reg_cfg;
};

/* MSA Memory Regions Infomation */
struct cnss_mem_region_info {
	uint64_t reg_addr;
	uint32_t size;
	uint8_t secure_flag;
};

/* driver modes */
enum cnss_driver_mode {
	CNSS_MISSION,
	CNSS_FTM,
	CNSS_EPPING,
	CNSS_WALTEST,
	CNSS_OFF,
	CNSS_CCPM,
	CNSS_QVIT,
};

extern int cnss_wlan_enable(struct cnss_wlan_enable_cfg *config,
			    enum cnss_driver_mode mode,
			    const char *host_version);
extern int cnss_wlan_disable(enum cnss_driver_mode mode);
extern int cnss_set_fw_debug_mode(bool enablefwlog);
/* End of QMI */


extern int cnss_get_fw_image(struct image_desc_info *image_desc_info);

extern int cnss_get_fw_image(struct image_desc_info *image_desc_info);
extern void cnss_runtime_init(struct device *dev, int auto_delay);
extern void cnss_runtime_exit(struct device *dev);
extern void cnss_device_crashed(void);
extern void cnss_device_self_recovery(void);
extern int cnss_get_ramdump_mem(unsigned long *address, unsigned long *size);
extern void *cnss_get_virt_ramdump_mem(unsigned long *size);
extern int cnss_set_wlan_unsafe_channel(u16 *unsafe_ch_list, u16 ch_count);
extern int cnss_get_wlan_unsafe_channel(u16 *unsafe_ch_list,
		u16 *ch_count, u16 buf_len);
extern int cnss_wlan_set_dfs_nol(void *info, u16 info_len);
extern int cnss_wlan_get_dfs_nol(void *info, u16 info_len);
extern void cnss_schedule_recovery_work(void);
extern void cnss_wlan_pci_link_down(void);
extern int cnss_pcie_shadow_control(struct pci_dev *dev, bool enable);
extern int cnss_wlan_register_driver(struct cnss_wlan_driver *driver);
extern void cnss_wlan_unregister_driver(struct cnss_wlan_driver *driver);
extern int cnss_get_fw_files(struct cnss_fw_files *pfw_files);
extern int cnss_get_fw_files_for_target(struct cnss_fw_files *pfw_files,
					u32 target_type, u32 target_version);
extern void cnss_flush_work(void *work);
extern void cnss_flush_delayed_work(void *dwork);
extern void cnss_get_monotonic_boottime(struct timespec *ts);
extern void cnss_get_boottime(struct timespec *ts);
extern void cnss_init_work(struct work_struct *work, work_func_t func);
extern void cnss_init_delayed_work(struct delayed_work *work, work_func_t func);
extern int cnss_vendor_cmd_reply(struct sk_buff *skb);
extern int cnss_request_bus_bandwidth(int bandwidth);

#ifdef CONFIG_CNSS_SECURE_FW
extern int cnss_get_sha_hash(const u8 *data, u32 data_len,
					u8 *hash_idx, u8 *out);
extern void *cnss_get_fw_ptr(void);
#endif

extern int cnss_get_codeswap_struct(struct codeswap_codeseg_info *swap_seg);
extern int cnss_get_bmi_setup(void);

extern void cnss_pm_wake_lock_init(struct wakeup_source *ws, const char *name);
extern void cnss_pm_wake_lock(struct wakeup_source *ws);
extern void cnss_pm_wake_lock_timeout(struct wakeup_source *ws, ulong msec);
extern void cnss_pm_wake_lock_release(struct wakeup_source *ws);
extern void cnss_pm_wake_lock_destroy(struct wakeup_source *ws);
#ifdef CONFIG_PCI_MSM
extern int cnss_wlan_pm_control(bool vote);
#endif
extern void cnss_lock_pm_sem(void);
extern void cnss_release_pm_sem(void);

extern int cnss_set_cpus_allowed_ptr(struct task_struct *task, ulong cpu);
extern void cnss_request_pm_qos(u32 qos_val);
extern void cnss_remove_pm_qos(void);
extern void cnss_sdio_request_pm_qos(int val);
extern void cnss_sdio_remove_pm_qos(void);
extern void cnss_pci_request_pm_qos(u32 qos_val);
extern void cnss_pci_remove_pm_qos(void);
extern int cnss_get_platform_cap(struct cnss_platform_cap *cap);
extern void cnss_set_driver_status(enum cnss_driver_status driver_status);
int cnss_pci_request_bus_bandwidth(int bandwidth);
int cnss_sdio_request_bus_bandwidth(int bandwidth);
extern int cnss_common_request_bus_bandwidth(
			struct device *dev, int bandwidth);

void cnss_sdio_device_self_recovery(void);
void cnss_pci_device_self_recovery(void);
extern void cnss_common_device_self_recovery(struct device *dev);

void cnss_sdio_schedule_recovery_work(void);
void cnss_pci_schedule_recovery_work(void);
extern void cnss_common_schedule_recovery_work(struct device *dev);

void cnss_sdio_device_crashed(void);
void cnss_pci_device_crashed(void);
extern void cnss_common_device_crashed(struct device *dev);

void *cnss_pci_get_virt_ramdump_mem(unsigned long *size);
void *cnss_sdio_get_virt_ramdump_mem(unsigned long *size);
extern void *cnss_common_get_virt_ramdump_mem(
		struct device *dev, unsigned long *size);

#ifndef CONFIG_WCNSS_MEM_PRE_ALLOC
static inline int wcnss_pre_alloc_reset(void) { return 0; }
#endif

#if !defined(CONFIG_WCNSS_MEM_PRE_ALLOC) || !defined(CONFIG_SLUB_DEBUG)
static inline void wcnss_prealloc_check_memory_leak(void) {}
#endif

#ifdef CONFIG_CNSS_ADRASTEA
extern void cnss_pcie_notify_q6(void);
extern void cnss_intr_notify_q6(void);
extern void *cnss_get_target_smem(void);
#endif

extern int msm_pcie_enumerate(u32 rc_idx);
extern int cnss_auto_suspend(void);
extern int cnss_auto_resume(void);
extern int cnss_prevent_auto_suspend(const char *caller_func);
extern int cnss_allow_auto_suspend(const char *caller_func);
extern int cnss_is_auto_suspend_allowed(const char *caller_func);

extern int cnss_pm_runtime_request(struct device *dev, enum
		cnss_runtime_request request);

extern int cnss_pcie_set_wlan_mac_address(const u8 *in, uint32_t len);
extern u8 *cnss_common_get_wlan_mac_address(struct device *dev, uint32_t *num);
extern u8 *cnss_pci_get_wlan_mac_address(uint32_t *num);
extern int cnss_power_up(struct device *dev);
extern int cnss_power_down(struct device *dev);
extern int cnss_pcie_power_up(struct device *dev);
extern int cnss_sdio_power_up(struct device *dev);
extern int cnss_sdio_power_down(struct device *dev);
extern int cnss_pcie_power_down(struct device *dev);
#endif /* _NET_CNSS_H_ */
