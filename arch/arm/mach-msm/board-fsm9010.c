/* Copyright (c) 2014-2017, The Linux Foundation. All rights reserved.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/memory.h>
#include <linux/uio_driver.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <soc/qcom/restart.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/smd.h>
#include "board-dt.h"
#include "platsmp.h"

#define FSM9010_MAC0_FUSE_PHYS	0xFC4B8440
#define FSM9010_MAC1_FUSE_PHYS	0xFC4B8448
#define FSM9010_MAC_FUSE_SIZE	0x10

#define FSM9010_SCLTE_DDR_PHYS		0x00100000
#define FSM9010_SCLTE_NL_SAMPLES_PHYS	0x15000000
#define FSM9010_CDU_PHYS		0x17000000
#define FSM9010_SCLTE_TRACE_DEBUG_PHYS	0x18100000
#define FSM9010_SCLTE_ETH_TRACE_PHYS	0x18180000
#define FSM9010_SCLTE_DEBUG_DUMP_PHYS	0x18280000
#define FSM9010_SCLTE_CB_TRACE_PHYS	0x18300000
#define FSM9010_SCLTE_RF_CAL_PHYS	0x29000000
#define FSM9010_SCLTE_GEN_DBG_PHYS	0xf5000000
#define FSM9010_QDSP6_0_DEBUG_DUMP_PHYS	0x18900000
#define FSM9010_QDSP6_1_DEBUG_DUMP_PHYS	0x18a80000
#define FSM9010_NSS_CORE_DUMP_PHYS	0x28000000
#define FSM9010_NSS_TCM_PHYS		0xe4000000

#define FSM9010_UIO_VERSION "1.0"

#define FSM9010_MEM_MAP_PHYS             0xFE800400
#define FSM9010_MEM_MAP_SIZE             0x400

#define FSM9010_MEM_TAG_NONE                 0x00000000
#define FSM9010_MEM_TAG_OEM_DEBUG            0x00000001
#define FSM9010_MEM_TAG_LOADABLE_HEX0        0x00000002
#define FSM9010_MEM_TAG_LOADABLE_HEX1        0x00000003
#define FSM9010_MEM_TAG_LOADABLE_HEX2        0x00000004
#define FSM9010_MEM_TAG_LOADABLE_HEX3        0x00000005
#define FSM9010_MEM_TAG_SHARED_LTEFAPI_UL    0x00000006
#define FSM9010_MEM_TAG_SHARED_LTEFAPI_DL    0x00000007
#define FSM9010_MEM_TAG_SHARED_LTEIPC        0x00000008
#define FSM9010_MEM_TAG_SHARED_LTEL2_DL      0x00000009
#define FSM9010_MEM_TAG_SHARED_LTEL2_UL      0x0000000A
#define FSM9010_MEM_TAG_LOADABLE_UBI         0x0000000B

#define VMID_NOACCESS			0
#define VMID_RPM			1
#define VMID_TZ				2
#define VMID_AP				3
#define VMID_HEX_0			4
#define VMID_HEX_1			5
#define VMID_HEX_2			6
#define VMID_HEX_3			7
#define VMID_CTTHRT			8
#define VMID_SCLTE			9
#define VMID_NAV			10
#define VMID_EMAC0			11
#define VMID_EMAC1			12
#define VMID_PCIE0			13
#define VMID_PCIE1			14

#define VMID_NOACCESS_BIT		(1<<VMID_NOACCESS)
#define VMID_RPM_BIT			(1<<VMID_RPM)
#define VMID_TZ_BIT			(1<<VMID_TZ)
#define VMID_AP_BIT			(1<<VMID_AP)
#define VMID_HEX_0_BIT			(1<<VMID_HEX_0)
#define VMID_HEX_1_BIT			(1<<VMID_HEX_1)
#define VMID_HEX_2_BIT			(1<<VMID_HEX_2)
#define VMID_HEX_3_BIT			(1<<VMID_HEX_3)
#define VMID_CTTHRT_BIT			(1<<VMID_CTTHRT)
#define VMID_SCLTE_BIT			(1<<VMID_SCLTE)
#define VMID_NAV_BIT			(1<<VMID_NAV)
#define VMID_EMAC0_BIT			(1<<VMID_EMAC0)
#define VMID_EMAC1_BIT			(1<<VMID_EMAC1)
#define VMID_PCIE0_BIT			(1<<VMID_PCIE0)
#define VMID_PCIE1_BIT			(1<<VMID_PCIE1)

struct mem_map_seg {
	u32 phy_addr;
	u32 sz;
	u32 rd_vmid;
	u32 wr_vmid;
	u32 tag;
} __attribute__((__packed__));

static struct uio_info fsm9010_uio_info[] = {
	{
		.name = "fsm9010-uio0",
		.version = FSM9010_UIO_VERSION,
	},
	{
		.name = "fsm9010-uio1",
		.version = FSM9010_UIO_VERSION,
	},
	{
		.name = "fsm9010-uio2",
		.version = FSM9010_UIO_VERSION,
	},
};

static struct resource fsm9010_uio0_resources[] = {
	{
		.start = FSM9010_SCLTE_DDR_PHYS,
		.end   = FSM9010_SCLTE_DDR_PHYS + 59 * SZ_1M - 1,
		.name  = "sclte_ddr",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_SCLTE_NL_SAMPLES_PHYS,
		.end   = FSM9010_SCLTE_NL_SAMPLES_PHYS + SZ_32M - 1,
		.name  = "nl_samples",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_CDU_PHYS,
		.end   = FSM9010_CDU_PHYS + SZ_16M - 1,
		.name  = "cdu",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_SCLTE_TRACE_DEBUG_PHYS,
		.end   = FSM9010_SCLTE_TRACE_DEBUG_PHYS + SZ_512K - 1,
		.name  = "sclte_trace_debug",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_SCLTE_ETH_TRACE_PHYS,
		.end   = FSM9010_SCLTE_ETH_TRACE_PHYS + SZ_1M - 1,
		.name  = "sclte_eth_trace",
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device fsm9010_uio0_device = {
	.name = "uio_pdrv",
	.id = 0,
	.dev = {
		.platform_data = &fsm9010_uio_info[0]
	},
	.num_resources = ARRAY_SIZE(fsm9010_uio0_resources),
	.resource = fsm9010_uio0_resources,
};

static struct resource fsm9010_uio1_resources[] = {
	{
		.start = FSM9010_SCLTE_DEBUG_DUMP_PHYS,
		.end   = FSM9010_SCLTE_DEBUG_DUMP_PHYS + SZ_512K - 1,
		.name  = "sclte_debug_dump",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_SCLTE_CB_TRACE_PHYS,
		.end   = FSM9010_SCLTE_CB_TRACE_PHYS + SZ_2M - 1,
		.name  = "sclte_cb_trace",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_SCLTE_RF_CAL_PHYS,
		.end   = FSM9010_SCLTE_RF_CAL_PHYS + 10 * SZ_1M - 1,
		.name  = "sclte_rf_cal",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_SCLTE_GEN_DBG_PHYS,
		.end   = FSM9010_SCLTE_GEN_DBG_PHYS + 48 * SZ_1M - 1,
		.name  = "sclte_gen_dbg",
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device fsm9010_uio1_device = {
	.name = "uio_pdrv",
	.id = 1,
	.dev = {
		.platform_data = &fsm9010_uio_info[1]
	},
	.num_resources = ARRAY_SIZE(fsm9010_uio1_resources),
	.resource = fsm9010_uio1_resources,
};

static struct resource fsm9010_uio2_resources[] = {
	{
		.start = FSM9010_QDSP6_0_DEBUG_DUMP_PHYS,
		.end   = FSM9010_QDSP6_0_DEBUG_DUMP_PHYS + SZ_1M + SZ_512K - 1,
		.name  = "qdsp6_0_debug_dump",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_QDSP6_1_DEBUG_DUMP_PHYS,
		.end   = FSM9010_QDSP6_1_DEBUG_DUMP_PHYS + SZ_1M + SZ_512K - 1,
		.name  = "qdsp6_1_debug_dump",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_NSS_CORE_DUMP_PHYS,
		.end   = FSM9010_NSS_CORE_DUMP_PHYS + SZ_16M - 1,
		.name  = "nss_core_dump",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = FSM9010_NSS_TCM_PHYS,
		.end   = FSM9010_NSS_TCM_PHYS + SZ_64K - 1,
		.name  = "nss_tcm",
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device fsm9010_uio2_device = {
	.name = "uio_pdrv",
	.id = 2,
	.dev = {
		.platform_data = &fsm9010_uio_info[2]
	},
	.num_resources = ARRAY_SIZE(fsm9010_uio2_resources),
	.resource = fsm9010_uio2_resources,
};

static struct platform_device *fsm9010_uio_devices[] = {
	&fsm9010_uio0_device,
	&fsm9010_uio1_device,
	&fsm9010_uio2_device,
};

static const char mac_addr_prop_name[] = "mac-address";
static const char mac_addr_prop_name1[] = "local-mac-address";
static const char shm_ul_bufs_prop_name[] = "ul-bufs";
static const char shm_dl_bufs_prop_name[] = "dl-bufs";

void __init fsm9010_reserve(void)
{
}

static struct mem_map_seg *find_mem_map_seg(struct mem_map_seg *tbl, u32 tag)
{
	if (tbl == NULL)
		return NULL;

	while (tbl->tag != FSM9010_MEM_TAG_NONE) {
		if (tbl->tag == tag)
			return tbl;

		tbl++;
	}

	return NULL;
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init fsm9010_add_drivers(void)
{
	msm_smd_init();
	platform_add_devices(fsm9010_uio_devices,
			     ARRAY_SIZE(fsm9010_uio_devices));
}

static void __init fsm9010_map_io(void)
{
	msm_map_fsm9010_io();
}

static int gmac_dt_update(int cell, phys_addr_t addr, unsigned long size)
{
	/*
	 * Use an array for the fuse. Corrected fuse data may be located
	 * at a different offsets.
	 */
	static int offset[ETH_ALEN] = { 0, 1, 2, 3, 4, 5};
	void __iomem *fuse_reg;
	struct device_node *np = NULL;
	struct property *pmac = NULL;
	struct property *pp = NULL;
	u8 buf[ETH_ALEN];
	int n;
	int retval = 0;

	fuse_reg = ioremap(addr, size);
	if (!fuse_reg) {
		pr_err("failed to ioremap efuse to read mac address");
		return -ENOMEM;
	}

	for (n = 0; n < ETH_ALEN; n++)
		buf[n] = ioread8(fuse_reg + offset[n]);

	iounmap(fuse_reg);

	if (!is_valid_ether_addr(buf)) {
		pr_err("invalid MAC address in efuse\n");
		return -ENODATA;
	}

	pmac = kzalloc(sizeof(*pmac) + ETH_ALEN, GFP_KERNEL);
	if (!pmac) {
		pr_err("failed to alloc memory for mac address\n");
		return -ENOMEM;
	}

	pmac->value = pmac + 1;
	pmac->length = ETH_ALEN;
	pmac->name = (char *)mac_addr_prop_name;
	memcpy(pmac->value, buf, ETH_ALEN);

	for_each_compatible_node(np, NULL, "qcom,qfec-nss") {
		if (of_property_read_u32(np, "cell-index", &n))
			continue;
		if (n == cell)
			break;
	}

	if (!np) {
		pr_err("failed to find dt node for gmac%d", cell);
		retval = -ENODEV;
		goto out;
	}

	pp = of_find_property(np, pmac->name, NULL);
	if (pp)
		of_update_property(np, pmac);
	else
		of_add_property(np, pmac);

	of_node_put(np);

	pmac = kzalloc(sizeof(*pmac) + ETH_ALEN, GFP_KERNEL);
	if (!pmac) {
		pr_err("failed to alloc memory for mac address\n");
		return -ENOMEM;
	}

	pmac->value = pmac + 1;
	pmac->length = ETH_ALEN;
	pmac->name = (char *)mac_addr_prop_name1;
	memcpy(pmac->value, buf, ETH_ALEN);

	for_each_compatible_node(np, NULL, "qcom,nss-gmac") {
		if (of_property_read_u32(np, "qcom,id", &n))
			continue;
		if (n == cell)
			break;
	}
	if (!np) {
		pr_err("failed to find dt node for gmac%d", cell);
		retval = -ENODEV;
		goto out;
	}

	pp = of_find_property(np, pmac->name, NULL);
	if (pp)
		of_update_property(np, pmac);
	else
		of_add_property(np, pmac);

	of_node_put(np);

out:
	if (retval && pmac)
		kfree(pmac);

	return retval;
}

int __init fsm9010_gmac_dt_update(void)
{
	gmac_dt_update(0, FSM9010_MAC0_FUSE_PHYS, FSM9010_MAC_FUSE_SIZE);
	gmac_dt_update(1, FSM9010_MAC1_FUSE_PHYS, FSM9010_MAC_FUSE_SIZE);
	return 0;
}

static int add_danipc_property(struct device_node *np,
			       struct mem_map_seg *region,
			       const char *name)
{
	u32 buf[2];
	struct property *pbuf = NULL;

	pr_info("%s: adding danipc DT prop %s=<0x%x 0x%x>\n",
		__func__, name, region->phy_addr, region->sz);

	if (!(region->rd_vmid & VMID_AP_BIT) ||
	    !(region->wr_vmid & VMID_AP_BIT)) {
		pr_err("do not have permissions for %s\n", name);
		return -EPERM;
	}

	buf[0] = region->phy_addr;
	buf[1] = region->sz;

	pbuf = kzalloc(sizeof(*pbuf) + sizeof(buf), GFP_KERNEL);

	if (pbuf == NULL)
		return -ENOMEM;

	pbuf->value = pbuf + 1;
	pbuf->length = sizeof(buf);
	pbuf->name = (char *)name;
	memcpy(pbuf->value, buf, sizeof(buf));

	of_add_property(np, pbuf);

	return 0;
}

static int __init fsm9010_ipc_buf_region_update(void)
{
	struct device_node *np = NULL;
	int ret = -ENODEV;
	void *mem_map_region = NULL;
	struct mem_map_seg __iomem *mem_map_table;
	struct mem_map_seg *ul_region;
	struct mem_map_seg *dl_region;

	/* We expect only a single danipc device node */
	np = of_find_compatible_node(NULL, NULL, "qcom,danipc");

	if (np == NULL) {
		pr_err("failed to find dt node for qcom,danipc\n");
		return -ENODEV;
	}

	mem_map_region = ioremap(FSM9010_MEM_MAP_PHYS, FSM9010_MEM_MAP_SIZE);

	if (mem_map_region == NULL) {
		pr_err("failed to map memory map");
		ret = -ENOMEM;
		goto out;
	}

	/* Version number comes first */
	mem_map_table = mem_map_region + sizeof(u32);

	/* Find the UL and DL regions and store them in the DT */
	ul_region = find_mem_map_seg(mem_map_table,
			FSM9010_MEM_TAG_SHARED_LTEL2_UL);

	dl_region = find_mem_map_seg(mem_map_table,
			FSM9010_MEM_TAG_SHARED_LTEL2_DL);

	if (dl_region == NULL || ul_region == NULL) {
		pr_err("could not find regions: ul=%pK, dl=%pK",
			ul_region, dl_region);
		ret = -ENODEV;
		goto out;
	}

	ret = add_danipc_property(np, dl_region, shm_dl_bufs_prop_name);

	if (ret != 0)
		goto out;

	ret = add_danipc_property(np, ul_region, shm_ul_bufs_prop_name);

	if (ret != 0)
		goto out;

out:
	of_node_put(np);

	if (mem_map_region)
		iounmap(mem_map_region);

	return ret;
}


void __init fsm9010_init(void)
{
	/*
	 * Populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init. socinfo_init needs smem support so call
	 * msm_smem_init before it.
	 */
	board_dt_populate(NULL);

	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	fsm9010_gmac_dt_update();
	fsm9010_ipc_buf_region_update();

	fsm9010_add_drivers();
}

static const char *fsm9010_dt_match[] __initconst = {
	"qcom,fsm9010",
	NULL
};

DT_MACHINE_START(FSM9010_DT,
		"Qualcomm Technologies, Inc. FSM 9010 (Flattened Device Tree)")
	.map_io			= fsm9010_map_io,
	.init_machine		= fsm9010_init,
	.dt_compat		= fsm9010_dt_match,
	.reserve		= fsm9010_reserve,
	.smp			= &arm_smp_ops,
MACHINE_END
