/*
 * arch/sh/mm/cache-j2.c
 *
 * Copyright (C) 2015-2016 Smart Energy Instruments, Inc.
 *
 * Released under the terms of the GNU GPL v2.0.
 */

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/cpumask.h>

#include <asm/cache.h>
#include <asm/addrspace.h>
#include <asm/processor.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

u32 j2_ccr_base;

static void j2_flush_icache(void *args)
{
	unsigned cpu;
	for_each_possible_cpu(cpu)
		__raw_writel(0x80000103, j2_ccr_base + 4*cpu);
}

static void j2_flush_dcache(void *args)
{
	unsigned cpu;
	for_each_possible_cpu(cpu)
		__raw_writel(0x80000203, j2_ccr_base + 4*cpu);
}

static void j2_flush_both(void *args)
{
	unsigned cpu;
	for_each_possible_cpu(cpu)
		__raw_writel(0x80000303, j2_ccr_base + 4*cpu);
}

void __init j2_cache_init(void)
{
	if (!j2_ccr_base)
		return;

	local_flush_cache_all = j2_flush_both;
	local_flush_cache_mm = j2_flush_both;
	local_flush_cache_dup_mm = j2_flush_both;
	local_flush_cache_page = j2_flush_both;
	local_flush_cache_range = j2_flush_both;
	local_flush_dcache_page = j2_flush_dcache;
	local_flush_icache_range = j2_flush_icache;
	local_flush_icache_page = j2_flush_icache;
	local_flush_cache_sigtramp = j2_flush_icache;

	pr_info("Initial J2 CCR is %.8x\n", __raw_readl(j2_ccr_base));
}
