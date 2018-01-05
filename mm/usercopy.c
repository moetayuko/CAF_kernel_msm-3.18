/*
 * This implements the various checks for CONFIG_HARDENED_USERCOPY*,
 * which are designed to protect kernel memory from needless exposure
 * and overwrite under many unintended conditions. This code is based
 * on PAX_USERCOPY, which is:
 *
 * Copyright (C) 2001-2016 PaX Team, Bradley Spengler, Open Source
 * Security Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/sched/task_stack.h>
#include <linux/thread_info.h>
#include <asm/sections.h>

/*
 * Checks if a given pointer and length is contained by the current
 * stack frame (if possible).
 *
 * Returns:
 *	NOT_STACK: not at all on the stack
 *	GOOD_FRAME: fully within a valid stack frame
 *	GOOD_STACK: fully on the stack (when can't do frame-checking)
 *	BAD_STACK: error condition (invalid stack position or bad stack frame)
 */
static noinline int check_stack_object(const void *obj, unsigned long len)
{
	const void * const stack = task_stack_page(current);
	const void * const stackend = stack + THREAD_SIZE;
	int ret;

	/* Object is not on the stack at all. */
	if (obj + len <= stack || stackend <= obj)
		return NOT_STACK;

	/*
	 * Reject: object partially overlaps the stack (passing the
	 * the check above means at least one end is within the stack,
	 * so if this check fails, the other end is outside the stack).
	 */
	if (obj < stack || stackend < obj + len)
		return BAD_STACK;

	/* Check if object is safely within a valid frame. */
	ret = arch_within_stack_frames(stack, stackend, obj, len);
	if (ret)
		return ret;

	return GOOD_STACK;
}

/*
 * If this function is reached, then CONFIG_HARDENED_USERCOPY has found an
 * unexpected state during a copy_from_user() or copy_to_user() call.
 * There are several checks being performed on the buffer by the
 * __check_object_size() function. Normal stack buffer usage should never
 * trip the checks, and kernel text addressing will always trip the check.
 * For cache objects, it is checking that only the whitelisted range of
 * bytes for a given cache is being accessed (via the cache's usersize and
 * useroffset fields). To adjust a cache whitelist, use the usercopy-aware
 * kmem_cache_create_usercopy() function to create the cache (and
 * carefully audit the whitelist range).
 */
int report_usercopy(const char *name, const char *detail, bool to_user,
		    unsigned long offset, unsigned long len)
{
	pr_emerg("kernel memory %s attempt detected %s %s%s%s%s (offset %lu, size %lu)\n",
		to_user ? "exposure" : "overwrite",
		to_user ? "from" : "to",
		name ? : "unknown?!",
		detail ? " '" : "", detail ? : "", detail ? "'" : "",
		offset, len);
	/*
	 * For greater effect, it would be nice to do do_group_exit(),
	 * but BUG() actually hooks all the lock-breaking and per-arch
	 * Oops code, so that is used here instead.
	 */
	BUG();

	return -1;
}

/* Returns true if any portion of [ptr,ptr+n) over laps with [low,high). */
static bool overlaps(const unsigned long ptr, unsigned long n,
		     unsigned long low, unsigned long high)
{
	const unsigned long check_low = ptr;
	unsigned long check_high = check_low + n;

	/* Does not overlap if entirely above or entirely below. */
	if (check_low >= high || check_high <= low)
		return false;

	return true;
}

/* Is this address range in the kernel text area? */
static inline int check_kernel_text_object(const unsigned long ptr,
					   unsigned long n, bool to_user)
{
	unsigned long textlow = (unsigned long)_stext;
	unsigned long texthigh = (unsigned long)_etext;
	unsigned long textlow_linear, texthigh_linear;

	if (overlaps(ptr, n, textlow, texthigh))
		return report_usercopy("kernel text", NULL, to_user,
				       ptr - textlow, n);

	/*
	 * Some architectures have virtual memory mappings with a secondary
	 * mapping of the kernel text, i.e. there is more than one virtual
	 * kernel address that points to the kernel image. It is usually
	 * when there is a separate linear physical memory mapping, in that
	 * __pa() is not just the reverse of __va(). This can be detected
	 * and checked:
	 */
	textlow_linear = (unsigned long)lm_alias(textlow);
	/* No different mapping: we're done. */
	if (textlow_linear == textlow)
		return 0;

	/* Check the secondary mapping... */
	texthigh_linear = (unsigned long)lm_alias(texthigh);
	if (overlaps(ptr, n, textlow_linear, texthigh_linear))
		return report_usercopy("linear kernel text", NULL, to_user,
				       ptr - textlow_linear, n);

	return 0;
}

static inline int check_bogus_address(const unsigned long ptr, unsigned long n,
				      bool to_user)
{
	/* Reject if object wraps past end of memory. */
	if (ptr + n < ptr)
		return report_usercopy("wrapped address", NULL, to_user,
				       0, ptr + n);

	/* Reject if NULL or ZERO-allocation. */
	if (ZERO_OR_NULL_PTR(ptr))
		return report_usercopy("null address", NULL, to_user, ptr, n);

	return 0;
}

/* Checks for allocs that are marked in some way as spanning multiple pages. */
static inline int check_page_span(const void *ptr, unsigned long n,
				  struct page *page, bool to_user)
{
#ifdef CONFIG_HARDENED_USERCOPY_PAGESPAN
	const void *end = ptr + n - 1;
	struct page *endpage;
	bool is_reserved, is_cma;

	/*
	 * Sometimes the kernel data regions are not marked Reserved (see
	 * check below). And sometimes [_sdata,_edata) does not cover
	 * rodata and/or bss, so check each range explicitly.
	 */

	/* Allow reads of kernel rodata region (if not marked as Reserved). */
	if (ptr >= (const void *)__start_rodata &&
	    end <= (const void *)__end_rodata) {
		if (!to_user)
			return report_usercopy("rodata", NULL, to_user, 0, n);
		return 0;
	}

	/* Allow kernel data region (if not marked as Reserved). */
	if (ptr >= (const void *)_sdata && end <= (const void *)_edata)
		return 0;

	/* Allow kernel bss region (if not marked as Reserved). */
	if (ptr >= (const void *)__bss_start &&
	    end <= (const void *)__bss_stop)
		return 0;

	/* Is the object wholly within one base page? */
	if (likely(((unsigned long)ptr & (unsigned long)PAGE_MASK) ==
		   ((unsigned long)end & (unsigned long)PAGE_MASK)))
		return 0;

	/* Allow if fully inside the same compound (__GFP_COMP) page. */
	endpage = virt_to_head_page(end);
	if (likely(endpage == page))
		return 0;

	/*
	 * Reject if range is entirely either Reserved (i.e. special or
	 * device memory), or CMA. Otherwise, reject since the object spans
	 * several independently allocated pages.
	 */
	is_reserved = PageReserved(page);
	is_cma = is_migrate_cma_page(page);
	if (!is_reserved && !is_cma)
		return report_usercopy("spans multiple pages", NULL, to_user,
				       0, n);

	for (ptr += PAGE_SIZE; ptr <= end; ptr += PAGE_SIZE) {
		page = virt_to_head_page(ptr);
		if (is_reserved && !PageReserved(page))
			return report_usercopy("spans Reserved and non-Reserved pages",
					       NULL, to_user, 0, n);
		if (is_cma && !is_migrate_cma_page(page))
			return report_usercopy("spans CMA and non-CMA pages",
					       NULL, to_user, 0, n);
	}
#endif

	return 0;
}

static inline int check_heap_object(const void *ptr, unsigned long n,
				    bool to_user)
{
	struct page *page;

	if (!virt_addr_valid(ptr))
		return 0;

	page = virt_to_head_page(ptr);

	/* Check slab allocator for flags and size. */
	if (PageSlab(page))
		return __check_heap_object(ptr, n, page, to_user);

	/* Verify object does not incorrectly span multiple pages. */
	return check_page_span(ptr, n, page, to_user);
}

/*
 * Validates that the given object is:
 * - not bogus address
 * - known-safe heap or stack object
 * - not in kernel text
 */
void __check_object_size(const void *ptr, unsigned long n, bool to_user)
{
	/* Skip all tests if size is zero. */
	if (!n)
		return;

	/* Check for invalid addresses. */
	if (check_bogus_address((const unsigned long)ptr, n, to_user))
		return;

	/* Check for bad heap object. */
	if (check_heap_object(ptr, n, to_user))
		return;

	/* Check for bad stack object. */
	switch (check_stack_object(ptr, n)) {
	case NOT_STACK:
		/* Object is not touching the current process stack. */
		break;
	case GOOD_FRAME:
	case GOOD_STACK:
		/*
		 * Object is either in the correct frame (when it
		 * is possible to check) or just generally on the
		 * process stack (when frame checking not available).
		 */
		return;
	default:
		report_usercopy("process stack", NULL, to_user, 0, n);
		return;
	}

	/* Check for object in kernel to avoid text exposure. */
	if (check_kernel_text_object((const unsigned long)ptr, n, to_user))
		return;
}
EXPORT_SYMBOL(__check_object_size);
