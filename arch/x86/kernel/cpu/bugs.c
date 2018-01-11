// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 1994  Linus Torvalds
 *
 *  Cyrix stuff, June 1998 by:
 *	- Rafael R. Reilova (moved everything from head.S),
 *        <rreilova@ececs.uc.edu>
 *	- Channing Corn (tests & fixes),
 *	- Andrew D. Balsa (code cleanup).
 */
#include <linux/init.h>
#include <linux/utsname.h>
#include <linux/cpu.h>

#include <asm/nospec-branch.h>
#include <asm/cmdline.h>
#include <asm/bugs.h>
#include <asm/processor.h>
#include <asm/processor-flags.h>
#include <asm/fpu/internal.h>
#include <asm/msr.h>
#include <asm/paravirt.h>
#include <asm/alternative.h>
#include <asm/pgtable.h>
#include <asm/set_memory.h>

static void __init spectre_v2_check_boottime_disable(void);

void __init check_bugs(void)
{
	identify_boot_cpu();

	if (!IS_ENABLED(CONFIG_SMP)) {
		pr_info("CPU: ");
		print_cpu_info(&boot_cpu_data);
	}

	/* Select the proper spectre mitigation before patching alternatives */
	spectre_v2_check_boottime_disable();

#ifdef CONFIG_X86_32
	/*
	 * Check whether we are able to run this kernel safely on SMP.
	 *
	 * - i386 is no longer supported.
	 * - In order to run on anything without a TSC, we need to be
	 *   compiled for a i486.
	 */
	if (boot_cpu_data.x86 < 4)
		panic("Kernel requires i486+ for 'invlpg' and other features");

	init_utsname()->machine[1] =
		'0' + (boot_cpu_data.x86 > 6 ? 6 : boot_cpu_data.x86);
	alternative_instructions();

	fpu__init_check_bugs();
#else /* CONFIG_X86_64 */
	alternative_instructions();

	/*
	 * Make sure the first 2MB area is not mapped by huge pages
	 * There are typically fixed size MTRRs in there and overlapping
	 * MTRRs into large pages causes slow downs.
	 *
	 * Right now we don't do that with gbpages because there seems
	 * very little benefit for that case.
	 */
	if (!direct_gbpages)
		set_memory_4k((unsigned long)__va(0), 1);
#endif
}

enum spectre_v2_mitigation {
	SPECTRE_V2_NONE,
	SPECTRE_V2_RETPOLINE_MINIMAL,
	SPECTRE_V2_RETPOLINE_MINIMAL_AMD,
	SPECTRE_V2_RETPOLINE_GENERIC,
	SPECTRE_V2_RETPOLINE_AMD,
};

#undef pr_fmt
#define pr_fmt(fmt)     "Spectre V2 mitigation: " fmt

static int spectre_v2_enabled = SPECTRE_V2_NONE;

static void __init spec2_print_if_insecure(const char *reason)
{
	if (boot_cpu_has_bug(X86_BUG_SPECTRE_V2))
		pr_info("%s\n", reason);
}

static void __init spec2_print_if_secure(const char *reason)
{
	if (!boot_cpu_has_bug(X86_BUG_SPECTRE_V2))
		pr_info("%s\n", reason);
}

static inline bool retp_compiler(void)
{
#ifdef RETPOLINE
	return true;
#else
	return false;
#endif
}

static inline bool match_option(const char *arg, int arglen, const char *opt)
{
	int len = strlen(opt);

	return len == arglen && !strncmp(arg, opt, len);
}

static void __init spectre_v2_check_boottime_disable(void)
{
	char arg[20];
	int ret;

	ret = cmdline_find_option(boot_command_line, "spectre_v2", arg,
				  sizeof(arg));
	if (ret > 0)  {
		if (match_option(arg, ret, "off")) {
			spec2_print_if_insecure("disabled on command line.");
			goto disable;
		} else if (match_option(arg, ret, "on")) {
			spec2_print_if_secure("force enabled on command line.");
			goto force;
		} else if (match_option(arg, ret, "retpoline")) {
			spec2_print_if_insecure("retpoline selected on command line.");
			goto retpoline;
		} else if (match_option(arg, ret, "retpoline,amd")) {
			spec2_print_if_insecure("AMD retpoline selected on command line.");
			goto retpoline_amd;
		} else if (match_option(arg, ret, "retpoline,generic")) {
			spec2_print_if_insecure("generic retpoline selected on command line.");
			goto retpoline_generic;
		} else if (match_option(arg, ret, "auto")) {
			goto autosel;
		}
	}

	if (cmdline_find_option_bool(boot_command_line, "nospectre_v2")) {
		spec2_print_if_insecure("disabled on command line.");
		goto disable;
	}

autosel:
	if (!boot_cpu_has_bug(X86_BUG_SPECTRE_V2))
		goto disable;

force:
#ifdef CONFIG_RETPOLINE
retpoline:
	if (boot_cpu_data.x86_vendor == X86_VENDOR_AMD) {
	retpoline_amd:
		if (boot_cpu_data.x86_vendor != X86_VENDOR_AMD ||
		    !boot_cpu_has(X86_FEATURE_LFENCE_RDTSC)) {
			pr_info("AMD retpoline not supported, fall back to generic\n");
			goto retpoline_generic;
		}

		spectre_v2_enabled = retp_compiler() ?
			SPECTRE_V2_RETPOLINE_AMD : SPECTRE_V2_RETPOLINE_MINIMAL_AMD;
		setup_force_cpu_cap(X86_FEATURE_RETPOLINE_AMD);
		setup_force_cpu_cap(X86_FEATURE_RETPOLINE);
		return;
	}
retpoline_generic:
	spectre_v2_enabled = retp_compiler() ?
		SPECTRE_V2_RETPOLINE_GENERIC : SPECTRE_V2_RETPOLINE_MINIMAL;
	setup_force_cpu_cap(X86_FEATURE_RETPOLINE);
	return;
#else
retpoline:
retpoline_amd:
retpoline_generic:
	pr_err("kernel not compiled with retpoline; no mitigation available!");
#endif
disable:
	setup_clear_cpu_cap(X86_FEATURE_RETPOLINE);
	setup_clear_cpu_cap(X86_FEATURE_RETPOLINE_AMD);
	return;
}

#ifdef CONFIG_SYSFS
ssize_t cpu_show_meltdown(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	if (!boot_cpu_has_bug(X86_BUG_CPU_MELTDOWN))
		return sprintf(buf, "Not affected\n");
	if (boot_cpu_has(X86_FEATURE_PTI))
		return sprintf(buf, "Mitigation: PTI\n");
	return sprintf(buf, "Vulnerable\n");
}

ssize_t cpu_show_spectre_v1(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	if (!boot_cpu_has_bug(X86_BUG_SPECTRE_V1))
		return sprintf(buf, "Not affected\n");
	return sprintf(buf, "Vulnerable\n");
}

ssize_t cpu_show_spectre_v2(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	if (!boot_cpu_has_bug(X86_BUG_SPECTRE_V2))
		return sprintf(buf, "Not affected\n");

	switch (spectre_v2_enabled) {
	case SPECTRE_V2_RETPOLINE_MINIMAL:
		return sprintf(buf, "Mitigation: Minimal generic ASM retpoline\n");
	case SPECTRE_V2_RETPOLINE_MINIMAL_AMD:
		return sprintf(buf, "Mitigation: Minimal AMD ASM retpoline\n");
	case SPECTRE_V2_RETPOLINE_GENERIC:
		return sprintf(buf, "Mitigation: Full generic retpoline\n");
	case SPECTRE_V2_RETPOLINE_AMD:
		return sprintf(buf, "Mitigation: Full AMD retpoline\n");
	default:
		return sprintf(buf, "Vulnerable\n");
	}
}
#endif
