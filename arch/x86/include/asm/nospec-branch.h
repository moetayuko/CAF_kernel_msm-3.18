/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __NOSPEC_BRANCH_H__
#define __NOSPEC_BRANCH_H__

#include <asm/alternative.h>
#include <asm/alternative-asm.h>
#include <asm/cpufeatures.h>

#ifdef __ASSEMBLY__

/*
 * These are the bare retpoline primitives for indirect jmp and call.
 * Do not use these directly; they only exist to make the ALTERNATIVE
 * invocation below less ugly.
 */
.macro RETPOLINE_JMP reg:req
	.align	16
	call	.Ldo_rop_\@
.Lspec_trap_\@:
	pause
	jmp	.Lspec_trap_\@
	.align	16
.Ldo_rop_\@:
	mov	\reg, (%_ASM_SP)
	ret
.endm

/*
 * This is a wrapper around RETPOLINE_JMP so the called function in reg
 * returns to the instruction after the macro.
 */
.macro RETPOLINE_CALL reg:req
	jmp	.Ldo_call_\@
.Ldo_retpoline_jmp_\@:
	RETPOLINE_JMP \reg
	.align	16
.Ldo_call_\@:
	call	.Ldo_retpoline_jmp_\@
.endm

/*
 * JMP_NOSPEC and CALL_NOSPEC macros can be used instead of a simple
 * indirect jmp/call which may be susceptible to the Spectre variant 2
 * attack.
 */
.macro JMP_NOSPEC reg:req
#ifdef CONFIG_RETPOLINE
	ALTERNATIVE_2 __stringify(jmp *\reg),				\
		__stringify(RETPOLINE_JMP \reg), X86_FEATURE_RETPOLINE,	\
		__stringify(lfence; jmp *\reg), X86_FEATURE_RETPOLINE_AMD
#else
	jmp	*\reg
#endif
.endm

.macro CALL_NOSPEC reg:req
#ifdef CONFIG_RETPOLINE
	ALTERNATIVE_2 __stringify(call *\reg),				\
		__stringify(RETPOLINE_CALL \reg), X86_FEATURE_RETPOLINE,\
		__stringify(lfence; call *\reg), X86_FEATURE_RETPOLINE_AMD
#else
	call	*\reg
#endif
.endm

/*
 * Use 32-N: 32 is the max return buffer size, but there should have been
 * at a minimum two controlled calls already: one into the kernel from
 * entry*.S and another into the function containing this macro. So N=2,
 * thus 30.
 */
#define NUM_BRANCHES_TO_FILL	30

/*
 * Fill the CPU return stack buffer to prevent indirect branch prediction
 * on underflow.
 *
 * A 'nop' after each call is required so it isn't interpreted by the CPU
 * as a simple 'push %eip', which would be handled specially and would not
 * put anything in the RSB.
 *
 * Required in various cases for retpoline and IBRS-based mitigations for
 * Spectre variant 2 vulnerability.
 */
.macro	FILL_RETURN_BUFFER reg:req
	mov	$NUM_BRANCHES_TO_FILL/2, \reg
	.align	16
.Ldo_call1_\@:
	call	.Ldo_call2_\@
.Ltrap1_\@:
	pause
	jmp	.Ltrap1_\@
	.align	16
.Ldo_call2_\@:
	call	.Ldo_loop_\@
.Ltrap2_\@:
	pause
	jmp	.Ltrap2_\@
	.align	16
.Ldo_loop_\@:
	dec	\reg
	jnz	.Ldo_call1_\@
#ifdef CONFIG_64BIT
	addq	$8*NUM_BRANCHES_TO_FILL, %rsp
#else
	addl    $4*NUM_BRANCHES_TO_FILL, %esp
#endif
.endm

#else /* __ASSEMBLY__ */

#if defined(CONFIG_X86_64) && defined(RETPOLINE)
/*
 * Since the inline asm uses the %V modifier which is only in newer GCC,
 * the 64-bit one is dependent on RETPOLINE not CONFIG_RETPOLINE.
 */
# define CALL_NOSPEC ALTERNATIVE(				\
	"call *%[thunk_target]\n",				\
	"call __x86_indirect_thunk_%V[thunk_target]\n",		\
	X86_FEATURE_RETPOLINE)
# define THUNK_TARGET(addr) [thunk_target] "r" (addr)
#elif defined(CONFIG_X86_32) && defined(CONFIG_RETPOLINE)
/*
 * For i386 we use the original ret-equivalent retpoline, because
 * otherwise we'll run out of registers. We don't care about CET
 * here, anyway.
 */
# define CALL_NOSPEC ALTERNATIVE(				\
	"call	*%[thunk_target]\n",				\
	""							\
	"       jmp    do_call%=;\n"				\
	"       .align 16\n"					\
	"do_retpoline%=:\n"					\
	"	call   do_rop%=;\n"				\
	"spec_trap%=:\n"					\
	"	pause;\n"					\
	"       jmp    spec_trap%=;\n"				\
	"       .align 16\n"					\
	"do_rop%=:\n"						\
	"	addl   $4, %%esp;\n"				\
	"       pushl  %[thunk_target];\n"			\
	"       ret;\n"						\
	"       .align 16\n"					\
	"do_call%=:\n"						\
	"	call   do_retpoline%=;\n",			\
	X86_FEATURE_RETPOLINE)

# define THUNK_TARGET(addr) [thunk_target] "rm" (addr)
#else /* No retpoline */
# define CALL_NOSPEC "call *%[thunk_target]\n"
# define THUNK_TARGET(addr) [thunk_target] "rm" (addr)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __NOSPEC_BRANCH_H__ */
