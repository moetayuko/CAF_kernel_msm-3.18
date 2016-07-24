/*
 * J-Core SoC PIT/clocksource driver
 *
 * Copyright (C) 2015-2016 Smart Energy Instruments, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/sched_clock.h>
#include <linux/cpu.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define PIT_IRQ_SHIFT 12
#define PIT_PRIO_SHIFT 20
#define PIT_ENABLE_SHIFT 26
#define PIT_IRQ_MASK 0x3f
#define PIT_PRIO_MASK 0xf

#define REG_PITEN 0x00
#define REG_THROT 0x10
#define REG_COUNT 0x14
#define REG_BUSPD 0x18
#define REG_SECHI 0x20
#define REG_SECLO 0x24
#define REG_NSEC  0x28

struct jcore_clocksource {
	struct clocksource cs;
	__iomem void *base;
};

struct jcore_pit {
	struct clock_event_device ced;
	__iomem void *base;
	unsigned long periodic_delta;
	unsigned cpu;
	u32 enable_val;
};

struct jcore_pit_nb {
	struct notifier_block nb;
	struct jcore_pit __percpu *pit_percpu;
};

static struct clocksource *jcore_cs;

static cycle_t jcore_clocksource_read(struct clocksource *cs)
{
	__iomem void *base =
		container_of(cs, struct jcore_clocksource, cs)->base;
	u32 sechi, seclo, nsec, sechi0, seclo0;

	sechi = __raw_readl(base + REG_SECHI);
	seclo = __raw_readl(base + REG_SECLO);
	do {
		sechi0 = sechi;
		seclo0 = seclo;
		nsec  = __raw_readl(base + REG_NSEC);
		sechi = __raw_readl(base + REG_SECHI);
		seclo = __raw_readl(base + REG_SECLO);
	} while (sechi0 != sechi || seclo0 != seclo);

	return ((u64)sechi << 32 | seclo) * NSEC_PER_SEC + nsec;
}

static notrace u64 jcore_sched_clock_read(void)
{
	return jcore_clocksource_read(jcore_cs);
}

static int jcore_pit_disable(struct jcore_pit *pit)
{
	__raw_writel(0, pit->base + REG_PITEN);
	return 0;
}

static int jcore_pit_set(unsigned long delta, struct jcore_pit *pit)
{
	jcore_pit_disable(pit);
	__raw_writel(delta, pit->base + REG_THROT);
	__raw_writel(pit->enable_val, pit->base + REG_PITEN);
	return 0;
}

static int jcore_pit_set_state_shutdown(struct clock_event_device *ced)
{
	struct jcore_pit *pit = container_of(ced, struct jcore_pit, ced);
	return jcore_pit_disable(pit);
}

static int jcore_pit_set_state_oneshot(struct clock_event_device *ced)
{
	struct jcore_pit *pit = container_of(ced, struct jcore_pit, ced);
	return jcore_pit_disable(pit);
}

static int jcore_pit_set_state_periodic(struct clock_event_device *ced)
{
	struct jcore_pit *pit = container_of(ced, struct jcore_pit, ced);
	return jcore_pit_set(pit->periodic_delta, pit);
}

static int jcore_pit_set_next_event(unsigned long delta,
				    struct clock_event_device *ced)
{
	struct jcore_pit *pit = container_of(ced, struct jcore_pit, ced);
	return jcore_pit_set(delta, pit);
}

static int jcore_pit_local_init(struct jcore_pit *pit)
{
	unsigned buspd, freq;

	pr_info("Local J-Core PIT init on cpu %u\n", pit->cpu);

	buspd = __raw_readl(pit->base + REG_BUSPD);
	freq = DIV_ROUND_CLOSEST(NSEC_PER_SEC, buspd);
	pit->periodic_delta = DIV_ROUND_CLOSEST(NSEC_PER_SEC, HZ*buspd);

	clockevents_config_and_register(&pit->ced, freq, 1, ULONG_MAX);

	return 0;
}

static int jcore_pit_cpu_notify(struct notifier_block *self,
			unsigned long action, void *hcpu)
{
	struct jcore_pit_nb *nb = container_of(self, struct jcore_pit_nb, nb);
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		jcore_pit_local_init(this_cpu_ptr(nb->pit_percpu));
		break;
	}
	return NOTIFY_OK;
}

static irqreturn_t jcore_timer_interrupt(int irq, void *dev_id)
{
	struct jcore_pit *pit = this_cpu_ptr(dev_id);

	if (clockevent_state_oneshot(&pit->ced))
		jcore_pit_disable(pit);

	pit->ced.event_handler(&pit->ced);

	return IRQ_HANDLED;
}

static void __init jcore_pit_init(struct device_node *node)
{
	int err;
	__iomem void *pit_base;
	unsigned pit_irq;
	unsigned cpu;
	struct jcore_pit_nb *nb = 0;
	struct jcore_clocksource *cs = 0;
	struct jcore_pit __percpu *pit_percpu = 0;
	unsigned long hwirq;
	u32 irqprio, enable_val;

	pit_base = of_iomap(node, 0);
	if (!pit_base) {
		pr_err("Error: Cannot map base address for J-Core PIT\n");
		goto out;
	}

	pit_irq = irq_of_parse_and_map(node, 0);
	if (!pit_irq) {
		pr_err("Error: J-Core PIT has no IRQ\n");
		goto out;
	}

	pr_info("Initializing J-Core PIT at %p IRQ %d\n", pit_base, pit_irq);

	cs = kzalloc(sizeof *cs, GFP_KERNEL);
	if (!cs) {
		pr_err("Failed to allocate memory for clocksource\n");
		goto out;
	}
	jcore_cs = &cs->cs;

	cs->base = pit_base;
	cs->cs.name = "jcore_pit_cs";
	cs->cs.rating = 400;
	cs->cs.read = jcore_clocksource_read;
	cs->cs.mult = 1;
	cs->cs.shift = 0;
	cs->cs.mask = CLOCKSOURCE_MASK(64);
	cs->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;

	err = clocksource_register_hz(&cs->cs, NSEC_PER_SEC);
	if (err) {
		pr_err("Error registering clocksource device: %d\n", err);
		goto out;
	}

	sched_clock_register(jcore_sched_clock_read, 64, NSEC_PER_SEC);

	pit_percpu = alloc_percpu(struct jcore_pit);
	if (!pit_percpu) {
		pr_err("Failed to allocate memory for clock event device\n");
		goto out;
	}

	nb = kzalloc(sizeof *nb, GFP_KERNEL);
	if (!nb) {
		pr_err("Failed to allocate memory for J-Core PIT notifier\n");
		goto out;
	}

	nb->pit_percpu = pit_percpu;
	nb->nb.notifier_call = jcore_pit_cpu_notify;
	err = register_cpu_notifier(&nb->nb);
	if (err) {
		pr_err("Error registering J-Core PIT notifier: %d\n", err);
		goto out;
	}

	err = request_irq(pit_irq, jcore_timer_interrupt,
		IRQF_TIMER | IRQF_PERCPU, "jcore_pit", pit_percpu);
	if (err) {
		pr_err("pit irq request failed: %d\n", err);
		goto out;
	}

	/* The J-Core PIT is not hard-wired to a particular IRQ, but
	 * integrated with the interrupt controller such that the IRQ it
	 * generates is programmable. The programming interface has a
	 * legacy field which was an interrupt priority for AIC1, but
	 * which is OR'd onto bits 2-5 of the generated IRQ number when
	 * used with J-Core AIC2, so set it to match these bits. */
	hwirq = irq_get_irq_data(pit_irq)->hwirq;
	irqprio = (hwirq >> 2) & PIT_PRIO_MASK;
	enable_val = (1U << PIT_ENABLE_SHIFT)
		   | (hwirq << PIT_IRQ_SHIFT)
		   | (irqprio << PIT_PRIO_SHIFT);

	for_each_possible_cpu(cpu) {
		struct jcore_pit *pit = per_cpu_ptr(pit_percpu, cpu);

		pit->base = of_iomap(node, cpu);
		if (!pit->base)
			continue;

		pit->ced.name = "jcore_pit";
		pit->ced.features = CLOCK_EVT_FEAT_PERIODIC
				  | CLOCK_EVT_FEAT_ONESHOT
				  | CLOCK_EVT_FEAT_PERCPU;
		pit->ced.cpumask = cpumask_of(cpu);
		pit->ced.rating = 400;
		pit->ced.irq = pit_irq;
		pit->ced.set_state_shutdown = jcore_pit_set_state_shutdown;
		pit->ced.set_state_periodic = jcore_pit_set_state_periodic;
		pit->ced.set_state_oneshot = jcore_pit_set_state_oneshot;
		pit->ced.set_next_event = jcore_pit_set_next_event;

		pit->cpu = cpu;
		pit->enable_val = enable_val;
	}

	jcore_pit_local_init(this_cpu_ptr(pit_percpu));

	return;

out:
	pr_err("Could not initialize J-Core PIT driver\n");
}

CLOCKSOURCE_OF_DECLARE(jcore_pit, "jcore,pit", jcore_pit_init);
