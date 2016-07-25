/*
 * J-Core SoC AIC driver
 *
 * Copyright (C) 2015-2016 Smart Energy Instruments, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#define AIC1_INTPRI 8

static struct aic_data {
	struct irq_chip chip;
	struct irq_domain *domain;
	struct notifier_block nb;
} aic_data;

static int aic_irqdomain_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hwirq)
{
	struct aic_data *aic = d->host_data;

	irq_set_chip_data(irq, aic);
	irq_set_chip_and_handler(irq, &aic->chip, handle_simple_irq);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops aic_irqdomain_ops = {
	.map = aic_irqdomain_map,
	.xlate = irq_domain_xlate_onecell,
};

static void noop(struct irq_data *data)
{
}

int __init aic_irq_of_init(struct device_node *node, struct device_node *parent)
{
	struct aic_data *aic = &aic_data;
	unsigned min_irq = 64;

	pr_info("Initializing J-Core AIC\n");

	if (!of_device_is_compatible(node, "jcore,aic2")) {
		unsigned cpu;
		for_each_possible_cpu(cpu) {
			void __iomem *base = of_iomap(node, cpu);
			if (!base)
				continue;
			pr_info("Local AIC1 enable for cpu %u at %p\n",
				cpu, base + AIC1_INTPRI);
			__raw_writel(0xffffffff, base + AIC1_INTPRI);
		}
		min_irq = 16;
	}

	aic->chip.name = node->name;
	aic->chip.irq_mask = noop;
	aic->chip.irq_unmask = noop;

	aic->domain = irq_domain_add_linear(node, 128, &aic_irqdomain_ops, aic);
	irq_create_strict_mappings(aic->domain, min_irq, min_irq, 128-min_irq);

	return 0;
}

IRQCHIP_DECLARE(jcore_aic2, "jcore,aic2", aic_irq_of_init);
IRQCHIP_DECLARE(jcore_aic1, "jcore,aic1", aic_irq_of_init);
