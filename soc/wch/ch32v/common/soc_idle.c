/*
 * Copyright (c) 2025 James Bennion-Pedley
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq.h>
#include <zephyr/tracing/tracing.h>

#ifdef CONFIG_SOC_CH32V_CPU_IDLE_GATING
/* Preventing CPU going into idle mode during command queue. */
static atomic_t cpu_idle_disabled;

void chip_permit_idle(void)
{
	atomic_dec(&cpu_idle_disabled);
}

void chip_block_idle(void)
{
	atomic_inc(&cpu_idle_disabled);
}

bool chip_idle_not_allowed(void)
{
	return !!(atomic_get(&cpu_idle_disabled));
}
#endif /* CONFIG_SOC_CH32V_CPU_IDLE_GATING */

void arch_cpu_idle(void)
{
#ifdef CONFIG_SOC_CH32V_CPU_IDLE_GATING
	if (chip_idle_not_allowed()) {
		irq_unlock(MSTATUS_IEN);
		return;
	}
#endif /* CONFIG_SOC_CH32V_CPU_IDLE_GATING */

	sys_trace_idle();
	__asm__ volatile("wfi");
	sys_trace_idle_exit();
	irq_unlock(MSTATUS_IEN);
}

void arch_cpu_atomic_idle(unsigned int key)
{
#ifdef CONFIG_SOC_CH32V_CPU_IDLE_GATING
	if (chip_idle_not_allowed()) {
		irq_unlock(key);
		return;
	}
#endif /* CONFIG_SOC_CH32V_CPU_IDLE_GATING */

	sys_trace_idle();
	__asm__ volatile("wfi");
	sys_trace_idle_exit();
	irq_unlock(key);
}
