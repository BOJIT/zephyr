/*
 * Copyright (c) 2025 James Bennion-Pedley
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SOC_CH32V_COMMON_H__

#include <stdbool.h>

#ifdef CONFIG_SOC_CH32V_CPU_IDLE_GATING
void chip_permit_idle(void);
void chip_block_idle(void);
bool chip_idle_not_allowed(void);
#endif /* CONFIG_SOC_CH32V_CPU_IDLE_GATING */

#endif /* __SOC_CH32V_COMMON_H__ */
