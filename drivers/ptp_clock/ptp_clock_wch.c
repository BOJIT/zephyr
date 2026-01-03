/*
 * Copyright 2026 James Bennion-Pedley
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ptp_clock

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ptp_wch, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/ptp_clock.h>

#include <hal_ch32fun.h>

/**
 * Addend * Increment = 2^63 / SysClk (144 MHz)
 * ptp_tick = Increment * 10^9 / 2^31
 *
 * +-----------+-----------+------------+
 * | ptp tick  | Increment | Addend     |
 * +-----------+-----------+------------+
 * |   20 ns   |    43     | 0x58C8EC2B |
 * |   14 ns   |    30     | 0x7F421F4F |
 * +-----------+-----------+------------+
 */

#define ADJ_FREQ_BASE_ADDEND    0x7F41992F
#define ADJ_FREQ_BASE_INCREMENT 30

struct ptp_clock_wch_config {
	ETH_TypeDef *regs;
	const struct device *clk_dev;
	uint8_t clk_id;
	const struct pinctrl_dev_config *pin_cfg;
};

struct ptp_clock_wch_data {
	struct k_mutex ptp_mutex;
	float clock_ratio;
};

static inline uint32_t subsec_to_nsec(uint32_t subsec)
{
	uint64_t val = subsec * 1000000000ll;
	val >>= 31;
	return val;
}

static inline uint32_t nsec_to_subsec(uint32_t nsec)
{
	uint64_t val = nsec * 0x80000000ll;
	val /= 1000000000;
	return val;
}

static int ptp_clock_wch_set(const struct device *dev, struct net_ptp_time *tm)
{
	struct ptp_clock_wch_data *data = dev->data;
	const struct ptp_clock_wch_config *config = dev->config;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	/* read old addend register value */
	uint32_t addend = config->regs->PTPTSAR;

	while ((config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTU) != 0U)
		;
	while ((config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTI) != 0U)
		;

	/* Load updated registers with absolute time */
	uint32_t seconds = (uint32_t)(tm->second);
	uint32_t subseconds = nsec_to_subsec(tm->nanosecond);

	config->regs->PTPTSHUR = seconds << 1; /* WCH Hardware Bug */
	config->regs->PTPTSLUR = ETH_PTP_PositiveTime | subseconds;

	/* Set TSSTI initialises the time */
	config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTI;
	while ((config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTI) != 0U)
		;

	/* Write back old addend register value. */
	config->regs->PTPTSAR = addend;
	config->regs->PTPTSCR |= ETH_PTPTSCR_TSARU;

	k_mutex_unlock(&data->ptp_mutex);

	return 0;
}

static int ptp_clock_wch_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct ptp_clock_wch_config *config = dev->config;

	tm->nanosecond = subsec_to_nsec(config->regs->PTPTSLR);
	tm->second = (uint64_t)(config->regs->PTPTSHR) >> 1; /* WCH Hardware Bug */

	return 0;
}

static int ptp_clock_wch_adjust(const struct device *dev, int increment)
{
	struct ptp_clock_wch_data *data = dev->data;
	const struct ptp_clock_wch_config *config = dev->config;

	int ret = 0;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	/* read old addend register value */
	uint32_t addend = config->regs->PTPTSAR;

	while ((config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTU) != 0U)
		;
	while ((config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTI) != 0U)
		;

	/* Write the offset (positive or negative) */
	uint32_t seconds = (uint32_t)(increment / NSEC_PER_SEC);
	uint32_t subseconds = nsec_to_subsec(increment % NSEC_PER_SEC);
	uint32_t sign = increment >= 0 ? ETH_PTP_PositiveTime : ETH_PTP_NegativeTime;

	ETH->PTPTSHUR = seconds;
	ETH->PTPTSLUR = sign | subseconds;

	/* Set TSSTU will adjust the clock by the offset */
	config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTU;
	while ((config->regs->PTPTSCR |= ETH_PTPTSCR_TSSTU) != 0U)
		;

	/* Write back old addend register value. */
	config->regs->PTPTSAR = addend;
	config->regs->PTPTSCR |= ETH_PTPTSCR_TSARU;

	k_mutex_unlock(&data->ptp_mutex);

	return ret;
}

static int ptp_clock_wch_rate_adjust(const struct device *dev, double ratio)
{
	struct ptp_clock_wch_data *data = dev->data;
	const struct ptp_clock_wch_config *config = dev->config;
	clock_control_subsys_t clock_sys;
	// int corr;
	// int32_t mul;
	// double val;
	uint32_t enet_ref_pll_rate;

	clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clk_id;
	(void)clock_control_get_rate(config->clk_dev, clock_sys, &enet_ref_pll_rate);
	int hw_inc = NSEC_PER_SEC / enet_ref_pll_rate;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	// TEMP, to test with WCH code
	int32_t adj = (ratio - 1) * 100000;
	if (adj > 5120000) {
		adj = 5120000;
	}
	if (adj < -5120000) {
		adj = -5120000;
	}

	// uint32_t addend = UINT32_MAX * (double)data->clock_ratio * ratio;
	uint32_t addend =
		((((275LL * adj) >> 8) * (ADJ_FREQ_BASE_ADDEND >> 24)) >> 6) + ADJ_FREQ_BASE_ADDEND;

	int32_t offset = addend - ADJ_FREQ_BASE_ADDEND;
	LOG_INF("Set Adj to: %u", adj);
	LOG_INF("Set Addend to: %u", offset);

	config->regs->PTPTSAR = (uint32_t)(addend);
	config->regs->PTPTSCR |= ETH_PTPTSCR_TSARU;

	k_mutex_unlock(&data->ptp_mutex);

	return 0;
}

static int ptp_clock_wch_init(const struct device *dev)
{
	struct ptp_clock_wch_data *data = dev->data;
	const struct ptp_clock_wch_config *config = dev->config;
	ETH_TypeDef *eth = config->regs;
	clock_control_subsys_t clock_sys;

	int ret = 0;

	/* enable clocks */
	clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clk_id;
	ret = clock_control_on(config->clk_dev, clock_sys);

	ret = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	k_mutex_init(&data->ptp_mutex);

	eth->PTPTSCR |= ETH_PTPTSCR_TSE;
	eth->PTPSSIR = ADJ_FREQ_BASE_INCREMENT;
	eth->PTPTSAR = ADJ_FREQ_BASE_ADDEND; // TODO calculate this from the clock prescaler
	eth->PTPTSCR |= ETH_PTPTSCR_TSARU;
	while ((eth->PTPTSCR & ETH_PTPTSCR_TSARU) != 0U)
		;

	eth->PTPTSCR |= ETH_PTPTSCR_TSFCU; /* Assume fine-update mode */
	eth->PTPTSHUR = 0x0;
	eth->PTPTSLUR = 0x0;

	eth->PTPTSCR |= ETH_PTPTSCR_TSSTI;
	while ((eth->PTPTSCR & ETH_PTPTSCR_TSSTI) != 0U)
		;

	// Set to sys time for now!
	// struct net_ptp_time t_sys = ns_to_net_ptp_time(k_uptime_get() * NSEC_PER_MSEC);
	// ptp_clock_wch_set(dev, &t_sys);
	// TODO set to ratio of subsecond counter to HCLK 144 MHz
	// data->clock_ratio =
	// 	((double)PTP_CLOCK_RATE) / ((double)ptp_hclk_rate);

	return 0;
}

static DEVICE_API(ptp_clock, ptp_clock_wch_api) = {
	.set = ptp_clock_wch_set,
	.get = ptp_clock_wch_get,
	.adjust = ptp_clock_wch_adjust,
	.rate_adjust = ptp_clock_wch_rate_adjust,
};

#define PTP_CLOCK_WCH_INIT(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct ptp_clock_wch_config ptp_clock_wch_config_##inst = {                   \
		.regs = (ETH_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst)),                          \
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_NAME(DT_INST_PARENT(inst), mac)),       \
		.clk_id = DT_CLOCKS_CELL_BY_NAME(DT_INST_PARENT(inst), mac, id),                   \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
	};                                                                                         \
	static struct ptp_clock_wch_data ptp_clock_wch_data_##inst;                                \
	DEVICE_DT_INST_DEFINE(inst, &ptp_clock_wch_init, NULL, &ptp_clock_wch_data_##inst,         \
			      &ptp_clock_wch_config_##inst, POST_KERNEL,                           \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_clock_wch_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_WCH_INIT)
