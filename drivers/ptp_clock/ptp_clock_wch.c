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

struct ptp_clock_wch_config {
	ETH_TypeDef *regs;
	const struct device *clk_dev;
	uint8_t clk_id;
	const struct pinctrl_dev_config *pin_cfg;
};

struct ptp_clock_wch_data {
	struct k_mutex ptp_mutex;
};

static int ptp_clock_wch_set(const struct device *dev, struct net_ptp_time *tm)
{
	struct ptp_clock_wch_data *data = dev->data;

	// TODO

	return 0;
}

static int ptp_clock_wch_get(const struct device *dev, struct net_ptp_time *tm)
{
	struct ptp_clock_wch_data *data = dev->data;

	// TODO

	return 0;
}

static int ptp_clock_wch_adjust(const struct device *dev, int increment)
{
	struct ptp_clock_wch_data *data = dev->data;
	int ret = 0;

	// TODO

	return ret;
}

static int ptp_clock_wch_rate_adjust(const struct device *dev, double ratio)
{
	const struct ptp_clock_wch_config *config = dev->config;
	struct ptp_clock_wch_data *data = dev->data;
	clock_control_subsys_t clock_sys;
	int corr;
	int32_t mul;
	double val;
	uint32_t enet_ref_pll_rate;

	clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clk_id;
	(void)clock_control_get_rate(config->clk_dev, clock_sys, &enet_ref_pll_rate);
	int hw_inc = NSEC_PER_SEC / enet_ref_pll_rate;

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	k_mutex_unlock(&data->ptp_mutex);

	return 0;
}

static int ptp_clock_wch_init(const struct device *dev)
{
	const struct ptp_clock_wch_config *config = dev->config;
	struct ptp_clock_wch_data *data = dev->data;
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

	LOG_WRN("PTP Clock Init");

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
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(inst))),                    \
		.clk_id = DT_CLOCKS_CELL(DT_INST_PARENT(inst), id),                                \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
	};                                                                                         \
	static struct ptp_clock_wch_data ptp_clock_wch_data_##inst;                                \
	DEVICE_DT_INST_DEFINE(inst, &ptp_clock_wch_init, NULL, &ptp_clock_wch_data_##inst,         \
			      &ptp_clock_wch_config_##inst, POST_KERNEL,                           \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &ptp_clock_wch_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_WCH_INIT)
