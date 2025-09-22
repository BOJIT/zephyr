/*
 * Copyright (c) 2025 James Bennion-Pedley <james@bojit.org>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>

#define DT_DRV_COMPAT wch_ch32_mdio

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdio_wch, CONFIG_MDIO_LOG_LEVEL);

struct mdio_wch_data {
	struct k_sem sem;
	int placeholder;
	// ETH_HandleTypeDef heth;
};

struct mdio_wch_config {
	const struct pinctrl_dev_config *pincfg;
};

static int mdio_wch_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct mdio_wch_data *const dev_data = dev->data;
	int ret = 0;

	k_sem_take(&dev_data->sem, K_FOREVER);

	k_sem_give(&dev_data->sem);

	return ret;
}

static int mdio_wch_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct mdio_wch_data *const dev_data = dev->data;
	int ret = 0;

	k_sem_take(&dev_data->sem, K_FOREVER);

	k_sem_give(&dev_data->sem);

	return ret;
}

static int mdio_wch_init(const struct device *dev)
{
	struct mdio_wch_data *const dev_data = dev->data;
	const struct mdio_wch_config *const config = dev->config;
	int ret = 0;

	/* enable clock */
	// ret = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
	// 		       (clock_control_subsys_t)&config->pclken);
	// if (ret < 0) {
	// 	LOG_ERR("Failed to enable ethernet clock needed for MDIO (%d)", ret);
	// 	return ret;
	// }

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&dev_data->sem, 1, 1);

	return 0;
}

static DEVICE_API(mdio, mdio_wch_api) = {
	.read = mdio_wch_read,
	.write = mdio_wch_write,
};

#define MDIO_WCH_DEVICE(inst)                                                                      \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static struct mdio_wch_data mdio_wch_data_##inst = {                                       \
		.placeholder = 0,                                                                  \
	};                                                                                         \
	static struct mdio_wch_config mdio_wch_config_##inst = {                                   \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &mdio_wch_init, NULL, &mdio_wch_data_##inst,                   \
			      &mdio_wch_config_##inst, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,     \
			      &mdio_wch_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_WCH_DEVICE)
