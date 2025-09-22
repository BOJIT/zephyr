/*
 * Copyright (c) 2025 James Bennion-Pedley <james@bojit.org>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_ethernet

#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ethernet_wch, LOG_LEVEL);

#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <ethernet/eth_stats.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/net/lldp.h>
#include <zephyr/drivers/hwinfo.h>

#include <hal_ch32fun.h>

#include "eth.h"

#if DT_INST_PROP(0, zephyr_random_mac_address)
#define ETH_WCH_RANDOM_MAC
#endif

#define ETH_10M_PHY_MODE HAL_ETH_10M_PHY_MODE
#define ETH_MII_MODE     HAL_ETH_MII_MODE
#define ETH_RMII_MODE    HAL_ETH_RMII_MODE
#define ETH_RGMII_MODE   HAL_ETH_RGMII_MODE

#define WCH_ETH_PHY_MODE(inst)                                                                     \
	((DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, rgmii)                                 \
		  ? ETH_RGMII_MODE                                                                 \
		  : (DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, mii) ? ETH_MII_MODE         \
									    : ETH_RMII_MODE)))

#define ETH_DMA_TX_TIMEOUT_MS (20U) /* transmit timeout in milliseconds */

#define ETH_RXBUFNB (4U)
#define ETH_TXBUFNB (4U)

struct eth_wch_config {
	ETH_TypeDef *regs;

	// void (*config_func)(void);

	// struct stm32_pclken pclken;
	// struct stm32_pclken pclken_rx;
	// struct stm32_pclken pclken_tx;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config_func)(const struct device *dev);
};

struct eth_wch_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_mutex tx_mutex;
	struct k_sem rx_int_sem;
#if defined(CONFIG_ETH_STM32_HAL_API_V2)
	struct k_sem tx_int_sem;
#endif /* CONFIG_ETH_STM32_HAL_API_V2 */
	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ETH_WCH_HAL_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
#if defined(CONFIG_ETH_STM32_MULTICAST_FILTER)
	uint8_t hash_index_cnt[64];
#endif /* CONFIG_ETH_STM32_MULTICAST_FILTER */
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

// TODO different if PTP enabled
typedef struct {
	uint32_t volatile Status;     /* Status */
	uint32_t ControlBufferSize;   /* Control and Buffer1, Buffer2 lengths */
	uint32_t Buffer1Addr;         /* Buffer1 address pointer */
	uint32_t Buffer2NextDescAddr; /* Buffer2 or next descriptor address pointer */
} ETH_DMADescTypeDef;                 // TODO make consistent with rest of Zephyr

static const struct device *eth_wch_phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle));

static ETH_DMADescTypeDef dma_rx_desc_tab[ETH_RXBUFNB] __aligned(4);
static ETH_DMADescTypeDef dma_tx_desc_tab[ETH_TXBUFNB] __aligned(4);

static uint8_t dma_rx_buffer[ETH_RXBUFNB][NET_ETH_MTU] __aligned(4);
static uint8_t dma_tx_buffer[ETH_TXBUFNB][NET_ETH_MTU] __aligned(4);

BUILD_ASSERT(NET_ETH_MTU % 4 == 0, "Rx buffer size must be a multiple of 4");

// static void setup_mac_filter(ETH_HandleTypeDef *heth)
// {
// 	__ASSERT_NO_MSG(heth != NULL);
// 	ETH_MACFilterConfigTypeDef MACFilterConf;

// 	HAL_ETH_GetMACFilterConfig(heth, &MACFilterConf);

// 	MACFilterConf.HashMulticast =
// 		IS_ENABLED(CONFIG_ETH_STM32_MULTICAST_FILTER) ? ENABLE : DISABLE;
// 	MACFilterConf.PassAllMulticast =
// 		IS_ENABLED(CONFIG_ETH_STM32_MULTICAST_FILTER) ? DISABLE : ENABLE;
// 	MACFilterConf.HachOrPerfectFilter = DISABLE;

// 	HAL_ETH_SetMACFilterConfig(heth, &MACFilterConf);

// 	k_sleep(K_MSEC(1));
// }

static int eth_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;
	int res;
	size_t total_len;
	size_t remaining_read;
	struct eth_stm32_tx_context *ctx = NULL;
	struct eth_stm32_tx_buffer_header *buf_header = NULL;
	// HAL_StatusTypeDef hal_ret = HAL_OK;

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);

	total_len = net_pkt_get_len(pkt);
	if (total_len > (NET_ETH_MTU * ETH_TXBUFNB)) {
		LOG_ERR("PKT too big");
		return -EIO;
	}

	/* the tx context is now owned by the HAL */
	ctx = NULL;

	/* Wait for end of TX buffer transmission */
	/* If the semaphore timeout breaks, it means */
	/* an error occurred or IT was not fired */
	// if (k_sem_take(&dev_data->tx_int_sem, K_MSEC(ETH_DMA_TX_TIMEOUT_MS)) != 0) {

	// 	goto error;
	// }

	res = 0;
error:

	return res;
}

static struct net_pkt *eth_rx(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;
	struct net_pkt *pkt;
	size_t total_len = 0;
	void *appbuf = NULL;
	struct eth_stm32_rx_buffer_header *rx_header;

	// if (HAL_ETH_ReadData(heth, &appbuf) != HAL_OK) {
	// 	/* no frame available */
	// 	return NULL;
	// }

	/* computing total length */
	// for (rx_header = (struct eth_stm32_rx_buffer_header *)appbuf; rx_header;
	//      rx_header = rx_header->next) {
	// 	total_len += rx_header->size;
	// }

	// pkt = net_pkt_rx_alloc_with_buffer(get_iface(dev_data), total_len, AF_UNSPEC, 0,
	// 				   K_MSEC(100));
	// if (!pkt) {
	// 	LOG_ERR("Failed to obtain RX buffer");
	// 	goto release_desc;
	// }

	// for (rx_header = (struct eth_stm32_rx_buffer_header *)appbuf; rx_header;
	//      rx_header = rx_header->next) {
	// 	const size_t index = rx_header - &dma_rx_buffer_header[0];

	// 	__ASSERT_NO_MSG(index < ETH_RXBUFNB);
	// 	if (net_pkt_write(pkt, dma_rx_buffer[index], rx_header->size)) {
	// 		LOG_ERR("Failed to append RX buffer to "
	// 			"context buffer");
	// 		net_pkt_unref(pkt);
	// 		pkt = NULL;
	// 		goto release_desc;
	// 	}
	// }

release_desc:
	// for (rx_header = (struct eth_stm32_rx_buffer_header *)appbuf; rx_header;
	//      rx_header = rx_header->next) {
	// 	rx_header->used = false;
	// }

	if (!pkt) {
		goto out;
	}

out:
	if (!pkt) {
		eth_stats_update_errors_rx(get_iface(dev_data));
	}

	return pkt;
}

static void rx_thread(void *arg1, void *unused1, void *unused2)
{
	const struct device *dev = (const struct device *)arg1;
	struct eth_wch_data *data = dev->data;
	struct net_if *iface;
	struct net_pkt *pkt;
	int res;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	while (1) {
		// res = k_sem_take(&dev_data->rx_int_sem, K_FOREVER);
		if (res == 0) {
			/* semaphore taken and receive packets */
			while ((pkt = eth_rx(dev)) != NULL) {
				iface = net_pkt_iface(pkt);
				res = net_recv_data(iface, pkt);
				if (res < 0) {
					eth_stats_update_errors_rx(net_pkt_iface(pkt));
					LOG_ERR("Failed to enqueue "
						"frame "
						"into RX queue: %d",
						res);
					net_pkt_unref(pkt);
				}
			}
		}
	}
}

static void eth_isr(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;

	// HAL_ETH_IRQHandler(heth);
}

static struct net_if *get_iface(struct eth_stm32_hal_dev_data *ctx)
{
	return NULL;
	// return ctx->iface;
}

static void generate_mac(uint8_t *mac_addr)
{
#if defined(ETH_WCH_RANDOM_MAC)
	gen_random_mac(mac_addr, ST_OUI_B0, ST_OUI_B1, ST_OUI_B2);
	mac_addr[0] |= 0x02; /* Ensure locally administered address */
#else                        /* Use user defined mac address */
	mac_addr[0] = 0x01;
	mac_addr[1] = 0x02;
	mac_addr[2] = 0x03;
	mac_addr[3] = 0x04;
	mac_addr[4] = 0x05;
	mac_addr[5] = 0x06;
	// TODO get builtin assigned WCH MAC
	// uint8_t unique_device_ID_12_bytes[12];
	// uint32_t result_mac_32_bits;
#endif
}

static int eth_wch_init(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	const struct eth_wch_config *config = dev->config;

	// ETH_HandleTypeDef *heth = &dev_data->heth;
	int ret = 0;

	// if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
	// 	LOG_ERR("clock control device not ready");
	// 	return -ENODEV;
	// }

	/* enable clock */
	// ret = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
	// 		       (clock_control_subsys_t)&cfg->pclken);
	// ret |= clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
	// 			(clock_control_subsys_t)&cfg->pclken_tx);
	// ret |= clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
	// 			(clock_control_subsys_t)&cfg->pclken_rx);

	if (ret) {
		LOG_ERR("Failed to enable ethernet clock");
		return -EIO;
	}

	/* configure pinmux */
	// ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	// if (ret < 0) {
	// 	LOG_ERR("Could not configure ethernet pins");
	// 	return ret;
	// }

	// generate_mac(dev_data->mac_addr);

	// heth->Init.MACAddr = dev_data->mac_addr;

	// LOG_DBG("MAC %02x:%02x:%02x:%02x:%02x:%02x", dev_data->mac_addr[0],
	// dev_data->mac_addr[1], 	dev_data->mac_addr[2], dev_data->mac_addr[3],
	// dev_data->mac_addr[4], 	dev_data->mac_addr[5]);

	return 0;
}

static int eth_mac_init_post(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;

	// heth->Init.TxDesc = dma_tx_desc_tab;
	// heth->Init.RxDesc = dma_rx_desc_tab;
	// heth->Init.RxBuffLen = ETH_STM32_RX_BUF_SIZE;

	// hal_ret = HAL_ETH_Init(heth);
	// if (hal_ret == HAL_TIMEOUT) {
	// 	/* HAL Init time out. This could be linked to */
	// 	/* a recoverable error. Log the issue and continue */
	// 	/* driver initialisation */
	// 	LOG_ERR("HAL_ETH_Init Timed out");
	// } else if (hal_ret != HAL_OK) {
	// 	LOG_ERR("HAL_ETH_Init failed: %d", hal_ret);
	// 	return -EINVAL;
	// }

	/* Initialize semaphores */
	// k_mutex_init(&dev_data->tx_mutex);
	// k_sem_init(&dev_data->rx_int_sem, 0, K_SEM_MAX_LIMIT);
	// k_sem_init(&dev_data->tx_int_sem, 0, K_SEM_MAX_LIMIT);

	/* Tx config init: */
	// memset(&tx_config, 0, sizeof(ETH_TxPacketConfig));
	// tx_config.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	// tx_config.ChecksumCtrl = IS_ENABLED(CONFIG_ETH_STM32_HW_CHECKSUM)
	// 				 ? ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC
	// 				 : ETH_CHECKSUM_DISABLE;
	// tx_config.CRCPadCtrl = ETH_CRC_PAD_INSERT;

	/* prepare tx buffer header */
	// for (uint16_t i = 0; i < ETH_TXBUFNB; ++i) {
	// 	dma_tx_buffer_header[i].tx_buff.buffer = dma_tx_buffer[i];
	// }

	return 0;
}

static void set_mac_config(const struct device *dev, struct phy_link_state *state)
{
	struct eth_wch_data *data = dev->data;

	// ETH_HandleTypeDef *heth = &dev_data->heth;
	// HAL_StatusTypeDef hal_ret = HAL_OK;
	// ETH_MACConfigTypeDef mac_config = {0};

	// HAL_ETH_GetMACConfig(heth, &mac_config);

	// mac_config.DuplexMode =
	// 	PHY_LINK_IS_FULL_DUPLEX(state->speed) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;

	// mac_config.Speed = IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32n6_ethernet),
	// 		PHY_LINK_IS_SPEED_1000M(state->speed) ? ETH_SPEED_1000M :)
	// PHY_LINK_IS_SPEED_100M(state->speed) 	? ETH_SPEED_100M 	: ETH_SPEED_10M;

	// hal_ret = HAL_ETH_SetMACConfig(heth, &mac_config);
	// if (hal_ret != HAL_OK) {
	// 	LOG_ERR("HAL_ETH_SetMACConfig: failed: %d", hal_ret);
	// }
}

static int eth_stm32_hal_start(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;
	// HAL_StatusTypeDef hal_ret = HAL_OK;

	LOG_DBG("Starting ETH HAL driver");

	// hal_ret = HAL_ETH_Start_IT(heth);
	// if (hal_ret != HAL_OK) {
	// 	LOG_ERR("HAL_ETH_Start{_IT} failed");
	// }

	return 0;
}

static int eth_stm32_hal_stop(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;
	// HAL_StatusTypeDef hal_ret = HAL_OK;

	LOG_DBG("Stopping ETH HAL driver");

	// hal_ret = HAL_ETH_Stop_IT(heth);
	// if (hal_ret != HAL_OK) {
	// 	/* HAL_ETH_Stop{_IT} returns HAL_ERROR only if ETH is
	// 	 * already stopped */
	// 	LOG_DBG("HAL_ETH_Stop{_IT} returned error (Ethernet "
	// 		"is already stopped)");
	// }

	return 0;
}

static void phy_link_state_changed(const struct device *phy_dev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct eth_wch_data *data = dev->data;

	ARG_UNUSED(phy_dev);

	/* The hal also needs to be stopped before changing the MAC
	 * config. The speed can change without receiving a link down
	 * callback before.
	 */
	eth_stm32_hal_stop(dev);
	if (state->is_up) {
		set_mac_config(dev, state);
		eth_stm32_hal_start(dev);
		// net_eth_carrier_on(dev_data->iface);
	} else {
		// net_eth_carrier_off(dev_data->iface);
	}
}

static void eth_wch_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_wch_data *data = dev->data;

	// ETH_HandleTypeDef *heth = &dev_data->heth;
	bool is_first_init = false;

	// if (dev_data->iface == NULL) {
	// 	dev_data->iface = iface;
	// 	is_first_init = true;
	// }

	/* Register Ethernet MAC Address with the upper layer */
	// net_if_set_link_addr(iface, dev_data->mac_addr, sizeof(dev_data->mac_addr),
	// 		     NET_LINK_ETHERNET);

	ethernet_init(iface);

	/* This function requires the Ethernet interface to be
	 * properly initialized. In auto-negotiation mode, it reads the
	 * speed and duplex settings to configure the driver
	 * accordingly.
	 */
	eth_mac_init_post(dev);

	// setup_mac_filter(heth);

	net_if_carrier_off(iface);
	net_lldp_set_lldpdu(iface);

	if (device_is_ready(eth_wch_phy_dev)) {
		phy_link_callback_set(eth_wch_phy_dev, phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}

	if (is_first_init) {
		const struct eth_wch_config *config = dev->config;
		/* Now that the iface is setup, we are safe to enable
		 * IRQs. */
		__ASSERT_NO_MSG(config->irq_config_func != NULL);
		config->irq_config_func(dev);

		/* Start interruption-poll thread */
		// k_thread_create(&dev_data->rx_thread, dev_data->rx_thread_stack,
		// 		K_KERNEL_STACK_SIZEOF(dev_data->rx_thread_stack), rx_thread,
		// 		(void *)dev, NULL, NULL,
		// 		IS_ENABLED(CONFIG_ETH_STM32_HAL_RX_THREAD_PREEMPTIVE)
		// 			? K_PRIO_PREEMPT(CONFIG_ETH_STM32_HAL_RX_THREAD_PRIO)
		// 			: K_PRIO_COOP(CONFIG_ETH_STM32_HAL_RX_THREAD_PRIO),
		// 		0, K_NO_WAIT);

		// k_thread_name_set(&dev_data->rx_thread, "wch_eth_rx");
	}
}

static enum ethernet_hw_caps eth_wch_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE
#if defined(CONFIG_NET_VLAN)
	       | ETHERNET_HW_VLAN
#endif
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	       | ETHERNET_PROMISC_MODE
#endif
#if defined(CONFIG_NET_LLDP)
	       | ETHERNET_LLDP
#endif
		;
}

static int eth_wch_set_config(const struct device *dev, enum ethernet_config_type type,
			      const struct ethernet_config *config)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		// memcpy(dev_data->mac_addr, config->mac_address.addr, 6);
		// heth->Instance->MACA0HR = (dev_data->mac_addr[5] << 8) | dev_data->mac_addr[4];
		// heth->Instance->MACA0LR = (dev_data->mac_addr[3] << 24) |
		// 			  (dev_data->mac_addr[2] << 16) |
		// 			  (dev_data->mac_addr[1] << 8) | dev_data->mac_addr[0];
		// net_if_set_link_addr(dev_data->iface, dev_data->mac_addr,
		// 		     sizeof(dev_data->mac_addr), NET_LINK_ETHERNET);
		return 0;
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		ETH_MACFilterConfigTypeDef MACFilterConf;

		HAL_ETH_GetMACFilterConfig(heth, &MACFilterConf);

		MACFilterConf.PromiscuousMode = config->promisc_mode ? ENABLE : DISABLE;

		HAL_ETH_SetMACFilterConfig(heth, &MACFilterConf);
		return 0;
#endif /* CONFIG_NET_PROMISCUOUS_MODE */
#if defined(ETH_WCH_MULTICAST_FILTER)
	case ETHERNET_CONFIG_TYPE_FILTER:
		// eth_stm32_mcast_filter(dev, &config->filter);
		return 0;
#endif /* ETH_WCH_MULTICAST_FILTER */
	default:
		break;
	}

	return -ENOTSUP;
}

static const struct device *eth_wch_get_phy(const struct device *dev)
{
	ARG_UNUSED(dev);
	return eth_wch_phy_dev;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_wch_get_stats(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;

	return &dev_data->stats;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static const struct ethernet_api eth_api = {
	.iface_api.init = eth_wch_iface_init,
	.get_capabilities = eth_wch_get_capabilities,
	.set_config = eth_wch_set_config,
	.get_phy = eth_wch_get_phy,
	.send = eth_tx,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_wch_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

#define ETH_WCH_IRQ_HANDLER_DECL(idx)                                                              \
	static void eth_wch_irq_config_func_##idx(const struct device *dev);
#define ETH_WCH_IRQ_HANDLER_FUNC(idx) .irq_config_func = eth_wch_irq_config_func_##idx,
#define ETH_WCH_IRQ_HANDLER(idx)                                                                   \
	static void eth_wch_irq_config_func_##idx(const struct device *dev)                        \
	{                                                                                          \
		/* IRQ 0 is core Interrupt (IRQ 1 is wakeup) */                                    \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), eth_isr,                \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}

#define ETH_WCH_DEVICE(inst)                                                                       \
	BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, mii) ||                     \
			     DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, rmii) ||            \
			     DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, rgmii) ||           \
			     DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, internal),          \
		     "Unsupported PHY connection type");                                           \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	ETH_WCH_IRQ_HANDLER_DECL(inst)                                                             \
	static const struct eth_wch_config eth_wch_config_##inst = {                               \
		.regs = (ETH_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(inst)),                          \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		ETH_WCH_IRQ_HANDLER_FUNC(inst)};                                                   \
	static struct eth_wch_data eth_wch_data_##inst;                                            \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, eth_wch_init, NULL, &eth_wch_data_##inst,              \
				      &eth_wch_config_##inst, CONFIG_ETH_INIT_PRIORITY, &eth_api,  \
				      NET_ETH_MTU);                                                \
	ETH_WCH_IRQ_HANDLER(inst)

DT_INST_FOREACH_STATUS_OKAY(ETH_WCH_DEVICE)
