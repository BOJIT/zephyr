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

	const struct device *clk_dev;
	uint8_t clk_id;
	const struct device *clk_tx_dev;
	uint8_t clk_tx_id;
	const struct device *clk_rx_dev;
	uint8_t clk_rx_id;

	bool use_random_mac;

	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config_func)(const struct device *dev);
};

struct eth_wch_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	struct k_mutex tx_mutex;
	struct k_sem rx_int_sem;
	struct k_sem tx_int_sem;
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
		res = k_sem_take(&data->rx_int_sem, K_FOREVER);
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
	const struct eth_wch_config *config = dev->config;
	ETH_TypeDef *eth = config->regs;

	uint32_t status_flags = ETH->DMASR;

	/* Error Flags */
	if (status_flags & ETH_DMA_IT_AIS) {
		if (status_flags & ETH_DMA_IT_RBU) {
			// if ((ChipId & 0xf0) == 0x10) {
			// 	((ETH_DMADESCTypeDef *)(((ETH_DMADESCTypeDef *)(ETH->DMACHRDR))
			// 					->Buffer2NextDescAddr))
			// 		->Status = ETH_DMARxDesc_OWN;

			// 	/* Resume DMA reception */
			// 	ETH->DMARPDR = 0;
			// }
			eth->DMASR = ETH_DMA_IT_RBU;
			// Out of descriptors. Write to RPDR to continue reception
		}
		eth->DMASR = ETH_DMA_IT_AIS;
	}

	/* Standard Flags */
	if (status_flags & ETH_DMA_IT_NIS) {
		if (status_flags & ETH_DMA_IT_R) {
			k_sem_give(&data->rx_int_sem);
			eth->DMASR = ETH_DMA_IT_R;
		}
		if (status_flags & ETH_DMA_IT_T) {
			k_sem_give(&data->tx_int_sem);
			eth->DMASR = ETH_DMA_IT_T;
		}
		if (status_flags & ETH_DMA_IT_PHYLINK) {
			// ETH_PHYLink();
			// TODO handle PHY status (different for internal and external)
			eth->DMASR = ETH_DMA_IT_PHYLINK;
		}
		eth->DMASR = ETH_DMA_IT_NIS;
	}
}

static struct net_if *get_iface(struct eth_wch_data *data)
{
	return data->iface;
}

static void generate_mac(uint8_t *mac_addr, bool random)
{
	/* WCH uses this as a MAC address, unclear if it is an IEEE-assigned MAC */
	uint8_t *mac_base = (uint8_t *)(ROM_CFG_USERADR_ID);

	for (size_t i = 0; i < NET_ETH_ADDR_LEN; i++) {
		mac_addr[i] = mac_base[NET_ETH_ADDR_LEN - 1 - i];
	}

	if (random) {
		// Generate random address based on WCH OUI (burned into the chip)
		gen_random_mac(mac_addr, mac_addr[0], mac_addr[1], mac_addr[2]);
	}
}

static int eth_mac_init(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	const struct eth_wch_config *config = dev->config;
	ETH_TypeDef *eth = config->regs;

	// Set MAC Address in Hardware
	eth->MACA0HR = (data->mac_addr[5] << 8) | data->mac_addr[4];
	eth->MACA0LR = (data->mac_addr[3] << 24) | (data->mac_addr[2] << 16) |
		       (data->mac_addr[1] << 8) | data->mac_addr[0];

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

static int eth_wch_start(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	// ETH_HandleTypeDef *heth = &dev_data->heth;
	// HAL_StatusTypeDef hal_ret = HAL_OK;

	LOG_DBG("Starting ETH HAL driver");

	// ETH_MACTransmissionCmd(ENABLE);
	// ETH_FlushTransmitFIFO();
	// ETH_MACReceptionCmd(ENABLE);
	// ETH_DMATransmissionCmd(ENABLE);
	// ETH_DMAReceptionCmd(ENABLE);
	// hal_ret = HAL_ETH_Start_IT(heth);
	// if (hal_ret != HAL_OK) {
	// 	LOG_ERR("HAL_ETH_Start{_IT} failed");
	// }

	return 0;
}

static int eth_wch_stop(const struct device *dev)
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

// TODO where is this used?
static void phy_link_state_changed(const struct device *phy_dev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct eth_wch_data *data = dev->data;

	ARG_UNUSED(phy_dev);

	/* The MAC also needs to be stopped before changing the MAC
	 * config. The speed can change without receiving a link down
	 * callback before.
	 */
	eth_wch_stop(dev);
	if (state->is_up) {
		set_mac_config(dev, state);
		eth_wch_start(dev);
		net_eth_carrier_on(data->iface);
	} else {
		net_eth_carrier_off(data->iface);
	}
}

static void eth_wch_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_wch_data *data = dev->data;
	const struct eth_wch_config *config = dev->config;

	/* Certain initialisation is only done once per interface */
	bool is_first_init = false;
	if (data->iface == NULL) {
		data->iface = iface;
		is_first_init = true;
	}

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);

	/* Initialise interface with relevant hardware settings */
	ethernet_init(iface);
	eth_mac_init(dev);

	// setup_mac_filter(heth);

	net_if_carrier_off(iface);
	net_lldp_set_lldpdu(iface);

	/* Initialize semaphores */
	k_mutex_init(&data->tx_mutex);
	k_sem_init(&data->rx_int_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->tx_int_sem, 0, K_SEM_MAX_LIMIT);

	if (device_is_ready(eth_wch_phy_dev)) {
		phy_link_callback_set(eth_wch_phy_dev, phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}

	if (is_first_init) {
		/* Now that the iface is setup, we are safe to enable IRQs. */
		__ASSERT_NO_MSG(config->irq_config_func != NULL);
		config->irq_config_func(dev);

		/* Start interruption-poll thread */
		k_thread_create(&data->rx_thread, data->rx_thread_stack,
				K_KERNEL_STACK_SIZEOF(data->rx_thread_stack), rx_thread,
				(void *)dev, NULL, NULL,
				K_PRIO_COOP(CONFIG_ETH_WCH_HAL_RX_THREAD_PRIO), 0, K_NO_WAIT);

		k_thread_name_set(&data->rx_thread, dev->name);
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
	const struct eth_wch_config *dev_config = dev->config;
	ETH_TypeDef *eth = dev_config->regs;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, 6);
		eth->MACA0HR = (data->mac_addr[5] << 8) | data->mac_addr[4];
		eth->MACA0LR = (data->mac_addr[3] << 24) | (data->mac_addr[2] << 16) |
			       (data->mac_addr[1] << 8) | data->mac_addr[0];
		net_if_set_link_addr(data->iface, data->mac_addr, sizeof(data->mac_addr),
				     NET_LINK_ETHERNET);
		return 0;
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		if (config->promisc_mode) {
			eth->MACFFR |= ETH_MACFFR_PM;
		} else {
			eth->MACFFR &= ~ETH_MACFFR_PM;
		}
		return 0;
#endif /* CONFIG_NET_PROMISCUOUS_MODE */
#if defined(ETH_WCH_MULTICAST_FILTER)
	case ETHERNET_CONFIG_TYPE_FILTER:
		// eth_stm32_mcast_filter(dev, &config->filter); // TODO
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

static int eth_wch_init(const struct device *dev)
{
	struct eth_wch_data *data = dev->data;
	const struct eth_wch_config *config = dev->config;

	int ret = 0;

	/* enable clocks */
	clock_control_subsys_t clock_sys;
	clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clk_id;
	ret = clock_control_on(config->clk_dev, clock_sys);
	clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clk_tx_id;
	ret |= clock_control_on(config->clk_tx_dev, clock_sys);
	clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clk_rx_id;
	ret |= clock_control_on(config->clk_rx_dev, clock_sys);

	if (ret) {
		LOG_ERR("Failed to enable ethernet clocks");
		return -EIO;
	}

	/* Enable Internal PHY (note - this is not controlled on a per-peripheral basis!) */
	// if(DT_NODE_HAS_PROP) {
	// 	EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN;
	// }

	/* configure pinmux */
	ret = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Could not configure ethernet pins (%d)", ret);
		return ret;
	}

	// Generate MAC address (once at boot)
	generate_mac(data->mac_addr, config->use_random_mac);
	LOG_DBG("MAC %02x:%02x:%02x:%02x:%02x:%02x", data->mac_addr[0], data->mac_addr[1],
		data->mac_addr[2], data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);

	return 0;
}

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
		.clk_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(inst))),                    \
		.clk_id = DT_CLOCKS_CELL(DT_INST_PARENT(inst), id),                                \
		.clk_tx_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(inst, 0)),                  \
		.clk_tx_id = DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, id),                              \
		.clk_rx_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_IDX(inst, 1)),                  \
		.clk_rx_id = DT_INST_CLOCKS_CELL_BY_IDX(inst, 1, id),                              \
		.use_random_mac = DT_INST_PROP(inst, zephyr_random_mac_address),                   \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		ETH_WCH_IRQ_HANDLER_FUNC(inst)};                                                   \
	static struct eth_wch_data eth_wch_data_##inst;                                            \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, eth_wch_init, NULL, &eth_wch_data_##inst,              \
				      &eth_wch_config_##inst, CONFIG_ETH_INIT_PRIORITY, &eth_api,  \
				      NET_ETH_MTU);                                                \
	ETH_WCH_IRQ_HANDLER(inst)

DT_INST_FOREACH_STATUS_OKAY(ETH_WCH_DEVICE)
