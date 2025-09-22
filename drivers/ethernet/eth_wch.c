/*
 * Copyright (c) 2025 James Bennion-Pedley <james@bojit.org>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT wch_ch32_ethernet

#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ethernet_wch);

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

#include "eth.h"

#if DT_INST_PROP(0, zephyr_random_mac_address)
#define ETH_WCH_RANDOM_MAC
#endif

static const struct device *eth_wch_phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle));

#define ETH_10M_PHY_MODE HAL_ETH_10M_PHY_MODE
#define ETH_MII_MODE     HAL_ETH_MII_MODE
#define ETH_RMII_MODE    HAL_ETH_RMII_MODE
#define ETH_RGMII_MODE   HAL_ETH_RGMII_MODE

#define WCH_ETH_PHY_MODE(inst)                                                                     \
	((DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, rgmii)                                 \
		  ? ETH_RGMII_MODE                                                                 \
		  : (DT_INST_ENUM_HAS_VALUE(inst, phy_connection_type, mii) ? ETH_MII_MODE         \
									    : ETH_RMII_MODE)))

#define ETH_DMA_TX_TIMEOUT_MS 20U /* transmit timeout in milliseconds */

static ETH_DMADescTypeDef dma_rx_desc_tab[ETH_RXBUFNB] __aligned(4);
static ETH_DMADescTypeDef dma_tx_desc_tab[ETH_TXBUFNB] __aligned(4);

static uint8_t dma_rx_buffer[ETH_RXBUFNB][ETH_WCH_RX_BUF_SIZE] __aligned(4);
static uint8_t dma_tx_buffer[ETH_TXBUFNB][ETH_WCH_TX_BUF_SIZE] __aligned(4);

BUILD_ASSERT(ETH_WCH_RX_BUF_SIZE % 4 == 0, "Rx buffer size must be a multiple of 4");

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
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	int res;
	size_t total_len;
	size_t remaining_read;
	struct eth_stm32_tx_context *ctx = NULL;
	struct eth_stm32_tx_buffer_header *buf_header = NULL;
	HAL_StatusTypeDef hal_ret = HAL_OK;

#if defined(CONFIG_PTP_CLOCK_STM32_HAL)
	bool timestamped_frame;
#endif /* CONFIG_PTP_CLOCK_STM32_HAL */

	__ASSERT_NO_MSG(pkt != NULL);
	__ASSERT_NO_MSG(pkt->frags != NULL);

	total_len = net_pkt_get_len(pkt);
	if (total_len > (ETH_STM32_TX_BUF_SIZE * ETH_TXBUFNB)) {
		LOG_ERR("PKT too big");
		return -EIO;
	}

	k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);

	ctx = allocate_tx_context(pkt);
	buf_header = &dma_tx_buffer_header[ctx->first_tx_buffer_index];

	remaining_read = total_len;
	/* fill and allocate buffer until remaining data fits in one
	 * buffer */
	while (remaining_read > ETH_STM32_TX_BUF_SIZE) {
		if (net_pkt_read(pkt, buf_header->tx_buff.buffer, ETH_STM32_TX_BUF_SIZE)) {
			res = -ENOBUFS;
			goto error;
		}
		const uint16_t next_buffer_id = allocate_tx_buffer();

		buf_header->tx_buff.len = ETH_STM32_TX_BUF_SIZE;
		/* append new buffer to the linked list */
		buf_header->tx_buff.next = &dma_tx_buffer_header[next_buffer_id].tx_buff;
		/* and adjust tail pointer */
		buf_header = &dma_tx_buffer_header[next_buffer_id];
		remaining_read -= ETH_STM32_TX_BUF_SIZE;
	}
	if (net_pkt_read(pkt, buf_header->tx_buff.buffer, remaining_read)) {
		res = -ENOBUFS;
		goto error;
	}
	buf_header->tx_buff.len = remaining_read;
	buf_header->tx_buff.next = NULL;

	tx_config.Length = total_len;
	tx_config.pData = ctx;
	tx_config.TxBuffer = &dma_tx_buffer_header[ctx->first_tx_buffer_index].tx_buff;

	/* Reset TX complete interrupt semaphore before TX request*/
	k_sem_reset(&dev_data->tx_int_sem);

	/* tx_buffer is allocated on function stack, we need */
	/* to wait for the transfer to complete */
	/* So it is not freed before the interrupt happens */
	hal_ret = HAL_ETH_Transmit_IT(heth, &tx_config);
	if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_ETH_Transmit: failed!");
		res = -EIO;
		goto error;
	}

	/* the tx context is now owned by the HAL */
	ctx = NULL;

	/* Wait for end of TX buffer transmission */
	/* If the semaphore timeout breaks, it means */
	/* an error occurred or IT was not fired */
	if (k_sem_take(&dev_data->tx_int_sem, K_MSEC(ETH_DMA_TX_TIMEOUT_MS)) != 0) {

		LOG_ERR("HAL_ETH_TransmitIT tx_int_sem take timeout");
		res = -EIO;

		/* Check for errors */
		/* Ethernet device was put in error state */
		/* Error state is unrecoverable ? */
		if (HAL_ETH_GetState(heth) == HAL_ETH_STATE_ERROR) {
			LOG_ERR("%s: ETH in error state: errorcode:%x", __func__,
				HAL_ETH_GetError(heth));
			/* TODO recover from error state by restarting
			 * eth */
		}

		/* Check for DMA errors */
		if (HAL_ETH_GetDMAError(heth)) {
			LOG_ERR("%s: ETH DMA error: dmaerror:%x", __func__,
				HAL_ETH_GetDMAError(heth));
			/* DMA fatal bus errors are putting in error
			 * state*/
			/* TODO recover from this */
		}

		/* Check for MAC errors */
		if (HAL_ETH_GetMACError(heth)) {
			LOG_ERR("%s: ETH MAC error: macerror:%x", __func__,
				HAL_ETH_GetMACError(heth));
			/* MAC errors are putting in error state*/
			/* TODO recover from this */
		}

		goto error;
	}

	res = 0;
error:

	if (!ctx) {
		/* The HAL owns the tx context */
		HAL_ETH_ReleaseTxPacket(heth);
	} else {
		/* We need to release the tx context and its buffers */
		HAL_ETH_TxFreeCallback((uint32_t *)ctx);
	}

	k_mutex_unlock(&dev_data->tx_mutex);

	return res;
}

static struct net_pkt *eth_rx(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	struct net_pkt *pkt;
	size_t total_len = 0;
	void *appbuf = NULL;
	struct eth_stm32_rx_buffer_header *rx_header;

#if defined(CONFIG_PTP_CLOCK_STM32_HAL)
	struct net_ptp_time timestamp;
	ETH_TimeStampTypeDef ts_registers;
	/* Default to invalid value. */
	timestamp.second = UINT64_MAX;
	timestamp.nanosecond = UINT32_MAX;
#endif /* CONFIG_PTP_CLOCK_STM32_HAL */

	if (HAL_ETH_ReadData(heth, &appbuf) != HAL_OK) {
		/* no frame available */
		return NULL;
	}

	/* computing total length */
	for (rx_header = (struct eth_stm32_rx_buffer_header *)appbuf; rx_header;
	     rx_header = rx_header->next) {
		total_len += rx_header->size;
	}

#if defined(CONFIG_PTP_CLOCK_STM32_HAL)
	if (HAL_ETH_PTP_GetRxTimestamp(heth, &ts_registers) == HAL_OK) {
		timestamp.second = ts_registers.TimeStampHigh;
		timestamp.nanosecond = ts_registers.TimeStampLow;
	}
#endif /* CONFIG_PTP_CLOCK_STM32_HAL */

	pkt = net_pkt_rx_alloc_with_buffer(get_iface(dev_data), total_len, AF_UNSPEC, 0,
					   K_MSEC(100));
	if (!pkt) {
		LOG_ERR("Failed to obtain RX buffer");
		goto release_desc;
	}

	for (rx_header = (struct eth_stm32_rx_buffer_header *)appbuf; rx_header;
	     rx_header = rx_header->next) {
		const size_t index = rx_header - &dma_rx_buffer_header[0];

		__ASSERT_NO_MSG(index < ETH_RXBUFNB);
		if (net_pkt_write(pkt, dma_rx_buffer[index], rx_header->size)) {
			LOG_ERR("Failed to append RX buffer to "
				"context buffer");
			net_pkt_unref(pkt);
			pkt = NULL;
			goto release_desc;
		}
	}

release_desc:
	for (rx_header = (struct eth_stm32_rx_buffer_header *)appbuf; rx_header;
	     rx_header = rx_header->next) {
		rx_header->used = false;
	}

	if (!pkt) {
		goto out;
	}

#if defined(CONFIG_PTP_CLOCK_STM32_HAL)
	pkt->timestamp.second = timestamp.second;
	pkt->timestamp.nanosecond = timestamp.nanosecond;
	if (timestamp.second != UINT64_MAX) {
		net_pkt_set_rx_timestamping(pkt, true);
	}
#endif /* CONFIG_PTP_CLOCK_STM32_HAL */

out:
	if (!pkt) {
		eth_stats_update_errors_rx(get_iface(dev_data));
	}

	return pkt;
}

static void rx_thread(void *arg1, void *unused1, void *unused2)
{
	const struct device *dev = (const struct device *)arg1;
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	struct net_if *iface;
	struct net_pkt *pkt;
	int res;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	while (1) {
		res = k_sem_take(&dev_data->rx_int_sem, K_FOREVER);
		if (res == 0) {
			/* semaphore taken and receive packets */
			while ((pkt = eth_rx(dev)) != NULL) {
				iface = net_pkt_iface(pkt);
#if defined(CONFIG_NET_DSA_DEPRECATED)
				iface = dsa_net_recv(iface, &pkt);
#endif
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
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;

	HAL_ETH_IRQHandler(heth);
}

static struct net_if *get_iface(struct eth_stm32_hal_dev_data *ctx)
{
	return ctx->iface;
}

static void generate_mac(uint8_t *mac_addr)
{
#if defined(ETH_WCH_RANDOM_MAC)
	gen_random_mac(mac_addr, ST_OUI_B0, ST_OUI_B1, ST_OUI_B2);
#else /* Use user defined mac address */
	mac_addr[0] = ST_OUI_B0;
	mac_addr[1] = ST_OUI_B1;
	mac_addr[2] = ST_OUI_B2;
#if NODE_HAS_VALID_MAC_ADDR(DT_DRV_INST(0))
	mac_addr[3] = NODE_MAC_ADDR_OCTET(DT_DRV_INST(0), 3);
	mac_addr[4] = NODE_MAC_ADDR_OCTET(DT_DRV_INST(0), 4);
	mac_addr[5] = NODE_MAC_ADDR_OCTET(DT_DRV_INST(0), 5);
#else
	uint8_t unique_device_ID_12_bytes[12];
	uint32_t result_mac_32_bits;

	/* Nothing defined by the user, use device id */
	hwinfo_get_device_id(unique_device_ID_12_bytes, 12);
	result_mac_32_bits = crc32_ieee((uint8_t *)unique_device_ID_12_bytes, 12);
	memcpy(&mac_addr[3], &result_mac_32_bits, 3);

	/**
	 * Set MAC address locally administered bit (LAA) as this is
	 * not assigned by the manufacturer
	 */
	mac_addr[0] |= 0x02;

#endif /* NODE_HAS_VALID_MAC_ADDR(DT_DRV_INST(0))) */
#endif
}

static int eth_initialize(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	const struct eth_stm32_hal_dev_cfg *cfg = dev->config;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	int ret = 0;

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* enable clock */
	ret = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			       (clock_control_subsys_t)&cfg->pclken);
	ret |= clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t)&cfg->pclken_tx);
	ret |= clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t)&cfg->pclken_rx);
#if DT_INST_CLOCKS_HAS_NAME(0, mac_clk_ptp)
	ret |= clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t)&cfg->pclken_ptp);
#endif

	if (ret) {
		LOG_ERR("Failed to enable ethernet clock");
		return -EIO;
	}

	/* configure pinmux */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Could not configure ethernet pins");
		return ret;
	}

	generate_mac(dev_data->mac_addr);

	heth->Init.MACAddr = dev_data->mac_addr;

	LOG_DBG("MAC %02x:%02x:%02x:%02x:%02x:%02x", dev_data->mac_addr[0], dev_data->mac_addr[1],
		dev_data->mac_addr[2], dev_data->mac_addr[3], dev_data->mac_addr[4],
		dev_data->mac_addr[5]);

	return 0;
}

static int eth_init_api_v2(const struct device *dev)
{
	HAL_StatusTypeDef hal_ret = HAL_OK;
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;

	heth->Init.TxDesc = dma_tx_desc_tab;
	heth->Init.RxDesc = dma_rx_desc_tab;
	heth->Init.RxBuffLen = ETH_STM32_RX_BUF_SIZE;

	hal_ret = HAL_ETH_Init(heth);
	if (hal_ret == HAL_TIMEOUT) {
		/* HAL Init time out. This could be linked to */
		/* a recoverable error. Log the issue and continue */
		/* driver initialisation */
		LOG_ERR("HAL_ETH_Init Timed out");
	} else if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_ETH_Init failed: %d", hal_ret);
		return -EINVAL;
	}

#if defined(CONFIG_PTP_CLOCK_STM32_HAL)
	/* Enable timestamping of RX packets. We enable all packets to
	 * be timestamped to cover both IEEE 1588 and gPTP.
	 */
#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet)
	heth->Instance->MACTSCR |= ETH_MACTSCR_TSENALL;
#else
	heth->Instance->PTPTSCR |= ETH_PTPTSCR_TSSARFE;
#endif /* DT_HAS_COMPAT_STATUS_OKAY(st_stm32h7_ethernet) */
#endif /* CONFIG_PTP_CLOCK_STM32_HAL */

	/* Initialize semaphores */
	k_mutex_init(&dev_data->tx_mutex);
	k_sem_init(&dev_data->rx_int_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&dev_data->tx_int_sem, 0, K_SEM_MAX_LIMIT);

	/* Tx config init: */
	memset(&tx_config, 0, sizeof(ETH_TxPacketConfig));
	tx_config.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	tx_config.ChecksumCtrl = IS_ENABLED(CONFIG_ETH_STM32_HW_CHECKSUM)
					 ? ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC
					 : ETH_CHECKSUM_DISABLE;
	tx_config.CRCPadCtrl = ETH_CRC_PAD_INSERT;

	/* prepare tx buffer header */
	for (uint16_t i = 0; i < ETH_TXBUFNB; ++i) {
		dma_tx_buffer_header[i].tx_buff.buffer = dma_tx_buffer[i];
	}

	return 0;
}

static void set_mac_config(const struct device *dev, struct phy_link_state *state)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	HAL_StatusTypeDef hal_ret = HAL_OK;
	ETH_MACConfigTypeDef mac_config = {0};

	HAL_ETH_GetMACConfig(heth, &mac_config);

	mac_config.DuplexMode =
		PHY_LINK_IS_FULL_DUPLEX(state->speed) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;

	mac_config.Speed = IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32n6_ethernet),
			PHY_LINK_IS_SPEED_1000M(state->speed) ? ETH_SPEED_1000M :) PHY_LINK_IS_SPEED_100M(state->speed)
		? ETH_SPEED_100M
		: ETH_SPEED_10M;

	hal_ret = HAL_ETH_SetMACConfig(heth, &mac_config);
	if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_ETH_SetMACConfig: failed: %d", hal_ret);
	}
}

static int eth_stm32_hal_start(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	HAL_StatusTypeDef hal_ret = HAL_OK;

	LOG_DBG("Starting ETH HAL driver");

	hal_ret = HAL_ETH_Start_IT(heth);
	if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_ETH_Start{_IT} failed");
	}

	return 0;
}

static int eth_stm32_hal_stop(const struct device *dev)
{
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	HAL_StatusTypeDef hal_ret = HAL_OK;

	LOG_DBG("Stopping ETH HAL driver");

	hal_ret = HAL_ETH_Stop_IT(heth);
	if (hal_ret != HAL_OK) {
		/* HAL_ETH_Stop{_IT} returns HAL_ERROR only if ETH is
		 * already stopped */
		LOG_DBG("HAL_ETH_Stop{_IT} returned error (Ethernet "
			"is already stopped)");
	}

	return 0;
}

static void phy_link_state_changed(const struct device *phy_dev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct eth_stm32_hal_dev_data *dev_data = dev->data;

	ARG_UNUSED(phy_dev);

	/* The hal also needs to be stopped before changing the MAC
	 * config. The speed can change without receiving a link down
	 * callback before.
	 */
	eth_stm32_hal_stop(dev);
	if (state->is_up) {
		set_mac_config(dev, state);
		eth_stm32_hal_start(dev);
		net_eth_carrier_on(dev_data->iface);
	} else {
		net_eth_carrier_off(dev_data->iface);
	}
}

static void eth_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;
	bool is_first_init = false;

	if (dev_data->iface == NULL) {
		dev_data->iface = iface;
		is_first_init = true;
	}

	/* Register Ethernet MAC Address with the upper layer */
	net_if_set_link_addr(iface, dev_data->mac_addr, sizeof(dev_data->mac_addr),
			     NET_LINK_ETHERNET);

	ethernet_init(iface);

	/* This function requires the Ethernet interface to be
	 * properly initialized. In auto-negotiation mode, it reads the
	 * speed and duplex settings to configure the driver
	 * accordingly.
	 */
	eth_init_api_v2(dev);

	setup_mac_filter(heth);

	net_if_carrier_off(iface);

	net_lldp_set_lldpdu(iface);

	if (device_is_ready(eth_wch_phy_dev)) {
		phy_link_callback_set(eth_wch_phy_dev, phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}

	if (is_first_init) {
		const struct eth_stm32_hal_dev_cfg *cfg = dev->config;
		/* Now that the iface is setup, we are safe to enable
		 * IRQs. */
		__ASSERT_NO_MSG(cfg->config_func != NULL);
		cfg->config_func();

		/* Start interruption-poll thread */
		k_thread_create(&dev_data->rx_thread, dev_data->rx_thread_stack,
				K_KERNEL_STACK_SIZEOF(dev_data->rx_thread_stack), rx_thread,
				(void *)dev, NULL, NULL,
				IS_ENABLED(CONFIG_ETH_STM32_HAL_RX_THREAD_PREEMPTIVE)
					? K_PRIO_PREEMPT(CONFIG_ETH_STM32_HAL_RX_THREAD_PRIO)
					: K_PRIO_COOP(CONFIG_ETH_STM32_HAL_RX_THREAD_PRIO),
				0, K_NO_WAIT);

		k_thread_name_set(&dev_data->rx_thread, "stm_eth");
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
	struct eth_stm32_hal_dev_data *dev_data = dev->data;
	ETH_HandleTypeDef *heth = &dev_data->heth;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(dev_data->mac_addr, config->mac_address.addr, 6);
		heth->Instance->MACA0HR = (dev_data->mac_addr[5] << 8) | dev_data->mac_addr[4];
		heth->Instance->MACA0LR = (dev_data->mac_addr[3] << 24) |
					  (dev_data->mac_addr[2] << 16) |
					  (dev_data->mac_addr[1] << 8) | dev_data->mac_addr[0];
		net_if_set_link_addr(dev_data->iface, dev_data->mac_addr,
				     sizeof(dev_data->mac_addr), NET_LINK_ETHERNET);
		return 0;
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		ETH_MACFilterConfigTypeDef MACFilterConf;

		HAL_ETH_GetMACFilterConfig(heth, &MACFilterConf);

		MACFilterConf.PromiscuousMode = config->promisc_mode ? ENABLE : DISABLE;

		HAL_ETH_SetMACFilterConfig(heth, &MACFilterConf);
		return 0;
#endif /* CONFIG_NET_PROMISCUOUS_MODE */
#if defined(CONFIG_ETH_STM32_MULTICAST_FILTER)
	case ETHERNET_CONFIG_TYPE_FILTER:
		eth_stm32_mcast_filter(dev, &config->filter);
		return 0;
#endif /* CONFIG_ETH_STM32_MULTICAST_FILTER */
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
	.iface_api.init = eth_iface_init,
	.get_capabilities = eth_wch_get_capabilities,
	.set_config = eth_wch_set_config,
	.get_phy = eth_wch_get_phy,
	.send = eth_tx,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_wch_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

static void eth0_irq_config(void)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), eth_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

PINCTRL_DT_INST_DEFINE(0);

static const struct eth_stm32_hal_dev_cfg eth0_config = {
	.config_func = eth0_irq_config,
	.pclken = {.bus = DT_CLOCKS_CELL_BY_NAME(DT_INST_PARENT(0), stm_eth, bus),
		   .enr = DT_CLOCKS_CELL_BY_NAME(DT_INST_PARENT(0), stm_eth, bits)},
	.pclken_tx = {.bus = DT_INST_CLOCKS_CELL_BY_NAME(0, mac_clk_tx, bus),
		      .enr = DT_INST_CLOCKS_CELL_BY_NAME(0, mac_clk_tx, bits)},
	.pclken_rx = {.bus = DT_INST_CLOCKS_CELL_BY_NAME(0, mac_clk_rx, bus),
		      .enr = DT_INST_CLOCKS_CELL_BY_NAME(0, mac_clk_rx, bits)},
#if DT_INST_CLOCKS_HAS_NAME(0, mac_clk_ptp)
	.pclken_ptp = {.bus = DT_INST_CLOCKS_CELL_BY_NAME(0, mac_clk_ptp, bus),
		       .enr = DT_INST_CLOCKS_CELL_BY_NAME(0, mac_clk_ptp, bits)},
#endif
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, mii) ||
	     DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rmii)
		     IF_ENABLED(DT_HAS_COMPAT_STATUS_OKAY(st_stm32n6_ethernet),
	(|| DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, rgmii)
	 || DT_INST_ENUM_HAS_VALUE(0, phy_connection_type, gmii))),
						       "Unsupported PHY connection type");

static struct eth_stm32_hal_dev_data eth0_data = {
	.heth =
		{
			.Instance = (ETH_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(0)),
			.Init =
				{
					.MediaInterface = STM32_ETH_PHY_MODE(0),
				},
		},
};

ETH_NET_DEVICE_DT_INST_DEFINE(0, eth_initialize, NULL, &eth0_data, &eth0_config,
			      CONFIG_ETH_INIT_PRIORITY, &eth_api, ETH_STM32_HAL_MTU);
