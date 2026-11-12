/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private types for the TI CC14xx TRX prop_rf backend.
 * Not part of the public driver API.
 */

#ifndef DRIVERS_PROP_RF_BACKENDS_TI_CC14XX_TRX_PROP_RF_CC14XX_TRX_H_
#define DRIVERS_PROP_RF_BACKENDS_TI_CC14XX_TRX_PROP_RF_CC14XX_TRX_H_

#include <stdint.h>

#include <zephyr/drivers/prop_rf.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "ti_cc14xx.h"

/*
 * TRX command slot assignments.
 */
#define CC14XX_TRX_SLOT_TX  0U
#define CC14XX_TRX_SLOT_RX  1U
#define CC14XX_TRX_SLOT_CW  2U

/*
 * Stream IDs. TX payload and RX data use separate stream IDs so the
 * TRX scheduler can demux them correctly.
 */
#define CC14XX_TRX_STREAM_TX 0U
#define CC14XX_TRX_STREAM_RX 1U

/*
 * PHY config ID stored on the TRX. ID 0 is the frontend config
 * (TRX_CONFIG_ID_FRONTEND). ID 1 is the first PHY / radio config.
 */
#define CC14XX_TRX_CONFIG_ID_PHY_BASE 1U

/*
 * RX packet metadata bytes appended after payload:
 * RSSI(1) + timestamp(4).
 */
#define CC14XX_TRX_RX_META_BYTES  5U

/*
 * Radio state machine. Radio returns to IDLE between every send() and
 * every recv(); no continuous background RX.
 */
enum cc14xx_trx_radio_state {
	RADIO_STATE_IDLE = 0,
	RADIO_STATE_TX_ACTIVE,
	RADIO_STATE_RX_ACTIVE,
};

/*
 * One entry in the compile-time PHY profile table built from DT child
 * nodes.
 */
struct cc14xx_trx_phy_profile {
	uint32_t frequency_hz;
	uint32_t sync_word;
	uint8_t  phy_config_id;
	uint16_t phy_option_mask;
};

/* DT-derived read-only device config. */
struct prop_rf_cc14xx_trx_config {
	const struct device                 *trx_dev;
	const struct cc14xx_trx_phy_profile *phy_table;
	uint8_t                              phy_count;
	uint16_t                             max_packet_len;
	uint32_t                             spi_bitrate; /**< SPI clock rate in Hz (from DT spi-max-frequency). */
	TRX_RfMode                           rf_mode;    /**< TRX RF firmware mode (SUN, PropFSK, …). */
	TRX_Dio2Config                       dio2;       /**< DIO2 function (PA control pin). */
	TRX_Dio3Config                       dio3;       /**< DIO3 function (LNA control pin). */
};

/* Per-instance mutable driver state. */
struct prop_rf_cc14xx_trx_data {
	atomic_t state;

	struct k_sem init_sem;      /* NV/device-config step completion */
	atomic_t init_error;        /* Set by init_config_callback on TRX error */
	struct k_sem tx_done;
	struct k_sem stream_done;   /* TX stream store completion */
	struct k_sem cmd_store_done; /* TX command store completion */
	struct k_sem cw_done;
	struct k_sem rx_sem;
	struct k_sem rx_cmd_store_done; /* RX command store completion */

	bool rx_stream_registered;

	/* Layer 1 config cache. */
	uint32_t cfg_frequency_hz;
	int8_t   cfg_tx_power_dbm;
	uint8_t  cfg_max_packet_len;

	/* TX command cache — store_cmds skipped when freq/power unchanged. */
	uint32_t stored_tx_freq_hz;
	int8_t   stored_tx_power_dbm;
	bool     tx_cmd_stored;

	/* Set by tx_callback / rx_callback on TRX_EventLastStatusError; checked by send()/recv(). */
	atomic_t tx_error;
	atomic_t rx_error;

	/* Observability counters. */
	uint32_t rx_ok_count;
	uint32_t rx_crc_err_count;
	uint32_t tx_ok_count;
	uint32_t tx_err_count;

	TRX_Request_CommandStore cmd_rx;
	TRX_Request_CommandStore cmd_tx;
	TRX_Request_CommandStore cmd_cw;
};

#endif /* DRIVERS_PROP_RF_BACKENDS_TI_CC14XX_TRX_PROP_RF_CC14XX_TRX_H_ */
