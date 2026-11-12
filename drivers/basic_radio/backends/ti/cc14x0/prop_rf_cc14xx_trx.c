/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TI CC14xx external transceiver prop_rf backend.
 *
 * Model: fully synchronous. Every recv() submits a one-shot RX command
 * (repeat=false); the radio is idle between send()/recv() calls.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/prop_rf.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "prop_rf_cc14xx_trx.h"
#include "radio_settings/rcl_settings_wisun.h"

LOG_MODULE_REGISTER(prop_rf_cc14xx_trx, CONFIG_PROP_RF_LOG_LEVEL);

#define DT_DRV_COMPAT ti_cc14xx_prop_rf

#define INST DT_DRV_INST(0)

#define MAX_PACKET_LEN   CONFIG_PROP_RF_MAX_PACKET_LEN

/* RX buffer: TRX_PayloadHeader_SunFSK(4) + payload + RSSI(1) + timestamp(4) */
#define RX_BUF_SIZE  (sizeof(TRX_PayloadHeader_SunFSK) + MAX_PACKET_LEN + CC14XX_TRX_RX_META_BYTES)

/* NV config version references matching the SmartRF Studio 8 generated blobs. */
#define FE_CONFIG_REFERENCE  0xFEFEFEFEU
#define RF_CONFIG_REFERENCE  0xF000000DU

/* ---- Static storage ------------------------------------------------------ */

static uint8_t rx_buf[RX_BUF_SIZE];

/* ---- PHY profile table (built from DT child nodes) ----------------------- */

#define PHY_PROFILE_INIT(node_id)                                 \
{                                                                 \
	.frequency_hz    = DT_PROP(node_id, frequency_hz),            \
	.sync_word       = DT_PROP(node_id, sync_word),               \
	.phy_config_id   = DT_PROP(node_id, phy_config_id),           \
	.phy_option_mask = DT_PROP_OR(node_id, phy_option_mask, 0),   \
},

static const struct cc14xx_trx_phy_profile phy_table[] = {
	DT_INST_FOREACH_CHILD(0, PHY_PROFILE_INIT)
};

/* ---- Forward declarations ------------------------------------------------ */

static void init_config_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg);
static void rx_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg);
static void tx_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg);
static void cw_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg);
static void general_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg);

/* ---- State helpers ------------------------------------------------------- */

static inline enum cc14xx_trx_radio_state state_get( const struct prop_rf_cc14xx_trx_data *dd)
{
	return (enum cc14xx_trx_radio_state)atomic_get(&dd->state);
}

static inline void state_set(struct prop_rf_cc14xx_trx_data *dd, enum cc14xx_trx_radio_state s)
{
	atomic_set(&dd->state, (atomic_val_t)s);
}

/* ---- TRX command builders ------------------------------------------------ */

static void build_rx_cmd(struct prop_rf_cc14xx_trx_data *dd, const struct prop_rf_cc14xx_trx_config *cfg, uint32_t frequency_hz)
{
	const struct cc14xx_trx_phy_profile *phy = &cfg->phy_table[0];
	TRX_Request_CommandStore *cmd = &dd->cmd_rx;

	memset(cmd, 0, sizeof(*cmd));
	cmd->slot           = CC14XX_TRX_SLOT_RX;
	cmd->cmd_id         = TRX_RadioCommand_Receive;
	cmd->trigger        = Command_Trigger_Immediate;
	cmd->conflict_policy = TRX_ConflictPolicy_AlwaysInterrupt;
	cmd->allow_delay    = false;

	cmd->params.rx.phy0.config_id   = phy->phy_config_id;
	cmd->params.rx.phy0.option_mask = phy->phy_option_mask;
	cmd->params.rx.stream_id        = CC14XX_TRX_STREAM_RX;
	cmd->params.rx.frequency        = frequency_hz / 1000U; /* Hz -> kHz */
	cmd->params.rx.modem            = TRX_RadioCommand_Modem_FSK;
	cmd->params.rx.stream_early     = true;
	cmd->params.rx.timeout          = 0U;
	cmd->params.rx.repeat           = false;   /* one-shot */
	cmd->params.rx.search_strategy  = TRX_Rx_SearchStrategy_Sync;
}

static void build_tx_cmd(struct prop_rf_cc14xx_trx_data *dd, const struct prop_rf_cc14xx_trx_config *cfg, uint32_t frequency_hz, int8_t power_dbm)
{
	const struct cc14xx_trx_phy_profile *phy = &cfg->phy_table[0];
	TRX_Request_CommandStore *cmd = &dd->cmd_tx;
	
	/* Use the last entry from the Wi-SUN High PA calibration table (21.5 dBm).
	 * Sentinel values like TRX_TxPower_Use_Max are not reliably handled by
	 * all firmware versions; using a concrete in-table entry avoids INVALID_PARAM. */
	ARG_UNUSED(power_dbm);
	TRX_TxPower pwr = TRX_txPowerLevelsHigh_wisun.powerLevels[TRX_txPowerLevelsHigh_wisun.numEntries - 1U];

	memset(cmd, 0, sizeof(*cmd));
	cmd->slot           			= CC14XX_TRX_SLOT_TX;
	cmd->cmd_id         			= TRX_RadioCommand_Transmit;
	cmd->trigger        			= Command_Trigger_Immediate;
	cmd->conflict_policy 			= TRX_ConflictPolicy_AlwaysInterrupt;
	cmd->allow_delay    			= false;

	cmd->params.tx.phy0.config_id   = phy->phy_config_id;
	cmd->params.tx.phy0.option_mask = phy->phy_option_mask;
	cmd->params.tx.stream_id        = CC14XX_TRX_STREAM_TX;
	cmd->params.tx.frequency        = frequency_hz / 1000U;
	cmd->params.tx.modem            = TRX_RadioCommand_Modem_FSK;
	cmd->params.tx.pa               = TRX_TxPa_High;
	cmd->params.tx.power            = pwr;
}

/* ---- TRX callbacks ------------------------------------------------------- */

static void init_config_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg)
{
	const struct device *dev = (const struct device *)(uintptr_t)arg;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;

	ARG_UNUSED(handle);
	ARG_UNUSED(p_cmd_or_data);
	ARG_UNUSED(request);

	if (events & TRX_EventLastStatusError) {
		const TRX_Request_LastStatus *ls = (const TRX_Request_LastStatus *)request;

		LOG_ERR("NV init error: req=%u status=%u", (unsigned)(ls ? ls->req : 0xFFu), (unsigned)(ls ? ls->status : 0xFFu));
		atomic_set(&dd->init_error, 1);
		k_sem_give(&dd->init_sem);
		return;
	}

	if (events & (TRX_EventNvEraseComplete           |
		      TRX_EventConfigStoreComplete           |
		      TRX_EventConfigPersistComplete         |
		      TRX_EventDioSetupComplete              |
		      TRX_EventSetRfModeComplete             |
		      TRX_EventDeviceConfigStoreComplete)) {
		k_sem_give(&dd->init_sem);
	}
}

static void general_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(p_cmd_or_data);
	ARG_UNUSED(request);
	ARG_UNUSED(arg);

	if (events & TRX_EventTransportError)
	{
		LOG_ERR("TRX transport error");
	}
	if (events & TRX_EventNvCorrupt)
	{
		LOG_ERR("TRX NV corrupt");
	}
}

static void rx_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg)
{
	const struct device *dev = (const struct device *)(uintptr_t)arg;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;

	ARG_UNUSED(handle);

	if (events & TRX_EventLastStatusError)
	{
		const TRX_Request_LastStatus *ls = (const TRX_Request_LastStatus *)request;

		LOG_ERR("RX LastStatus error: req=%u status=%u", (unsigned)(ls ? ls->req : 0xFFu), (unsigned)(ls ? ls->status : 0xFFu));
		dd->rx_crc_err_count++;
		atomic_set(&dd->rx_error, 1);
		k_sem_give(&dd->rx_cmd_store_done);
		k_sem_give(&dd->rx_sem);
		return;
	}

	if (events & TRX_EventCmdStoreComplete)
	{
		k_sem_give(&dd->rx_cmd_store_done);
	}

	if (events & TRX_EventStreamStoreOverflow)
	{
		LOG_WRN("RX buffer overflow");
		return;
	}

	if (events & TRX_EventFinalStreamStoreReceived)
	{
		if ((uintptr_t)NULL == p_cmd_or_data) {
			return;
		}
		dd->rx_ok_count++;
		k_sem_give(&dd->rx_sem);
	}
}

static void tx_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg)
{
	const struct device *dev = (const struct device *)(uintptr_t)arg;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;

	ARG_UNUSED(handle);
	ARG_UNUSED(p_cmd_or_data);

	if (events & TRX_EventLastStatusError) {
		const TRX_Request_LastStatus *ls = (const TRX_Request_LastStatus *)request;

		LOG_ERR("TX last-status error: req=%u status=%u", (unsigned)(ls ? ls->req : 0xFFu), (unsigned)(ls ? ls->status : 0xFFu));
		atomic_set(&dd->tx_error, 1);
		/* Give all pending TX semaphores to prevent deadlock in send(). */
		k_sem_give(&dd->cmd_store_done);
		k_sem_give(&dd->stream_done);
		k_sem_give(&dd->tx_done);
		return;
	}

	if (events & TRX_EventStreamStoreComplete) {
		k_sem_give(&dd->stream_done);
	}

	if (events & TRX_EventCmdStoreComplete) {
		k_sem_give(&dd->cmd_store_done);
	}

	if (events & TRX_EventCmdStatus) {
		TRX_Request_CommandStatus *cs = (TRX_Request_CommandStatus *)request;

		if (cs->status == TRX_CommandStatus_Finished) {
			dd->tx_ok_count++;
		} else {
			LOG_WRN("TX ended status=%d", cs->status);
			dd->tx_err_count++;
		}
		k_sem_give(&dd->tx_done);
	}
}

static void cw_callback(TRX_Host_Handle handle, uintptr_t p_cmd_or_data, TRX_Request *request, uint64_t events, uintptr_t arg)
{
	const struct device *dev = (const struct device *)(uintptr_t)arg;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;

	ARG_UNUSED(handle);
	ARG_UNUSED(p_cmd_or_data);
	ARG_UNUSED(request);

	if (events & TRX_EventCmdStatus) {
		k_sem_give(&dd->cw_done);
	}
}

/* ---- Driver API ---------------------------------------------------------- */

static int prop_rf_cc14xx_trx_config(const struct device *dev, const struct prop_rf_config *cfg)
{
	struct prop_rf_cc14xx_trx_data *dd = dev->data;

	if (cfg->frequency == 0U) {
		return -EINVAL;
	}

	dd->cfg_frequency_hz   = cfg->frequency;
	dd->cfg_tx_power_dbm   = cfg->tx_power;
	dd->cfg_max_packet_len = (uint8_t)MIN(cfg->max_packet_len, MAX_PACKET_LEN);

	return 0;
}

static int prop_rf_cc14xx_trx_send(const struct device *dev, const uint8_t *data, uint16_t len, const struct prop_rf_tx_params *params, k_timeout_t timeout)
{
	const struct prop_rf_cc14xx_trx_config *cfg = dev->config;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;
	int ret = 0;

	TRX_Host_Status s;
	
	static const uint32_t supported_tx_flags = PROP_RF_TX_FLAG_SCHED_ABS | PROP_RF_TX_FLAG_CHANNEL | PROP_RF_TX_FLAG_POWER;
	if (params && (params->flags & ~supported_tx_flags))
	{
		return -ENOTSUP;
	}

	if (len == 0 || len > dd->cfg_max_packet_len)
	{
		return -EINVAL;
	}

	if (!atomic_cas(&dd->state, RADIO_STATE_IDLE, RADIO_STATE_TX_ACTIVE))
	{
		return -EBUSY;
	}

	LOG_INF("TI-cc14xx TRX");

	atomic_set(&dd->tx_error, 0);

	uint32_t freq  = (params && (params->flags & PROP_RF_TX_FLAG_CHANNEL)) ? params->frequency_hz : dd->cfg_frequency_hz;
	int8_t   power = (params && (params->flags & PROP_RF_TX_FLAG_POWER)) ? params->tx_power_dbm : dd->cfg_tx_power_dbm;
	bool sched_abs = params && (params->flags & PROP_RF_TX_FLAG_SCHED_ABS);

	/* Re-store TX command only when parameters differ from what the TRX has.
	 * Scheduled-absolute triggers change every call, so always re-store those. */
	bool needs_store = !dd->tx_cmd_stored || (freq  != dd->stored_tx_freq_hz) || (power != dd->stored_tx_power_dbm) || sched_abs;

	if (needs_store) {
		build_tx_cmd(dd, cfg, freq, power);
		if (sched_abs) {
			dd->cmd_tx.trigger       = Command_Trigger_Time_Absolute;
			dd->cmd_tx.trigger_param = params->abs_start_time;
		}
		k_sem_reset(&dd->cmd_store_done);
		s = ti_cc14xx_store_cmds(cfg->trx_dev, &dd->cmd_tx, 1U, NULL, tx_callback, TRX_EventCmdStoreComplete | TRX_EventCmdStatus | TRX_EventLastStatusError | TRX_EventCmdSubmitComplete);
		if (s != TRX_Host_Success)
		{
			LOG_ERR("TX cmd store failed: %d", s);
			ret = -EIO;
			goto out;
		}
		if (k_sem_take(&dd->cmd_store_done, timeout) != 0)
		{
			ret = -ETIMEDOUT;
			goto out;
		}
		if (atomic_get(&dd->tx_error)) {
			ret = -EIO;
			goto out;
		}
		dd->stored_tx_freq_hz   = freq;
		dd->stored_tx_power_dbm = power;
		dd->tx_cmd_stored       = true;
	}

	static uint8_t tx_pkt[sizeof(TRX_PayloadHeader_SunFSK) + 255];
	TRX_PayloadHeader_SunFSK *hdr = (TRX_PayloadHeader_SunFSK *)tx_pkt;

	memset(hdr, 0, sizeof(*hdr));
	hdr->length      = len;
	hdr->whitening   = 1U;
	hdr->fcs_mode    = 0U;
	hdr->mode_switch = 0U;
	hdr->modulation  = TRX_PayloadHeader_Modulation_FSK;
	memcpy(tx_pkt + sizeof(TRX_PayloadHeader_SunFSK), data, len);
	uint16_t pkt_len = (uint16_t)(sizeof(TRX_PayloadHeader_SunFSK) + len);

	k_sem_reset(&dd->stream_done);
	s = ti_cc14xx_store_stream(cfg->trx_dev, CC14XX_TRX_STREAM_TX, tx_pkt, pkt_len, Stream_Retention_Flush_Streaming, tx_callback, TRX_EventStreamStoreComplete | TRX_EventLastStatusError);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("TX stream store failed: %d", s);
		ret = -EIO;
		goto out;
	}
	if (k_sem_take(&dd->stream_done, timeout) != 0)
	{
		ret = -ETIMEDOUT;
		goto out;
	}
	if (atomic_get(&dd->tx_error)) {
		ret = -EIO;
		goto out;
	}

	k_sem_reset(&dd->tx_done);
	s = ti_cc14xx_submit_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_TX);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("TX cmd submit failed: %d", s);
		ret = -EIO;
		goto out;
	}
	if (k_sem_take(&dd->tx_done, timeout) != 0)
	{
		ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_TX, TRX_Command_StopType_Hard);
		ret = -ETIMEDOUT;
	} else if (atomic_get(&dd->tx_error)) {
		ret = -EIO;
	}

out:
	state_set(dd, RADIO_STATE_IDLE);
	return ret;
}

static int prop_rf_cc14xx_trx_recv(const struct device *dev, uint8_t *buf, uint16_t buf_len, const struct prop_rf_rx_params *params, k_timeout_t timeout, struct prop_rf_rx_info *info)
{
	const struct prop_rf_cc14xx_trx_config *cfg = dev->config;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;
	
	TRX_Host_Status s;

	static const uint32_t supported_rx_flags = PROP_RF_RX_FLAG_SCHED_ABS | PROP_RF_RX_FLAG_ONE_SHOT | PROP_RF_RX_FLAG_DISCARD_PAYL | PROP_RF_RX_FLAG_CHANNEL;
	if (params && (params->flags & ~supported_rx_flags))
	{
		return -ENOTSUP;
	}

	if (!atomic_cas(&dd->state, RADIO_STATE_IDLE, RADIO_STATE_RX_ACTIVE))
	{
		return -EBUSY;
	}

	atomic_set(&dd->rx_error, 0);

	if (!dd->rx_stream_registered)
	{
		s = ti_cc14xx_register_rx_stream(cfg->trx_dev, CC14XX_TRX_STREAM_RX, rx_buf, sizeof(rx_buf), rx_callback, TRX_EventFinalStreamStoreReceived | TRX_EventStreamStoreOverflow);
		if (s != TRX_Host_Success)
		{
			LOG_ERR("register_rx_stream failed: %d", s);
			state_set(dd, RADIO_STATE_IDLE);
			return -EIO;
		}
		dd->rx_stream_registered = true;
	}

	LOG_INF("TI-cc14xx TRX");

	uint32_t freq = (params && (params->flags & PROP_RF_RX_FLAG_CHANNEL)) ? params->frequency_hz : dd->cfg_frequency_hz;

	build_rx_cmd(dd, cfg, freq);

	if (params && (params->flags & PROP_RF_RX_FLAG_SCHED_ABS))
	{
		dd->cmd_rx.trigger       = Command_Trigger_Time_Absolute;
		dd->cmd_rx.trigger_param = params->abs_start_time;
	}

	/* Phase 1: store the command (no submit yet). */
	k_sem_reset(&dd->rx_cmd_store_done);
	k_sem_reset(&dd->rx_sem);
	s = ti_cc14xx_store_cmds(cfg->trx_dev, &dd->cmd_rx, 1U, NULL, rx_callback, TRX_EventCmdStoreComplete | TRX_EventLastStatusError);
	if (s != TRX_Host_Success) {
		LOG_ERR("RX cmd store failed: %d", s);
		state_set(dd, RADIO_STATE_IDLE);
		return -EIO;
	}
	if (k_sem_take(&dd->rx_cmd_store_done, timeout) != 0) {
		ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_RX, TRX_Command_StopType_Hard);
		state_set(dd, RADIO_STATE_IDLE);
		return -ETIMEDOUT;
	}
	if (atomic_get(&dd->rx_error)) {
		ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_RX, TRX_Command_StopType_Hard);
		state_set(dd, RADIO_STATE_IDLE);
		return -EIO;
	}

	/* Phase 2: submit the command and wait for received data. */
	s = ti_cc14xx_submit_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_RX);
	if (s != TRX_Host_Success) {
		LOG_ERR("RX cmd submit failed: %d", s);
		ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_RX, TRX_Command_StopType_Hard);
		state_set(dd, RADIO_STATE_IDLE);
		return -EIO;
	}

	if (k_sem_take(&dd->rx_sem, timeout) != 0) {
		ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_RX, TRX_Command_StopType_Hard);
		state_set(dd, RADIO_STATE_IDLE);
		return -ETIMEDOUT;
	}
	if (atomic_get(&dd->rx_error)) {
		ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_RX, TRX_Command_StopType_Hard);
		state_set(dd, RADIO_STATE_IDLE);
		return -EIO;
	}

	const TRX_PayloadHeader_SunFSK *hdr = (const TRX_PayloadHeader_SunFSK *)rx_buf;
	uint16_t payload_len = hdr->length;

	if (payload_len > buf_len)
	{
		state_set(dd, RADIO_STATE_IDLE);
		return -ENOMEM;
	}

	if (!(params && (params->flags & PROP_RF_RX_FLAG_DISCARD_PAYL)))
	{
		memcpy(buf, rx_buf + sizeof(TRX_PayloadHeader_SunFSK), payload_len);
	}

	if (info != NULL) {
		int8_t rssi = (int8_t)(rx_buf + sizeof(TRX_PayloadHeader_SunFSK))[payload_len];
		info->rssi      = rssi;
		info->crc_ok    = true;
		info->timestamp = 0;
	}

	state_set(dd, RADIO_STATE_IDLE);
	return (int)payload_len;
}

static int prop_rf_cc14xx_trx_test_cw(const struct device *dev, uint32_t frequency, int8_t tx_power, uint16_t duration_ms)
{
	const struct prop_rf_cc14xx_trx_config *cfg = dev->config;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;
	TRX_Host_Status s;
	TRX_TxPower pwr =
	{
		.dBm      = tx_power,
		.fraction = 0,
	};
	const struct cc14xx_trx_phy_profile *phy = &cfg->phy_table[0];

	if (!atomic_cas(&dd->state, RADIO_STATE_IDLE, RADIO_STATE_TX_ACTIVE))
	{
		return -EBUSY;
	}

	memset(&dd->cmd_cw, 0, sizeof(dd->cmd_cw));
	dd->cmd_cw.slot            = CC14XX_TRX_SLOT_CW;
	dd->cmd_cw.cmd_id          = TRX_RadioCommand_TransmitTest;
	dd->cmd_cw.trigger         = Command_Trigger_Immediate;
	dd->cmd_cw.conflict_policy = TRX_ConflictPolicy_AlwaysInterrupt;

	dd->cmd_cw.params.txTest.phy0.config_id    = phy->phy_config_id;
	dd->cmd_cw.params.txTest.phy0.option_mask  = phy->phy_option_mask;
	dd->cmd_cw.params.txTest.frequency         = frequency / 1000U;
	dd->cmd_cw.params.txTest.modem             = TRX_RadioCommand_Modem_FSK;
	dd->cmd_cw.params.txTest.mode              = TRX_TxTestMode_Unmodulated;
	dd->cmd_cw.params.txTest.pattern           = 0x55U;
	dd->cmd_cw.params.txTest.pa                = TRX_TxPa_High;
	dd->cmd_cw.params.txTest.power             = pwr;
	dd->cmd_cw.params.txTest.stream_id         = CC14XX_TRX_STREAM_TX;

	k_sem_reset(&dd->cw_done);
	s = ti_cc14xx_store_cmds(cfg->trx_dev, &dd->cmd_cw, 1U, &dd->cmd_cw, cw_callback,TRX_EventCmdStatus | TRX_EventLastStatusError);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("CW cmd store failed: %d", s);
		state_set(dd, RADIO_STATE_IDLE);
		return -EIO;
	}

	k_sleep(K_MSEC(duration_ms));

	ti_cc14xx_stop_cmd(cfg->trx_dev, CC14XX_TRX_SLOT_CW, TRX_Command_StopType_Graceful);
	k_sem_take(&dd->cw_done, K_MSEC(100));

	state_set(dd, RADIO_STATE_IDLE);
	return 0;
}

static uint32_t prop_rf_cc14xx_trx_get_caps(const struct device *dev)
{
	ARG_UNUSED(dev);
	return PROP_RF_CAP_CRC_HW | PROP_RF_CAP_TEST_CW | PROP_RF_CAP_HALF_DUPLEX;
}

static int prop_rf_cc14xx_trx_cca(const struct device *dev, bool *channel_clear)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_clear);
	return -ENOTSUP;
}

static int prop_rf_cc14xx_trx_ext_ctl(const struct device *dev, uint32_t cmd,
				       void *arg, size_t arg_sz)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cmd);
	ARG_UNUSED(arg);
	ARG_UNUSED(arg_sz);
	return -ENOTSUP;
}

/* ---- API vtable ---------------------------------------------------------- */

static const struct prop_rf_driver_api prop_rf_cc14xx_trx_api = {
	.config   = prop_rf_cc14xx_trx_config,
	.send     = prop_rf_cc14xx_trx_send,
	.recv     = prop_rf_cc14xx_trx_recv,
	.test_cw  = prop_rf_cc14xx_trx_test_cw,
	.get_caps = prop_rf_cc14xx_trx_get_caps,
	.cca      = prop_rf_cc14xx_trx_cca,
	.ext_ctl  = prop_rf_cc14xx_trx_ext_ctl,
};

/* ---- Init ---------------------------------------------------------------- */

static int prop_rf_cc14xx_trx_init(const struct device *dev)
{
	const struct prop_rf_cc14xx_trx_config *cfg = dev->config;
	struct prop_rf_cc14xx_trx_data *dd = dev->data;
	int ret;

	TRX_Host_Status s;
	
	if (!device_is_ready(cfg->trx_dev)) {
		LOG_ERR("ti_cc14xx device not ready");
		return -ENODEV;
	}

	k_sem_init(&dd->init_sem, 0, 1);
	atomic_set(&dd->init_error, 0);
	k_sem_init(&dd->tx_done, 0, 1);
	k_sem_init(&dd->stream_done, 0, 1);
	k_sem_init(&dd->cmd_store_done, 0, 1);
	k_sem_init(&dd->rx_sem, 0, 1);
	k_sem_init(&dd->rx_cmd_store_done, 0, 1);
	k_sem_init(&dd->cw_done, 0, 1);
	atomic_set(&dd->tx_error, 0);
	atomic_set(&dd->rx_error, 0);

	dd->rx_stream_registered = false;

	atomic_set(&dd->state, RADIO_STATE_IDLE);

	if (cfg->phy_count > 0U)
	{
		dd->cfg_frequency_hz = cfg->phy_table[0].frequency_hz;
	}
	dd->cfg_tx_power_dbm   = DT_INST_PROP_OR(0, tx_power_dbm, 0);
	dd->cfg_max_packet_len = (uint8_t)MIN(cfg->max_packet_len, MAX_PACKET_LEN);

	TRX_Host_Params params = 
	{
		.bitRate   = cfg->spi_bitrate,
		.generalCb = general_callback,
		.arg       = (uintptr_t)dev,
	};
	ret = ti_cc14xx_host_open(cfg->trx_dev, &params);
	if (ret != 0)
	{
		LOG_ERR("ti_cc14xx_host_open failed: %d", ret);
		return ret;
	}

	k_sleep(K_MSEC(100));

#define INIT_TIMEOUT K_MSEC(5000)
#define INIT_WAIT(label)                                      \
	if (k_sem_take(&dd->init_sem, INIT_TIMEOUT) != 0) {       \
		LOG_ERR(label " timed out");                          \
		return -ETIMEDOUT;                                    \
	}                                                         \
	if (atomic_get(&dd->init_error)) {                        \
		LOG_ERR(label " failed (TRX returned error)");        \
		return -EIO;                                          \
	}

	/* Configure DIO routing, clock source, power policy, and RF firmware
	 * in a single TRX packet. */
	TRX_DeviceConfigData device_cfg = {
		.dioConfig = {
			.dio0 = DIO0_UNCHANGED,
			.dio2 = (TRX_Dio2Config)cfg->dio2,
			.dio3 = (TRX_Dio3Config)cfg->dio3,
			.dio4 = DIO4_UNCHANGED,
			.dio5 = DIO5_UNCHANGED,
			.dio6 = DIO6_UNCHANGED,
		},
		.clockConfig = TRX_ClockConfig_XOSC,
		.powerMode = {
			.dwellTimeUs  = 0U,
			.powerPolicy  = TRX_PowerPolicy_StandbyDisallow,
		},
		.rfMode = cfg->rf_mode,
	};
	s = ti_cc14xx_store_device_config(cfg->trx_dev, device_cfg, init_config_callback);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("store_device_config failed: %d", s);
		return -EIO;
	}
	INIT_WAIT("store_device_config");

	/* Program TRX NV: erase, store FE config, persist, store PHY, persist. */
	s = ti_cc14xx_erase_nv(cfg->trx_dev, init_config_callback);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("erase_nv failed: %d", s);
		return -EIO;
	}
	INIT_WAIT("erase_nv");

	s = ti_cc14xx_store_config(cfg->trx_dev, TRX_CONFIG_ID_FRONTEND,
				   (uint8_t *)LRF_frontendRegConfig_wisun,
				   (uint16_t)LRF_frontendRegConfig_wisun_byteCount,
				   FE_CONFIG_REFERENCE, true,
				   init_config_callback,
				   TRX_EventConfigStoreComplete |
				   TRX_EventConfigPersistComplete |
				   TRX_EventLastStatusError);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("store_config FE failed: %d", s);
		return -EIO;
	}
	INIT_WAIT("store_config FE");

	s = ti_cc14xx_persist_config(cfg->trx_dev, TRX_CONFIG_ID_FRONTEND);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("persist_config FE failed: %d", s);
		return -EIO;
	}
	INIT_WAIT("persist_config FE");

	s = ti_cc14xx_store_config(cfg->trx_dev, CC14XX_TRX_CONFIG_ID_PHY_BASE,
				   (uint8_t *)LRF_mainRegConfig_wisun,
				   (uint16_t)LRF_mainRegConfig_wisun_byteCount,
				   RF_CONFIG_REFERENCE, false,
				   init_config_callback,
				   TRX_EventConfigStoreComplete |
				   TRX_EventConfigPersistComplete |
				   TRX_EventLastStatusError);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("store_config PHY failed: %d", s);
		return -EIO;
	}
	INIT_WAIT("store_config PHY");

	s = ti_cc14xx_persist_config(cfg->trx_dev, CC14XX_TRX_CONFIG_ID_PHY_BASE);
	if (s != TRX_Host_Success)
	{
		LOG_ERR("persist_config PHY failed: %d", s);
		return -EIO;
	}
	INIT_WAIT("persist_config PHY");

	LOG_INF("CC14xx TRX prop_rf backend ready");
	return 0;
}

/* ---- Device instantiation ------------------------------------------------ */

static const struct prop_rf_cc14xx_trx_config prop_rf_cc14xx_trx_cfg_0 =
{
	.trx_dev        = DEVICE_DT_GET(DT_INST_PHANDLE(0, trx_dev)),
	.phy_table      = phy_table,
	.phy_count      = ARRAY_SIZE(phy_table),
	.max_packet_len = MAX_PACKET_LEN,
	.spi_bitrate    = DT_PROP(DT_INST_PHANDLE(0, trx_dev), spi_max_frequency),
	.rf_mode        = (TRX_RfMode)DT_INST_PROP_OR(0, rf_mode, 0),
	.dio2           = (TRX_Dio2Config)DT_INST_PROP_OR(0, dio2_function, 0x54),
	.dio3           = (TRX_Dio3Config)DT_INST_PROP_OR(0, dio3_function, 0x12),
};

static struct prop_rf_cc14xx_trx_data prop_rf_cc14xx_trx_data_0;

DEVICE_DT_INST_DEFINE(0, prop_rf_cc14xx_trx_init, NULL,
					  &prop_rf_cc14xx_trx_data_0, &prop_rf_cc14xx_trx_cfg_0,
	                  POST_KERNEL, CONFIG_PROP_RF_INIT_PRIORITY, &prop_rf_cc14xx_trx_api);
