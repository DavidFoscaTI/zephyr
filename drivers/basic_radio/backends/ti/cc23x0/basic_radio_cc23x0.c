/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TI CC23x0 Basic Radio backend.
 *
 * Wraps the TI Radio Control Layer (RCL) commands behind the
 * Zephyr basic_radio driver API.
 *
 * PHY selection comes from devicetree. Each basic-radio-base PHY
 * child of the radio node selects one operating mode of a
 * ti,cc23x0-radio-phy-config node through phy-mode. That node names
 * the LRF_Config (SmartRF Studio C export) the driver links against:
 * certified PHYs come from hal_ti, custom PHYs from the application.
 */

#define DT_DRV_COMPAT ti_cc23x0_basic_radio

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/basic_radio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <ti/devices/DeviceFamily.h>
#include <ti/drivers/rcl/RCL.h>
#include <ti/drivers/rcl/LRF.h>
#include <ti/drivers/rcl/commands/generic.h>

#include <inc/hw_memmap.h>
#include <inc/hw_systim.h>
#include <inc/hw_types.h>
#include <inc/pbe_generic_ram_regs.h>
#include <rf_patches/lrf_pbe_binary_generic.h>

#include "basic_radio_cc23x0.h"

LOG_MODULE_REGISTER(basic_radio_cc23x0, CONFIG_BASIC_RADIO_LOG_LEVEL);

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1, "basic_radio_cc23x0 backend supports at most one enabled instance");

#define INST                0
#define RX_BUFFER_COUNT     CONFIG_BASIC_RADIO_RX_BUFFER_COUNT
#define RX_BUFFER_SIZE      CONFIG_BASIC_RADIO_CC23X0_RX_BUFFER_SIZE
#define TX_BUFFER_COUNT     CONFIG_BASIC_RADIO_TX_BUFFER_COUNT
#define MAX_PACKET_LEN      CONFIG_BASIC_RADIO_MAX_PACKET_LEN

/*****************************************************************************************************************/
/*
 * PHY table: one entry per basic-radio-base PHY child.
 *
 * Hierarchy: instance → phy child (phy-mode → PHY node in radio-phys)
 *
 * PHY node (radio-phys child) provides:
 *   - ti,lrf-config: C symbol name of LRF_Config
 *   - ti,len-pos, ti,num-len-bits: LENCFG for buffer layout
 *   - ti,phy-feature: phyFeatures selector
 *   - frequency-min-hz, frequency-max-hz: characterized range
 *   - datarate-bps, modulation, deviation-hz, crc-enable, variable-length: waveform metadata
 *
 * PHY child (basic-radio-base child) provides optional boot-time defaults:
 *   - frequency-hz, sync-word, max-packet-length, tx-power-dbm
 *   These are overridden by basic_radio_config() at runtime.
 */
#define PHY_MODE(child)    DT_PHANDLE(child, phy_mode)
#define PHY_LRF_SYM(child) DT_STRING_TOKEN(PHY_MODE(child), ti_lrf_config)

/*
 * Declare only the LRF_Configs the enabled phy children reference, so
 * unused PHYs are dropped at link time.
 */
#define PHY_LRF_EXTERN(child) extern const LRF_Config PHY_LRF_SYM(child);

DT_INST_FOREACH_CHILD_STATUS_OKAY(INST, PHY_LRF_EXTERN)

/*
 * TX header layout comes from the FIRST PHY's PBE_GENERIC_RAM.LENCFG.
 * ALL PHYs UNDER THIS INSTANCE MUST USE THE SAME LENCFG because TX/RX
 * buffers are allocated ONCE at compile time using HDR_LEN/LEN_INDEX.
 *
 * LENCFG exists only in generic PBE exports (ti,pbe-image = "generic").
 * BLE5/IEEE PBE images hardcode length position in firmware — no LENCFG.
 * This driver uses RCL generic commands (RCL_CmdGenericTx/Rx) which require
 * generic PBE. If a PHY uses ble5/ieee PBE, LENCFG is absent and the build
 * fails here.
 *
 * Mixing PHYs with different LENCFG (e.g., BLE LENPOS=8 + IEEE LENPOS=0)
 * in one instance is NOT supported — would corrupt TX/RX parsing.
 *
 * CURRENT ASSERT ONLY CHECKS PROPERTIES EXIST, NOT THAT VALUES MATCH.
 * PHY0: LENPOS=8, NUMLENBITS=8  and  PHY1: LENPOS=0, NUMLENBITS=7
 * will BUILD OK but CORRUPT RX/TX at runtime. No cross-PHY check exists yet.
 */
#define PHY_ASSERT_GENERIC_PBE(child)                                              \
    BUILD_ASSERT(DT_NODE_HAS_PROP(PHY_MODE(child), ti_len_pos) &&               \
                 DT_NODE_HAS_PROP(PHY_MODE(child), ti_num_len_bits),            \
                 "phy-mode needs a generic PBE radio config "                 \
                 "(ti,len-pos and ti,num-len-bits missing)");

DT_INST_FOREACH_CHILD_STATUS_OKAY(INST, PHY_ASSERT_GENERIC_PBE)

#define PHY_LEN_POS(child)      DT_PROP_OR(PHY_MODE(child), ti_len_pos, 0)
#define PHY_NUM_LEN_BITS(child) DT_PROP_OR(PHY_MODE(child), ti_num_len_bits, 0)
#define BOOT_LEN_POS            GET_ARG_N(1, DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(INST, PHY_LEN_POS, (,)))
#define BOOT_NUM_LEN_BITS       GET_ARG_N(1, DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(INST, PHY_NUM_LEN_BITS, (,)))
#define LEN_INDEX               (BOOT_LEN_POS / CHAR_BIT)
#define HDR_LEN                 (LEN_INDEX + (BOOT_NUM_LEN_BITS / CHAR_BIT))

#define CHILD_PHY_INIT(child)                                                                           \
{                                                                                                       \
    .lrf_config     = &PHY_LRF_SYM(child),                                  /* SmartRF LRF_Config */    \
    .dt_id          = BASIC_RADIO_PHY_DT_ID(child),                         /* phy index for API */     \
    .frequency_hz   = DT_PROP_OR(PHY_MODE(child), boot_frequency_hz, 0),    /* boot default */         \
    .sync_word      = DT_PROP_OR(PHY_MODE(child), boot_sync_word, 0),       /* boot default */         \
    .tx_power_dbm   = DT_PROP_OR(PHY_MODE(child), boot_tx_power_dbm, 0),    /* boot default */         \
    .max_packet_len = DT_PROP_OR(PHY_MODE(child), boot_max_packet_length, 255), /* boot default */    \
    .frequency_min  = DT_PROP_OR(PHY_MODE(child), frequency_min_hz, 0),     /* min freq allowed */      \
    .frequency_max  = DT_PROP_OR(PHY_MODE(child), frequency_max_hz, 0),     /* max freq allowed */      \
    .phy_feature    = DT_PROP_OR(PHY_MODE(child), ti_phy_feature, 0),       /* phyFeatures */           \
}

static const struct phy_profile phy_table[] = {
    DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(INST, CHILD_PHY_INIT, (,))
};

BUILD_ASSERT(ARRAY_SIZE(phy_table) >= 1, "basic-radio-base node needs at least one PHY child");

/*****************************************************************************************************************/
/* RCL command templates from TI Radio Control Layer (RCL).
 * RCL_CmdGenericTx_Default()/RCL_CmdGenericRx_Default() provide zero-initialized
 * structs with default cmdId, scheduling, status, etc. Runtime fields
 * (frequency, sync word, power, buffers) are filled in by rf_params_set()
 * and send()/recv().
 */
static RCL_CmdGenericTx tx_cmd = RCL_CmdGenericTx_Default();
static RCL_CmdGenericRx rx_cmd = RCL_CmdGenericRx_Default();
static RCL_CmdGenericTxTest tone_cmd = RCL_CmdGenericTxTest_Default();

static uint32_t rx_bufs[RX_BUFFER_COUNT][RCL_MultiBuffer_len_u32(RX_BUFFER_SIZE)];
static uint32_t tx_bufs[TX_BUFFER_COUNT][RCL_TxBuffer_len_u32(NUM_PAD_BYTES, HDR_LEN, MAX_PACKET_LEN)];

static const struct device *basic_radio_cc23x0_dev;

/* 64-bit extension of SYSTIM TIME1U: wrap count and last read value */
static uint32_t time_hi;
static uint32_t time_last;
static struct k_timer time_refresh_timer;

/* Forward declarations */
static void rf_params_set(struct basic_radio_cc23x0_data *data, uint32_t frequency_hz, uint32_t syncWord,
                          int8_t txPowerDbm, uint8_t maxPacketLen);
static int start_time_set(RCL_Command *cmd, bool isScheduled, uint64_t startUs);
static uint64_t rx_timestamp_get(void);
static uint64_t basic_radio_cc23x0_time_get(const struct device *dev);
static void rx_callback(RCL_Command *cmd, LRF_Events lrfEvt, RCL_Events rclEvt);
static void tx_callback(RCL_Command *cmd, LRF_Events lrfEvt, RCL_Events rclEvt);
static void tone_callback(RCL_Command *cmd, LRF_Events lrfEvt, RCL_Events rclEvt);
static void time_refresh(struct k_timer *timer);

/* Additional forward declarations */
static void rx_entries_report(struct basic_radio_cc23x0_data *data);
static void op_done(struct basic_radio_cc23x0_data *data, enum basic_radio_evt_type type, RCL_CommandStatus status);
static int status_to_errno(RCL_CommandStatus status);

/*
 * Carrier frequency inside the characterized range of a PHY
 */
static bool frequency_is_valid(const struct phy_profile *phy, uint32_t frequency_hz)
{
    return ((frequency_hz >= phy->frequency_min) && (frequency_hz <= phy->frequency_max));
}

/*
 * Select a PHY. Uses precomputed header and TX power range from the PHY object.
 * Returns 0 on success, -EIO if RCL reopen fails.
 */
static int phy_select(struct basic_radio_cc23x0_data *data, const struct basic_radio_cc23x0_config *cfg, uint8_t idx)
{
    const struct phy_profile *phy = &cfg->phy_table[idx];
    const struct phy_profile *current = &cfg->phy_table[data->active_phy_idx];

    if ((data->handle == NULL) || (phy->lrf_config != current->lrf_config))
    {
        if (data->handle != NULL)
        {
            RCL_close(data->handle);
            data->handle = NULL;
        }

        data->handle = RCL_open(&data->rcl_client, phy->lrf_config);

        if (data->handle == NULL)
        {
            LOG_ERR("RCL_open failed for PHY %u", idx);
            data->active_phy_idx = 0xFF;

            return (-EIO);
        }
    }

    tx_cmd.common.phyFeatures   = phy->phy_feature;
    rx_cmd.common.phyFeatures   = phy->phy_feature;
    tone_cmd.common.phyFeatures = phy->phy_feature;
    data->active_phy_idx        = idx;
    data->hdr.lenIndex          = LEN_INDEX;
    data->hdr.len               = HDR_LEN;
    data->hdr.lenMask           = (1 << (BOOT_NUM_LEN_BITS % CHAR_BIT)) - 1;

    /* TX power range: unknown for custom PHYs unless added to DT */
    data->isTxPowerKnown = false;
    data->tx_power_min_dbm = -128;
    data->tx_power_max_dbm = 127;

    return (0);
}

/* Index of the phy_table entry built from DT node id, or -1 */
static int phy_find(const struct basic_radio_cc23x0_config *cfg, uint16_t dtId)
{
    for (uint8_t i = 0; i < cfg->phy_count; i++)
    {
        if (cfg->phy_table[i].dt_id == dtId)
        {
            return (i);
        }
    }

    return (-1);
}

/* TX power inside the TX power table of the active PHY, if it has one */
static bool tx_power_is_valid(const struct basic_radio_cc23x0_data *data, int8_t txPowerDbm)
{
    if (!data->isTxPowerKnown) {
        return true;
    }
    return ((txPowerDbm >= data->tx_power_min_dbm) && (txPowerDbm <= data->tx_power_max_dbm));
}

/*****************************************************************************************************************/
/* Driver API: Time Get */
static uint64_t basic_radio_cc23x0_time_get(const struct device *dev)
{
    ARG_UNUSED(dev);

    /* Read and extend in one step, so concurrent callers see one wrap */
    unsigned int key = irq_lock();
    uint32_t now = HWREG(SYSTIM_BASE + SYSTIM_O_TIME1U);

    if (now < time_last)
    {
        time_hi++;
    }

    time_last = now;

    uint64_t time = ((uint64_t)time_hi << 32) | now;

    irq_unlock(key);

    return (time);
}

/* Read the time often enough to catch every wrap of TIME1U */
static void time_refresh(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    (void)basic_radio_cc23x0_time_get(basic_radio_cc23x0_dev);
}

/*
 * Program the start of a command. An absolute start must be in the future
 * and within the RCL compare window.
 */
static int start_time_set(RCL_Command *cmd, bool isScheduled, uint64_t startUs)
{
    if (!isScheduled)
    {
        cmd->scheduling = RCL_Schedule_Now;

        return (0);
    }

    uint64_t now = basic_radio_cc23x0_time_get(basic_radio_cc23x0_dev);

    if (startUs <= now)
    {
        return (-ETIME);
    }

    /* RCL compares start times as signed 32-bit differences of 250 ns steps,
     * a window of +-536 s. Keep a margin below it.
     */
    #define SCHED_HORIZON_US 500000000U
    if ((startUs - now) > SCHED_HORIZON_US)
    {
        return (-EINVAL);
    }

    /* Low 32 bits of the 250 ns SYSTIM count */
    #define SYSTIM_TICKS_PER_US 4U
    cmd->scheduling = RCL_Schedule_AbsTime;
    cmd->timing.absStartTime = (uint32_t)(startUs * SYSTIM_TICKS_PER_US);

    return (0);
}

/* RX timestamp on the 64-bit time base, 0 if the radio has none */
static uint64_t rx_timestamp_get(void)
{
    if (!rx_cmd.stats->timestampValid)
    {
        return (0);
    }

    /* The packet is in the past: age in 250 ns steps, modulo 2^32 */
    #define SYSTIM_TICKS_PER_US 4U
    uint64_t now = basic_radio_cc23x0_time_get(basic_radio_cc23x0_dev);
    uint32_t age = (uint32_t)(now * SYSTIM_TICKS_PER_US) - rx_cmd.stats->lastTimestamp;

    return (now - (age / SYSTIM_TICKS_PER_US));
}

/*
 * Write the runtime parameters into the driver cache and both RCL commands.
 * Shared by init and basic_radio_config().
 */
static void rf_params_set(struct basic_radio_cc23x0_data *data, uint32_t frequency_hz, uint32_t syncWord,
                          int8_t txPowerDbm, uint8_t maxPacketLen)
{
    #define BASIC_RADIO_FREQ_UNIT_HZ 10U
    data->cfg_frequency_hz   = frequency_hz;
    data->cfg_sync_word      = syncWord;
    data->cfg_tx_power_dbm   = txPowerDbm;
    data->cfg_max_packet_len = MIN(maxPacketLen, MAX_PACKET_LEN);

    tx_cmd.rfFrequency      = frequency_hz * BASIC_RADIO_FREQ_UNIT_HZ;
    rx_cmd.rfFrequency      = frequency_hz * BASIC_RADIO_FREQ_UNIT_HZ;

    tx_cmd.syncWord         = syncWord;
    rx_cmd.syncWordA        = syncWord;

    tx_cmd.txPower.dBm      = txPowerDbm;
    tx_cmd.txPower.fraction = 0;

    rx_cmd.maxPktLen        = data->cfg_max_packet_len;
}

/* RCL Rx Callback */
static void rx_callback(RCL_Command *cmd, LRF_Events lrfEvt, RCL_Events rclEvt)
{
    struct basic_radio_cc23x0_data *data = basic_radio_cc23x0_dev->data;

    if (lrfEvt.rxOk)
    {
        data->rx_ok_count++;
    }

    if (lrfEvt.rxNok)
    {
        data->rx_crc_err_count++;
    }

    if (rclEvt.cmdStarted)
    {
        data->cmd_started_count++;
    }

    if (rclEvt.startRejected || rclEvt.startDelayed)
    {
        data->sched_reject_count++;
    }

    /* Discarded packets are not stored, report them from the LRF event */
    if (rx_cmd.config.discardRxPackets && lrfEvt.rxOk)
    {
        struct basic_radio_evt evt = {
            .type = BASIC_RADIO_EVT_RX_PKT,
            .info = {.timestamp_us = rx_timestamp_get(), .rssi = rx_cmd.stats->lastRssi, .crc_ok = true},
        };

        data->cb(basic_radio_cc23x0_dev, &evt, data->user_data);
    }

    /* Packets stored before the end are reported before RX_DONE */
    if (rclEvt.rxEntryAvail || rclEvt.lastCmdDone)
    {
        rx_entries_report(data);
    }

    if (rclEvt.lastCmdDone)
    {
        op_done(data, BASIC_RADIO_EVT_RX_DONE, cmd->status);
    }
}

/* RCL Tx Callback */
static void tx_callback(RCL_Command *cmd, LRF_Events lrfEvt, RCL_Events rclEvt)
{
    ARG_UNUSED(lrfEvt);
    struct basic_radio_cc23x0_data *data = basic_radio_cc23x0_dev->data;

    if (rclEvt.cmdStarted)
    {
        data->cmd_started_count++;
    }

    if (rclEvt.startRejected || rclEvt.startDelayed)
    {
        data->sched_reject_count++;
    }

    if (!rclEvt.lastCmdDone)
    {
        return;
    }

    if (cmd->status >= RCL_CommandStatus_Error)
    {
        LOG_ERR("TX error: status=%d", cmd->status);
    }

    op_done(data, BASIC_RADIO_EVT_TX_DONE, cmd->status);
}

/* RCL Tone Callback */
static void tone_callback(RCL_Command *cmd, LRF_Events lrfEvt, RCL_Events rclEvt)
{
    ARG_UNUSED(lrfEvt);
    struct basic_radio_cc23x0_data *data = basic_radio_cc23x0_dev->data;

    if (rclEvt.lastCmdDone)
    {
        op_done(data, BASIC_RADIO_EVT_TONE_DONE, cmd->status);
    }
}

/* Report every stored packet, then hand the drained buffers back to RX */
static void rx_entries_report(struct basic_radio_cc23x0_data *data)
{
    List_List consumed = {0};
    RCL_Buffer_DataEntry *entry;
    RCL_MultiBuffer *mb;

    while ((entry = RCL_MultiBuffer_RxEntry_get(&rx_cmd.rxBuffers, &consumed)) != NULL)
    {
        uint8_t *pkt = &entry->pad0 + entry->numPad;
        struct basic_radio_evt evt = {
            .type = BASIC_RADIO_EVT_RX_PKT,
            .data = pkt + data->hdr.len,
            .len  = (uint16_t)(pkt[data->hdr.lenIndex] & data->hdr.lenMask),
            .info = {.timestamp_us = rx_timestamp_get(), .rssi = rx_cmd.stats->lastRssi, .crc_ok = true},
        };

        data->cb(basic_radio_cc23x0_dev, &evt, data->user_data);
    }

    while ((mb = RCL_MultiBuffer_get(&consumed)) != NULL)
    {
        RCL_MultiBuffer_clear(mb);
        RCL_MultiBuffer_put(&rx_cmd.rxBuffers, mb);
    }
}

/*
 * Report the final event of the active operation. The radio is idle before
 * the callback runs, so the callback can start the next operation.
 */
static void op_done(struct basic_radio_cc23x0_data *data, enum basic_radio_evt_type type, RCL_CommandStatus status)
{
    basic_radio_callback_t cb = data->cb;
    void *userData = data->user_data;
    struct basic_radio_evt evt = {
        .type   = type,
        .status = status_to_errno(status),
    };

    atomic_set(&data->state, RADIO_STATE_IDLE);
    cb(basic_radio_cc23x0_dev, &evt, userData);
}

/* Final status of an RCL command as reported in basic_radio_evt::status */
static int status_to_errno(RCL_CommandStatus status)
{
    /* A timeout stop is the programmed end of a tone */
    if ((status == RCL_CommandStatus_Finished) || RCL_CommandStatus_isAnyTimeoutStop(status))
    {
        return (0);
    }

    if (RCL_CommandStatus_isAnyApiStop(status))
    {
        return (-ECANCELED);
    }

    if ((status == RCL_CommandStatus_RejectedStart) || (status == RCL_CommandStatus_Error_StartTooLate))
    {
        return (-ETIME);
    }

    return (-EIO);
}

/*****************************************************************************************************************/
/* Driver API: Configuration */
static int basic_radio_cc23x0_config_impl(const struct device *dev, const struct basic_radio_config *cfg)
{
    struct basic_radio_cc23x0_data *dd = dev->data;
    const struct basic_radio_cc23x0_config *drvCfg = dev->config;
    int idx = dd->active_phy_idx;
    int ret = 0;

    if (cfg->phy != BASIC_RADIO_PHY_CURRENT)
    {
        idx = phy_find(drvCfg, cfg->phy);

        if (idx < 0)
        {
            return (-EINVAL);
        }
    }

    const struct phy_profile *phy = &drvCfg->phy_table[idx];

    /* Merge provided config with current values (0 = keep current) */
    uint32_t frequency = cfg->frequency ? cfg->frequency : dd->cfg_frequency_hz;
    uint32_t sync_word = cfg->sync_word ? cfg->sync_word : dd->cfg_sync_word;
    int8_t   tx_power  = cfg->tx_power  ? cfg->tx_power  : dd->cfg_tx_power_dbm;
    uint8_t  max_len   = cfg->max_packet_len ? cfg->max_packet_len : dd->cfg_max_packet_len;

    /* Evaluate the frequency */
    if (!frequency_is_valid(phy, frequency))
    {
        LOG_ERR("frequency %u outside the PHY's characterized range", frequency);

        return (-EINVAL);
    }

    /* Own the radio so no send()/recv() runs while RCL is reconfigured */
    if (!atomic_cas(&dd->state, RADIO_STATE_IDLE, RADIO_STATE_TX_ACTIVE))
    {
        return (-EBUSY);
    }

    /* If a new phy node is detected, then change the RCL settings */
    if (idx != dd->active_phy_idx)
    {
        ret = phy_select(dd, drvCfg, (uint8_t)idx);
    }

    /* After changing the PHY, modify the runtime parameters */
    if (ret == 0)
    {
        rf_params_set(dd, frequency, sync_word, tx_power, max_len);
    }

    atomic_set(&dd->state, RADIO_STATE_IDLE);

    return (ret);
}

/*****************************************************************************************************************/
/* Driver API: Get Caps */
static int basic_radio_cc23x0_get_caps(const struct device *dev, struct basic_radio_caps *caps)
{
    const struct basic_radio_cc23x0_data *data = dev->data;
    const struct basic_radio_cc23x0_config *cfg = dev->config;

    if (data->active_phy_idx >= cfg->phy_count)
    {
        return (-ENODEV);
    }

    const struct phy_profile *phy = &cfg->phy_table[data->active_phy_idx];

    #define SYNTH_STEP_MILLIHZ ((48000000ULL * 1000ULL) >> 18)
    #define RETUNE_TIME_US 116U
    #define SCHED_HORIZON_US 500000000U

    *caps = (struct basic_radio_caps){
        .flags                   = BASIC_RADIO_CAP_SCHED | BASIC_RADIO_CAP_TONE | BASIC_RADIO_CAP_STOP_GRACEFUL,
        .freq_min                = phy->frequency_min,
        .freq_max                = phy->frequency_max,
        .freq_resolution_millihz = (uint32_t)SYNTH_STEP_MILLIHZ,
        .retune_time_us          = RETUNE_TIME_US,
        .sched_horizon_us        = SCHED_HORIZON_US,
        .tx_power_min_dbm        = data->tx_power_min_dbm,
        .tx_power_max_dbm        = data->tx_power_max_dbm,
    };

    return (0);
}

/*****************************************************************************************************************/
/* Driver API: Send */
static int basic_radio_cc23x0_send_async(const struct device *dev, const uint8_t *data, uint16_t len,
                                         const struct basic_radio_tx_params *params, basic_radio_callback_t cb,
                                         void *user_data)
{
    struct basic_radio_cc23x0_data *dd = dev->data;

    LOG_INF("TI-cc23xx");

    if (len == 0 || len > MAX_PACKET_LEN || cb == NULL)
    {
        return (-EINVAL);
    }

    static const uint32_t supported_tx_flags = BASIC_RADIO_TX_FLAG_SCHED_ABS | BASIC_RADIO_TX_FLAG_CHANNEL | BASIC_RADIO_TX_FLAG_SYNC_WORD | BASIC_RADIO_TX_FLAG_POWER;

    if (params && (params->flags & ~supported_tx_flags))
    {
        return (-ENOTSUP);
    }

    const struct basic_radio_cc23x0_config *cfg = dev->config;
    if (dd->active_phy_idx >= cfg->phy_count)
    {
        return (-ENODEV);
    }

    /* Length must fit the length field of the active PHY */
    if (len > dd->hdr.lenMask)
    {
        return (-EINVAL);
    }

    uint32_t frequency = (params && (params->flags & BASIC_RADIO_TX_FLAG_CHANNEL)) ? params->frequency_hz : dd->cfg_frequency_hz;
    uint32_t syncWord = (params && (params->flags & BASIC_RADIO_TX_FLAG_SYNC_WORD)) ? params->sync_word : dd->cfg_sync_word;
    int8_t txPowerDbm = (params && (params->flags & BASIC_RADIO_TX_FLAG_POWER)) ? params->tx_power_dbm : dd->cfg_tx_power_dbm;

    const struct phy_profile *phy = &cfg->phy_table[dd->active_phy_idx];

    if (!frequency_is_valid(phy, frequency) || !tx_power_is_valid(dd, txPowerDbm))
    {
        return (-EINVAL);
    }

    if (!atomic_cas(&dd->state, RADIO_STATE_IDLE, RADIO_STATE_TX_ACTIVE))
    {
        return (-EBUSY);
    }

    dd->cb = cb;
    dd->user_data = user_data;

    uint8_t slot = dd->tx_pool_idx;
    dd->tx_pool_idx = (slot + 1) % TX_BUFFER_COUNT;
    RCL_Buffer_TxBuffer *txBuf = (RCL_Buffer_TxBuffer *)tx_bufs[slot];
    uint8_t *txData = RCL_TxBuffer_init(txBuf, NUM_PAD_BYTES, dd->hdr.len, len);
    memset(txData, 0, dd->hdr.lenIndex);
    txData[dd->hdr.lenIndex] = (uint8_t)len;
    memcpy(&txData[dd->hdr.len], data, len);
    tx_cmd.common.status = RCL_CommandStatus_Idle;
    memset(&tx_cmd.txBuffers, 0, sizeof(tx_cmd.txBuffers));
    RCL_TxBuffer_put(&tx_cmd.txBuffers, txBuf);

    tx_cmd.rfFrequency = frequency * BASIC_RADIO_FREQ_UNIT_HZ;
    tx_cmd.syncWord = syncWord;
    tx_cmd.txPower.dBm = txPowerDbm;

    int ret = start_time_set(&tx_cmd.common, (params && (params->flags & BASIC_RADIO_TX_FLAG_SCHED_ABS)),
                             (params != NULL) ? params->abs_start_time_us : 0U);

    if (ret != 0)
    {
        atomic_set(&dd->state, RADIO_STATE_IDLE);

        return (ret);
    }

    /* A rejected submit reports no event. Otherwise tx_callback ends the operation */
    RCL_CommandStatus st = RCL_Command_submit(dd->handle, &tx_cmd.common);

    if (st >= RCL_CommandStatus_Error)
    {
        atomic_set(&dd->state, RADIO_STATE_IDLE);

        return (-EIO);
    }

    return (0);
}

/*****************************************************************************************************************/
/* Driver API: Receive */
static int basic_radio_cc23x0_recv_async(const struct device *dev, const struct basic_radio_rx_params *params,
                                         basic_radio_callback_t cb, void *user_data)
{
    struct basic_radio_cc23x0_data *data = dev->data;

    LOG_INF("TI-cc23xx");

    static const uint32_t supported_rx_flags = BASIC_RADIO_RX_FLAG_SCHED_ABS | BASIC_RADIO_RX_FLAG_ONE_SHOT | BASIC_RADIO_RX_FLAG_DISCARD_PAYL | BASIC_RADIO_RX_FLAG_CHANNEL | BASIC_RADIO_RX_FLAG_SYNC_WORD | BASIC_RADIO_RX_FLAG_MAX_LEN;

    if (cb == NULL)
    {
        return (-EINVAL);
    }

    if (params && (params->flags & ~supported_rx_flags))
    {
        return (-ENOTSUP);
    }

    const struct basic_radio_cc23x0_config *cfg = dev->config;
    if (data->active_phy_idx >= cfg->phy_count)
    {
        return (-ENODEV);
    }

    uint32_t frequency = (params && (params->flags & BASIC_RADIO_RX_FLAG_CHANNEL)) ? params->frequency_hz : data->cfg_frequency_hz;
    uint32_t syncWord = (params && (params->flags & BASIC_RADIO_RX_FLAG_SYNC_WORD)) ? params->sync_word : data->cfg_sync_word;

    const struct phy_profile *phy = &cfg->phy_table[data->active_phy_idx];

    if (!frequency_is_valid(phy, frequency))
    {
        return (-EINVAL);
    }

    /* Own the radio so no send()/config() touches RCL while RX is pending */
    if (!atomic_cas(&data->state, RADIO_STATE_IDLE, RADIO_STATE_RX_ACTIVE))
    {
        return (-EBUSY);
    }

    data->cb = cb;
    data->user_data = user_data;

    rx_cmd.config.repeated = (params && (params->flags & BASIC_RADIO_RX_FLAG_ONE_SHOT)) ? 0 : 1;
    rx_cmd.config.discardRxPackets = (params && (params->flags & BASIC_RADIO_RX_FLAG_DISCARD_PAYL)) ? 1 : 0;
    rx_cmd.rfFrequency = frequency * BASIC_RADIO_FREQ_UNIT_HZ;
    rx_cmd.syncWordA = syncWord;
    rx_cmd.maxPktLen = (params && (params->flags & BASIC_RADIO_RX_FLAG_MAX_LEN)) ? params->max_packet_len : data->cfg_max_packet_len;

    int ret = start_time_set(&rx_cmd.common, (params && (params->flags & BASIC_RADIO_RX_FLAG_SCHED_ABS)),
                             (params != NULL) ? params->abs_start_time_us : 0U);

    if (ret != 0)
    {
        atomic_set(&data->state, RADIO_STATE_IDLE);

        return (ret);
    }

    /* Rebuild the list: the last RX may have ended with buffers still in it */
    List_clearList(&rx_cmd.rxBuffers);

    for (uint8_t i = 0; i < RX_BUFFER_COUNT; i++)
    {
        RCL_MultiBuffer *mb = (RCL_MultiBuffer *)rx_bufs[i];
        RCL_MultiBuffer_clear(mb);
        RCL_MultiBuffer_put(&rx_cmd.rxBuffers, mb);
    }

    rx_cmd.common.status = RCL_CommandStatus_Idle;

    /* A rejected submit reports no event. Otherwise rx_callback ends the operation */
    RCL_CommandStatus st = RCL_Command_submit(data->handle, &rx_cmd.common);

    if (st >= RCL_CommandStatus_Error)
    {
        atomic_set(&data->state, RADIO_STATE_IDLE);

        return (-EIO);
    }

    return (0);
}

/*****************************************************************************************************************/
/* Driver API: Stop */
static int basic_radio_cc23x0_stop(const struct device *dev, enum basic_radio_stop_mode mode)
{
    struct basic_radio_cc23x0_data *data = dev->data;
    RCL_Command *cmd;
    RCL_StopType stopType;

    switch (mode)
    {
        case BASIC_RADIO_STOP_DESCHEDULE:
            stopType = RCL_StopType_DescheduleOnly;
            break;

        case BASIC_RADIO_STOP_GRACEFUL:
            stopType = RCL_StopType_Graceful;
            break;

        case BASIC_RADIO_STOP_ABORT:
            stopType = RCL_StopType_Hard;
            break;

        default:
            return (-EINVAL);
    }

    /* The operation ends in the RCL interrupt; keep it from ending between state and stop */
    unsigned int key = irq_lock();

    switch (atomic_get(&data->state))
    {
        case RADIO_STATE_TX_ACTIVE:
            cmd = &tx_cmd.common;
            break;

        case RADIO_STATE_RX_ACTIVE:
            cmd = &rx_cmd.common;
            break;

        case RADIO_STATE_TONE_ACTIVE:
            cmd = &tone_cmd.common;
            break;

        default:
            cmd = NULL;
            break;
    }

    /* The final event follows once the stop takes effect */
    RCL_CommandStatus st = (cmd != NULL) ? RCL_Command_stop(cmd, stopType) : RCL_CommandStatus_Idle;

    irq_unlock(key);

    if ((st < RCL_CommandStatus_Scheduled) || (st >= RCL_CommandStatus_Finished))
    {
        return (-EALREADY);
    }

    if ((stopType == RCL_StopType_DescheduleOnly) && (st != RCL_CommandStatus_Scheduled))
    {
        return (-EBUSY);
    }

    return (0);
}

/*****************************************************************************************************************/
/* Driver API: Tone */
static int basic_radio_cc23x0_tone_async(const struct device *dev, const struct basic_radio_tone_params *params,
                                         basic_radio_callback_t cb, void *user_data)
{
    struct basic_radio_cc23x0_data *data = dev->data;

    if ((params == NULL) || (cb == NULL))
    {
        return (-EINVAL);
    }

    if ((params->flags & ~BASIC_RADIO_TONE_FLAG_SCHED_ABS) != 0U)
    {
        return (-ENOTSUP);
    }

    /* RCL needs an open PHY to drive the synthesizer */
    const struct basic_radio_cc23x0_config *cfg = dev->config;
    if (data->active_phy_idx >= cfg->phy_count)
    {
        return (-ENODEV);
    }

    const struct phy_profile *phy = &cfg->phy_table[data->active_phy_idx];

    if (!frequency_is_valid(phy, params->frequency_hz) || !tx_power_is_valid(data, params->tx_power_dbm))
    {
        return (-EINVAL);
    }

    if (!atomic_cas(&data->state, RADIO_STATE_IDLE, RADIO_STATE_TONE_ACTIVE))
    {
        return (-EBUSY);
    }

    data->cb = cb;
    data->user_data = user_data;

    tone_cmd.rfFrequency = params->frequency_hz * BASIC_RADIO_FREQ_UNIT_HZ;
    tone_cmd.txPower.dBm = params->tx_power_dbm;
    tone_cmd.txPower.fraction = 0;
    tone_cmd.config.sendCw = 1;
    /* The hard stop ends the tone; 0 runs it until basic_radio_stop() */
    #define SYSTIM_TICKS_PER_US 4U
    tone_cmd.common.timing.relHardStopTime = params->duration_us * SYSTIM_TICKS_PER_US;

    int ret = start_time_set(&tone_cmd.common, ((params->flags & BASIC_RADIO_TONE_FLAG_SCHED_ABS) != 0U),
                             params->abs_start_time_us);

    if (ret != 0)
    {
        atomic_set(&data->state, RADIO_STATE_IDLE);

        return (ret);
    }

    tone_cmd.common.status = RCL_CommandStatus_Idle;

    /* A rejected submit reports no event. Otherwise tone_callback ends the operation */
    RCL_CommandStatus st = RCL_Command_submit(data->handle, &tone_cmd.common);

    if (st >= RCL_CommandStatus_Error)
    {
        atomic_set(&data->state, RADIO_STATE_IDLE);

        return (-EIO);
    }

    return (0);
}

/*****************************************************************************************************************/
/* Driver API: External Function */
static int basic_radio_cc23x0_ext_ctl(const struct device *dev, uint32_t cmd, void *arg, size_t arg_sz)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cmd);
    ARG_UNUSED(arg);
    ARG_UNUSED(arg_sz);

    return (-ENOTSUP);
}

/*****************************************************************************************************************/
/* Driver API: Init */
static int basic_radio_cc23x0_init(const struct device *dev)
{
    struct basic_radio_cc23x0_data *data = dev->data;
    const struct basic_radio_cc23x0_config *cfg = dev->config;
    const struct phy_profile *boot = &cfg->phy_table[0];

    atomic_set(&data->state, RADIO_STATE_IDLE);

    data->active_phy_idx = 0xFF;
    data->handle = NULL;

    RCL_init();

    int ret = phy_select(data, cfg, 0);

    if (ret != 0)
    {
        return (ret);
    }

    rf_params_set(data, boot->frequency_hz, boot->sync_word, boot->tx_power_dbm, boot->max_packet_len);

    tone_cmd.common.runtime.callback = tone_callback;
    tone_cmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value;

    tx_cmd.common.runtime.callback = tx_callback;
    tx_cmd.common.runtime.rclCallbackMask.value = RCL_EventLastCmdDone.value | RCL_EventCmdStarted.value |
                                                  RCL_EventStartRejected.value | RCL_EventStartDelayed.value;

    rx_cmd.common.runtime.callback = rx_callback;
    rx_cmd.common.runtime.rclCallbackMask.value = RCL_EventRxEntryAvail.value | RCL_EventLastCmdDone.value |
                                                  RCL_EventCmdStarted.value | RCL_EventStartRejected.value |
                                                  RCL_EventStartDelayed.value;

    rx_cmd.common.runtime.lrfCallbackMask.value = LRF_EventRxOk.value | LRF_EventRxNok.value;

    data->stats = RCL_StatsGeneric_DefaultRuntime();
    rx_cmd.stats = &data->stats;
    rx_cmd.stats->config.activeUpdate = 1;

    for (uint8_t i = 0; i < RX_BUFFER_COUNT; i++)
    {
        RCL_MultiBuffer *mb = (RCL_MultiBuffer *)rx_bufs[i];
        RCL_MultiBuffer_init(mb, RCL_MultiBuffer_len(RX_BUFFER_SIZE));
    }

    basic_radio_cc23x0_dev = dev;

    #define TIME_REFRESH_PERIOD K_MINUTES(30)
    k_timer_init(&time_refresh_timer, time_refresh, NULL);
    k_timer_start(&time_refresh_timer, TIME_REFRESH_PERIOD, TIME_REFRESH_PERIOD);

    return (0);
}

/*****************************************************************************************************************/
static const struct basic_radio_driver_api basic_radio_cc23x0_api = {
    /* Main Functions */
    .config     = basic_radio_cc23x0_config_impl,
    .send_async = basic_radio_cc23x0_send_async,
    .recv_async = basic_radio_cc23x0_recv_async,
    .tone_async = basic_radio_cc23x0_tone_async,
    .stop       = basic_radio_cc23x0_stop,
    .time_get   = basic_radio_cc23x0_time_get,
    /* Optional Functions */
    .get_caps = basic_radio_cc23x0_get_caps,
	/* Vendor-Specific */
    .ext_ctl  = basic_radio_cc23x0_ext_ctl,
};

static const struct basic_radio_cc23x0_config basic_radio_cc23x0_cfg_0 = {
    .phy_table = phy_table,
    .phy_count = ARRAY_SIZE(phy_table),
};

static struct basic_radio_cc23x0_data basic_radio_cc23x0_data_0;

DEVICE_DT_INST_DEFINE(INST,
                      basic_radio_cc23x0_init,
                      NULL,
                      &basic_radio_cc23x0_data_0,
                      &basic_radio_cc23x0_cfg_0,
                      POST_KERNEL,
                      CONFIG_BASIC_RADIO_INIT_PRIORITY,
                      &basic_radio_cc23x0_api);