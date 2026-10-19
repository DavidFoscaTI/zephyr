/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public Basic Radio driver API
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_BASIC_RADIO_H_
#define ZEPHYR_INCLUDE_DRIVERS_BASIC_RADIO_H_

/**
 * @brief Basic Radio APIs
 * @defgroup basic_radio_api Basic Radio APIs
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Modulation schemes supported by Basic Radio backends. */
enum basic_radio_modulation {
	BASIC_RADIO_MOD_2GFSK = 0,
	BASIC_RADIO_MOD_4GFSK,
	BASIC_RADIO_MOD_2FSK,
	BASIC_RADIO_MOD_OOK,
	BASIC_RADIO_MOD_MSK,
	BASIC_RADIO_MOD_OQPSK,
	BASIC_RADIO_MOD_OFDM,
};

/** @brief Optional radio capabilities. Bitmask returned by basic_radio_get_caps(). */
enum basic_radio_capability {
	BASIC_RADIO_CAP_SCHED       = BIT(0), /**< Scheduled operations (absolute start time) */
	BASIC_RADIO_CAP_TONE        = BIT(1), /**< Continuous wave (tone) transmission */
	BASIC_RADIO_CAP_STOP_GRACEFUL = BIT(2), /**< Graceful stop (finish current frame) */
	/* others to be added */
};

/** @brief Stop mode for basic_radio_stop(). */
enum basic_radio_stop_mode {
	BASIC_RADIO_STOP_DESCHEDULE = 0, /**< Cancel scheduled start only, no effect if running */
	BASIC_RADIO_STOP_GRACEFUL     = 1, /**< Finish current frame then stop */
	BASIC_RADIO_STOP_ABORT        = 2, /**< Stop immediately */
};

/** @brief Event types reported via callback. */
enum basic_radio_evt_type {
	BASIC_RADIO_EVT_TX_DONE    = 0,
	BASIC_RADIO_EVT_RX_PKT     = 1,
	BASIC_RADIO_EVT_RX_DONE    = 2,
	BASIC_RADIO_EVT_TONE_DONE  = 3,
};

/** @brief basic_radio_config::phy value that keeps the active PHY. */
#define BASIC_RADIO_PHY_CURRENT 0U

/**
 * @brief Identifier of a PHY child node, for basic_radio_config::phy.
 *
 * Uses the node's dependency ordinal, which is unique and never 0 for a
 * child node (0 is the root), so it cannot collide with BASIC_RADIO_PHY_CURRENT.
 */
#define BASIC_RADIO_PHY_DT_ID(phy_node) ((uint16_t)DT_DEP_ORD(phy_node))

/**
 * @brief Runtime radio configuration (Layer 1 parameters only).
 *
 * phy selects the PHY child node whose waveform is used. The backend
 * switches to the waveform of that node's phy-mode, reconfiguring the radio
 * if needed. BASIC_RADIO_PHY_CURRENT keeps the active PHY.
 *
 * frequency, sync_word, max_packet_len and tx_power are the common runtime
 * parameters (basic-radio-base.yaml) and every backend applies them.
 *
 * @note **Contract:** Any field set to 0 (or BASIC_RADIO_PHY_CURRENT for phy)
 *       keeps the current value (from boot defaults or previous config()).
 *       Set a field to non-zero to change it. All 4 fields must be non-zero
 *       for the first config() call if boot defaults are not available.
 *
 * Waveform properties (datarate, modulation, etc.) are fixed per PHY mode
 * and come from the devicetree (basic-radio-phy-mode.yaml). They are not
 * part of runtime configuration.
 */
struct basic_radio_config {
	uint16_t phy;                     /**< BASIC_RADIO_PHY_DT_ID() or BASIC_RADIO_PHY_CURRENT. */
	uint32_t frequency;               /**< Carrier frequency in Hz (0 = keep current). */
	uint32_t sync_word;               /**< Sync pattern, LSB-aligned (0 = keep current). */
	uint8_t  max_packet_len;          /**< Maximum payload length in bytes (0 = keep current). */
	int8_t   tx_power;                /**< TX power in dBm (0 = keep current). */
};

/**
 * @brief Initializer for struct basic_radio_config from a DT PHY child node.
 *
 * @p phy_node is a PHY child of a basic-radio-base node. The PHY is
 * selected via its phy-mode property. Runtime parameters (frequency,
 * sync_word, max_packet_len, tx_power) use boot defaults from the PHY's
 * DTSI node, or can be set via basic_radio_config() at runtime.
 *
 * Typical usage:
 * @code
 *   static const struct basic_radio_config cfg =
 *       BASIC_RADIO_CONFIG_DT(DT_NODELABEL(phy0));
 * @endcode
 */
#define BASIC_RADIO_CONFIG_DT(phy_node) &(struct basic_radio_config){ \
	.phy = BASIC_RADIO_PHY_DT_ID(phy_node), \
}

/** @brief Per-packet RX metadata. */
struct basic_radio_rx_info {
	int8_t   rssi;                    /**< Received signal strength in dBm. */
	uint64_t timestamp_us;            /**< Timestamp in microseconds (radio time base); 0 if unsupported. */
	bool     crc_ok;
};

/** @brief Event reported via async callback. */
struct basic_radio_evt {
	enum basic_radio_evt_type type;  /**< Event type. */
	int status;                      /**< 0 on success, negative errno on error. */
	union {
		struct {
			const uint8_t *data;     /**< Received payload (valid for RX_PKT). */
			uint16_t len;            /**< Payload length in bytes. */
			struct basic_radio_rx_info info; /**< RX metadata. */
		};
	};
};

/** @brief Async callback signature. */
typedef void (*basic_radio_callback_t)(const struct device *dev, const struct basic_radio_evt *evt, void *user_data);

/* --- Layer 2: per-operation adverb flags ---------------------------------- */

/** @brief TX adverbs passed to basic_radio_send_async().
 *
 *  Pass NULL for params (or set flags = 0) to use Layer 1 defaults from the
 *  last basic_radio_config() call.  Each BIT opts into exactly one override;
 *  unknown bits cause basic_radio_send_async() to return -ENOTSUP.
 */
#define BASIC_RADIO_TX_FLAG_SCHED_ABS  BIT(0) /**< Schedule TX at abs_start_time_us.  Default: immediate. */
#define BASIC_RADIO_TX_FLAG_CHANNEL    BIT(1) /**< Use frequency_hz instead of Layer 1 default. */
#define BASIC_RADIO_TX_FLAG_POWER      BIT(2) /**< Use tx_power_dbm instead of Layer 1 default. */
#define BASIC_RADIO_TX_FLAG_SYNC_WORD  BIT(3) /**< Use sync_word instead of Layer 1 default. */

struct basic_radio_tx_params {
	uint32_t flags;
	/** Absolute start time in microseconds (radio time base). Used only when BASIC_RADIO_TX_FLAG_SCHED_ABS is set. */
	uint64_t abs_start_time_us;
	/** Carrier frequency in Hz. Used only when BASIC_RADIO_TX_FLAG_CHANNEL is set. */
	uint32_t frequency_hz;
	/** TX power in dBm. Used only when BASIC_RADIO_TX_FLAG_POWER is set. */
	int8_t   tx_power_dbm;
	/** Sync word. Used only when BASIC_RADIO_TX_FLAG_SYNC_WORD is set. */
	uint32_t sync_word;
};

/** @brief RX adverbs passed to basic_radio_recv_async().
 *
 *  Pass NULL for params (or set flags = 0) to use Layer 1 defaults from the
 *  last basic_radio_config() call.  Unknown bits cause basic_radio_recv_async() to return
 *  -ENOTSUP.
 */
#define BASIC_RADIO_RX_FLAG_SCHED_ABS     BIT(0) /**< Start RX at abs_start_time_us.  Default: immediate. */
#define BASIC_RADIO_RX_FLAG_ONE_SHOT      BIT(1) /**< Stop after first packet.  Default: continuous. */
#define BASIC_RADIO_RX_FLAG_DISCARD_PAYL  BIT(2) /**< Discard payload bytes (PER test).  Default: keep. */
#define BASIC_RADIO_RX_FLAG_CHANNEL       BIT(3) /**< Use frequency_hz instead of Layer 1 default. */
#define BASIC_RADIO_RX_FLAG_SYNC_WORD     BIT(4) /**< Use sync_word instead of Layer 1 default. */
#define BASIC_RADIO_RX_FLAG_MAX_LEN       BIT(5) /**< Cap received payload at max_packet_len bytes. */

struct basic_radio_rx_params {
	uint32_t flags;
	/** Absolute start time in microseconds (radio time base). Used only when BASIC_RADIO_RX_FLAG_SCHED_ABS is set. */
	uint64_t abs_start_time_us;
	/** Carrier frequency in Hz. Used only when BASIC_RADIO_RX_FLAG_CHANNEL is set. */
	uint32_t frequency_hz;
	/** Sync word. Used only when BASIC_RADIO_RX_FLAG_SYNC_WORD is set. */
	uint32_t sync_word;
	/** Maximum payload length cap in bytes. Used only when BASIC_RADIO_RX_FLAG_MAX_LEN is set. */
	uint16_t max_packet_len;
};

/** @brief Tone adverbs passed to basic_radio_tone_async(). */
#define BASIC_RADIO_TONE_FLAG_SCHED_ABS  BIT(0) /**< Start tone at abs_start_time_us.  Default: immediate. */

struct basic_radio_tone_params {
	uint32_t flags;
	/** Absolute start time in microseconds (radio time base). Used only when BASIC_RADIO_TONE_FLAG_SCHED_ABS is set. */
	uint64_t abs_start_time_us;
	/** Carrier frequency in Hz. */
	uint32_t frequency_hz;
	/** TX power in dBm. */
	int8_t   tx_power_dbm;
	/** Duration in microseconds. 0 = run until basic_radio_stop(). */
	uint64_t duration_us;
};

/** @brief Radio capabilities returned by basic_radio_get_caps(). */
struct basic_radio_caps {
	uint32_t flags;                    /**< Bitmask of BASIC_RADIO_CAP_* */
	uint32_t freq_min;                 /**< Minimum carrier frequency in Hz. */
	uint32_t freq_max;                 /**< Maximum carrier frequency in Hz. */
	uint32_t freq_resolution_millihz;  /**< Frequency step in millihertz. */
	uint32_t retune_time_us;           /**< Time to retune between frequencies. */
	uint32_t sched_horizon_us;         /**< Maximum scheduling horizon in microseconds. */
	int8_t   tx_power_min_dbm;         /**< Minimum TX power in dBm. */
	int8_t   tx_power_max_dbm;         /**< Maximum TX power in dBm. */
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * Driver API - implemented by each backend, consumed by inline wrappers below.
 */
__subsystem struct basic_radio_driver_api {
	/* Main Functions */
	int (*config)(const struct device *dev, const struct basic_radio_config *cfg);
	int (*send_async)(const struct device *dev, const uint8_t *data, uint16_t len,
			  const struct basic_radio_tx_params *params, basic_radio_callback_t cb, void *user_data);
	int (*recv_async)(const struct device *dev, const struct basic_radio_rx_params *params,
			  basic_radio_callback_t cb, void *user_data);
	int (*tone_async)(const struct device *dev, const struct basic_radio_tone_params *params,
			  basic_radio_callback_t cb, void *user_data);
	int (*stop)(const struct device *dev, enum basic_radio_stop_mode mode);
	uint64_t (*time_get)(const struct device *dev);
	/* Optional Functions */
	int (*get_caps)(const struct device *dev, struct basic_radio_caps *caps);
	/* Vendor-Specific */
	int (*ext_ctl)(const struct device *dev, uint32_t cmd, void *arg, size_t arg_sz);
};
/** @endcond */

/**
 * @brief Apply a runtime radio configuration.
 *
 * Selects the PHY in @p cfg->phy (when the backend supports several) and
 * applies the runtime parameters on top of it. Changing to a PHY with
 * another radio configuration may reinitialize the radio. The radio must
 * be idle.
 *
 * @param dev  Radio device.
 * @param cfg  Configuration to apply.
 *
 * @retval 0       Success.
 * @retval -EBUSY  Radio is not idle.
 * @retval -EINVAL @p cfg->phy is not a PHY of @p dev, or a parameter is out
 *                 of range for the selected PHY.
 * @retval -EIO    Radio reconfiguration failed.
 */
static inline int basic_radio_config(const struct device *dev, const struct basic_radio_config *cfg)
{
	const struct basic_radio_driver_api *api = dev->api;

	return api->config(dev, cfg);
}

/**
 * @brief Transmit a packet asynchronously.
 *
 * The @p cb is invoked with BASIC_RADIO_EVT_TX_DONE when the frame is on the air
 * (or the operation fails).
 *
 * @param dev      Radio device.
 * @param data     Payload to transmit.
 * @param len      Length of @p data in bytes.
 * @param params   Optional per-packet overrides (may be NULL).
 * @param cb       Completion callback.
 * @param user_data Opaque pointer passed to @p cb.
 *
 * @return 0 on success (operation started), negative errno on failure.
 */
static inline int basic_radio_send_async(const struct device *dev, const uint8_t *data, uint16_t len,
					 const struct basic_radio_tx_params *params,
					 basic_radio_callback_t cb, void *user_data)
{
	const struct basic_radio_driver_api *api = dev->api;
	return api->send_async(dev, data, len, params, cb, user_data);
}

/**
 * @brief Start asynchronous receive.
 *
 * The @p cb is invoked with BASIC_RADIO_EVT_RX_PKT for each received packet
 * and BASIC_RADIO_EVT_RX_DONE when RX stops.
 *
 * @param dev      Radio device.
 * @param params   Optional per-operation overrides (may be NULL).
 * @param cb       Event callback.
 * @param user_data Opaque pointer passed to @p cb.
 *
 * @return 0 on success (RX started), negative errno on failure.
 */
static inline int basic_radio_recv_async(const struct device *dev, const struct basic_radio_rx_params *params,
					 basic_radio_callback_t cb, void *user_data)
{
	const struct basic_radio_driver_api *api = dev->api;
	return api->recv_async(dev, params, cb, user_data);
}

/**
 * @brief Start asynchronous continuous wave (tone) transmission.
 *
 * The @p cb is invoked with BASIC_RADIO_EVT_TONE_DONE when the tone ends
 * (duration elapsed or stopped).
 *
 * @param dev      Radio device.
 * @param params   Tone parameters (frequency, power, duration).
 * @param cb       Completion callback.
 * @param user_data Opaque pointer passed to @p cb.
 *
 * @return 0 on success (tone started), negative errno on failure.
 */
static inline int basic_radio_tone_async(const struct device *dev, const struct basic_radio_tone_params *params,
					 basic_radio_callback_t cb, void *user_data)
{
	const struct basic_radio_driver_api *api = dev->api;

	if (api->tone_async == NULL) {
		return -ENOSYS;
	}
	return api->tone_async(dev, params, cb, user_data);
}

/**
 * @brief Stop an ongoing operation.
 *
 * @param dev  Radio device.
 * @param mode Stop mode (deschedule, graceful, abort).
 *
 * @retval 0       Stop initiated (final event follows via callback).
 * @retval -EALREADY No operation in progress.
 * @retval -EBUSY  DESCHEDULE requested but operation already running.
 * @retval -ENOSYS Backend does not support stop.
 */
static inline int basic_radio_stop(const struct device *dev, enum basic_radio_stop_mode mode)
{
	const struct basic_radio_driver_api *api = dev->api;

	if (api->stop == NULL) {
		return -ENOSYS;
	}
	return api->stop(dev, mode);
}

/**
 * @brief Get current radio time in microseconds.
 *
 * @param dev Radio device.
 *
 * @return Current time in microseconds (radio time base).
 */
static inline uint64_t basic_radio_time_get(const struct device *dev)
{
	const struct basic_radio_driver_api *api = dev->api;

	if (api->time_get == NULL) {
		return 0;
	}
	return api->time_get(dev);
}

/**
 * @brief Query supported optional features and radio capabilities.
 *
 * @param dev  Radio device.
 * @param caps Output structure to fill.
 *
 * @retval 0 Success.
 * @retval -ENOSYS Backend does not implement get_caps.
 * @retval -ENODEV No PHY active (cannot determine capabilities).
 */
static inline int basic_radio_get_caps(const struct device *dev, struct basic_radio_caps *caps)
{
	const struct basic_radio_driver_api *api = dev->api;

	if (api->get_caps == NULL) {
		return -ENOSYS;
	}
	return api->get_caps(dev, caps);
}

/**
 * @brief Vendor-specific escape hatch. @c cmd space is per-backend.
 *
 * @retval 0        Command handled.
 * @retval -ENOSYS  Backend does not implement ext_ctl.
 * @retval -ENOTSUP Backend does not recognize @p cmd.
 */
static inline int basic_radio_ext_ctl(const struct device *dev, uint32_t cmd, void *arg, size_t arg_sz)
{
	const struct basic_radio_driver_api *api = dev->api;

	if (api->ext_ctl == NULL) {
		return -ENOSYS;
	}
	return api->ext_ctl(dev, cmd, arg, arg_sz);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_BASIC_RADIO_H_ */