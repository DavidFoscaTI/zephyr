/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/basic_radio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(basic_radio_cw, LOG_LEVEL_INF);

#define BASIC_RADIO_NODE 	DT_NODELABEL(basic_radio0)
#define PHY0_NODE    		DT_NODELABEL(phy0)

static const struct device *const rf_dev = DEVICE_DT_GET(BASIC_RADIO_NODE);

#define CW_DURATION_MS   	5000U

static struct k_sem tone_done_sem;

static void tone_done_cb(const struct device *dev, const struct basic_radio_evt *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (evt->type == BASIC_RADIO_EVT_TONE_DONE) {
		if (evt->status == 0) {
			LOG_INF("Tone done");
		} else {
			LOG_ERR("Tone failed: %d", evt->status);
		}
		k_sem_give(&tone_done_sem);
	}
}

int main(void)
{
	int ret;

	if (!device_is_ready(rf_dev)) {
		LOG_ERR("basic_radio device not ready");
		return -ENODEV;
	}

	/* BASIC_RADIO_CONFIG_DT only sets .phy; other fields are 0 = keep boot defaults */
	ret = basic_radio_config(rf_dev, BASIC_RADIO_CONFIG_DT(PHY0_NODE));
	if (ret != 0) {
		LOG_ERR("basic_radio_config failed: %d", ret);
		return ret;
	}

	/* Boot defaults from DTSI are applied at init.
	 * App can override with basic_radio_config() if needed. */

	struct basic_radio_caps caps;
	ret = basic_radio_get_caps(rf_dev, &caps);
	if (ret == 0) {
		LOG_INF("caps: flags=0x%08x freq=%u-%u Hz power=%d..%d dBm",
			caps.flags, caps.freq_min, caps.freq_max,
			caps.tx_power_min_dbm, caps.tx_power_max_dbm);
	}

	LOG_INF("Emitting CW at boot-default frequency and power");
	k_sem_init(&tone_done_sem, 0, 1);

	struct basic_radio_tone_params tone_params = {
		.duration_us = CW_DURATION_MS * 1000,
	};

	ret = basic_radio_tone_async(rf_dev, &tone_params, tone_done_cb, NULL);
	if (ret != 0) {
		LOG_ERR("tone_async failed: %d", ret);
		return ret;
	}

	k_sem_take(&tone_done_sem, K_FOREVER);

	LOG_INF("CW done");
	return 0;
}