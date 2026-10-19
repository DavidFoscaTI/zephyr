/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/basic_radio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(basic_radio_rx, LOG_LEVEL_INF);

#define BASIC_RADIO_NODE 	DT_NODELABEL(basic_radio0)
#define PHY0_NODE    	 	DT_NODELABEL(phy0)

static const struct device *const rf_dev = DEVICE_DT_GET(BASIC_RADIO_NODE);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define RX_BUF_LEN 255

static uint8_t rx_buf[RX_BUF_LEN];
static uint32_t cnt = 0;

static void rx_cb(const struct device *dev, const struct basic_radio_evt *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case BASIC_RADIO_EVT_RX_PKT:
		cnt++;
		LOG_INF("RX #%u: %u bytes RSSI=%d dBm crc_ok=%d",
			cnt, evt->len, evt->info.rssi, (int)evt->info.crc_ok);
		LOG_HEXDUMP_INF(evt->data, evt->len, "payload");
		gpio_pin_set_dt(&led, 1);
		k_sleep(K_MSEC(100));
		gpio_pin_set_dt(&led, 0);
		break;

	case BASIC_RADIO_EVT_RX_DONE:
		LOG_INF("RX done: %d", evt->status);
		break;
	}
}

int main(void)
{
	int ret;

	if (!device_is_ready(rf_dev))
	{
		LOG_ERR("basic_radio device not ready");
		return -ENODEV;
	}

	if (!device_is_ready(led.port)) {
		LOG_ERR("LED device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("LED configure failed: %d", ret);
		return ret;
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

	LOG_INF("Listening for packets");
	ret = basic_radio_recv_async(rf_dev, NULL, rx_cb, NULL);

	if (ret != 0)
	{
		LOG_ERR("recv_async failed: %d", ret);
		return ret;
	}

	/* RX runs asynchronously, just sleep forever */
	while (true) {
		k_sleep(K_FOREVER);
	}

	return 0;
}