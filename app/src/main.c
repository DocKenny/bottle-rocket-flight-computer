/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include <ncs_version.h>

LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS (2 * MSEC_PER_SEC)

/**
 * @brief Print firmware version and other useful information.
 */
static void prv_boot_msg(void)
{
	LOG_INF("---------------------------------------------");
	LOG_INF("App version:\t %s", APP_VERSION_EXTENDED_STRING);
	LOG_INF("App git hash:\t %s", STRINGIFY(APP_BUILD_VERSION));
	LOG_INF("NCS version:\t %s", NCS_VERSION_STRING);
	LOG_INF("Board:\t\t %s", CONFIG_BOARD);
	LOG_INF("---------------------------------------------");
}

int main(void)
{
	prv_boot_msg();

	const struct device *acc = DEVICE_DT_GET(DT_ALIAS(accel0));
	if (!device_is_ready(acc)) {
		LOG_ERR("Accelerometer not ready");
		return 0;
	}

	while (1) {
		struct sensor_value accel[3];
		if (sensor_sample_fetch_chan(acc, SENSOR_CHAN_ACCEL_XYZ) < 0) {
			LOG_ERR("Accelerometer sample fetch error\n");
			return 0;
		}

		sensor_channel_get(acc, SENSOR_CHAN_ACCEL_XYZ, accel);
		LOG_INF("Acceleration (m/s^2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));

		k_sleep(K_MSEC(SLEEP_TIME_MS));
	}
}
