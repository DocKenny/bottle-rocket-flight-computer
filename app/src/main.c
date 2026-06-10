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

K_SEM_DEFINE(data_ready_sem, 0, 1);

static const struct device *prv_acc = DEVICE_DT_GET(DT_ALIAS(accel0));

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

static void prv_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trig);

	k_sem_give(&data_ready_sem);
}

static void prv_data_processing_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	static uint32_t sample_count = 0;
	static uint32_t last_print_time = 0;

	while (1) {
		k_sem_take(&data_ready_sem, K_FOREVER);

		int rc = sensor_sample_fetch_chan(prv_acc, SENSOR_CHAN_ACCEL_XYZ);
		if (rc) {
			LOG_ERR("Failed to fetch sample: %d", rc);
			continue;
		}

		struct sensor_value accel_val[3];
		sensor_channel_get(prv_acc, SENSOR_CHAN_ACCEL_XYZ, accel_val);

		sample_count++;
		uint32_t now = k_uptime_get_32();
		if (now - last_print_time >= 1000) {
			LOG_INF("Actual samples captured in the last second: %d", sample_count);
			sample_count = 0;
			last_print_time = now;
		}
	}
}

int main(void)
{
	prv_boot_msg();

	if (!device_is_ready(prv_acc)) {
		LOG_ERR("Accelerometer not ready");
		return 0;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	sensor_trigger_set(prv_acc, &trig, prv_trigger_handler);

	return 0;
}

K_THREAD_DEFINE(data_processing_thread_id, CONFIG_DATA_PROCESSING_THREAD_STACK_SIZE,
		prv_data_processing_thread, NULL, NULL, NULL,
		CONFIG_DATA_PROCESSING_THREAD_PRIORITY, 0, 0);
