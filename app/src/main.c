/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include <ncs_version.h>

LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS (2 * MSEC_PER_SEC)

#define STATE_IDLE   0x00
#define STATE_FLIGHT 0x01
#define STATE_LANDED 0x02

#define FLIGHT_WAKEUP_THRESHOLD_UG CONFIG_FLIGHT_WAKEUP_THRESHOLD_UG
#define STATIONARY_WINDOW_MS       CONFIG_FLIGHT_STATIONARY_WINDOW_MS
#define G_TARGET                   ((float)CONFIG_FLIGHT_G_TARGET_X100 / 100.0f)
#define STATIONARY_TOLERANCE       ((float)CONFIG_FLIGHT_STATIONARY_TOLERANCE_X100 / 100.0f)

K_SEM_DEFINE(data_ready_sem, 0, 1);
K_SEM_DEFINE(motion_sem, 0, 1);
K_SEM_DEFINE(stationary_sem, 0, 1);

static const struct device *prv_acc = DEVICE_DT_GET(DT_ALIAS(accel0));

static volatile bool prv_track_stationary = false;

struct sensor_trigger data_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

struct sensor_trigger motion_trig = {
	.type = SENSOR_TRIG_MOTION,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

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
	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		k_sem_give(&data_ready_sem);
		break;
	case SENSOR_TRIG_MOTION:
		k_sem_give(&motion_sem);
		break;
	default:
		LOG_WRN("Unknown trigger type: %d", trig->type);
		break;
	}
}

static void prv_detect_stationary(const float magnitude)
{
	static uint32_t still_start_time = 0;
	static bool was_still = false;

	if (!prv_track_stationary) {
		was_still = false;
		return;
	}

	/* Evaluate if magnitude falls within our resting gravity tolerance window */
	bool is_currently_still = (magnitude >= (G_TARGET - STATIONARY_TOLERANCE)) &&
				  (magnitude <= (G_TARGET + STATIONARY_TOLERANCE));

	if (is_currently_still) {
		if (!was_still) {
			still_start_time = k_uptime_get_32();
			was_still = true;
			LOG_DBG("Device settled. Starting timer.");
		}

		if (k_uptime_get_32() - still_start_time >= STATIONARY_WINDOW_MS) {
			LOG_INF("STATIONARY DETECTED. Still for %d ms", STATIONARY_WINDOW_MS);
			prv_track_stationary = false;
			was_still = false;
			k_sem_give(&stationary_sem);
		}
	} else {
		if (was_still) {
			LOG_DBG("Device moved! Noise spike: %.3f m/s^2. Resetting timer.",
				(double)magnitude);
		}
		was_still = false;
	}
}

static void prv_data_processing_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("DATA PROCESSING THREAD");
	while (1) {
		k_sem_take(&data_ready_sem, K_FOREVER);

		int rc = sensor_sample_fetch_chan(prv_acc, SENSOR_CHAN_ACCEL_XYZ);
		if (rc) {
			LOG_ERR("Failed to fetch sample: %d", rc);
			continue;
		}

		struct sensor_value accel_val[3];
		sensor_channel_get(prv_acc, SENSOR_CHAN_ACCEL_XYZ, accel_val);

		double x = sensor_value_to_double(&accel_val[0]);
		double y = sensor_value_to_double(&accel_val[1]);
		double z = sensor_value_to_double(&accel_val[2]);
		float magnitude = (float)sqrt((x * x) + (y * y) + (z * z));

		prv_detect_stationary(magnitude);
	}
}

static void prv_state_machine(void)
{
	static uint8_t flight_state = STATE_IDLE;

	switch (flight_state) {
	case STATE_IDLE:
		LOG_INF("State 0: Idle");

		k_sem_reset(&motion_sem);
		sensor_trigger_set(prv_acc, &motion_trig, prv_trigger_handler);

		k_sem_take(&motion_sem, K_FOREVER);

		sensor_trigger_set(prv_acc, &motion_trig, NULL);
		flight_state = STATE_FLIGHT;

		break;
	case STATE_FLIGHT:
		LOG_INF("State 1: Flight");
		sensor_trigger_set(prv_acc, &data_trig, prv_trigger_handler);

		prv_track_stationary = true;

		LOG_INF("Waiting for stationary trigger");
		k_sem_take(&stationary_sem, K_FOREVER);

		sensor_trigger_set(prv_acc, &data_trig, NULL);
		flight_state = STATE_LANDED;

		break;
	case STATE_LANDED:
		LOG_INF("State 2: Landed");

		k_msleep(CONFIG_BLE_ADVERTISEMENT_TIME_SEC *
			 MSEC_PER_SEC); // Sleep before resetting to idle

		/* Reset semaphores in case hard landing triggered motion interrupt */
		k_sem_reset(&motion_sem);
		k_sem_reset(&stationary_sem);

		flight_state = STATE_IDLE;

		break;
	default:
		LOG_WRN("Unknown state");
		break;
	}
}

int main(void)
{
	prv_boot_msg();

	if (!device_is_ready(prv_acc)) {
		LOG_ERR("Accelerometer not ready");
		return 0;
	}

	k_msleep(50);

	int rc;
	struct sensor_value threshold;
	sensor_ug_to_ms2(FLIGHT_WAKEUP_THRESHOLD_UG, &threshold);

	rc = sensor_attr_set(prv_acc, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_UPPER_THRESH, &threshold);
	if (rc != 0) {
		LOG_ERR("Failed to set wakeup threshold: %d", rc);
	}

	struct sensor_value dummy_val[3];
	sensor_sample_fetch_chan(prv_acc, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(prv_acc, SENSOR_CHAN_ACCEL_XYZ, dummy_val);

	LOG_INF("STARTING STATE MACHINE");
	while (1) {
		prv_state_machine();
		k_msleep(10);
	}

	return 0;
}

K_THREAD_DEFINE(data_processing_thread_id, CONFIG_DATA_PROCESSING_THREAD_STACK_SIZE,
		prv_data_processing_thread, NULL, NULL, NULL,
		CONFIG_DATA_PROCESSING_THREAD_PRIORITY, 0, 0);
