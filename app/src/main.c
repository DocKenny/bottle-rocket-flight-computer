/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include <ncs_version.h>

LOG_MODULE_REGISTER(main);

#define SLEEP_TIME_MS (2 * MSEC_PER_SEC)

#define LIS2DW12_REG_CTRL7 0x3F

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

K_MUTEX_DEFINE(sensor_mutex);

static const struct device *prv_acc = DEVICE_DT_GET(DT_ALIAS(accel0));

static volatile bool prv_track_stationary = false;

static uint16_t prv_launch_count = 0;

static float prv_g_target = ((float)CONFIG_FLIGHT_G_TARGET_X100 / 100.0f);

struct sensor_trigger data_trig = {
	.type = SENSOR_TRIG_DATA_READY,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

struct sensor_trigger motion_trig = {
	.type = SENSOR_TRIG_MOTION,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};

/* 2B Dummy ID, 1B launch count + 2B max altitude */
static uint8_t prv_adv_data[5] = {0xFF, 0xFF, 0x00, 0x00, 0x00};
static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, prv_adv_data, sizeof(prv_adv_data)),
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

	/* Evaluate if magnitude falls within resting gravity tolerance window */
	bool is_currently_still = (magnitude >= (prv_g_target - STATIONARY_TOLERANCE)) &&
				  (magnitude <= (prv_g_target + STATIONARY_TOLERANCE));

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
		// Keep the thread entirely asleep if the state machine is not in STATE_FLIGHT
		if (!prv_track_stationary) {
			k_msleep(50);
			continue;
		}

		k_sem_take(&data_ready_sem, K_FOREVER);

		k_mutex_lock(&sensor_mutex, K_FOREVER);

		int rc = sensor_sample_fetch_chan(prv_acc, SENSOR_CHAN_ACCEL_XYZ);
		if (rc) {
			LOG_ERR("Failed to fetch sample: %d", rc);
			k_mutex_unlock(&sensor_mutex);
			continue;
		}

		struct sensor_value accel_val[3];
		sensor_channel_get(prv_acc, SENSOR_CHAN_ACCEL_XYZ, accel_val);

		k_mutex_unlock(&sensor_mutex);

		double x = accel_val[0].val1 + (accel_val[0].val2 / 1000000.0);
		double y = accel_val[1].val1 + (accel_val[1].val2 / 1000000.0);
		double z = accel_val[2].val1 + (accel_val[2].val2 / 1000000.0);
		float magnitude = (float)sqrt((x * x) + (y * y) + (z * z));

		prv_detect_stationary(magnitude);
	}
}

static void prv_calibrate_resting_g(void)
{
	LOG_INF("Calibrating baseline gravity. Keep device still");

	double sum_magnitude = 0.0;
	const uint8_t samples = 20;
	uint8_t valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		k_mutex_lock(&sensor_mutex, K_FOREVER);
		if (sensor_sample_fetch_chan(prv_acc, SENSOR_CHAN_ACCEL_XYZ) == 0) {
			struct sensor_value accel_val[3];
			sensor_channel_get(prv_acc, SENSOR_CHAN_ACCEL_XYZ, accel_val);

			double x = accel_val[0].val1 + (accel_val[0].val2 / 1000000.0);
			double y = accel_val[1].val1 + (accel_val[1].val2 / 1000000.0);
			double z = accel_val[2].val1 + (accel_val[2].val2 / 1000000.0);

			sum_magnitude += sqrt((x * x) + (y * y) + (z * z));
			valid_samples++;
		}
		k_mutex_unlock(&sensor_mutex);
		k_msleep(10);
	}

	if (valid_samples > 0) {
		prv_g_target = (float)(sum_magnitude / valid_samples);
		LOG_INF("Calibration complete. baseline G set to: %.3f m/s^2",
			(double)prv_g_target);
	} else {
		prv_g_target = ((float)CONFIG_FLIGHT_G_TARGET_X100 / 100.0f);
		LOG_WRN("Calibration failed! Using Kconfig default: %.3f m/s^2",
			(double)prv_g_target);
	}
}

static void prv_state_machine(void)
{
	static uint8_t flight_state = STATE_IDLE;

	switch (flight_state) {
	case STATE_IDLE:
		LOG_INF("State 0: Idle");

		prv_track_stationary = false;

		prv_calibrate_resting_g();

		k_mutex_lock(&sensor_mutex, K_FOREVER);
		sensor_trigger_set(prv_acc, &motion_trig, prv_trigger_handler);
		k_mutex_unlock(&sensor_mutex);

		k_msleep(100);
		k_sem_reset(&motion_sem);

		k_sem_take(&motion_sem, K_FOREVER);

		k_mutex_lock(&sensor_mutex, K_FOREVER);
		sensor_trigger_set(prv_acc, &motion_trig, NULL);
		k_mutex_unlock(&sensor_mutex);

		prv_launch_count++;
		flight_state = STATE_FLIGHT;

		break;
	case STATE_FLIGHT:
		LOG_INF("State 1: Flight");

		k_sem_reset(&data_ready_sem);
		k_sem_reset(&stationary_sem);

		k_mutex_lock(&sensor_mutex, K_FOREVER);
		sensor_trigger_set(prv_acc, &data_trig, prv_trigger_handler);
		k_mutex_unlock(&sensor_mutex);

		// NOW enable the background thread processing safely
		prv_track_stationary = true;

		LOG_INF("Waiting for stationary trigger");
		k_sem_take(&stationary_sem, K_FOREVER);

		prv_track_stationary = false;

		k_mutex_lock(&sensor_mutex, K_FOREVER);
		sensor_trigger_set(prv_acc, &data_trig, NULL);
		k_mutex_unlock(&sensor_mutex);

		flight_state = STATE_LANDED;
		break;
	case STATE_LANDED:
		LOG_INF("State 2: Landed");

		int err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			LOG_ERR("Advertising failed to start (err %d)", err);
		} else {
			LOG_INF("BLE Advertising started...");
		}

		k_msleep(CONFIG_BLE_ADVERTISEMENT_TIME_SEC *
			 MSEC_PER_SEC); // Sleep before resetting to idle

		/* Stop advertising before cycling back to idle */
		err = bt_le_adv_stop();
		if (err) {
			LOG_ERR("Advertising failed to stop (err %d)", err);
		} else {
			LOG_INF("BLE Advertising stopped.");
		}

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

	/* FIX: clear CTRL7 reg in order to fix immediate movement trigger bug */
	const struct i2c_dt_spec i2c_bus = I2C_DT_SPEC_GET(DT_ALIAS(accel0));

	if (device_is_ready(i2c_bus.bus)) {
		int err = i2c_reg_write_byte_dt(&i2c_bus, LIS2DW12_REG_CTRL7, 0x00);
		if (err) {
			LOG_WRN("Failed to force reset CTRL7 register: %d", err);
		}
	} else {
		LOG_ERR("I2C bus for accelerometer not ready");
	}

	/* Initialize Bluetooth */
	int rc = bt_enable(NULL);
	if (rc) {
		LOG_ERR("Bluetooth init failed (err %d)", rc);
		return 0;
	}

	k_msleep(50);

	struct sensor_value threshold;
	sensor_ug_to_ms2(FLIGHT_WAKEUP_THRESHOLD_UG, &threshold);

	k_mutex_lock(&sensor_mutex, K_FOREVER);
	rc = sensor_attr_set(prv_acc, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_UPPER_THRESH, &threshold);
	if (rc != 0) {
		LOG_ERR("Failed to set wakeup threshold: %d", rc);
	}

	struct sensor_value dummy_val[3];
	sensor_sample_fetch_chan(prv_acc, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(prv_acc, SENSOR_CHAN_ACCEL_XYZ, dummy_val);
	k_mutex_unlock(&sensor_mutex);

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
