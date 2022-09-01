/*
 * Copyright (c) 2022 HAW Hamburg FTZ-DIWIP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

union sensor_val {
	struct sensor_value str;
	uint64_t wert;

};

void main(void)
{
	k_msleep(1000);

	const struct device *max_sensor = DEVICE_DT_GET_ANY(maxim_max31865);
	union sensor_val sv;
	int r;

	while (1) {

		r = sensor_sample_fetch(max_sensor);
		if (r != 0) {
			continue;
		}
		sensor_channel_get(max_sensor, SENSOR_CHAN_AMBIENT_TEMP, &sv.str);

		printk("TEMP: %d.%06d\n", sv.str.val1, sv.str.val2);

		sv.wert = 0;
		k_msleep(1000);
	}
}
