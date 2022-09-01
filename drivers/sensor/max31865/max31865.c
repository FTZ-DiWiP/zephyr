/*
 * Copyright (c) 2022 HAW Hamburg FTZ-DIWIP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "max31865.h"

static int max31865_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	const struct max31865_config *cfg = dev->config;
	uint8_t cmd[10];

	cmd[0] = reg;
	memcpy(&cmd[1], data, length);
	struct spi_buf bufs[] = {

		{.buf = cmd, .len = length + 1}};
	struct spi_buf_set tx = {.buffers = bufs, .count = 1};

	return spi_write_dt(&cfg->spi, &tx);
}

static int max31865_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	const struct max31865_config *cfg = dev->config;

	reg &= 0x7F;
	const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf[] = {{
					   .buf = &reg,
					   .len = 1,
				   },
				   {.buf = data, .len = len}};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = 2};

	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

/**
 * @brief Set device configuration register
 *
 * @param device device instance
 * @return 0 if successful, or negative error code from SPI API
 */
static int configure_device(const struct device *dev)
{
	struct max31865_data *data = dev->data;
	uint8_t cmd[] = {data->config_control_bits};
	int err = max31865_spi_write(dev, WR(REG_CONFIG), cmd, 1);

	if (err < 0) {
		LOG_ERR("Error write SPI%d\n", err);
	}
	return err;
}

/**
 * @brief Set device fail threshold registers
 *
 * @param device device instance
 * @return 0 if successful, or negative error code from SPI API
 */
static int set_threshold_values(const struct device *dev)
{
	const struct max31865_config *config = dev->config;
	uint8_t cmd[] = {
		(config->high_threshold >> 7) & 0x00ff, (config->high_threshold << 1) & 0x00ff,
		(config->low_threshold >> 7) & 0x00ff, (config->low_threshold << 1) & 0x00ff};
	int err = max31865_spi_write(dev, WR(REG_HIGH_FAULT_THR_MSB), cmd, 4);

	if (err < 0) {
		LOG_ERR("Error write SPI%d\n", err);
	}
	return err;
}

#ifdef CONFIG_NEWLIB_LIBC

/**
 * Apply the Callendar-Van Dusen equation to convert the RTD resistance
 * to temperature:
 * Tr = (-A + SQRT(delta) ) / 2*B
 * delta = A^2 - 4B*(1-Rt/Ro)
 * @param resistance measured resistance
 * @param resistance_0 constant resistance at 0oC
 * @return calculated temperature
 */
static double calculate_temperature(double resistance, double resistance_0)
{
	double temperature;
	double delta = (RTD_A * RTD_A) - 4 * RTD_B * (1.0 - resistance / resistance_0);

	temperature = (-RTD_A + sqrt(delta)) / (2 * RTD_B);
	return temperature;
}

#else

/**
 * Apply a very good linear approximation of the Callendar-Van Dusen equation to convert the RTD
 * resistance to temperature:
 * @param resistance measured resistance
 * @param resistance_0 constant resistance at 0oC
 * @return calculated temperature
 */
static double calculate_temperature(double resistance, double resistance_0)
{
	double temperature;

	temperature = (resistance - resistance_0) / (resistance_0 * RTD_A);
	return temperature;
}

#endif

/**
 * @brief Enable/Disable Vbias for MAX31865
 *
 * @param device device instance
 * @param enable true, turn on vbias, false, turn off vbias
 * @return 0 if successful, or negative error code from SPI API
 */
static int max31865_set_vbias(const struct device *dev, bool enable)
{
	struct max31865_data *data = dev->data;

	WRITE_BIT(data->config_control_bits, 7, enable);
	return configure_device(dev);
}

/**
 * @brief Get temperature value in oC for device
 *
 * @param device device instance
 * @param temperature measured temperature
 * @return 0 if successful, or negative error code
 */
static int max31865_get_temperature(const struct device *dev, double *temperature)
{
	struct max31865_data *data = dev->data;

	max31865_set_vbias(dev, true);
	union read_reg_u {
		uint8_t u8[2];
		uint16_t u16;
	} read_reg;

	read_reg.u16 = 0;
	double resistance = 0;
	/* Waiting Time for Temerature Conversion (Page 3 of the datasheet)*/
	k_sleep(K_MSEC(66));
	/* Read resistance measured value */
	int err = max31865_spi_read(dev, (REG_RTD_MSB), read_reg.u8, 2);

	if (err < 0) {
		LOG_ERR("SPI read %d\n", err);
		return -EIO;
	}
	byteswp(read_reg.u8, read_reg.u8 + 1, 1);

	LOG_DBG("RAW: %02X %02X , %04X", read_reg.u8[0], read_reg.u8[1], read_reg.u16);
	if (TESTBIT(read_reg.u16, 0)) {
		uint8_t fault_register;

		max31865_spi_read(dev, (REG_FAULT_STATUS), &fault_register, 1);
		/*Clear fault register */
		WRITE_BIT(data->config_control_bits, 1, 1);
		configure_device(dev);
		LOG_ERR("Fault Register: %10x", fault_register);
		WRITE_BIT(data->config_control_bits, 1, 0);
		return -EIO;
	}
	max31865_set_vbias(dev, false);
	read_reg.u16 = read_reg.u16 >> 1;
	resistance = (double)read_reg.u16;
	resistance /= 32768;
	resistance *= data->resistance_reference;
	*temperature = calculate_temperature(resistance, data->resistance_at_zero);
	return 0;
}


static int max31865_init(const struct device *dev)
{
	const struct max31865_config *config = dev->config;

	if (!spi_is_ready(&config->spi)) {
		return -ENODEV;
	}
	struct max31865_data *data = dev->data;
	/* Set the confgiuration register */
	data->config_control_bits = 0;

	WRITE_BIT(data->config_control_bits, 6, config->conversion_mode);
	WRITE_BIT(data->config_control_bits, 5, config->one_shot);
	WRITE_BIT(data->config_control_bits, 4, config->three_wire);
	data->config_control_bits |= config->fault_cycle & 0b00001100;
	WRITE_BIT(data->config_control_bits, 0, config->filter_50hz);
	/* Set the PTD. */
	data->resistance_at_zero = config->resistance_at_zero;
	data->resistance_reference = config->resistance_reference;

	configure_device(dev);
	set_threshold_values(dev);
	max31865_set_vbias(dev, false);
	return 0;
}



static int max31865_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct max31865_data *data = dev->data;
	int ret;
	double temperature = 0;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		LOG_ERR("Invalid channel provided");
		return -ENOTSUP;
	}
	ret = max31865_get_temperature(dev, &temperature);
	if (ret < 0) {
		return ret;
	}
	sensor_value_from_double(&data->val, temperature);
	LOG_DBG("TEMP: %d.%06d\n", data->val.val1, data->val.val2);
	return 0;
}

static int max31865_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct max31865_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		val->val1 = data->val.val1;
		val->val2 = data->val.val2;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct sensor_driver_api max31865_api_funcs = {
	.sample_fetch = max31865_sample_fetch,
	.channel_get = max31865_channel_get,
};

#define MAX31865_DEFINE(inst)                                                                      \
                                                                                                   \
	static struct max31865_data max31865_data_##inst;                                          \
                                                                                                   \
	static const struct max31865_config max31865_config_##inst = {                             \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_MODE_CPHA | SPI_WORD_SET(8), 0),             \
		.resistance_at_zero = DT_INST_PROP(inst, resistance_at_zero),                      \
		.resistance_reference = DT_INST_PROP(inst, resistance_reference),                  \
		.conversion_mode = false,                                                          \
		.one_shot = true,                                                                  \
		.three_wire = DT_INST_PROP(inst, maxim_3_wire),                                    \
		.fault_cycle = false,                                                              \
		.filter_50hz = DT_INST_PROP(inst, filter_50hz),                                    \
		.low_threshold = DT_INST_PROP(inst, low_threshold),                                \
		.high_threshold = DT_INST_PROP(inst, high_threshold),                              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, max31865_init, NULL, &max31865_data_##inst,                    \
			      &max31865_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,   \
			      &max31865_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(MAX31865_DEFINE)
