/*
 * Minimal GPIO driver for NXP PCF857x family (PCF8574 / PCF8575).
 * Supports 8 or 16 GPIOs, selected via 'ngpios' DT property.
 *
 * This is good enough for keyboard matrix use: basic input/output,
 * no advanced interrupt handling.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT nxp_pcf857x
#define PCF857X_INIT_PRIORITY 80

struct pcf857x_cfg {
	struct i2c_dt_spec bus;
	uint8_t ngpios;
};

struct pcf857x_data {
	struct gpio_driver_data common;
	struct k_mutex lock;
	uint16_t output_state;
};

/* Read current pin state from the expander */
static int pcf857x_reg_read(const struct device *dev, uint16_t *val)
{
	const struct pcf857x_cfg *cfg = dev->config;
	uint8_t buf[2] = {0};
	int len = (cfg->ngpios > 8) ? 2 : 1;
	int ret = i2c_read_dt(&cfg->bus, buf, len);

	if (ret < 0) {
		return ret;
	}

	uint16_t v = buf[0];
	if (cfg->ngpios > 8) {
		v |= ((uint16_t)buf[1]) << 8;
	}

	*val = v;
	return 0;
}

/* Write output_state to the expander */
static int pcf857x_reg_write(const struct device *dev, uint16_t val)
{
	const struct pcf857x_cfg *cfg = dev->config;
	uint8_t buf[2];

	buf[0] = (uint8_t)(val & 0xFF);
	if (cfg->ngpios > 8) {
		buf[1] = (uint8_t)((val >> 8) & 0xFF);
	}

	int len = (cfg->ngpios > 8) ? 2 : 1;
	return i2c_write_dt(&cfg->bus, buf, len);
}

static int pcf857x_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	uint16_t v;
	int ret = pcf857x_reg_read(dev, &v);

	if (ret < 0) {
		return ret;
	}

	*value = v;
	return 0;
}

static int pcf857x_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
				       gpio_port_value_t value)
{
	struct pcf857x_data *data = dev->data;
	const struct pcf857x_cfg *cfg = dev->config;

	uint16_t valid_mask = (cfg->ngpios == 16) ? 0xFFFFU : 0x00FFU;

	mask &= valid_mask;
	value &= valid_mask;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->output_state = (data->output_state & ~mask) | (value & mask);
	int ret = pcf857x_reg_write(dev, data->output_state);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int pcf857x_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return pcf857x_port_set_masked_raw(dev, pins, pins);
}

static int pcf857x_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return pcf857x_port_set_masked_raw(dev, pins, 0);
}

static int pcf857x_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	struct pcf857x_data *data = dev->data;
	const struct pcf857x_cfg *cfg = dev->config;

	uint16_t valid_mask = (cfg->ngpios == 16) ? 0xFFFFU : 0x00FFU;

	pins &= valid_mask;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->output_state ^= pins;
	int ret = pcf857x_reg_write(dev, data->output_state);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int pcf857x_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct pcf857x_data *data = dev->data;
	const struct pcf857x_cfg *cfg = dev->config;

	if (pin >= cfg->ngpios) {
		return -EINVAL;
	}

	/* No support for push-pull vs open-drain per pin; it's quasi-bidirectional.
	 * Ignore pull-up/down flags, the chip uses internal pull-ups.
	 */
	if ((flags & GPIO_SINGLE_ENDED) != 0U &&
		(flags & GPIO_OPEN_DRAIN) == 0U) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (flags & GPIO_OUTPUT) {
		/* Output mode: bit 0 = drive low, bit 1 = high (or input-high) */
		if (flags & GPIO_OUTPUT_INIT_LOW) {
			data->output_state &= ~BIT(pin);
		} else {
			/* Default to high */
			data->output_state |= BIT(pin);
		}
	} else {
		/* Input mode: write 1 to let pin float high (input) */
		data->output_state |= BIT(pin);
	}

	int ret = pcf857x_reg_write(dev, data->output_state);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int pcf857x_init(const struct device *dev)
{
	struct pcf857x_data *data = dev->data;
	const struct pcf857x_cfg *cfg = dev->config;
	uint16_t v;
	int ret;

	if (!device_is_ready(cfg->bus.bus)) {
		return -ENODEV;
	}

	k_mutex_init(&data->lock);

	/* Read current state so we don't blindly clobber lines */
	ret = pcf857x_reg_read(dev, &v);
	if (ret < 0) {
		return ret;
	}

	data->output_state = v;
	return 0;
}

static const struct gpio_driver_api pcf857x_api = {
    .pin_configure       = pcf857x_pin_configure,
    .port_get_raw        = pcf857x_port_get_raw,
    .port_set_masked_raw = pcf857x_port_set_masked_raw,
    .port_set_bits_raw   = pcf857x_port_set_bits_raw,
    .port_clear_bits_raw = pcf857x_port_clear_bits_raw,
    .port_toggle_bits    = pcf857x_port_toggle_bits,
    /* no pin_get_raw member in this Zephyr version */
};


#define PCF857X_INIT(inst)                                                                         \
	static struct pcf857x_data pcf857x_data_##inst;                                            \
                                                                                                   \
	static const struct pcf857x_cfg pcf857x_cfg_##inst = {                                     \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.ngpios = DT_INST_PROP(inst, ngpios),                                              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, pcf857x_init, NULL, &pcf857x_data_##inst, &pcf857x_cfg_##inst, \
			      POST_KERNEL, PCF857X_INIT_PRIORITY, &pcf857x_api);

DT_INST_FOREACH_STATUS_OKAY(PCF857X_INIT);
