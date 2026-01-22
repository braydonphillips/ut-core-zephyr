/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* LED aliases from devicetree */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

/* I2C bus node (change if your bus label isn't i2c1) */
#define I2C_BUS_NODE DT_NODELABEL(i2c1)

/* Your sensor 7-bit address: 1001000b = 0x48 */
#define TEMP_SENSOR_ADDR 0x48

/* Most of these sensors use register pointer 0x00 for temperature */
#define TEMP_REG 0x00

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

static const struct device *i2c_bus = DEVICE_DT_GET(I2C_BUS_NODE);

static int read_temp_c(float *temp_c)
{
	uint8_t reg = TEMP_REG;
	uint8_t buf[2];

	/* Write register pointer, then read 2 bytes */
	int ret = i2c_write_read(i2c_bus, TEMP_SENSOR_ADDR, &reg, 1, buf, 2);
	if (ret != 0) {
		return ret;
	}

	/* Combine bytes: MSB first */
	int16_t raw16 = (int16_t)((buf[0] << 8) | buf[1]);

	/*
	 * Table indicates 0.0625°C/LSB with left alignment (lower 4 bits unused).
	 * So shift right by 4 to get signed temperature counts.
	 */
	int16_t counts = raw16 >> 4;

	*temp_c = (float)counts * 0.0625f;
	return 0;
}

int main(void)
{
	int ret;

	/* LEDs */
	if (!gpio_is_ready_dt(&led0) || !gpio_is_ready_dt(&led1)) {
		printk("LED GPIO not ready\n");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) return 0;

	ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) return 0;

	/* I2C bus */
	if (!device_is_ready(i2c_bus)) {
		printk("I2C bus not ready\n");
		return 0;
	}

	/* Probe sensor by attempting a temperature read once */
	float temp = 0.0f;
	ret = read_temp_c(&temp);
	if (ret == 0) {
		gpio_pin_set_dt(&led0, 1); /* Found sensor */
		printk("Temp sensor found at 0x%02X. First read: %.4f C\n", TEMP_SENSOR_ADDR, temp);
	} else {
		gpio_pin_set_dt(&led0, 0);
		printk("Temp sensor NOT found at 0x%02X (err %d)\n", TEMP_SENSOR_ADDR, ret);
		/* You can choose to stop here; I’ll keep looping and retrying */
	}

	while (1) {
		ret = read_temp_c(&temp);
		if (ret == 0) {
			printk("Temp: %.4f C\n", temp);

			/* LED1 on if temp > 40 C */
			gpio_pin_set_dt(&led1, (temp > 40.0f) ? 1 : 0);

			/* Keep LED0 on if comms are good */
			gpio_pin_set_dt(&led0, 1);
		} else {
			printk("I2C read error: %d\n", ret);
			gpio_pin_set_dt(&led0, 0);
			gpio_pin_set_dt(&led1, 0);
		}

		k_msleep(500);
	}

	return 0;
}
