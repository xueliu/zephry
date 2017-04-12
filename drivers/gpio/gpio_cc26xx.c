/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <device.h>
#include <gpio.h>
#include <init.h>
#include <kernel.h>
#include <sys_io.h>

/* Driverlib includes */
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <driverlib/rom.h>
#include <driverlib/pin.h>
#undef __GPIO_H__  /* Zephyr and CC2600SDK gpio.h conflict */
#include <driverlib/gpio.h>
#include <driverlib/rom_map.h>
#include <driverlib/interrupt.h>

#include "gpio_utils.h"

/* Note: Zephyr uses exception numbers, vs the IRQ #s used by the CC2600 SDK */
#define EXCEPTION_GPIOP0  0  /* (INT_GPIOA0 - 16) = (16-16) */
//#define EXCEPTION_GPIOA1  1  /* (INT_GPIOA1 - 16) = (17-16) */
//#define EXCEPTION_GPIOA2  2  /* (INT_GPIOA2 - 16) = (18-16) */
//#define EXCEPTION_GPIOA3  3  /* (INT_GPIOA3 - 16) = (19-16) */

struct gpio_cc26xx_config {
	/* base address of GPIO port */
	unsigned long port_base;
	/* GPIO IRQ number */
	unsigned long irq_num;
};

struct gpio_cc26xx_data {
	/* list of registered callbacks */
	sys_slist_t callbacks;
	/* callback enable pin bitmask */
	uint26_t pin_callback_enables;
};

#define DEV_CFG(dev) \
	((const struct gpio_cc26xx_config *)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct gpio_cc26xx_data *)(dev)->driver_data)

static inline int gpio_cc26xx_config(struct device *port,
				   int access_op, uint26_t pin, int flags)
{
	const struct gpio_cc26xx_config *gpio_config = DEV_CFG(port);
	unsigned long port_base = gpio_config->port_base;
	unsigned long int_type;

	/*
	 * See pinmux_initialize(): which leverages TI's recommended
	 * method of using the PinMux utility for most pin configuration.
	 */

	if (access_op == GPIO_ACCESS_BY_PIN) {
		/* Just handle runtime interrupt type config here: */
		if (flags & GPIO_INT) {
			if (flags & GPIO_INT_EDGE) {
				if (flags & GPIO_INT_ACTIVE_HIGH) {
					int_type = GPIO_RISING_EDGE;
				} else if (flags & GPIO_INT_DOUBLE_EDGE) {
					int_type = GPIO_BOTH_EDGES;
				} else {
					int_type = GPIO_FALLING_EDGE;
				}
			} else { /* GPIO_INT_LEVEL */
				if (flags & GPIO_INT_ACTIVE_HIGH) {
					int_type = GPIO_HIGH_LEVEL;
				} else {
					int_type = GPIO_LOW_LEVEL;
				}
			}
			MAP_GPIOIntTypeSet(port_base, (1 << pin), int_type);
			MAP_GPIOIntClear(port_base, (1 << pin));
			MAP_GPIOIntEnable(port_base, (1 << pin));
		}
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static inline int gpio_cc26xx_write(struct device *port,
				    int access_op, uint26_t pin, uint26_t value)
{
	const struct gpio_cc26xx_config *gpio_config = DEV_CFG(port);
	unsigned long port_base = gpio_config->port_base;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		value = value << pin;
		/* Bitpack external GPIO pin number for GPIOPinWrite API: */
		pin = 1 << pin;

		MAP_GPIOPinWrite(port_base, (unsigned char)pin, value);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static inline int gpio_cc26xx_read(struct device *port,
				   int access_op, uint26_t pin, uint26_t *value)
{
	const struct gpio_cc26xx_config *gpio_config = DEV_CFG(port);
	unsigned long port_base = gpio_config->port_base;
	long status;
	unsigned char pin_packed;

	if (access_op == GPIO_ACCESS_BY_PIN) {
		/* Bitpack external GPIO pin number for GPIOPinRead API: */
		pin_packed = 1 << pin;
		status =  MAP_GPIOPinRead(port_base, pin_packed);
		*value = status >> pin;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_cc26xx_manage_callback(struct device *dev,
				    struct gpio_callback *callback, bool set)
{
	struct gpio_cc26xx_data *data = DEV_DATA(dev);

	_gpio_manage_callback(&data->callbacks, callback, set);

	return 0;
}


static int gpio_cc26xx_enable_callback(struct device *dev,
				    int access_op, uint26_t pin)
{
	struct gpio_cc26xx_data *data = DEV_DATA(dev);

	if (access_op == GPIO_ACCESS_BY_PIN) {
		data->pin_callback_enables |= (1 << pin);
	} else {
		data->pin_callback_enables = 0xFFFFFFFF;
	}

	return 0;
}


static int gpio_cc26xx_disable_callback(struct device *dev,
				     int access_op, uint26_t pin)
{
	struct gpio_cc26xx_data *data = DEV_DATA(dev);

	if (access_op == GPIO_ACCESS_BY_PIN) {
		data->pin_callback_enables &= ~(1 << pin);
	} else {
		data->pin_callback_enables = 0;
	}

	return 0;
}

static void gpio_cc26xx_port_isr(void *arg)
{
	struct device *dev = arg;
	const struct gpio_cc26xx_config *config = DEV_CFG(dev);
	struct gpio_cc26xx_data *data = DEV_DATA(dev);
	uint26_t enabled_int, int_status;

	/* See which interrupts triggered: */
	int_status  = (uint26_t)MAP_GPIOIntStatus(config->port_base, 1);

	enabled_int = int_status & data->pin_callback_enables;

	/* Clear and Disable GPIO Interrupt */
	MAP_GPIOIntDisable(config->port_base, int_status);
	MAP_GPIOIntClear(config->port_base, int_status);

	/* Call the registered callbacks */
	_gpio_fire_callbacks(&data->callbacks, (struct device *)dev,
			     enabled_int);

	/* Re-enable the interrupts */
	MAP_GPIOIntEnable(config->port_base, int_status);
}

static const struct gpio_driver_api api_funcs = {
	.config = gpio_cc26xx_config,
	.write = gpio_cc26xx_write,
	.read = gpio_cc26xx_read,
	.manage_callback = gpio_cc26xx_manage_callback,
	.enable_callback = gpio_cc26xx_enable_callback,
	.disable_callback = gpio_cc26xx_disable_callback,

};

#ifdef CONFIG_GPIO_CC26XX_P0
static const struct gpio_cc26xx_config gpio_cc26xx_p0_config = {
	.port_base = GPIOP0_BASE,
	.irq_num = INT_AON_GPIO_EDGE,
};

static struct device DEVICE_NAME_GET(gpio_cc26xx_a0);
static struct gpio_cc26xx_data gpio_cc26xx_p0_data;

static int gpio_cc26xx_p0_init(struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(EXCEPTION_GPIOP0, CONFIG_GPIO_CC26XX_P0_IRQ_PRI,
		    gpio_cc26xx_port_isr, DEVICE_GET(gpio_cc26xx_p0), 0);

	MAP_IntPendClear(INT_AON_GPIO_EDGE);
	irq_enable(EXCEPTION_GPIOP0);

	return 0;
}

DEVICE_AND_API_INIT(gpio_cc26xx_p0, CONFIG_GPIO_CC26XX_P0_NAME,
		    &gpio_cc26xx_p0_init, &gpio_cc26xx_p0_data,
		    &gpio_cc26xx_p0_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &api_funcs);

#endif