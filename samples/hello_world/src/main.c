/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <board.h>
#include <device.h>
#include <misc/printk.h>

#include <driverlib/ioc.h>
//#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/cpu.h>
#include <driverlib/prcm.h>
#include <driverlib/uart.h>

#include <inc/hw_gpio.h>

/* Change this if you have an LED connected to a custom port */
#define PORT	LED0_GPIO_PORT

/* Change this if you have an LED connected to a custom pin */
#define LED	LED0_GPIO_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME 	1000

#define BOARD_IOID_LED_1          IOID_10
#define BOARD_IOID_LED_2          IOID_15
#define BOARD_LED_1               (1 << BOARD_IOID_LED_1)
#define BOARD_LED_2               (1 << BOARD_IOID_LED_2)
#define BOARD_LED_ALL             (BOARD_LED_1 | BOARD_LED_2)

/*---------------------------------------------------------------------------*/
/* Which events to trigger a UART interrupt */
#define CC26XX_UART_RX_INTERRUPT_TRIGGERS (UART_INT_RX | UART_INT_RT)

/* All interrupt masks */
#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
                                   UART_INT_FE | UART_INT_RT | UART_INT_TX | \
                                   UART_INT_RX | UART_INT_CTS)

void main(void)
{
	int cnt = 0;
//	struct device *dev;

//	dev = device_get_binding(PORT);
	/* Set LED pin as output */
//	gpio_pin_configure(dev, LED, GPIO_DIR_OUT);

//	IntMasterDisable();

//	power_domains_on();

	PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);

	PRCMLoadSet();

	ROM_IOCPinTypeGpioOutput(BOARD_IOID_LED_1);
	ROM_IOCPinTypeGpioOutput(BOARD_IOID_LED_2);

	GPIO_clearMultiDio(BOARD_LED_ALL);

//	HWREG( GPIO_BASE + GPIO_O_DOUTCLR31_0 ) = BOARD_LED_ALL;


	//printk("Hello World! %s\n", CONFIG_ARCH);

	while (1) {
		/* Set pin to HIGH/LOW every 1 second */
//		gpio_pin_write(dev, LED, cnt % 2);
		cnt++;

		GPIO_toggleDio(BOARD_IOID_LED_2);

//		HWREG( GPIO_BASE + GPIO_O_DOUTTGL31_0 ) = ( 1 << BOARD_IOID_LED_2 );

		k_sleep(SLEEP_TIME);
	}
}
