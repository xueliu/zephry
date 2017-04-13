/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* See www.ti.com/lit/pdf/swru367, section 6, for CC2600 UART info. */

#include <kernel.h>
#include <arch/cpu.h>
#include <uart.h>

/* Driverlib includes */
#include <inc/hw_types.h>
#include <driverlib/rom.h>
#include <driverlib/ioc.h>
#include <driverlib/prcm.h>
#include <driverlib/uart.h>
#include <driverlib/sys_ctrl.h>

#define CC26XX_UART_CONF_BAUD_RATE    115200

/* All interrupt masks */
#define CC26XX_UART_INTERRUPT_ALL (UART_INT_OE | UART_INT_BE | UART_INT_PE | \
                                   UART_INT_FE | UART_INT_RT | UART_INT_TX | \
                                   UART_INT_RX | UART_INT_CTS)

struct uart_cc26xx_dev_data_t {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_t cb; /**< Callback function pointer */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define DEV_CFG(dev) \
	((const struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_cc26xx_dev_data_t * const)(dev)->driver_data)

/* Forward decls: */
static struct device DEVICE_NAME_GET(uart_cc26xx_0);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_cc26xx_isr(void *arg);
#endif

static const struct uart_device_config uart_cc26xx_dev_cfg_0 = {
	.base = (void *)TI_CC26XX_UART_40001000_BASE_ADDRESS,
	.sys_clk_freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
};

static struct uart_cc26xx_dev_data_t uart_cc26xx_dev_data_0 = {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.cb = NULL,
#endif
};

/*
 *  CC26XX UART has a configurable FIFO length, from 1 to 8 characters.
 *  However, the Zephyr console driver, and the Zephyr uart sample test, assume
 *  a RX FIFO depth of one: meaning, one interrupt == one character received.
 *  Keeping with this assumption, this driver leaves the FIFOs disabled,
 *  and at depth 1.
 */
static int uart_cc26xx_init(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	// MAP_PRCMPeripheralReset(PRCM_UARTA0);

	// /* This also calls MAP_UARTEnable() to enable the FIFOs: */
	// MAP_UARTConfigSetExpClk((unsigned long)config->base,
	// 			MAP_PRCMPeripheralClockGet(PRCM_UARTA0),
	// 			TI_CC26XX_UART_4000C000_BAUD_RATE,
	// 			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE
	// 			 | UART_CONFIG_PAR_NONE));
	// MAP_UARTFlowControlSet((unsigned long)config->base,
	// 		       UART_FLOWCONTROL_NONE);
	// /* Re-disable the FIFOs: */
	// MAP_UARTFIFODisable((unsigned long)config->base);
	uint32_t ctl_val = UART_CTL_UARTEN | UART_CTL_TXE;
	/*
	* Make sure the TX pin is output / high before assigning it to UART control
	* to avoid falling edge glitches
	*/
  	// IOCPinTypeGpioOutput(BOARD_IOID_UART_TX);
  	// GPIO_setDio(BOARD_IOID_UART_TX);

	/*
	* Map UART signals to the correct GPIO pins and configure them as
	* hardware controlled.
	*/
	// IOCPinTypeUart(UART0_BASE, BOARD_IOID_UART_RX, BOARD_IOID_UART_TX,
    //                        BOARD_IOID_UART_CTS, BOARD_IOID_UART_RTS);

	/* Configure the UART for 115,200, 8-N-1 operation. */
	UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(),
                                 CC26XX_UART_CONF_BAUD_RATE,
                                 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                  UART_CONFIG_PAR_NONE));
					

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/* Clear any pending UART interrupts: we only care about RX, TX: */
	// MAP_UARTIntClear((unsigned long)config->base,
	// 	(UART_INT_RX | UART_INT_TX));

	UARTIntClear((unsigned long)config->base, CC26XX_UART_INTERRUPT_ALL);

	/*
	* Generate an RX interrupt at FIFO 1/8 full.
	* We don't really care about the TX interrupt
	*/
  	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX7_8, UART_FIFO_RX1_8);

	IRQ_CONNECT(TI_CC26XX_UART_40001000_IRQ_0,
		    TI_CC26XX_UART_40001000_IRQ_0_PRIORITY,
		    uart_cc26xx_isr, DEVICE_GET(uart_cc26xx_0),
		    0);
	irq_enable(TI_CC26XX_UART_40001000_IRQ_0);
#endif

	/* Enable FIFOs */
  	HWREG(UART0_BASE + UART_O_LCRH) |= UART_LCRH_FEN;

	ctl_val += UART_CTL_RXE;

	/* Enable TX, RX (conditionally), and the UART. */
	HWREG(UART0_BASE + UART_O_CTL) = ctl_val;	

	return 0;
}

static int uart_cc26xx_poll_in(struct device *dev, unsigned char *c)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	if (UARTCharsAvail((unsigned long)config->base)) {
		*c = UARTCharGetNonBlocking((unsigned long)config->base);
	} else {
		return (-1);
	}
	return 0;
}

static unsigned char uart_cc26xx_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	UARTCharPut((unsigned long)config->base, c);

	return c;
}

static int uart_cc26xx_err_check(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);
	unsigned long cc26xx_errs = 0L;
	unsigned int z_err = 0;

	cc26xx_errs = UARTRxErrorGet((unsigned long)config->base);

	/* Map cc2600 SDK uart.h defines to zephyr uart.h defines */
	z_err = ((cc26xx_errs & UART_RXERROR_OVERRUN) ? UART_ERROR_OVERRUN : 0) |
		((cc26xx_errs & UART_RXERROR_BREAK) ? UART_ERROR_BREAK : 0) |
		((cc26xx_errs & UART_RXERROR_PARITY) ? UART_ERROR_PARITY : 0) |
		((cc26xx_errs & UART_RXERROR_FRAMING) ? UART_ERROR_FRAMING : 0);

	UARTRxErrorClear((unsigned long)config->base);

	return (int)z_err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_cc26xx_fifo_fill(struct device *dev, const uint8_t *tx_data,
				 int size)
{
	const struct uart_device_config *config = DEV_CFG(dev);
	unsigned int num_tx = 0;

	while ((size - num_tx) > 0) {
		/* Send a character */
		if (UARTCharPutNonBlocking((unsigned long)config->base,
					       tx_data[num_tx])) {
			num_tx++;
		} else {
			break;
		}
	}

	return (int)num_tx;
}

static int uart_cc26xx_fifo_read(struct device *dev, uint8_t *rx_data,
				 const int size)
{
	const struct uart_device_config *config = DEV_CFG(dev);
	unsigned int num_rx = 0;

	while (((size - num_rx) > 0) &&
		UARTCharsAvail((unsigned long)config->base)) {

		/* Receive a character */
		rx_data[num_rx++] =
			UARTCharGetNonBlocking((unsigned long)config->base);
	}

	return num_rx;
}

static void uart_cc26xx_irq_tx_enable(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	UARTIntEnable((unsigned long)config->base, UART_INT_TX);
}

static void uart_cc26xx_irq_tx_disable(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	UARTIntDisable((unsigned long)config->base, UART_INT_TX);
}

static int uart_cc26xx_irq_tx_ready(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);
	unsigned int int_status;

	int_status = UARTIntStatus((unsigned long)config->base, 1);

	return (int_status & UART_INT_TX);
}

static void uart_cc26xx_irq_rx_enable(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	/* FIFOs are left disabled from reset, so UART_INT_RT flag not used. */
	UARTIntEnable((unsigned long)config->base, UART_INT_RX);
}

static void uart_cc26xx_irq_rx_disable(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	UARTIntDisable((unsigned long)config->base, UART_INT_RX);
}

static int uart_cc26xx_irq_tx_empty(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);

	return (!UARTBusy((unsigned long)config->base));
}

static int uart_cc26xx_irq_rx_ready(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);
	unsigned int int_status;

	int_status = UARTIntStatus((unsigned long)config->base, 1);

	return (int_status & UART_INT_RX);
}

static void uart_cc26xx_irq_err_enable(struct device *dev)
{
	/* Not yet used in zephyr */
}

static void uart_cc26xx_irq_err_disable(struct device *dev)
{
	/* Not yet used in zephyr */
}

static int uart_cc26xx_irq_is_pending(struct device *dev)
{
	const struct uart_device_config *config = DEV_CFG(dev);
	unsigned int int_status;

	int_status = UARTIntStatus((unsigned long)config->base, 1);

	return (int_status & (UART_INT_TX | UART_INT_RX));
}

static int uart_cc26xx_irq_update(struct device *dev)
{
	return 1;
}

static void uart_cc26xx_irq_callback_set(struct device *dev,
					 uart_irq_callback_t cb)
{
	struct uart_cc26xx_dev_data_t * const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * Note: CC26XX UART Tx interrupts when ready to send; Rx interrupts when char
 * received.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
static void uart_cc26xx_isr(void *arg)
{
	struct device *dev = arg;
	const struct uart_device_config *config = DEV_CFG(dev);
	struct uart_cc26xx_dev_data_t * const dev_data = DEV_DATA(dev);

	unsigned long intStatus = UARTIntStatus((unsigned long)config->base,
						    1);

	if (dev_data->cb) {
		dev_data->cb(dev);
	}
	/*
	 * Clear interrupts only after cb called, as Zephyr UART clients expect
	 * to check interrupt status during the callback.
	 */
	UARTIntClear((unsigned long)config->base, intStatus);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_cc26xx_driver_api = {
	.poll_in = uart_cc26xx_poll_in,
	.poll_out = uart_cc26xx_poll_out,
	.err_check = uart_cc26xx_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill	  = uart_cc26xx_fifo_fill,
	.fifo_read	  = uart_cc26xx_fifo_read,
	.irq_tx_enable	  = uart_cc26xx_irq_tx_enable,
	.irq_tx_disable	  = uart_cc26xx_irq_tx_disable,
	.irq_tx_ready	  = uart_cc26xx_irq_tx_ready,
	.irq_rx_enable	  = uart_cc26xx_irq_rx_enable,
	.irq_rx_disable	  = uart_cc26xx_irq_rx_disable,
	.irq_tx_empty	  = uart_cc26xx_irq_tx_empty,
	.irq_rx_ready	  = uart_cc26xx_irq_rx_ready,
	.irq_err_enable	  = uart_cc26xx_irq_err_enable,
	.irq_err_disable  = uart_cc26xx_irq_err_disable,
	.irq_is_pending	  = uart_cc26xx_irq_is_pending,
	.irq_update	  = uart_cc26xx_irq_update,
	.irq_callback_set = uart_cc26xx_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

DEVICE_AND_API_INIT(uart_cc26xx_0, CONFIG_UART_CONSOLE_ON_DEV_NAME,
		    uart_cc26xx_init, &uart_cc26xx_dev_data_0,
		    &uart_cc26xx_dev_cfg_0,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    (void *)&uart_cc26xx_driver_api);
