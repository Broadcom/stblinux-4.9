/*
 *  8250-core based driver for Broadcom ns16550a UARTs
 *
 *  Copyright (C) 2017 Broadcom
 *
 * This driver uses the standard 8250 driver core but adds the ability
 * to use a baud rate clock mux for more accurate high speed baud rate
 * selection.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>

#include "8250.h"

#define KHZ    1000
#define MHZ(x) ((x) * KHZ * KHZ)

static const u32 brcmstb_rate_table[] = {
	MHZ(81),
	MHZ(108),
	MHZ(64),		/* Actually 64285715 for some chips */
	MHZ(48),
};

static const u32 brcmstb_rate_table_7278[] = {
	MHZ(81),
	MHZ(108),
	0,
	MHZ(48),
};

struct brcmuart_priv {
	int		line;
	struct clk	*baud_mux_clk;
	unsigned long	default_mux_rate;
	u32		real_rates[ARRAY_SIZE(brcmstb_rate_table)];
	const u32	*rate_table;
	ktime_t		char_wait;
	struct uart_port *up;
	struct hrtimer	hrt;
	u32		bad_rx_timeout_keeps;
	u32		bad_rx_timeout_discards;
	u32		flags;
#define BRCMUART_PRIV_FLAGS_SHUTDOWN 1
};

/*
 * Not all clocks run at the exact specified rate, so set each requested
 * rate and then get the actual rate.
 */
static void init_real_clk_rates(struct device *dev, struct brcmuart_priv *priv)
{
	int x;
	int rc;

	priv->default_mux_rate = clk_get_rate(priv->baud_mux_clk);
	dev_dbg(dev, "Default BAUD MUX Clock rate is %lu\n",
		priv->default_mux_rate);

	for (x = 0; x < ARRAY_SIZE(priv->real_rates); x++) {
		if (priv->rate_table[x] == 0) {
			priv->real_rates[x] = 0;
			continue;
		}
		rc = clk_set_rate(priv->baud_mux_clk, priv->rate_table[x]);
		if (rc) {
			dev_err(dev, "Error selecting BAUD MUX clock for %u\n",
				priv->rate_table[x]);
			priv->real_rates[x] = priv->rate_table[x];
		} else {
			priv->real_rates[x] = clk_get_rate(priv->baud_mux_clk);
		}
	}
	 clk_set_rate(priv->baud_mux_clk, priv->default_mux_rate);
}

static void set_clock_mux(struct uart_port *up, struct brcmuart_priv *priv,
			u32 baud)
{
	u32 percent;
	u32 best_percent = UINT_MAX;
	u32 quot;
	u32 best_quot = 1;
	u32 rate;
	int best_index = -1;
	u64 hires_rate;
	u64 hires_baud;
	u64 hires_err;
	int rc;
	int i;
	int real_baud;

	/* If the Baud Mux Clock was not specified, just return */
	if (priv->baud_mux_clk == NULL)
		return;

	/* Find the closest match for specified baud */
	for (i = 0; i < ARRAY_SIZE(priv->real_rates); i++) {
		if (priv->real_rates[i] == 0)
			continue;
		rate = priv->real_rates[i] / 16;
		quot = DIV_ROUND_CLOSEST(rate, baud);
		if (!quot)
			continue;

		/* increase resolution to get xx.xx percent */
		hires_rate = (u64)rate * 10000;
		hires_baud = (u64)baud * 10000;

		hires_err = div_u64(hires_rate, (u64)quot);

		/* get the delta */
		if (hires_err > hires_baud)
			hires_err = (hires_err - hires_baud);
		else
			hires_err = (hires_baud - hires_err);

		percent = (unsigned long)DIV_ROUND_CLOSEST_ULL(hires_err, baud);
		dev_dbg(up->dev,
			"Baud rate: %u, MUX Clk: %u, Error: %u.%u%%\n",
			baud, priv->real_rates[i], percent / 100,
			percent % 100);
		if (percent < best_percent) {
			best_percent = percent;
			best_index = i;
			best_quot = quot;
		}
	}
	if (best_index == -1) {
		dev_err(up->dev, "Error, %d BAUD rate is too fast.\n", baud);
		return;
	}
	rate = priv->real_rates[best_index];
	rc = clk_set_rate(priv->baud_mux_clk, rate);
	if (rc)
		dev_err(up->dev, "Error selecting BAUD MUX clock\n");

	/* Error over 3 percent will cause data errors */
	if (best_percent > 300)
		dev_err(up->dev, "Error, baud: %d has %u.%u%% error\n",
			baud, percent / 100, percent % 100);

	real_baud = rate / 16 / best_quot;
	dev_dbg(up->dev, "Selecting BAUD MUX rate: %u\n", rate);
	dev_dbg(up->dev, "Requested baud: %u, Actual baud: %u\n",
		baud, real_baud);

	/* calc nanoseconds for 1.5 characters time at the given baud rate */
	i = 1000000000 / real_baud / 10;
	i += (i / 2);
	priv->char_wait = ns_to_ktime(i);

	up->uartclk = rate;
}

static void brcmstb_set_termios(struct uart_port *up,
				struct ktermios *termios,
				struct ktermios *old)
{
	struct uart_8250_port *p8250 = up_to_u8250p(up);
	struct brcmuart_priv *priv = up->private_data;

	set_clock_mux(up, priv, tty_termios_baud_rate(termios));
	serial8250_do_set_termios(up, termios, old);
	if (p8250->mcr & UART_MCR_AFE)
		p8250->port.status |= UPSTAT_AUTOCTS;
}

static int brcmuart_startup(struct uart_port *port)
{
	struct brcmuart_priv *priv = port->private_data;

	priv->flags &= ~BRCMUART_PRIV_FLAGS_SHUTDOWN;
	return serial8250_do_startup(port);
}

static void brcmuart_shutdown(struct uart_port *port)
{
	struct brcmuart_priv *priv = port->private_data;

	priv->flags |= BRCMUART_PRIV_FLAGS_SHUTDOWN;
	return serial8250_do_shutdown(port);
}

static int brcmuart_handle_irq(struct uart_port *p)
{
	unsigned int iir = serial_port_in(p, UART_IIR);
	struct brcmuart_priv *priv = p->private_data;
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int status;
	unsigned long flags;
	unsigned int ier;
	unsigned int mcr;
	int handled = 0;

	/*
	 * There's a bug in some 8250 cores where we get a timeout
	 * interrupt but there is no data ready.
	 */
	if (((iir & UART_IIR_ID) == UART_IIR_RX_TIMEOUT) &&
	    ((priv->flags & BRCMUART_PRIV_FLAGS_SHUTDOWN) == 0)) {
		spin_lock_irqsave(&p->lock, flags);
		status = serial_port_in(p, UART_LSR);
		if ((status & UART_LSR_DR) == 0) {

			ier = serial_port_in(p, UART_IER);
			/*
			 * if Receive Data Interrupt is enabled and
			 * we're uing hardware flow control, deassert
			 * RTS and wait for any chars in the pipline to
			 * arrive and then check for DR again.
			 */
			if ((ier & UART_IER_RDI) && (up->mcr & UART_MCR_AFE)) {
				ier &= ~(UART_IER_RLSI | UART_IER_RDI);
				serial_port_out(p, UART_IER, ier);
				mcr = serial_port_in(p, UART_MCR);
				mcr &= ~UART_MCR_RTS;
				serial_port_out(p, UART_MCR, mcr);
				hrtimer_start(&priv->hrt, priv->char_wait,
					      HRTIMER_MODE_REL);
			} else {
				serial_port_in(p, UART_RX);
			}

			handled = 1;
		}
		spin_unlock_irqrestore(&p->lock, flags);
		if (handled)
			return 1;
	}
	return serial8250_handle_irq(p, iir);
}

static enum hrtimer_restart brcmuart_hrtimer_func(struct hrtimer *t)
{
	struct brcmuart_priv *priv = container_of(t, struct brcmuart_priv, hrt);
	struct uart_port *p = priv->up;
	struct uart_8250_port *up = up_to_u8250p(p);
	unsigned int status;
	unsigned long flags;

	if (priv->flags & BRCMUART_PRIV_FLAGS_SHUTDOWN)
		return HRTIMER_NORESTART;

	spin_lock_irqsave(&p->lock, flags);
	status = serial_port_in(p, UART_LSR);

	/*
	 * If a character did not arrive after the timeout, clear the false
	 * receive timeout.
	 */
	if ((status & UART_LSR_DR) == 0) {
		serial_port_in(p, UART_RX);
		priv->bad_rx_timeout_discards++;
	} else {
		priv->bad_rx_timeout_keeps++;
	}

	/* re-enable receive unless upper layer has disabled it */
	if ((up->ier & (UART_IER_RLSI | UART_IER_RDI)) ==
	    (UART_IER_RLSI | UART_IER_RDI)) {
		status = serial_port_in(p, UART_IER);
		status |= (UART_IER_RLSI | UART_IER_RDI);
		serial_port_out(p, UART_IER, status);
		status = serial_port_in(p, UART_MCR);
		status |= UART_MCR_RTS;
		serial_port_out(p, UART_MCR, status);
	}
	spin_unlock_irqrestore(&p->lock, flags);
	return HRTIMER_NORESTART;
}

static ssize_t bad_rx_timeouts_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct brcmuart_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "No chars: %d, Late chars: %d\n",
		       priv->bad_rx_timeout_discards,
		       priv->bad_rx_timeout_keeps);
}
static DEVICE_ATTR_RO(bad_rx_timeouts);

static const struct of_device_id brcmuart_dt_ids[] = {
	{
		.compatible = "brcm,bcm7278-uart",
		.data = brcmstb_rate_table_7278,
	},
	{
		.compatible = "brcm,bcm7271-uart",
		.data = brcmstb_rate_table,
	},
	{},
};

MODULE_DEVICE_TABLE(of, brcmuart_dt_ids);

static int brcmuart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id = NULL;
	struct uart_8250_port *new_port;
	struct device *dev = &pdev->dev;
	struct brcmuart_priv *priv;
	struct clk *baud_mux_clk;
	struct resource *res_mem;
	struct uart_8250_port up;
	struct resource *irq;
	u32 clk_rate = 0;
	int ret;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(dev, "missing irq\n");
		return -EINVAL;
	}
	priv = devm_kzalloc(dev, sizeof(struct brcmuart_priv),
			GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	of_id = of_match_node(brcmuart_dt_ids, np);
	if (!of_id || !of_id->data)
		priv->rate_table = brcmstb_rate_table;
	else
		priv->rate_table = of_id->data;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem) {
		dev_err(dev, "Registers not specified.\n");
		return -ENODEV;
	}

	of_property_read_u32(np, "clock-frequency", &clk_rate);

	/* See if a Baud clock has been specified */
	baud_mux_clk = of_clk_get_by_name(np, "sw_baud");
	if (IS_ERR(baud_mux_clk)) {
		if (PTR_ERR(baud_mux_clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "BAUD MUX clock not specified\n");
	} else {
		dev_info(dev, "BAUD MUX clock found\n");
		ret = clk_prepare_enable(baud_mux_clk);
		if (ret)
			return ret;
		priv->baud_mux_clk = baud_mux_clk;
		init_real_clk_rates(dev, priv);
		clk_rate = priv->default_mux_rate;
	}

	if (clk_rate == 0) {
		dev_err(dev, "clock-frequency or clk not defined\n");
		return -EINVAL;
	}

	memset(&up, 0, sizeof(up));
	up.port.type = PORT_16550A;
	up.port.uartclk = clk_rate;
	up.port.dev = dev;
	up.port.mapbase = res_mem->start;
	up.port.irq = irq->start;
	up.port.handle_irq = brcmuart_handle_irq;
	up.port.regshift = 2;
	up.port.iotype = of_device_is_big_endian(np) ?
		UPIO_MEM32BE : UPIO_MEM32;
	up.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
		| UPF_FIXED_PORT | UPF_FIXED_TYPE | UPF_IOREMAP;
	up.port.dev = dev;
	up.port.private_data = priv;
	up.capabilities = UART_CAP_FIFO | UART_CAP_AFE;
	up.port.fifosize = 32;

	/* Check for a fixed line number */
	ret = of_alias_get_id(np, "serial");
	if (ret >= 0)
		up.port.line = ret;

	/* setup HR timer */
	hrtimer_init(&priv->hrt, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	priv->hrt.function = brcmuart_hrtimer_func;
	up.port.shutdown = brcmuart_shutdown;
	up.port.startup = brcmuart_startup;

	up.port.set_termios = brcmstb_set_termios;
	ret = serial8250_register_8250_port(&up);
	if (ret < 0)
		return ret;
	priv->line = ret;
	platform_set_drvdata(pdev, priv);
	new_port = serial8250_get_port(priv->line);
	priv->up = &new_port->port;

	ret = sysfs_create_file(&dev->kobj, &dev_attr_bad_rx_timeouts.attr);
	if (ret)
		dev_warn(dev, "Error creating sysfs attributes\n");

	return 0;
}

static int brcmuart_remove(struct platform_device *pdev)
{
	struct brcmuart_priv *priv = platform_get_drvdata(pdev);

	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_bad_rx_timeouts.attr);
	hrtimer_cancel(&priv->hrt);
	serial8250_unregister_port(priv->line);
	return 0;
}

static int __maybe_unused brcmuart_suspend(struct device *dev)
{
	struct brcmuart_priv *priv = dev_get_drvdata(dev);

	serial8250_suspend_port(priv->line);
	clk_disable_unprepare(priv->baud_mux_clk);

	return 0;
}

static int __maybe_unused brcmuart_resume(struct device *dev)
{
	struct brcmuart_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(priv->baud_mux_clk);
	if (ret)
		dev_err(dev, "Error enabling BAUD MUX clock\n");

	/*
	 * The hardware goes back to it's default after suspend
	 * so get the "clk" back in sync.
	 */
	ret = clk_set_rate(priv->baud_mux_clk, priv->default_mux_rate);
	if (ret)
		dev_err(dev, "Error restoring default BAUD MUX clock\n");
	serial8250_resume_port(priv->line);
	return 0;
}

static const struct dev_pm_ops brcmuart_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(brcmuart_suspend, brcmuart_resume)
};

static struct platform_driver brcmuart_platform_driver = {
	.driver = {
		.name	= "bcm7271-uart",
		.pm		= &brcmuart_dev_pm_ops,
		.of_match_table = brcmuart_dt_ids,
	},
	.probe		= brcmuart_probe,
	.remove		= brcmuart_remove,
};
module_platform_driver(brcmuart_platform_driver);

MODULE_AUTHOR("Al Cooper");
MODULE_DESCRIPTION("Broadcom NS16550A compatible serial port driver");
MODULE_LICENSE("GPL v2");
