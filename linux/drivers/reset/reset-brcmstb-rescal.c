// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018 Broadcom */


#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/types.h>

#define BRCM_RESCAL_START	0
#define	BRCM_RESCAL_START_BIT	BIT(0)
#define BRCM_RESCAL_CTRL	4
#define BRCM_RESCAL_STATUS	8
#define BRCM_RESCAL_STATUS_BIT	BIT(0)

struct brcm_rescal_reset {
	void __iomem	*base;
	struct device *dev;
	struct reset_controller_dev rcdev;
};

static int brcm_rescal_reset_assert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	return 0;
}

static int brcm_rescal_reset_deassert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct brcm_rescal_reset *data =
		container_of(rcdev, struct brcm_rescal_reset, rcdev);
	void __iomem *base = data->base;
	const int NUM_RETRIES = 10;
	u32 reg;
	int i;

	reg = readl(base + BRCM_RESCAL_START);
	writel(reg | BRCM_RESCAL_START_BIT, base + BRCM_RESCAL_START);
	reg = readl(base + BRCM_RESCAL_START);
	if (!(reg & BRCM_RESCAL_START_BIT)) {
		dev_err(data->dev, "failed to start sata/pcie rescal\n");
		return -EIO;
	}

	reg = readl(base + BRCM_RESCAL_STATUS);
	for (i = NUM_RETRIES; i >= 0 &&  !(reg & BRCM_RESCAL_STATUS_BIT); i--) {
		udelay(100);
		reg = readl(base + BRCM_RESCAL_STATUS);
	}
	if (!(reg & BRCM_RESCAL_STATUS_BIT)) {
		dev_err(data->dev, "timedout on sata/pcie rescal\n");
		return -ETIMEDOUT;
	}

	reg = readl(base + BRCM_RESCAL_START);
	writel(reg ^ BRCM_RESCAL_START_BIT, base + BRCM_RESCAL_START);
	reg = readl(base + BRCM_RESCAL_START);
	dev_dbg(data->dev, "sata/pcie rescal success\n");

	return 0;
}

static int brcm_rescal_reset_xlate(struct reset_controller_dev *rcdev,
				   const struct of_phandle_args *reset_spec)
{
	/* This is needed if #reset-cells == 0. */
	return 0;
}

static const struct reset_control_ops brcm_rescal_reset_ops = {
	.assert = brcm_rescal_reset_assert,
	.deassert = brcm_rescal_reset_deassert,
};

static int brcm_rescal_reset_probe(struct platform_device *pdev)
{
	struct brcm_rescal_reset *data;
	struct resource *res;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	platform_set_drvdata(pdev, data);

	data->rcdev.owner = THIS_MODULE;
	data->rcdev.nr_resets = 1;
	data->rcdev.ops = &brcm_rescal_reset_ops;
	data->rcdev.of_node = pdev->dev.of_node;
	data->rcdev.of_xlate = brcm_rescal_reset_xlate;
	data->dev = &pdev->dev;

	return devm_reset_controller_register(&pdev->dev, &data->rcdev);
}

static const struct of_device_id brcm_rescal_reset_of_match[] = {
	{ .compatible = "brcm,bcm7216-pcie-sata-rescal" },
	{ },
};
MODULE_DEVICE_TABLE(of, brcm_rescal_reset_of_match);

static struct platform_driver brcm_rescal_reset_driver = {
	.probe = brcm_rescal_reset_probe,
	.driver = {
		.name	= "brcm-rescal-reset",
		.of_match_table	= brcm_rescal_reset_of_match,
	}
};
module_platform_driver(brcm_rescal_reset_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Sata/PCIe rescal reset controller");
MODULE_LICENSE("GPL v2");
