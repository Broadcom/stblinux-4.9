// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2017, Broadcom */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#if defined(CONFIG_ARM64) || defined(CONFIG_ARM)
#include <linux/arm-smccc.h>
#endif

#define BRCM_SCMI_SMC_OEM_FUNC	0x400

static const u32 BRCM_FID = ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,
					       IS_ENABLED(CONFIG_ARM64),
					       ARM_SMCCC_OWNER_OEM,
					       BRCM_SCMI_SMC_OEM_FUNC);

struct brcm_mbox {
	struct mbox_controller mbox;
	int irq;
};

static struct mbox_chan *chans;

#if defined(CONFIG_ARM64) || defined(CONFIG_ARM)
static int announce_msg(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(BRCM_FID, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;
	return 0;
}
#else
#error Func announce_msg() not defined for the current ARCH
#endif

static int brcm_mbox_send_data(struct mbox_chan *chan, void *data)
{
	return announce_msg();
}

static int brcm_mbox_startup(struct mbox_chan *chan)
{
	return 0;
}

static const struct mbox_chan_ops brcm_mbox_ops = {
	.send_data = brcm_mbox_send_data,
	.startup = brcm_mbox_startup,
};

static irqreturn_t brcm_isr(void)
{
	mbox_chan_received_data(&chans[0], NULL);
	return IRQ_HANDLED;
}

static int brcm_mbox_probe(struct platform_device *pdev)
{
	struct brcm_mbox *mbox;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -EINVAL;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	/* Allocate one channel */
	chans = devm_kzalloc(&pdev->dev, sizeof(*chans), GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	/* Get SGI interrupt number */
	ret = of_property_read_u32_index(np, "interrupts", 1, &mbox->irq);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get SGI intr from DT\n");
		return ret;
	}

	ret = set_ipi_handler(mbox->irq, brcm_isr,
			      "brcm: EL3 to Linux SCMI reply msg");
	if (ret) {
		dev_err(&pdev->dev, "failed to setup IPI isr\n");
		return -EINVAL;
	}

	mbox->mbox.dev = &pdev->dev;
	mbox->mbox.ops = &brcm_mbox_ops;
	mbox->mbox.chans = chans;
	mbox->mbox.num_chans = 1;

	ret = mbox_controller_register(&mbox->mbox);
	if (ret) {
		dev_err(dev, "failed to register BrcmSTB mbox\n");
		return ret;
	}

	platform_set_drvdata(pdev, mbox);
	return 0;
}

static int brcm_mbox_remove(struct platform_device *pdev)
{
	struct brcm_mbox *mbox = platform_get_drvdata(pdev);

	clear_ipi_handler(mbox->irq);
	mbox_controller_unregister(&mbox->mbox);
	return 0;
}

static const struct of_device_id brcm_mbox_of_match[] = {
	{ .compatible = "brcm,brcmstb-mbox", },
	{}
};
MODULE_DEVICE_TABLE(of, brcm_mbox_of_match);

static struct platform_driver brcm_mbox_driver = {
	.probe = brcm_mbox_probe,
	.remove = brcm_mbox_remove,
	.driver = {
		.name = "brcm_mbox",
		.of_match_table = brcm_mbox_of_match,
	},
};

module_platform_driver(brcm_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Broadcom STB SCMI driver");
MODULE_AUTHOR("Broadcom");
