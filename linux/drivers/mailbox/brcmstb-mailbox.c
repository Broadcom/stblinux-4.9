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
#include <linux/workqueue.h>
#if defined(CONFIG_ARM64) || defined(CONFIG_ARM)
#include <linux/arm-smccc.h>
#endif

#define BRCM_SCMI_SMC_OEM_FUNC	0x400

#define BRCM_FID(ch) ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, \
			IS_ENABLED(CONFIG_ARM64), \
			ARM_SMCCC_OWNER_OEM, \
			BRCM_SCMI_SMC_OEM_FUNC + (ch))
enum {
	A2P_CHAN = 0,
	P2A_CHAN,
	NUM_CHAN
};

struct brcm_work {
	struct work_struct w;
	unsigned int mbox_num;
};

struct chan_priv {
	unsigned int mbox_num;
	struct brcm_work work;
	unsigned int ch;
};

struct brcm_mbox {
	struct brcm_work work;
	struct mbox_controller controller;
	int irqs[NUM_CHAN];
};

static struct mbox_chan *brcm_mbox_of_xlate(struct mbox_controller *controller,
					    const struct of_phandle_args *sp)
{
	unsigned int ch = sp->args[0];
	struct brcm_mbox *mbox
		= container_of(controller, struct brcm_mbox, controller);

	if (!mbox || ch >= NUM_CHAN)
		return ERR_PTR(-ENOENT);

	return &mbox->controller.chans[ch];
}

#if defined(CONFIG_ARM64) || defined(CONFIG_ARM)
static int announce_msg(unsigned int mbox_num, unsigned int ch)
{
	struct arm_smccc_res res;

	if (ch >= NUM_CHAN)
		return -EIO;
	arm_smccc_smc(BRCM_FID(ch), mbox_num, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return -EIO;
	return 0;
}
#else
#error Func announce_msg() not defined for the current ARCH
#endif

static void p2a_work_func(struct work_struct *w)
{
	struct brcm_work *work = (struct brcm_work *)w;

	/* Kick the platform's P2A Tx queue */
	(void)announce_msg(work->mbox_num, P2A_CHAN);
}

static int brcm_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct chan_priv *priv = chan->con_priv;

	return announce_msg(priv->mbox_num, priv->ch);
}

static int brcm_mbox_startup(struct mbox_chan *chan)
{
	return 0;
}

static const struct mbox_chan_ops brcm_mbox_ops = {
	.send_data = brcm_mbox_send_data,
	.startup = brcm_mbox_startup,
};

static irqreturn_t brcm_a2p_isr(int irq, void *data)
{
	struct mbox_chan *chan = data;

	mbox_chan_received_data(chan, NULL);
	return IRQ_HANDLED;
}

static irqreturn_t brcm_p2a_isr(int irq, void *data)
{
	struct mbox_chan *chan = data;
	struct chan_priv *priv = (struct chan_priv *) chan->con_priv;
	struct work_struct *work = (struct work_struct *) &priv->work;

	mbox_chan_received_data(chan, NULL);

	/*
	 * If we are getting this interrupt on the p2a channel
	 * we need to kick the platform's queue. But, we have to
	 * do it after the interrupt is cleared as the kick may
	 * set the interrupt again.  So we schedule deferred
	 * work to do the kick.
	 */
	if (!work_pending(work))
		schedule_work(work);

	return IRQ_HANDLED;
}

static int brcm_mbox_probe(struct platform_device *pdev)
{
	struct brcm_mbox *mbox;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct chan_priv *chan_priv;
	unsigned int mbox_num;
	int ret;

	if (!np)
		return -EINVAL;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox_num = of_alias_get_id(np, "mailbox") < 0 ?
		0 : (unsigned int)of_alias_get_id(np, "mailbox");

	/* Allocate channels */
	mbox->controller.chans = devm_kzalloc(
		&pdev->dev, NUM_CHAN * sizeof(struct mbox_chan), GFP_KERNEL);
	if (!mbox->controller.chans)
		return -ENOMEM;
	chan_priv = devm_kzalloc(
		&pdev->dev, NUM_CHAN * sizeof(struct chan_priv), GFP_KERNEL);
	if (!chan_priv)
		return -ENOMEM;

	mbox->irqs[A2P_CHAN] = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, mbox->irqs[A2P_CHAN], brcm_a2p_isr,
				IRQF_NO_SUSPEND,
				"brcm: SCMI a2p intr",
				&mbox->controller.chans[A2P_CHAN]);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup SCMI a2p isr\n");
		return ret;
	}
	chan_priv[A2P_CHAN].mbox_num = mbox_num;
	chan_priv[A2P_CHAN].ch = A2P_CHAN;
	mbox->controller.chans[A2P_CHAN].con_priv = &chan_priv[A2P_CHAN];
	mbox->controller.num_chans++;

	/* Get SGI interrupt number for p2a */
	mbox->irqs[P2A_CHAN] = platform_get_irq(pdev, 1);
	if (ret >= 0) {
		ret = devm_request_irq(&pdev->dev, mbox->irqs[P2A_CHAN],
				       brcm_p2a_isr,
				       IRQF_NO_SUSPEND,
				       "brcm: SCMI p2a intr",
				       &mbox->controller.chans[P2A_CHAN]);
		if (ret) {
			dev_err(&pdev->dev, "failed to setup SCMI p2a isr\n");
			return ret;
		}
		chan_priv[P2A_CHAN].mbox_num = mbox_num;
		chan_priv[P2A_CHAN].ch = P2A_CHAN;
		INIT_WORK((struct work_struct *)&chan_priv[P2A_CHAN].work,
			  p2a_work_func);
		mbox->controller.chans[P2A_CHAN].con_priv
			= &chan_priv[P2A_CHAN];
		mbox->controller.num_chans++;
	}

	mbox->controller.dev = &pdev->dev;
	mbox->controller.ops = &brcm_mbox_ops;
	mbox->controller.of_xlate = brcm_mbox_of_xlate;
	ret = mbox_controller_register(&mbox->controller);
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

	if (mbox->controller.num_chans == NUM_CHAN) {
		struct chan_priv *priv = (struct chan_priv *)
			mbox->controller.chans[P2A_CHAN].con_priv;

		cancel_work_sync((struct work_struct *)&priv->work);
	}
	mbox_controller_unregister(&mbox->controller);

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
