// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018, Broadcom */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include "core.h"
#include "pwrseq.h"

struct mmc_pwrseq_brcm {
	struct mmc_pwrseq pwrseq;
};

static int mmc_go_bootmode(struct mmc_host *host)
{
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_GO_IDLE_STATE;
	cmd.arg = 0xf0f0f0f0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_NONE | MMC_CMD_BC;

	return mmc_wait_for_cmd(host, &cmd, 0);
}

static void mmc_pwrseq_brcm_power_off(struct mmc_host *host)
{
	mmc_go_bootmode(host);
}

static const struct mmc_pwrseq_ops mmc_pwrseq_brcm_ops = {
	.power_off = mmc_pwrseq_brcm_power_off,
};

static const struct of_device_id mmc_pwrseq_brcm_of_match[] = {
	{ .compatible = "brcm,bcm7211a0-mmc-pwrseq",},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, mmc_pwrseq_brcm_of_match);

static int mmc_pwrseq_brcm_probe(struct platform_device *pdev)
{
	struct mmc_pwrseq_brcm *pwrseq;
	struct device *dev = &pdev->dev;

	pwrseq = devm_kzalloc(dev, sizeof(*pwrseq), GFP_KERNEL);
	if (!pwrseq)
		return -ENOMEM;

	pwrseq->pwrseq.dev = dev;
	pwrseq->pwrseq.ops = &mmc_pwrseq_brcm_ops;
	pwrseq->pwrseq.owner = THIS_MODULE;
	platform_set_drvdata(pdev, pwrseq);

	return mmc_pwrseq_register(&pwrseq->pwrseq);
}

static int mmc_pwrseq_brcm_remove(struct platform_device *pdev)
{
	struct mmc_pwrseq_brcm *pwrseq = platform_get_drvdata(pdev);

	mmc_pwrseq_unregister(&pwrseq->pwrseq);

	return 0;
}

static struct platform_driver mmc_pwrseq_brcm_driver = {
	.probe = mmc_pwrseq_brcm_probe,
	.remove = mmc_pwrseq_brcm_remove,
	.driver = {
		.name = "pwrseq_emmc_brcm",
		.of_match_table = mmc_pwrseq_brcm_of_match,
	},
};

module_platform_driver(mmc_pwrseq_brcm_driver);
MODULE_LICENSE("GPL v2");
