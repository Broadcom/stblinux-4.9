/*
 * phy-brcm-usb.c - Broadcom USB Phy Driver
 *
 * Copyright (C) 2015-2017 Broadcom
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/soc/brcmstb/brcmstb.h>
#include <dt-bindings/phy/phy.h>

#include "phy-brcm-usb-init.h"

static DEFINE_MUTEX(sysfs_lock);

enum brcm_usb_phy_id {
	BRCM_USB_PHY_2_0 = 0,
	BRCM_USB_PHY_3_0,
	BRCM_USB_PHY_ID_MAX
};

struct value_to_name_map {
	int value;
	const char *name;
};

static struct value_to_name_map brcm_device_mode_to_name[] = {
	{ USB_CTLR_DEVICE_OFF, "off" },
	{ USB_CTLR_DEVICE_ON, "on" },
	{ USB_CTLR_DEVICE_DUAL, "dual" },
	{ USB_CTLR_DEVICE_TYPEC_PD, "typec-pd" }
};

static struct value_to_name_map brcm_dual_mode_to_name[] = {
	{ 0, "host" },
	{ 1, "device" },
	{ 2, "auto" },
};

struct brcm_usb_phy_data {
	struct  brcm_usb_common_init_params ini;
	bool			has_eohci;
	bool			has_xhci;
	struct clk		*usb_20_clk;
	struct clk		*usb_30_clk;
	struct mutex		mutex;
	int			init_count;
	struct brcm_usb_phy {
		struct phy *phy;
		unsigned int id;
		bool inited;
	} phys[BRCM_USB_PHY_ID_MAX];
};

#define to_brcm_usb_phy_data(phy) \
	container_of((phy), struct brcm_usb_phy_data, phys[(phy)->id])

static int brcm_usb_phy_init(struct phy *gphy)
{
	struct brcm_usb_phy *phy = phy_get_drvdata(gphy);
	struct brcm_usb_phy_data *priv = to_brcm_usb_phy_data(phy);

	/*
	 * Use a lock to make sure a second caller waits until
	 * the base phy is inited before using it.
	 */
	mutex_lock(&priv->mutex);
	if (priv->init_count++ == 0) {
		clk_enable(priv->usb_20_clk);
		clk_enable(priv->usb_30_clk);
		brcm_usb_init_common(&priv->ini);
	}
	mutex_unlock(&priv->mutex);
	if (phy->id == BRCM_USB_PHY_2_0)
		brcm_usb_init_eohci(&priv->ini);
	else if (phy->id == BRCM_USB_PHY_3_0)
		brcm_usb_init_xhci(&priv->ini);
	phy->inited = true;

	dev_dbg(&gphy->dev, "INIT, id: %d, total: %d\n", phy->id,
		priv->init_count);

	return 0;
}

static int brcm_usb_phy_exit(struct phy *gphy)
{
	struct brcm_usb_phy *phy = phy_get_drvdata(gphy);
	struct brcm_usb_phy_data *priv = to_brcm_usb_phy_data(phy);

	dev_dbg(&gphy->dev, "EXIT\n");
	if (phy->id == BRCM_USB_PHY_2_0)
		brcm_usb_uninit_eohci(&priv->ini);
	if (phy->id == BRCM_USB_PHY_3_0)
		brcm_usb_uninit_xhci(&priv->ini);

	/* If both xhci and eohci are gone, reset everything else */
	mutex_lock(&priv->mutex);
	if (--priv->init_count == 0) {
		brcm_usb_uninit_common(&priv->ini);
		clk_disable(priv->usb_20_clk);
		clk_disable(priv->usb_30_clk);
	}
	mutex_unlock(&priv->mutex);
	phy->inited = false;
	return 0;
}

static struct phy_ops brcm_usb_phy_ops = {
	.init		= brcm_usb_phy_init,
	.exit		= brcm_usb_phy_exit,
	.owner		= THIS_MODULE,
};

static struct phy *brcm_usb_phy_xlate(struct device *dev,
				struct of_phandle_args *args)
{
	struct brcm_usb_phy_data *data = dev_get_drvdata(dev);

	/*
	 * values 0 and 1 are for backward compatibility with
	 * device tree nodes from older bootloaders.
	 */
	switch (args->args[0]) {
	case 0:
	case PHY_TYPE_USB2:
		if (data->phys[BRCM_USB_PHY_2_0].phy)
			return data->phys[BRCM_USB_PHY_2_0].phy;
		dev_info(dev, "2.0 Phy not found\n");
		break;
	case 1:
	case PHY_TYPE_USB3:
		if (data->phys[BRCM_USB_PHY_3_0].phy)
			return data->phys[BRCM_USB_PHY_3_0].phy;
		dev_info(dev, "3.0 Phy not found\n");
		break;
	}
	return ERR_PTR(-ENODEV);
}

/*
 * Fix the case where the #phy-cells in the Device Tree node is
 * incorrectly set to 0 instead of 1. This happens with versions of
 * BOLT <= v1.18
 */
static int fix_phy_cells(struct device *dev, struct device_node *dn)
{
	struct property *new;
	u32 count;
	const char phy_cells[] = "#phy-cells";

	if (of_property_read_u32(dn, phy_cells, &count))
		return 1;

	if (count == 1)
		return 0;

	dev_info(dev, "Fixing incorrect #phy-cells in Device Tree node\n");
	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		return 1;

	new->name = kstrdup(phy_cells, GFP_KERNEL);
	if (!new->name)
		goto cleanup;

	new->length = sizeof(u32);
	new->value = kmalloc(new->length, GFP_KERNEL);
	if (!new->value)
		goto cleanup;

	*((u32 *)new->value) = cpu_to_be32(1);
	of_update_property(dn, new);
	return 0;

cleanup:
	kfree(new->name);
	kfree(new->value);
	kfree(new);
	return 1;
}

static int name_to_value(struct value_to_name_map *table, int count,
			const char *name, int *value)
{
	int x;

	*value = 0;
	for (x = 0; x < count; x++) {
		if (sysfs_streq(name, table[x].name)) {
			*value = x;
			return 0;
		}
	}
	return -EINVAL;
}

static const char *value_to_name(struct value_to_name_map *table, int count,
				int value)
{
	if (value >= count)
		return "unknown";
	return table[value].name;
}

static ssize_t device_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct brcm_usb_phy_data *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n",
		value_to_name(&brcm_device_mode_to_name[0],
			ARRAY_SIZE(brcm_device_mode_to_name),
			priv->ini.device_mode));
}
static DEVICE_ATTR_RO(device_mode);



static ssize_t dual_select_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct brcm_usb_phy_data *priv = dev_get_drvdata(dev);
	int value;
	int res;

	mutex_lock(&sysfs_lock);
	res = name_to_value(&brcm_dual_mode_to_name[0],
			ARRAY_SIZE(brcm_dual_mode_to_name), buf, &value);
	if (!res) {
		brcm_usb_init_set_dual_select(&priv->ini, value);
		res = len;
	}
	mutex_unlock(&sysfs_lock);
	return res;
}

static ssize_t dual_select_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct brcm_usb_phy_data *priv = dev_get_drvdata(dev);
	int value;

	mutex_lock(&sysfs_lock);
	value = brcm_usb_init_get_dual_select(&priv->ini);
	mutex_unlock(&sysfs_lock);
	return sprintf(buf, "%s\n",
		value_to_name(&brcm_dual_mode_to_name[0],
			ARRAY_SIZE(brcm_dual_mode_to_name),
			value));
}
static DEVICE_ATTR_RW(dual_select);

static struct attribute *brcm_usb_phy_attrs[] = {
	&dev_attr_device_mode.attr,
	&dev_attr_dual_select.attr,
	NULL
};

static const struct attribute_group brcm_usb_phy_group = {
	.attrs = brcm_usb_phy_attrs,
};

static int brcm_usb_phy_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct brcm_usb_phy_data *priv;
	struct phy *gphy;
	struct phy_provider *phy_provider;
	struct device_node *dn = pdev->dev.of_node;
	int err;
	const char *device_mode;
	char err_msg_ioremap[] = "can't map register space\n";

	dev_dbg(&pdev->dev, "brcm_usb_phy_probe\n");
	fix_phy_cells(dev, dn);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	priv->ini.family_id = brcmstb_get_family_id();
	priv->ini.product_id = brcmstb_get_product_id();
	brcm_usb_set_family_map(&priv->ini);
	dev_dbg(&pdev->dev, "Best mapping table is for %s\n",
		priv->ini.family_name);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "can't get USB_CTRL base address\n");
		return -EINVAL;
	}
	priv->ini.ctrl_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->ini.ctrl_regs)) {
		dev_err(dev, err_msg_ioremap);
		return -EINVAL;
	}

	/* The XHCI EC registers are optional */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res != NULL) {
		priv->ini.xhci_ec_regs =
			devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->ini.xhci_ec_regs)) {
			dev_err(&pdev->dev, err_msg_ioremap);
			return -EINVAL;
		}
	}

	of_property_read_u32(dn, "ipp", &priv->ini.ipp);
	of_property_read_u32(dn, "ioc", &priv->ini.ioc);

	priv->ini.device_mode = USB_CTLR_DEVICE_OFF;
	err = of_property_read_string(dn, "device", &device_mode);
	if (err == 0)
		name_to_value(&brcm_device_mode_to_name[0],
			ARRAY_SIZE(brcm_device_mode_to_name),
			device_mode, &priv->ini.device_mode);

	if (of_property_read_bool(dn, "has_xhci_only")) {
		priv->has_xhci = true;
	} else {
		priv->has_eohci = true;
		if (of_property_read_bool(dn, "has_xhci"))
			priv->has_xhci = true;
	}
	if (priv->has_eohci) {
		gphy = devm_phy_create(dev, NULL, &brcm_usb_phy_ops);
		if (IS_ERR(gphy)) {
			dev_err(dev, "failed to create EHCI/OHCI PHY\n");
			return PTR_ERR(gphy);
		}
		priv->phys[BRCM_USB_PHY_2_0].phy = gphy;
		priv->phys[BRCM_USB_PHY_2_0].id = BRCM_USB_PHY_2_0;
		phy_set_drvdata(gphy, &priv->phys[BRCM_USB_PHY_2_0]);
	}
	if (priv->has_xhci) {
		gphy = devm_phy_create(dev, NULL, &brcm_usb_phy_ops);
		if (IS_ERR(gphy)) {
			dev_err(dev, "failed to create XHCI PHY\n");
			return PTR_ERR(gphy);
		}
		priv->phys[BRCM_USB_PHY_3_0].phy = gphy;
		priv->phys[BRCM_USB_PHY_3_0].id = BRCM_USB_PHY_3_0;
		phy_set_drvdata(gphy, &priv->phys[BRCM_USB_PHY_3_0]);
	}
	mutex_init(&priv->mutex);
	phy_provider = devm_of_phy_provider_register(dev,
			brcm_usb_phy_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	priv->usb_20_clk = of_clk_get_by_name(dn, "sw_usb");
	if (IS_ERR(priv->usb_20_clk)) {
		dev_err(&pdev->dev, "Clock not found in Device Tree\n");
		priv->usb_20_clk = NULL;
	}
	err = clk_prepare_enable(priv->usb_20_clk);
	if (err)
		return err;

	/* Get the USB3.0 clocks if we have XHCI */
	if (priv->has_xhci) {
		priv->usb_30_clk = of_clk_get_by_name(dn, "sw_usb3");
		if (IS_ERR(priv->usb_30_clk)) {
			/* Older device-trees are missing this clock */
			dev_info(&pdev->dev,
				"USB3.0 clock not found in Device Tree\n");
			priv->usb_30_clk = NULL;
		}
		err = clk_prepare_enable(priv->usb_30_clk);
		if (err)
			return err;
	}

	brcm_usb_init_ipp(&priv->ini);

	/*
	 * Create sysfs entries for device_mode.
	 * Remove "dual_select" attribute if not in dual mode
	 */
	if (priv->ini.device_mode != USB_CTLR_DEVICE_DUAL)
		brcm_usb_phy_attrs[1] = NULL;
	err = sysfs_create_group(&dev->kobj, &brcm_usb_phy_group);
	if (err)
		dev_warn(&pdev->dev, "Error creating sysfs attributes\n");

	/* start with everything off */
	if (priv->has_xhci)
		brcm_usb_uninit_xhci(&priv->ini);
	if (priv->has_eohci)
		brcm_usb_uninit_eohci(&priv->ini);
	brcm_usb_uninit_common(&priv->ini);
	clk_disable(priv->usb_20_clk);
	clk_disable(priv->usb_30_clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int brcm_usb_phy_suspend(struct device *dev)
{
	struct brcm_usb_phy_data *priv = dev_get_drvdata(dev);

	if (priv->init_count) {
		clk_disable(priv->usb_20_clk);
		clk_disable(priv->usb_30_clk);
	}
	return 0;
}

static int brcm_usb_phy_resume(struct device *dev)
{
	struct brcm_usb_phy_data *priv = dev_get_drvdata(dev);

	clk_enable(priv->usb_20_clk);
	clk_enable(priv->usb_30_clk);
	brcm_usb_init_ipp(&priv->ini);

	/*
	 * Initialize anything that was previously initialized.
	 * Uninitialize anything that wasn't previously initialized.
	 */
	if (priv->init_count) {
		brcm_usb_init_common(&priv->ini);
		if (priv->phys[BRCM_USB_PHY_2_0].inited) {
			brcm_usb_init_eohci(&priv->ini);
		} else if (priv->has_eohci) {
			brcm_usb_uninit_eohci(&priv->ini);
			clk_disable(priv->usb_20_clk);
		}
		if (priv->phys[BRCM_USB_PHY_3_0].inited) {
			brcm_usb_init_xhci(&priv->ini);
		} else if (priv->has_xhci) {
			brcm_usb_uninit_xhci(&priv->ini);
			clk_disable(priv->usb_30_clk);
		}
	} else {
		if (priv->has_xhci)
			brcm_usb_uninit_xhci(&priv->ini);
		if (priv->has_eohci)
			brcm_usb_uninit_eohci(&priv->ini);
		brcm_usb_uninit_common(&priv->ini);
		clk_disable(priv->usb_20_clk);
		clk_disable(priv->usb_30_clk);
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops brcm_usb_phy_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(brcm_usb_phy_suspend, brcm_usb_phy_resume)
};

static const struct of_device_id brcm_usb_dt_ids[] = {
	{ .compatible = "brcm,brcmstb-usb-phy" },
	{ .compatible = "brcm,usb-phy" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, brcm_usb_dt_ids);

static struct platform_driver brcm_usb_driver = {
	.probe		= brcm_usb_phy_probe,
	.driver		= {
		.name	= "brcmstb-usb-phy",
		.owner	= THIS_MODULE,
		.pm = &brcm_usb_phy_pm_ops,
		.of_match_table = brcm_usb_dt_ids,
	},
};

module_platform_driver(brcm_usb_driver);

MODULE_ALIAS("platform:brcmstb-usb-phy");
MODULE_AUTHOR("Al Cooper <acooper@broadcom.com>");
MODULE_DESCRIPTION("BRCM USB PHY driver");
MODULE_LICENSE("GPL v2");
