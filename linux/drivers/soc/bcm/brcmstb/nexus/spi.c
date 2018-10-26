/*
 * Nexus SPI SHIM registration
 *
 * Copyright (C) 2017-2018, Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

struct brcmstb_spi_controller {
	const char *compat;
	unsigned int max_cs;
};

static __initconst const struct brcmstb_spi_controller spi_ctls[] = {
	{
		.compat = "brcm,spi-brcmstb-mspi",
		.max_cs = 4,
	},
	{
		.compat = "brcm,bcm2835-spi",
		/* Maximum number of native CS */
		.max_cs = 2,
	},
};

static int __init brcmstb_register_spi_devices(void)
{
	struct device_node *dn = NULL, *child;
	struct spi_board_info *spi_bdinfo;
	u32 addr, dt_enabled_cs = 0;
	struct spi_board_info *bd;
	unsigned int cs, i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(spi_ctls); i++) {
		dn = of_find_compatible_node(NULL, NULL, spi_ctls[i].compat);
		if (dn)
			break;
	}

	if (!dn)
		return 0;

	spi_bdinfo = kcalloc(spi_ctls[i].max_cs, sizeof(*spi_bdinfo),
			     GFP_KERNEL);
	if (!spi_bdinfo)
		return -ENOMEM;

	/* Scan for DT enabled SPI devices */
	for_each_available_child_of_node(dn, child) {
		ret = of_property_read_u32(child, "reg", &addr);
		if (ret)
			continue;

		dt_enabled_cs |= 1 << addr;
	}

	/* Populate SPI board info with non DT enabled SPI devices */
	for (cs = 0; cs < spi_ctls[i].max_cs; cs++) {
		bd = &spi_bdinfo[cs];

		/* Skip over DT enabled CS */
		if ((1 << cs) & dt_enabled_cs)
			continue;

		strcpy(bd->modalias, "nexus_spi_shim");
		bd->of_node = dn;
		bd->chip_select = cs;
		bd->max_speed_hz = 13500000;
	}

	ret = spi_register_board_info(spi_bdinfo, spi_ctls[i].max_cs);
	if (ret)
		pr_err("Failed to register SPI devices: %d\n", ret);

	/* spi_register_board_info copies the structure so this can be freed */
	kfree(spi_bdinfo);

	return ret;
}
arch_initcall(brcmstb_register_spi_devices);
