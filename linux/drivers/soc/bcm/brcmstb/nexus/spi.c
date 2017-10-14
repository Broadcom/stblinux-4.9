/*
 * Nexus SPI SHIM registration
 *
 * Copyright (C) 2017, Broadcom
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

#define UPG_MSPI_MAX_CS		4

static struct spi_board_info mspi_bdinfo[UPG_MSPI_MAX_CS];

static int __init brcmstb_register_spi_devices(void)
{
	struct device_node *dn, *child;
	u32 addr, dt_enabled_cs = 0;
	struct spi_board_info *bd;
	unsigned int cs;
	int ret;

	/* Lookup the UPG_MSPI controller */
	dn = of_find_compatible_node(NULL, NULL, "brcm,spi-brcmstb-mspi");
	if (!dn)
		return 0;

	/* Scan for DT enabled SPI devices */
	for_each_available_child_of_node(dn, child) {
		ret = of_property_read_u32(child, "reg", &addr);
		if (ret)
			continue;

		dt_enabled_cs |= 1 << addr;
	}

	/* Populate SPI board info with non DT enabled SPI devices */
	for (cs = 0; cs < UPG_MSPI_MAX_CS; cs++) {
		bd = &mspi_bdinfo[cs];

		/* Skip over DT enabled CS */
		if ((1 << cs) & dt_enabled_cs)
			continue;

		strcpy(bd->modalias, "nexus_spi_shim");
		bd->of_node = dn;
		bd->chip_select = cs;
		bd->max_speed_hz = 13500000;
	}

	ret = spi_register_board_info(mspi_bdinfo, ARRAY_SIZE(mspi_bdinfo));
	if (ret)
		pr_err("Failed to register SPI devices: %d\n", ret);

	return ret;
}
arch_initcall(brcmstb_register_spi_devices);
