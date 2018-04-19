/*
 * System Control and Management Interface (SCMI) Performance Protocol
 *
 * Copyright (C) 2017 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/brcmstb/avs_dvfs.h>
#include <linux/brcmstb/brcmstb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>

#include "../../../../firmware/arm_scmi/common.h"

#define SCMI_PROTOCOL_BRCM 0x80

enum brcm_protocol_cmd {
	BRCM_SEND_AVS_CMD = 0x3,
};

static const struct scmi_handle *handle;
static struct platform_device *cpufreq_dev;

static int avs_ret_to_linux_ret(int avs_ret)
{
	int ret;

	/* Convert firmware errors to errno's as much as possible. */
	switch (avs_ret) {
	case AVS_STATUS_SUCCESS:
		ret = 0;
		break;
	case AVS_STATUS_INVALID:
		ret = -EINVAL;
		break;
	case AVS_STATUS_NO_SUPP:
		ret = -ENOTSUPP;
		break;
	case AVS_STATUS_NO_MAP:
		ret = -ENOENT;
		break;
	case AVS_STATUS_MAP_SET:
		ret = -EEXIST;
		break;

	default:
	case AVS_STATUS_FAILURE:
		ret = -EIO;
		break;
	}

	return ret;
}

static int brcm_send_avs_cmd_via_scmi(const struct scmi_handle *handle,
				      unsigned int cmd, unsigned int num_in,
				      unsigned int num_out, u32 *params)
{
	int ret, avs_ret;
	unsigned int i;
	struct scmi_xfer *t;
	__le32 *p;

	ret = scmi_one_xfer_init(handle, BRCM_SEND_AVS_CMD, SCMI_PROTOCOL_BRCM,
				 sizeof(u32) * (num_in + 2),
				 sizeof(u32) * (num_out + 1), &t);
	if (ret)
		return ret;

	p = (__le32 *)t->tx.buf;
	/* First word is meta-info to be used by EL3 */
	p[0] = cpu_to_le32((num_out << 16) | (num_in << 8) | cmd);
	/* Then the full AVS command */
	p[1] = cpu_to_le32(cmd);
	for (i = 0; i < num_in; i++)
		p[i + 2] = cpu_to_le32(params[i]);

	ret = scmi_do_xfer(handle, t);

	if (!ret) {
		p = t->rx.buf;
		avs_ret = le32_to_cpu(p[0]);
		for (i = 0; i < num_out; i++)
			params[i] = (u32)le32_to_cpu(p[i + 1]);
	}

	scmi_one_xfer_put(handle, t);

	return ret ? ret : avs_ret_to_linux_ret(avs_ret);
}

static int scmi_brcm_protocol_init(struct scmi_handle *handle)
{
	u32 version;

	scmi_version_get(handle, SCMI_PROTOCOL_BRCM, &version);

	dev_dbg(handle->dev, "Brcm SCMI Version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	return 0;
}

static int __init scmi_brcm_init(void)
{
	return scmi_protocol_register(SCMI_PROTOCOL_BRCM,
				      &scmi_brcm_protocol_init);
}
subsys_initcall(scmi_brcm_init);

/**
 * brcmstb_stb_dvfs_get_pstate() - Get the pstate for a core/island.
 *
 * @idx: index; 0 == cpu/combined, 1 == reserved, 2 == HVD core, ...) (in).
 * @pstate: the current pstate (out).
 * @info: four values, each taking a byte: [31:24] reserved, [23:16] num
 *     cores, [15:8] num pstates, [7:0] idx given (out).
 *
 * Return: 0 on success
 */
int brcmstb_stb_dvfs_get_pstate(unsigned int idx, unsigned int *pstate,
				u32 *info)
{
	u32 args[AVS_MAX_PARAMS];
	int ret = -ENODEV;

	args[0] = idx;

	if (handle) {
		ret = brcm_send_avs_cmd_via_scmi(handle, AVS_CMD_GET_PSTATE,
						 1, 2, args);
	} else if (cpufreq_dev) {
		ret = brcmstb_issue_avs_command(
			cpufreq_dev, AVS_CMD_GET_PSTATE, 1, 2, args);
	}
	if (!ret) {
		*pstate = args[0];
		*info = args[1];
	}
	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_dvfs_get_pstate);

/**
 * brcmstb_stb_dvfs_set_pstate() -- Set the pstate for a core/island.
 *
 * @idx: index; 0 == cpu/combined, 1 == reserved, 2 == HVD core, ... (in).
 * @pstate: desired pstate (in).
 * @clk_writes -- the number of clocks regs to write [0..3] (in).
 * @clk_params: array of (3*num_clk_writes) u32s; every set of
 *     three u32s is { addr, data, mask } of a clock register write (in).
 *
 *  Return: 0 on success.
 */
int brcmstb_stb_dvfs_set_pstate(unsigned int idx, unsigned int pstate,
				unsigned int clk_writes,
				const u32 *clk_params)
{
	u32 args[AVS_MAX_PARAMS];
	unsigned int i, j, num_in;
	int ret = -ENODEV;

	args[0] = (pstate & 0xff) | ((idx & 0xff) << 8)
		| ((clk_writes & 0xff) << 16);
	for (i = 0, num_in = 1; i < clk_writes; i++)
		for (j = 0; j < 3; j++, num_in++)
			args[3 * i + 1 + j] = clk_params[3 * i + j];
	if (handle) {
		ret = brcm_send_avs_cmd_via_scmi(handle, AVS_CMD_SET_PSTATE,
						 num_in, 0, args);
	} else if (cpufreq_dev) {
		ret = brcmstb_issue_avs_command(
			cpufreq_dev, AVS_CMD_GET_PSTATE, num_in, 0, args);
	}
	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_dvfs_set_pstate);

/**
 * brcmstb_stb_avs_read_debug() -- get debug value via EL3/AVS.
 *
 * @debug_idx: see AVS API documentation (in).
 * @value: value of the indicated debug_idx (out).
 *
 * Return: 0 on success.
 */
int brcmstb_stb_avs_read_debug(unsigned int debug_idx, u32 *value)
{
	u32 args[AVS_MAX_PARAMS];
	int ret = -ENODEV;

	args[0] = debug_idx;

	if (handle) {
		ret = brcm_send_avs_cmd_via_scmi(handle, AVS_CMD_READ_DEBUG,
						 1, 2, args);
	} else if (cpufreq_dev) {
		ret = brcmstb_issue_avs_command(
			cpufreq_dev, AVS_CMD_GET_PSTATE, 1, 2, args);
	}

	if (!ret)
		*value = args[1];

	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_avs_read_debug);

static int brcm_scmi_dvfs_probe(struct scmi_device *sdev)
{
	int ret, value;

	handle = sdev->handle;

	if (!handle)
		return -ENODEV;

	/* This tells AVS we are using the new API */
	ret = brcmstb_stb_avs_read_debug(0, &value);

	return ret;
}

static void brcm_scmi_dvfs_remove(struct scmi_device *sdev)
{
}

static const struct scmi_device_id brcm_scmi_id_table[] = {
	{ SCMI_PROTOCOL_BRCM },
	{ },
};
MODULE_DEVICE_TABLE(scmi, brcm_scmi_id_table);

static int __init get_brcm_avs_cpufreq_dev(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "brcm,avs-cpu-data-mem");
	cpufreq_dev = np ? of_find_device_by_node(np) : NULL;
	return 0;
}

late_initcall(get_brcm_avs_cpufreq_dev);

static struct scmi_driver brcmstb_scmi_dvfs_drv = {
	.name		= "brcmstb-scmi-dvfs",
	.probe		= brcm_scmi_dvfs_probe,
	.remove		= brcm_scmi_dvfs_remove,
	.id_table	= brcm_scmi_id_table,
};
module_scmi_driver(brcmstb_scmi_dvfs_drv);

MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
