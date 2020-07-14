/* System Control and Management Interface (SCMI) BRCM Protocol
 *
 * Copyright (C) 2019, Broadcom Corporation
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

#include <linux/brcmstb/avs_dvfs.h>
#include <linux/brcmstb/brcmstb.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "../../../../firmware/arm_scmi/common.h"

#define SCMI_PROTOCOL_BRCM 0x80
#define SCMI_MAX_STRINGLEN 116
#define MAX_PSTATES 16

#define SEQ_PRINTF(m, x...)			\
do {						\
	if (m)					\
		seq_printf(m, x);		\
	else					\
		printk(x);			\
} while (0)

enum brcm_protocol_cmd {
	BRCM_SEND_AVS_CMD = 0x3,
	BRCM_CLK_SHOW_CMD = 0x4,
	BRCM_PMAP_SHOW_CMD = 0x5,
	BRCM_CLK_SHOW_NEW_CMD = 0x6,
	BRCM_RESET_ENABLE_CMD = 0x7,
	BRCM_RESET_DISABLE_CMD = 0x8,
	BRCM_OVERTEMP_RESET_CMD = 0x9,
};

static const char *pmap_cores[BCLK_SW_NUM_CORES - BCLK_SW_OFFSET] = {
	[BCLK_SW_CPU_CORE - BCLK_SW_OFFSET] = "cpu",
	[BCLK_SW_V3D - BCLK_SW_OFFSET] = "v3d0",
	[BCLK_SW_SYSIF - BCLK_SW_OFFSET] = "sysif0",
	[BCLK_SW_SCB - BCLK_SW_OFFSET] = "scb0",
	[BCLK_SW_HVD0 - BCLK_SW_OFFSET] = "hvd0",
	[BCLK_SW_RAAGA0 - BCLK_SW_OFFSET] = "raaga0",
	[BCLK_SW_VICE0 - BCLK_SW_OFFSET] = "vice0",
	[BCLK_SW_VICE0_PSS - BCLK_SW_OFFSET] = "vice0_pss",
	[BCLK_SW_VICE1 - BCLK_SW_OFFSET] = "vice1",
	[BCLK_SW_VICE1_PSS - BCLK_SW_OFFSET] = "vice1_pss",
	[BCLK_SW_XPT - BCLK_SW_OFFSET] = "xpt",
	[BCLK_SW_M2MC0 - BCLK_SW_OFFSET] = "m2mc0",
	[BCLK_SW_M2MC1 - BCLK_SW_OFFSET] = "m2mc1",
	[BCLK_SW_MIPMAP0 - BCLK_SW_OFFSET] = "mipmap0",
	[BCLK_SW_TSX0 - BCLK_SW_OFFSET] = "tsx0",
	[BCLK_SW_SMARTCARD0 - BCLK_SW_OFFSET] = "sc0",
	[BCLK_SW_SMARTCARD1 - BCLK_SW_OFFSET] = "sc1",
	[BCLK_SW_VPU0 - BCLK_SW_OFFSET] = "vpu0",
	[BCLK_SW_BNE0 - BCLK_SW_OFFSET] = "bne0",
	[BCLK_SW_ASP0 - BCLK_SW_OFFSET] = "asp0",
};

static const struct scmi_handle *handle;
static struct platform_device *cpufreq_dev;
static DEFINE_MUTEX(clk_api_mutex);

static void clk_api_lock(void)
{
	mutex_lock(&clk_api_mutex);
}

static void clk_api_unlock(void)
{
	mutex_unlock(&clk_api_mutex);
}

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

static int brcm_send_cmd_via_scmi(const struct scmi_handle *handle,
				  unsigned int cmd, unsigned int sub_cmd,
				  unsigned int protocol,
				  unsigned int num_in, unsigned int num_out,
				  u32 *params)
{
	int ret, ret_out;
	struct scmi_xfer *t;
	__le32 *p;
	int i, j = 0;

	if (!handle)
		return -ENODEV;

	if ((num_in || num_out) && !params)
		return -EINVAL;

	ret = scmi_one_xfer_init(handle, cmd, protocol,
				 sizeof(u32) * (num_in + 2),
				 sizeof(u32) * (num_out + 1), &t);
	if (ret)
		return ret;

	p = (__le32 *)t->tx.buf;
	if (cmd == BRCM_SEND_AVS_CMD) {
		/* First word is meta-info to be used by EL3 */
		p[0] = cpu_to_le32((num_out << 16) | (num_in << 8) | sub_cmd);
		/* Then the full AVS command */
		p[1] = cpu_to_le32(sub_cmd);
		j = 2;
	}

	for (i = 0; i < num_in; i++)
		p[i + j] = cpu_to_le32(params[i]);

	ret = scmi_do_xfer(handle, t);

	if (!ret) {
		p = t->rx.buf;
		ret_out = le32_to_cpu(p[0]);
		for (i = 0; i < num_out; i++)
			params[i] = (u32)le32_to_cpu(p[i + 1]);
	}

	scmi_one_xfer_put(handle, t);

	if (cmd == BRCM_SEND_AVS_CMD)
		ret = ret ? ret : avs_ret_to_linux_ret(ret_out);
	else
		ret = ret ? ret : ret_out;

	return ret;
}

static int brcm_send_avs_cmd_via_scmi(const struct scmi_handle *handle,
				      unsigned int sub_cmd, unsigned int num_in,
				      unsigned int num_out, u32 *params)
{
	int ret;

	clk_api_lock();
	ret = brcm_send_cmd_via_scmi(handle, BRCM_SEND_AVS_CMD, sub_cmd,
				     SCMI_PROTOCOL_BRCM, num_in, num_out,
				     params);
	clk_api_unlock();

	return ret;
}

static int brcm_send_show_cmd_via_scmi(struct seq_file *s,
				       const struct scmi_handle *handle,
				       unsigned int cmd)
{
	int state = 0;
	u32 params[SCMI_MAX_STRINGLEN/4 + 1]; /* state + string len */
	char *str = (char *) &params[0]; /* out */;

	do {
		params[0] = state; /* in */
		state = brcm_send_cmd_via_scmi(handle, cmd, 0,
					     SCMI_PROTOCOL_BRCM,
					     1, ARRAY_SIZE(params),
					     params);
		if (state <= 0)
			break;

		SEQ_PRINTF(s, "%s\n", str);
	} while (state);

	if (state == 0)
		/* last line if there is no scmi error */
		SEQ_PRINTF(s, "%s\n", str);

	return state ? state : 0;
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
				      &scmi_brcm_protocol_init, NULL);
}
subsys_initcall(scmi_brcm_init);

static int brcmstb_send_avs_cmd(unsigned int cmd, unsigned int in,
			    unsigned int out, u32 args[AVS_MAX_PARAMS])
{
	int ret = -ENODEV;

	if (handle)
		ret = brcm_send_avs_cmd_via_scmi(handle, cmd,
						 in, out, args);
	else if (cpufreq_dev)
		ret = brcmstb_issue_avs_command(cpufreq_dev, cmd,
						in, out, args);
	return ret;
}

int brcm_pmap_show(void)
{
	int ret;

	clk_api_lock();
	ret = brcm_send_show_cmd_via_scmi(NULL, handle, BRCM_PMAP_SHOW_CMD);
	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_pmap_show);

int brcm_pmap_num_pstates(unsigned int core_id, unsigned int *num_pstates)
{
	struct scmi_perf_ops *perf_ops;
	unsigned int domain = core_id - BCLK_SW_OFFSET;
	int ret;

	if (!handle)
		return -EINVAL;
	perf_ops = handle->perf_ops;
	if (core_id <= BCLK_SW_OFFSET || core_id >= BCLK_SW_NUM_CORES)
		return -EINVAL;

	clk_api_lock();
	ret = perf_ops->get_num_domain_opps(handle, domain, num_pstates);
	clk_api_unlock();

	return (ret == 0 && *num_pstates == 0) ? -EINVAL : ret;
}
EXPORT_SYMBOL(brcm_pmap_num_pstates);

int brcm_pmap_get_pstate(unsigned int core_id, unsigned int *pstate)
{
	struct scmi_perf_ops *perf_ops = handle->perf_ops;
	unsigned int domain = core_id - BCLK_SW_OFFSET;
	int ret;

	if (core_id <= BCLK_SW_OFFSET || core_id >= BCLK_SW_NUM_CORES)
		return -EINVAL;

	clk_api_lock();
	ret = perf_ops->level_get(handle, domain, pstate, false);
	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_pmap_get_pstate);

int brcm_pmap_set_pstate(unsigned int core_id, unsigned int pstate)
{

	struct scmi_perf_ops *perf_ops = handle->perf_ops;
	unsigned int domain = core_id - BCLK_SW_OFFSET;
	int ret;

	if (core_id <= BCLK_SW_OFFSET || core_id >= BCLK_SW_NUM_CORES)
		return -EINVAL;

	clk_api_lock();
	ret = perf_ops->level_set(handle, domain, pstate, false);
	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_pmap_set_pstate);

int brcm_pmap_get_pstate_freqs(unsigned int core_id, u32 *freqs)
{
	int ret;
	u32 freq_cnt, i;
	struct scmi_perf_ops *perf_ops = handle->perf_ops;
	unsigned int domain = core_id - BCLK_SW_OFFSET;

	if (core_id <= BCLK_SW_OFFSET || core_id >= BCLK_SW_NUM_CORES)
		return -EINVAL;

	ret = brcm_pmap_num_pstates(core_id, &freq_cnt);
	if (ret)
		return -EINVAL;

	clk_api_lock();
	ret = perf_ops->domain_freqs_get(handle, domain, freqs);
	/* sort frequencies in descending order */
	for (i = 0; i < freq_cnt/2; i++)
		swap(freqs[i], freqs[freq_cnt - i - 1]);

	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_pmap_get_pstate_freqs);

int brcm_reset_assert(unsigned int reset_id)
{
	int ret;
	u32 params = reset_id - BRST_SW_OFFSET;

	if (reset_id <= BRST_SW_OFFSET || reset_id >= BRST_SW_NUM_CORES)
		return -EINVAL;

	clk_api_lock();
	ret = brcm_send_cmd_via_scmi(handle, BRCM_RESET_ENABLE_CMD, 0,
				       SCMI_PROTOCOL_BRCM,
				       1, 0,
				       &params);
	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_reset_assert);

int brcm_reset_deassert(unsigned int reset_id)
{
	int ret;
	u32 params = reset_id - BRST_SW_OFFSET;

	if (reset_id <= BRST_SW_OFFSET || reset_id >= BRST_SW_NUM_CORES)
		return -EINVAL;

	clk_api_lock();
	ret = brcm_send_cmd_via_scmi(handle, BRCM_RESET_DISABLE_CMD, 0,
				       SCMI_PROTOCOL_BRCM,
				       1, 0,
				       &params);
	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_reset_deassert);

int brcm_overtemp_reset(unsigned int reset_temp)
{
	int ret;
	u32 params = reset_temp;

	clk_api_lock();
	ret = brcm_send_cmd_via_scmi(handle, BRCM_OVERTEMP_RESET_CMD, 0,
				     SCMI_PROTOCOL_BRCM,
				     1, 0,
				     &params);
	clk_api_unlock();

	return ret;
}
EXPORT_SYMBOL(brcm_overtemp_reset);

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>

static struct dentry *rootdir;

static int brcm_scmi_clk_summary_show(struct seq_file *s, void *data)
{
	int ret;

	clk_api_lock();
	ret = brcm_send_show_cmd_via_scmi(s, handle, BRCM_CLK_SHOW_NEW_CMD);
	clk_api_unlock();

	return ret;
}

static int brcm_scmi_pmap_show(struct seq_file *s, void *data)
{
	int ret;

	clk_api_lock();
	ret = brcm_send_show_cmd_via_scmi(s, handle, BRCM_PMAP_SHOW_CMD);
	clk_api_unlock();

	return ret;
}

/* Our debugfs helpers */
static int filp_to_core_id(struct file *filp)
{
	const char **p = filp->f_path.dentry->d_inode->i_private;

	return BCLK_SW_OFFSET + (p - &pmap_cores[0]);
}

static int uint_from_buf(const char *ubuf, size_t len, unsigned int *result)
{
	char buf[32];

	if (len > sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	return kstrtouint(buf, 10, result);
}

static int get_all_pmap_freq_info(int core_id, unsigned int *num_pstates,
				  unsigned int *cur_pstate, unsigned int *freqs)
{
	int ret;

	ret = brcm_pmap_num_pstates(core_id, num_pstates);
	if (ret < 0)
		return ret;
	if (*num_pstates > MAX_PSTATES)
		return -EINVAL;
	ret = brcm_pmap_get_pstate(core_id, cur_pstate);
	if (ret < 0)
		return ret;
	return brcm_pmap_get_pstate_freqs(core_id, freqs);
}


static int brcm_scmi_clk_summary_open(struct inode *inode, struct file *file)
{
	return single_open(file, brcm_scmi_clk_summary_show, inode->i_private);
}

static int brcm_scmi_pmap_open(struct inode *inode, struct file *file)
{
	return single_open(file, brcm_scmi_pmap_show, inode->i_private);
}

static ssize_t brcm_scmi_pmap_enable_wt(struct file *filp,
					const char __user *ubuf,
					size_t len, loff_t *offp)
{
	const int core_id = filp_to_core_id(filp);
	unsigned int en;
	int ret;

	ret = uint_from_buf(ubuf, len, &en);
	if (ret < 0)
		return ret;
	if (en)
		ret = brcm_clk_prepare_enable(core_id);
	else
		brcm_clk_disable_unprepare(core_id);

	return ret < 0 ? ret : len;
}

static ssize_t brcm_scmi_pmap_cur_freq_rd(struct file *filp, char __user *ubuf,
					  size_t count, loff_t *offp)
{
	int ret;
	char buf[32];
	unsigned int freqs[MAX_PSTATES], size, num_pstates, cur_pstate;
	const int core_id = filp_to_core_id(filp);

	ret = get_all_pmap_freq_info(core_id, &num_pstates, &cur_pstate,
				     freqs);
	if (ret < 0)
		return ret;
	size = snprintf(buf, sizeof(buf), "%u\n", freqs[cur_pstate]);
	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t brcm_scmi_pmap_cur_freq_wt(struct file *filp,
					  const char __user *ubuf,
					  size_t len, loff_t *offp)
{
	int ret, i;
	unsigned int freqs[MAX_PSTATES], num_pstates, cur_pstate, freq;
	const int core_id = filp_to_core_id(filp);

	ret = get_all_pmap_freq_info(core_id, &num_pstates, &cur_pstate,
				     freqs);
	if (ret < 0)
		return ret;
	ret = uint_from_buf(ubuf, len, &freq);
	for (i = 0; i < num_pstates; i++)
		if (freqs[i] == freq)
			break;
	if (i >= num_pstates)
		return -EINVAL;
	ret = brcm_pmap_set_pstate(core_id, i);
	if (ret < 0)
		return ret;
	return len;
}

static ssize_t brcm_scmi_pmap_all_freqs(struct file *filp, char __user *ubuf,
					size_t count, loff_t *offp)
{
	int ret, i;
	char buf[16 * MAX_PSTATES];
	unsigned int freqs[MAX_PSTATES], size, num_pstates, cur_pstate;
	const int core_id = filp_to_core_id(filp);

	ret = get_all_pmap_freq_info(core_id, &num_pstates, &cur_pstate, freqs);
	if (ret < 0)
		return ret;

	for (size = 0, i = 0; i < num_pstates; i++) {
		ret = snprintf(buf + size, sizeof(buf) - size, "%u\n",
			       freqs[i]);
		if (ret < 0)
			return ret;
		size += ret;
	}
	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t brcm_scmi_pmap_cur_pstate_rd(struct file *filp,
					    char __user *ubuf,
					    size_t count, loff_t *offp)
{
	int ret;
	char buf[32];
	unsigned int pstate, size;
	const int core_id = filp_to_core_id(filp);

	ret = brcm_pmap_get_pstate(core_id, &pstate);
	if (ret < 0)
		return ret;
	size = snprintf(buf, sizeof(buf), "%u\n", pstate);
	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}

static ssize_t brcm_scmi_pmap_cur_pstate_wt(struct file *filp,
					    const char __user *ubuf,
					    size_t len, loff_t *offp)
{
	int ret;
	unsigned int pstate;
	const int core_id = filp_to_core_id(filp);

	ret = uint_from_buf(ubuf, len, &pstate);
	if (ret < 0)
		return ret;
	ret = brcm_pmap_set_pstate(core_id, pstate);
	if (ret < 0)
		return ret;
	return len;
}

static ssize_t brcm_scmi_pmap_num_pstates(struct file *filp, char __user *ubuf,
					  size_t count, loff_t *offp)
{
	int ret;
	char buf[32];
	unsigned int num_pstates, size;
	const int core_id = filp_to_core_id(filp);

	ret = brcm_pmap_num_pstates(core_id, &num_pstates);
	if (ret < 0)
		return ret;
	size = snprintf(buf, sizeof(buf), "%u\n", num_pstates);
	return simple_read_from_buffer(ubuf, count, offp, buf, size);
}



static const struct file_operations brcm_scmi_clk_summary_fops = {
	.open		= brcm_scmi_clk_summary_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations brcm_scmi_pmap_fops = {
	.open		= brcm_scmi_pmap_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations brcm_scmi_pmap_enable_fops = {
	.write		= brcm_scmi_pmap_enable_wt,
};

static const struct file_operations brcm_scmi_pmap_cur_freq_fops = {
	.read		= brcm_scmi_pmap_cur_freq_rd,
	.write		= brcm_scmi_pmap_cur_freq_wt,
};

static const struct file_operations brcm_scmi_pmap_all_freqs_fops = {
	.read		= brcm_scmi_pmap_all_freqs,
};

static const struct file_operations brcm_scmi_pmap_cur_pstate_fops = {
	.read		= brcm_scmi_pmap_cur_pstate_rd,
	.write		= brcm_scmi_pmap_cur_pstate_wt,
};

static const struct file_operations brcm_scmi_pmap_num_pstates_fops = {
	.read		= brcm_scmi_pmap_num_pstates,
};

/**
 * brcm_scmi_debug_init - lazily populate the debugfs brcm_scmi directory
 *
 * clks are often initialized very early during boot before memory can be
 * dynamically allocated and well before debugfs is setup. This function
 * populates the debugfs brcm_scmi directory once at boot-time when we
 * know that debugfs is setup. It should only be called once at boot-time.
 */
static int brcm_scmi_debug_init(void)
{
	struct dentry *d, *d_cores;
	unsigned int i, n;

	rootdir = debugfs_create_dir("brcm-scmi", NULL);

	if (!rootdir)
		return -ENOMEM;

	d = debugfs_create_file("clk_summary", 0444, rootdir, NULL,
				&brcm_scmi_clk_summary_fops);

	if (!d)
		return -ENOMEM;

	d = debugfs_create_file("pmap", 0444, rootdir, NULL,
				&brcm_scmi_pmap_fops);

	if (!d)
		return -ENOMEM;

	d_cores = debugfs_create_dir("pmap_cores", rootdir);
	if (!d_cores)
		return -ENOMEM;

	for (i = 0; i < BCLK_SW_NUM_CORES - BCLK_SW_OFFSET; i++) {
		struct dentry *d_core;

		if (brcm_pmap_num_pstates(i + BCLK_SW_OFFSET, &n))
			continue;
		if (n == 0 || !pmap_cores[i])
			continue;
		d_core = debugfs_create_dir(pmap_cores[i], d_cores);
		if (!d_core)
			return -ENOMEM;

		d = debugfs_create_file("enable", 0644, d_core, &pmap_cores[i],
					&brcm_scmi_pmap_enable_fops);
		if (!d)
			return -ENOMEM;

		d = debugfs_create_file("cur_freq", 0644, d_core,
					&pmap_cores[i],
					&brcm_scmi_pmap_cur_freq_fops);
		if (!d)
			return -ENOMEM;

		d = debugfs_create_file("all_freqs", 0444, d_core,
					&pmap_cores[i],
					&brcm_scmi_pmap_all_freqs_fops);
		if (!d)
			return -ENOMEM;

		d = debugfs_create_file("cur_pstate", 0644, d_core,
					&pmap_cores[i],
					&brcm_scmi_pmap_cur_pstate_fops);
		if (!d)
			return -ENOMEM;

		d = debugfs_create_file("num_pstates", 0444, d_core,
					&pmap_cores[i],
					&brcm_scmi_pmap_num_pstates_fops);
		if (!d)
			return -ENOMEM;
	}



	return 0;
}
#endif

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

	ret = brcmstb_send_avs_cmd(AVS_CMD_GET_PSTATE, 1, 2, args);
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
		if (idx || clk_writes)
			ret = -EINVAL;
		else
			ret = brcmstb_issue_avs_command(cpufreq_dev,
							AVS_CMD_SET_PSTATE,
							num_in, 0, args);
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

	ret = brcmstb_send_avs_cmd(AVS_CMD_READ_DEBUG, 1, 2, args);
	if (ret)
		return ret;

	if (!ret)
		*value = args[1];

	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_avs_read_debug);

/**
 * brcmstb_stb_avs_get_pmic_info -- get PMIC information via EL3/AVS.
 *
 * @info: PMIC information (out).
 *
 * Return: 0 on success.
 */
int brcmstb_stb_avs_get_pmic_info(struct brcmstb_avs_pmic_info *info)
{
	u32 args[AVS_MAX_PARAMS];
	unsigned int i, byte_offset;
	int ret;

	ret = brcmstb_send_avs_cmd(AVS_CMD_GET_PMIC_INFO, 0, 4, args);
	if (ret)
		return ret;

	/* Now fill in the structure */
	info->num_pmic_devices = args[0];
	info->num_regulators = args[0] >> 8;
	info->num_gpios = args[0] >> 16;
	for (i = 0; i < ARRAY_SIZE(info->ext_infos); i++) {
		byte_offset = 8 * i;
		info->ext_infos[i].i2c_addr = args[1] >> byte_offset;
		info->ext_infos[i].chip_id = args[2] >> byte_offset;
		info->ext_infos[i].caps = args[3] >> byte_offset;
	}

	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_avs_get_pmic_info);

/**
 * brcmstb_stb_avs_set_pmic_config -- set PMIC configuration via EL3/AVS.
 *
 * @pmic: PMIC index (in)
 * @ovr_temp: over-temperature threshold (in)
 * @standby_regulators: regulators selected for S3_STANDBY
 *
 * Return: 0 on success.
 */
int brcmstb_stb_avs_set_pmic_config(u8 pmic,
				    u32 ovr_temp,
				    u32 standby_regulators)
{
	u32 num_in = 3, num_out = 1;
	u32 args[AVS_MAX_PARAMS];
	int ret;

	args[0] = pmic;
	args[1] = ovr_temp;
	args[2] = standby_regulators;

	ret = brcmstb_send_avs_cmd(AVS_CMD_SET_PMIC_CONFIG, num_in, num_out,
				   args);
	if (ret)
		return ret;

	if ((args[0] & 0xff) != pmic) {
		pr_err("Invalid PMIC return value: %d vs %d\n",
		       pmic, args[0]);
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_avs_set_pmic_config);

/**
 * brcmstb_stb_avs_get_pmic_status -- get PMIC status via EL3/AVS.
 *
 * @pmic: PMIC index (in)
 * @die_temp: PMIC die temperature (out)
 * @ext_therm_temp: External thermistor temperature (out)
 * @overall_power: Overall power consumption (out)
 *
 * Return: 0 on success.
 */
int brcmstb_stb_avs_get_pmic_status(u8 pmic,
				    u32 *die_temp,
				    u32 *ext_therm_temp,
				    u32 *overall_power)
{
	u32 num_in = 1, num_out = 4;
	u32 args[AVS_MAX_PARAMS];
	int ret;

	args[0] = pmic;

	ret = brcmstb_send_avs_cmd(AVS_CMD_GET_PMIC_STATUS, num_in, num_out,
				   args);
	if (ret)
		return ret;

	if ((args[0] & 0xff) != pmic) {
		pr_err("Invalid PMIC return value: %d vs %d\n",
		       pmic, args[0]);
		return -EINVAL;
	}

	*die_temp = args[1];
	*ext_therm_temp = args[2];
	*overall_power = args[3];

	return ret;
}
EXPORT_SYMBOL(brcmstb_stb_avs_get_pmic_status);

/**
 * brcmstb_stb_avs_get_pmic_reg_info -- get PMIC regulator configuration via
 * EL3/AVS.
 *
 * @pmic: PMIC index (in)
 * @die_temp: PMIC die temperature (out)
 * @ext_therm_temp: External thermistor temperature (out)
 * @overall_power: Overall power consumption (out)
 *
 * Return: 0 on success.
 */
int brcmstb_avs_get_pmic_reg_info(u8 regulator, u16 *nom_volt)
{
	u32 num_in = 1, num_out = 2;
	u32 args[AVS_MAX_PARAMS];
	int ret;

	args[0] = regulator;

	ret = brcmstb_send_avs_cmd(AVS_CMD_GET_PMIC_REG_INFO, num_in,
			       num_out, args);
	if (ret)
		return ret;

	if ((args[0] & 0xff) != regulator) {
		pr_err("Invalid regulator return value: %d vs %d\n",
		       regulator, args[0]);
		ret = -EINVAL;
	}

	*nom_volt = args[1];

	return ret;
}
EXPORT_SYMBOL(brcmstb_avs_get_pmic_reg_info);

/**
 * brcmstb_stb_avs_set_pmic_reg_config -- set PMIC regulator configuration via
 * EL3/AVS.
 *
 * @pmic: PMIC index (in)
 * @die_temp: PMIC die temperature (out)
 * @ext_therm_temp: External thermistor temperature (out)
 * @overall_power: Overall power consumption (out)
 *
 * Return: 0 on success.
 */
int brcmstb_avs_set_pmic_reg_config(u8 regulator, u16 voltage,
				    u16 over_current_thres)
{
	u32 num_in = 2, num_out = 1;
	u32 args[AVS_MAX_PARAMS];
	int ret;

	args[0] = regulator;
	args[1] = voltage;
	args[1] |= (u32)over_current_thres << 16;

	ret = brcmstb_send_avs_cmd(AVS_CMD_SET_PMIC_REG_CONFIG, num_in, num_out,
				   args);
	if (ret)
		return ret;

	if ((args[0] & 0xff) != regulator) {
		pr_err("Invalid regulator return value: %d vs %d\n",
		       regulator, args[0]);
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(brcmstb_avs_set_pmic_reg_config);

/**
 * brcmstb_stb_avs_get_pmic_reg_status -- get PMIC regulator status via
 * EL3/AVS.
 *
 * @pmic: PMIC index (in)
 * @die_temp: PMIC die temperature (out)
 * @ext_therm_temp: External thermistor temperature (out)
 * @overall_power: Overall power consumption (out)
 *
 * Return: 0 on success.
 */
int brcmstb_avs_get_pmic_reg_status(u8 regulator, u16 *voltage,
				    u16 *curr)
{
	u32 num_in = 1, num_out = 2;
	u32 args[AVS_MAX_PARAMS];
	int ret;

	args[0] = regulator;

	ret = brcmstb_send_avs_cmd(AVS_CMD_GET_PMIC_REG_STATUS, num_in, num_out,
			       args);
	if (ret)
		return ret;

	if ((args[0] & 0xff) != regulator) {
		pr_err("Invalid regulator return value: %d vs %d\n",
		       regulator, args[0]);
		ret = -EINVAL;
	}

	*voltage = args[1];
	*curr = args[1] >> 16;

	return ret;
}
EXPORT_SYMBOL(brcmstb_avs_get_pmic_reg_status);

static int brcm_scmi_dvfs_probe(struct scmi_device *sdev)
{
	int value = 0;

	handle = sdev->handle;

	if (!handle)
		return -ENODEV;

#ifdef CONFIG_DEBUG_FS
	brcm_scmi_debug_init();
#endif

	/* This tells AVS we are using the new API */
	(void)brcmstb_stb_avs_read_debug(0, &value);

	return 0;
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
