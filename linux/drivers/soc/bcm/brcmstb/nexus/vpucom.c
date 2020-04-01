/*
 * System Control and Management Interface (SCMI) VPUCom Protocol
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

#include <linux/brcmstb/brcmstb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

#include "../../../../firmware/arm_scmi/common.h"

#define SCMI_PROTOCOL_VPUCOM    0x82
#define VPUCOM_TIMEOUT_MS       100

enum vpucom_protocol_cmd {
	VPUCOM_SEND_VPU_MSG     = 0x3,
	VPUCOM_ASYNC_NOTIF      = 0x4,
};

struct vpucom_info {
	const struct scmi_handle *handle;
	brcmstb_vpucom_callback_fn_t callback;
};

static struct vpucom_info *info;

/**
 * brcmstb_vpucom_register_callback() - Register callback for async notify.
 *
 * @callback: callback function pointer
 *
 * Return: 0 on success
 */
int brcmstb_vpucom_register_callback(brcmstb_vpucom_callback_fn_t callback)
{
        if (!info)
		return -ENODEV;

	info->callback = callback;
	return 0;
}
EXPORT_SYMBOL(brcmstb_vpucom_register_callback);

/**
 * brcmstb_vpucom_unregister_callback() - Unregister callback for async notify.
 *
 * Return: 0 on success
 */
int brcmstb_vpucom_unregister_callback(void)
{
        if (!info)
		return -ENODEV;

	info->callback = NULL;
	return 0;
}
EXPORT_SYMBOL(brcmstb_vpucom_unregister_callback);

/**
 * brcmstb_vpucom_send_vpu_msg() - Send VPU msg, block and wait for reply.
 *
 * @pmsg: VPU message (input and output)
 * @msg_words: VPU message size in words
 *
 * Return: 0 on success
 */
int brcmstb_vpucom_send_vpu_msg(u32 *pmsg, size_t msg_words)
{
	int ret;
	const struct scmi_handle *handle;
	struct scmi_xfer *xfer;
	__le32 *p;
	size_t i;
	int timeout;

        if (!info || !info->handle)
		return -ENODEV;

	handle = info->handle;

	ret = scmi_one_xfer_init(handle,
				 VPUCOM_SEND_VPU_MSG,
				 SCMI_PROTOCOL_VPUCOM,
				 sizeof(u32) * (msg_words),
				 sizeof(u32) * (msg_words + 1), &xfer);
	if (ret)
		return ret;

	/* Copy outgoing VPU msg */
	p = (__le32 *)xfer->tx.buf;
	for (i = 0; i < msg_words; i++)
		p[i] = cpu_to_le32(pmsg[i]);

	ret = scmi_do_xfer(handle, xfer);

	if (ret == -ETIMEDOUT) {
		/*
                 * If SCMI driver timed out, continue to wait here.
                 * The default timeout in SCMI driver is 30ms which
                 * may NOT be enough in case of VPUCom.
                 */
		ret = 0;
		timeout = msecs_to_jiffies(VPUCOM_TIMEOUT_MS);
		if (!wait_for_completion_timeout(&xfer->done, timeout)) {
			dev_err(handle->dev, "VPU timed out in response\n");
			ret = -ETIMEDOUT;
		}
	}

	if (!ret) {
		/* Copy incoming VPU msg */
		p = (__le32 *)xfer->rx.buf;
		for (i = 0; i < msg_words; i++)
			pmsg[i] = (u32)le32_to_cpu(p[i]);
	}

	scmi_one_xfer_put(handle, xfer);
	return ret;
}
EXPORT_SYMBOL(brcmstb_vpucom_send_vpu_msg);

static void scmi_vpucom_callback(struct scmi_xfer *xfer)
{
	const struct scmi_handle *handle;
	size_t msg_words;
	u32 *pmsg;
	__le32 *p;
	size_t i;

        if (!info || !info->handle)
		return;

	handle = info->handle;

	/* Only async notify callback is supported */
	if (xfer->hdr.id != VPUCOM_ASYNC_NOTIF) {
		dev_err(handle->dev, "SCMI VPUCom msg %d callback unsupported",
			xfer->hdr.id);
		goto exit;
	}

	if (xfer->rx.len == 0) {
		dev_err(handle->dev, "SCMI VPUCom msg has zero length");
		goto exit;
	}

	/* Callback if registered */
	if (info->callback) {
		/* Convert incoming msg on spot */
		msg_words = xfer->rx.len / sizeof(u32);
		p = (__le32 *)xfer->rx.buf;
		pmsg = (u32 *)xfer->rx.buf;
		for (i = 0; i < msg_words; i++)
			pmsg[i] = (u32)cpu_to_le32(p[i]);
		info->callback(pmsg, msg_words);
	}
exit:
	scmi_one_xfer_put(handle, xfer);
}

static int scmi_vpucom_protocol_init(struct scmi_handle *handle)
{
	u32 version;

	scmi_version_get(handle, SCMI_PROTOCOL_VPUCOM, &version);

	dev_dbg(handle->dev, "SCMI VPUCom Version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	return 0;
}

static int __init scmi_vpucom_init(void)
{
	return scmi_protocol_register(SCMI_PROTOCOL_VPUCOM,
				      &scmi_vpucom_protocol_init,
				      &scmi_vpucom_callback);
}
subsys_initcall(scmi_vpucom_init);

static int scmi_vpucom_probe(struct scmi_device *sdev)
{
	struct device *dev = &sdev->dev;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (!sdev->handle)
		return -ENODEV;

	info->handle = sdev->handle;
	return 0;
}

static void scmi_vpucom_remove(struct scmi_device *sdev)
{
	info = NULL;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_VPUCOM },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver brcmstb_scmi_vpucom_drv = {
	.name		= "brcmstb-scmi-vpucom",
	.probe		= scmi_vpucom_probe,
	.remove		= scmi_vpucom_remove,
	.id_table	= scmi_id_table,
};
module_scmi_driver(brcmstb_scmi_vpucom_drv);

MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
