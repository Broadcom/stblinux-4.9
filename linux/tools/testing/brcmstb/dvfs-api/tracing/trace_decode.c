/******************************************************************************
 *  Copyright (C) 2019 Broadcom.
 *  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
 *
 *  This program is the proprietary software of Broadcom and/or its licensors,
 *  and may only be used, duplicated, modified or distributed pursuant to
 *  the terms and conditions of a separate, written license agreement executed
 *  between you and Broadcom (an "Authorized License").  Except as set forth in
 *  an Authorized License, Broadcom grants no license (express or implied),
 *  right to use, or waiver of any kind with respect to the Software, and
 *  Broadcom expressly reserves all rights in and to the Software and all
 *  intellectual property rights therein. IF YOU HAVE NO AUTHORIZED LICENSE,
 *  THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY WAY, AND SHOULD
 *  IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF THE SOFTWARE.
 *
 *  Except as expressly set forth in the Authorized License,
 *
 *  1.     This program, including its structure, sequence and organization,
 *  constitutes the valuable trade secrets of Broadcom, and you shall use all
 *  reasonable efforts to protect the confidentiality thereof, and to use this
 *  information only in connection with your use of Broadcom integrated circuit
 *  products.
 *
 *  2.     TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
 *  "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS
 *  OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *  RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *  IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS FOR
 *  A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS, QUIET
 *  ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU ASSUME
 *  THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 *
 *  3.     TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM
 *  OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL,
 *  INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY
 *  RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM
 *  HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN
 *  EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1,
 *  WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY
 *  FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 ******************************************************************************/

#include <stdio.h>
#include "scmi_private.h"
#include "trace.h"

#define STR_ARRAY_SIZE(__strs) (sizeof(__strs)/sizeof(__strs[0]))

static const char *proto_to_str(int proto)
{
	static const char *strs[] = { "Base", "Power", "System", "Perf",
				      "Clock", "Sensor", };
	static const int N = STR_ARRAY_SIZE(strs);
	int idx = proto - SCMI_PROTOCOL_BASE;

	if (proto == 0x80)
		return "Brcm";
	if (proto == 0x82)
		return "VPUCom";
	if (idx < 0 || idx >= N)
		return "unknown";
	else
		return strs[idx];
}

static const char  *msg_id_to_str(int proto, unsigned int id)
{
	static const char *base_strs[]
		= { "Version", "Attrs", "Msg_attrs", "Vendor", "Sub_vendor",
		    "Impl_version", "List_protos", "Agent", "Notify_errors", };
	static const char *perf_strs[]
		= { "Version", "Attrs", "Msg_attrs", "Dom_attrs",
		    "Desc_levels", "Limits_set", "Limits_get", "Level_set",
		    "Level_get", "Notify_limits", "Notify_level", };
	static const char *sens_strs[]
		= { "Version", "Attrs", "Msg_attrs", "Desc_get",
		    "Trip_Pt_notify", "Trip_Pt_config", "Reading_get",
		    "Trip_Pt_event", };
	static const char *clock_strs[]
		= { "Version", "Attrs", "Msg_attrs", "Clk_attrs", "Desc_rates",
		    "Rate_set", "Rate_get", "Config_set", "Rate_set_cmplt", };
	static const char *brcm_strs[]
		= { "Version", "Attrs", "Msg_attrs", "Send_AVS_Cmd",
		    "Clk_ShowV1", "Pmap_Show", "Clk_ShowV2", "Reset_Enable",
		    "Reset_Disable", "Overtemp_Reset", "Stats_Show",
		    "Stats_Reset", "Trace_Enable", "Trace_Show"};
	static const char *vpucom_strs[]
		= { "Version", "Attrs", "Msg_attrs", "Send_VPU_Msg",
		    "Async_Notify"};

	const unsigned int base_num = STR_ARRAY_SIZE(base_strs);
	const unsigned int perf_num = STR_ARRAY_SIZE(perf_strs);
	const unsigned int sens_num = STR_ARRAY_SIZE(sens_strs);
	const unsigned int clock_num = STR_ARRAY_SIZE(clock_strs);
	const unsigned int brcm_num = STR_ARRAY_SIZE(brcm_strs);
	const unsigned int vpucom_num = STR_ARRAY_SIZE(vpucom_strs);

	if (proto == SCMI_PROTOCOL_BASE && id < base_num)
		return base_strs[id];
	else if (proto == SCMI_PROTOCOL_PERF && id < perf_num)
		return perf_strs[id];
	else if (proto == SCMI_PROTOCOL_SENSOR && id < sens_num)
		return sens_strs[id];
	else if (proto == SCMI_PROTOCOL_CLOCK && id < clock_num)
		return clock_strs[id];
	else if (proto == SCMI_PROTOCOL_BRCM && id < brcm_num)
		return brcm_strs[id];
	else if (proto == SCMI_PROTOCOL_VPUCOM && id < vpucom_num)
		return vpucom_strs[id];
	else
		return "Unknown";
}

static const char *event_type_to_str(enum event_type evtype)
{
	static const char *evtype_strs[]
		= {"Scmi", "Psci", "Avs", "Cpufreq", "String_io"};

	int n = STR_ARRAY_SIZE(evtype_strs);

	if (evtype < 0 || evtype >= n)
		return "unknown";
	else
		return evtype_strs[evtype];
}

static const char *event_source_to_str(enum event_source evsrc)
{
	static const char *evsrc_strs[] = {"Linux", "Astra", "Vpu"};
	int n = STR_ARRAY_SIZE(evsrc_strs);

	if (evsrc < 0 || evsrc >= n)
		return "unknown";
	else
		return evsrc_strs[evsrc];
}

static const char *event_attr_to_str(enum event_source evattr)
{
	static const char *evattr_strs[] = {"L", "I", "O"};
	int n = STR_ARRAY_SIZE(evattr_strs);

	if (evattr < 0 || evattr >= n)
		return "unknown";
	else
		return evattr_strs[evattr];
}

void prn_event_decoded(struct trace_header *th, struct event *evp,
		       uint32_t seqnum)
{
	int i;
	uint64_t ts = uint48_get(&evp->ts_event.t[0]);
	uint64_t time_us = (1000 * 1000 * ts/th->clk_freq);

	printf("<%s:%d> %llu %s:%s ",
	       event_attr_to_str(evp->ev_attrib),
	       seqnum,
	       (unsigned long long)time_us,
	       event_source_to_str(EVID_SOURCE(evp->event_id)),
	       event_type_to_str(EVID_TYPE(evp->event_id)));

	if (EVID_TYPE(evp->event_id) == STRING_IO) {
		printf(" str: %s\n", (char *) evp->params);
	} else {
		printf("%s %s ", proto_to_str(evp->params[0]),
		       msg_id_to_str(evp->params[0], evp->params[1]));
		if (evp->params[0] == SCMI_PROTOCOL_CLOCK &&
		    evp->params[1] == 7) {
			printf("clk_%sable(%s)",
			       (evp->params[3] & 0x1) ? "en" : "dis",
			       trace_decode_clk_name(evp->params[2]));
		} else {
			printf("[ ");
			for (i = 2; i < evp->num_params; i++)
				printf("%d ", evp->params[i]);
			printf("]\n");
		}
	}
}
