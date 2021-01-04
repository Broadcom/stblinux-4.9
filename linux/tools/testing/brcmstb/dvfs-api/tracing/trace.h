/******************************************************************************
 *  Copyright (C) 2019-2020 Broadcom.
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
#ifndef __TRACE_H_
#define __TRACE_H_

#include <stdint.h>

#define TRC_MAGIC_WORD		0x54524342  /* trace buffer "TRCB" */
#define TRC_MINOR_VER		2
#define TRC_MAJOR_VER		0
#define TRC_EVENT_PARAMS_MAX	20

/* event id source and type conversion macros */
#define EVID(__src, __typ)	(__src | (__typ << 4))
#define EVID_FLAG(__evid)	(1 << (__evid & 0x0f))
#define EVID_SOURCE(__evid)	(__evid & 0x0f)
#define EVID_TYPE(__evid)	((__evid & 0xf0) >> 4)

enum event_type {
	SCMI,
	PSCI,
	AVS,
	CPUFREQ,
	STRING_IO
};

enum event_source {
	LINUX,
	ASTRA,
	VPU,
};

enum event_attribute {
	EV_LOG,
	EV_ENTER,
	EV_EXIT,
};

struct trace_header {
	uint32_t magic_word;
	uint16_t major_ver;
	uint16_t minor_ver;
	uint32_t trcbuf_size;
	uint32_t trcbuf_offset;
	uint32_t trcbuf_head_offset;
	uint32_t trcbuf_tail_offset;
	uint32_t trcbuf_head_seqnum;
	uint8_t	 event_sz;
	uint8_t  event_params_max;
	uint16_t flags;
	uint32_t clk_freq;
	uint8_t reserved[16];
	uint32_t crc32;
} __attribute__ ((aligned(4)));

struct ts48 {
	uint16_t t[3];
};

struct event {
	struct ts48 ts_event;
	uint16_t ev_attrib;	/* event trace point attribute */
	uint16_t ev_desc;	/* event specific data */
	uint8_t event_id;	/* type source encoded in 1 byte */
	uint8_t num_params;
	uint32_t params[TRC_EVENT_PARAMS_MAX];
} __attribute__ ((aligned(4)));

struct trace_info {
	struct trace_header *trc_hdr;
	struct event *start_ev_ptr;
	struct event *end_ev_ptr;
	struct ts48 cur_ev_ts;
	struct event *head;
	struct event *tail;
	uint32_t trace_on;
};

static inline uint64_t uint48_get(const uint16_t *a16)
{
	return a16[0] | (uint32_t) a16[1] << 16 | (uint64_t) a16[2] << 32;
}

static inline void uint48_set(uint16_t *a16, uint64_t value)
{
	a16[0] = (uint16_t) value;
	a16[1] = (uint16_t) (value >> 16);
	a16[2] = (uint16_t) (value >> 32);
}

static inline uint8_t trace_header_size(void)
{
	return sizeof(struct trace_header);
}

static inline uint8_t trace_event_size(struct trace_header const *th)
{
	return th->event_sz;
}

static inline uint8_t trace_event_max_params(struct trace_header *th)
{
	return th->event_params_max;
}

void prn_event_decoded(struct trace_header *th, struct event *evp,
		       uint32_t seqnum);
struct trace_info *get_trace_info(void);

/* Clock name decode functions */
int trace_decode_clk_name_init(const char *filename);
void trace_decode_clk_name_unint(void);
const char *trace_decode_clk_name(unsigned int clk_id);
#endif /* _TRACE_H_ */
