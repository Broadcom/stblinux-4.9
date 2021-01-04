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
#include <stdlib.h>
#include <ctype.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include "trace.h"

#define to_evp(__ev, __offset)				\
	((void *) ((uint8_t *) __ev + __offset))

#define header_to_ev_headp(__hp, __offset)		\
	((void *) ((uint8_t *)__hp + __offset))

enum debugfs_file {
	CLK_SUM,
	TRC_BUF_SIZE,
	TRC_BUF_ADDR,
};

static char *dfs[] = {
	"/sys/kernel/debug/brcm-scmi/clk_summary",
	"/sys/kernel/debug/brcm-trace/buf_size",
	"/sys/kernel/debug/brcm-trace/phys_addr64",
};

enum trcbuf_src {
	LOGFILE,
	LOGMEM
};

struct options {
	uint64_t phys_addr;
	uint32_t buf_size;
	char *of_name;
	char *if_name;
	int decode;
	int dump;
	int auto_rd;
	const char *clk_file;
};

static struct trace_info trcinfo;

struct trace_info *get_trace_info(void)
{
	return &trcinfo;
}

struct event *trace_get_next_event(struct trace_info *ti,
				   struct event *cur_ev)
{
	struct event *next_ev = NULL;
	struct trace_header *th = ti->trc_hdr;

	/* empty trace */
	if ((ti->tail == ti->head) && uint48_get(ti->cur_ev_ts.t) == 0)
		return NULL;

	/* roll over */
	if (cur_ev == ti->end_ev_ptr)
		next_ev = ti->start_ev_ptr;
	else
		next_ev = to_evp(cur_ev, trace_event_size(th));

	return next_ev;
}

static void prn_trace_info(struct trace_info *ti)
{
	printf("trace: start %p end %p head %p tail %p\n",
	       ti->start_ev_ptr, ti->end_ev_ptr,
	       ti->head, ti->tail);

}

static void trace_iterate_events(struct trace_info *ti)
{
	struct event *next_evp = ti->head;
	struct trace_header *th = ti->trc_hdr;
	uint32_t ev_seqnum = th->trcbuf_head_seqnum;

	prn_trace_info(ti);
	while (uint48_get(next_evp->ts_event.t)) {
		prn_event_decoded(th, next_evp, ev_seqnum++);

		if (ti->tail == next_evp)
			break;

		next_evp = trace_get_next_event(ti, next_evp);
	}
}

static void trace_events_print(void)
{
	struct trace_info *ti;

	ti = get_trace_info();
	if (ti->head == ti->tail)// && uint48_get(ti->head->ts_event.t) == 0)
		printf("trace empty %p %x %llu %d\n", ti->trc_hdr,
		       ti->trc_hdr->flags,
		       (unsigned long long) uint48_get(ti->cur_ev_ts.t),
		       ti->trace_on);
	else
		trace_iterate_events(ti);
}

static int trace_log_from_mem(void)
{
	trace_events_print();
	return 0;
}

static void *trace_mmap_trcbuf(uintptr_t trcbuf_paddr, uint32_t trcbuf_size)
{
	void *mem = NULL;
	int fd;

	printf("%s: %llx %d\n", __func__,
	       (unsigned long long)trcbuf_paddr, trcbuf_size);
	if (trcbuf_paddr && trcbuf_size) {
		if (trcbuf_size < trace_header_size()) {
			fprintf(stderr, "trcbuf too small\n");
			goto map_out;
		}

		fd = open("/dev/mem", O_RDWR | O_SYNC);
		if (fd < 0) {
			fprintf(stderr, "can't open /dev/mem\n");
			goto map_out;
		}

		mem = mmap(NULL, trcbuf_size, PROT_READ,
			   MAP_SHARED, fd, trcbuf_paddr);

		close(fd);

		if (mem == MAP_FAILED) {
			fprintf(stderr, "mmap failed\n");
			mem = NULL;
		}
	}

map_out:
	return mem;
}

static void trace_munmap_trcbuf(void *paddr, uint32_t trcbuf_size)
{
	munmap(paddr, trcbuf_size);
}

static int prn_trace_header(struct trace_header *th, enum trcbuf_src src)
{
	char buf[5];
	uint32_t magic = th->magic_word;

	if (magic != TRC_MAGIC_WORD) {
		fprintf(stderr, "tracelog not initialized\n");
		return -1;
	}

	magic = __builtin_bswap32(th->magic_word);
	memset(buf, 0, sizeof(buf));
	memcpy(buf, &magic, 4);

	printf("%s: ver%d.%d %s\n", buf, th->major_ver, th->minor_ver,
		src == LOGFILE ? "[file]" : "[memory]");
	printf("trcbsz %d offset 0x%x flags 0x%x tsfreq %uHz ts in usec\n",
	       th->trcbuf_size, th->trcbuf_offset, th->flags, th->clk_freq);
	printf("headersz %d eventsz %d\n", trace_header_size(),
	       trace_event_size(th));

	return 0;
}

static void trace_info_init(struct trace_header *th, uint32_t trcbuf_size)
{
	void *evp;
	struct trace_info *ti = get_trace_info();
	uint32_t bufsz, trc_bufsz;
	uint32_t th_sz = trace_header_size();
	uint32_t ev_sz = trace_event_size(th);
	uint16_t *t;
	uint64_t ts;

	memset(ti, 0, sizeof(struct trace_info));

	bufsz = trcbuf_size;
	/* only event trace buffer size aligned to event size */
	trc_bufsz = ((bufsz - th_sz) / ev_sz) * ev_sz;
	/* initialize the trace event info */
	ti->trc_hdr = th;
	evp = header_to_ev_headp(th, th_sz);
	ti->start_ev_ptr = to_evp(evp, 0);
	ti->end_ev_ptr =  to_evp(evp, (trc_bufsz - ev_sz));
	ti->head = to_evp(ti->start_ev_ptr, th->trcbuf_head_offset);
	ti->tail = to_evp(ti->start_ev_ptr, th->trcbuf_tail_offset);

	t =  &ti->head->ts_event.t[0];
	ts = uint48_get(t);
	t =  &ti->cur_ev_ts.t[0];
	uint48_set(t, ts);
}

static int trace_init(uintptr_t trcbuf_paddr, uint32_t trcbuf_size)
{
	struct trace_header *th;
	uint8_t *trcbuf;
	int sts = 0;

	trcbuf = (uint8_t *) trace_mmap_trcbuf(trcbuf_paddr, trcbuf_size);
	if (!trcbuf)
		return -1;

	th = (struct trace_header *) trcbuf;
	sts = prn_trace_header(th, LOGMEM);

	if (sts == 0)
		trace_info_init(th, trcbuf_size);

	return sts;
}

static void usage(char *progname)
{
	printf("\nUSAGE: %s [OPTION]... [FILE]\n\n"
	       "example 1: dumping trace buffer to binary file on/from target\n"
	       "     %s -a | [-p <phys addr> -l <length>] -o <output file>\n\n"
	       "example 2: decoding binary trace buffer on target\n"
	       "     %s -d -a | [-p <phys addr> -l <length>]\n\n"
	       "example 3: decoding binary trace buffer input file on host\n"
	       "     %s -d -i <input file> [-c <target-clk-summary-file>]\n\n"
	       "Options with arguments:\n"
	       "-p     64-bit target physical address where logs are stored\n"
	       "-l     length of the trace buffer\n"
	       "-i     input file name containg logs for decoding\n"
	       "-o     output file name to dump trace buffer data to\n"
	       "-c     target clk-summary file\n"
	       "Options:\n"
	       "-d     decode input trace buffer file\n"
	       "-a     skims debugfs files for trace buffer addr, length\n"
	       "       replaces -p & -l option\n"
	       "\n", basename(progname), basename(progname), basename(progname),
	       basename(progname));
}

static int debugfs_get_val(char *fname, size_t width, uint64_t *val)
{
	FILE *f = fopen(fname, "r");

	if (f == NULL) {
		fprintf(stderr, "file %s does not exist?!\n", fname);
		return -1;
	}

	if (width == 4)
		fscanf(f, "%u", (unsigned int *) val);
	else if (width == 8)
		fscanf(f, "%llx", (unsigned long long *) val);

	fclose(f);

	return 0;
}

int parse_options(int argc, char **argv, struct options *opts)
{
	int index;
	int c, sts = 0;

	opterr = 0;

	if (!opts)
		return -1;

	memset(opts, 0, sizeof(struct options));

	while ((c = getopt(argc, argv, "p:l:i:o:sc:dha")) != -1) {
		switch (c) {
		case 'p':
			opts->phys_addr = strtoll(optarg, NULL, 16);
			break;
		case 'l':
			opts->buf_size  = atoi(optarg);
			break;
		case 's':
			opts->dump = 1;
			break;
		case 'c':
			opts->clk_file = optarg;
			break;
		case 'd':
			opts->decode = 1;
			break;
		case 'i':
			opts->if_name = optarg;
			break;
		case 'o':
			opts->of_name = optarg;
			break;
		case 'h':
			usage(argv[0]);
			sts = -1;
			break;
		case 'a':
			opts->auto_rd = 1;
			debugfs_get_val(dfs[TRC_BUF_ADDR], 8, &opts->phys_addr);
			debugfs_get_val(dfs[TRC_BUF_SIZE], 4,
					(uint64_t *)&opts->buf_size);
			break;
		case '?':
			if (optopt == 'o' || optopt == 'i' || optopt == 'p' ||
			    optopt == 'l')
				fprintf(stderr, "Option -%c requires an "
						"argument.\n", optopt);
			else if (isprint (optopt))
				fprintf(stderr, "Unknown option `-%c'.\n",
					 optopt);
			else
				fprintf(stderr,
					 "Unknown option character `\\x%x'.\n",
					 optopt);
			sts = -1;
			break;
		default:
			usage(argv[0]);
			abort();
		}
	}

	sts = -1;
	/* validate arguments */
	if (opts->if_name && opts->of_name) {
		fprintf(stderr, "cannot specify both input and output files\n");
	} else if (opts->dump && (!opts->of_name || (!opts->auto_rd ||
				(!opts->phys_addr && !opts->buf_size)))) {
		fprintf(stderr, "please provide output file name with "
			"option along with phy addr and buffer length\n");
	} else if (opts->decode) {
		if (!opts->if_name && (!opts->phys_addr || !opts->buf_size ||
			    !opts->auto_rd))
			fprintf(stderr, "need input file name or trace buffer "
					"address, length\n");
		else if (opts->if_name && (opts->phys_addr || opts->buf_size))
			fprintf(stderr, "invalid arguments phys_addr or "
					"length\n");
		else
			sts = 0;
	} else
		sts = 0;

	for (index = optind; index < argc; index++) {
		fprintf(stderr, "Non-option argument %s\n", argv[index]);
		sts = -1;
	}

	if (sts != 0) {
		fprintf(stderr, "bad options for %s\n", basename(argv[0]));
		usage(argv[0]);
	}

	return sts;
}

static int dump_to_file(char *of_name, char *buf, int length)
{
	int outfd;
	int ret = 0;

	outfd = open(of_name, O_WRONLY | O_CREAT);
	if (outfd < 0) {
		perror("file open failed");
		return errno;
	}

	ret = write(outfd, buf, length);
	if (ret < length)
		fprintf(stderr, "failed to write output file %s\n", of_name);

	close(outfd);

	printf("Wrote %d bytes log buffer to %s\n", ret, of_name);

	return ret;
}

static int tracelog_from_file(char *if_name)
{
	int infd;
	int ret;
	struct event ev;
	uint32_t ev_seqnum;
	off_t start, end, head, tail;
	uint8_t trc_ev_sz;
	struct trace_header th;
	size_t th_sz = sizeof(struct trace_header);

	infd = open(if_name, O_RDONLY);
	if (infd < 0) {
		perror("file open failed");
		return errno;
	}

	ret = read(infd, &th, th_sz);
	if (ret < trace_header_size()) {
		fprintf(stderr, "failed to read input file %s\n", if_name);
		goto tracelog_out;
	}

	ret = prn_trace_header(&th, LOGFILE);
	if (ret != 0) {
		fprintf(stderr, "uninitialized or corrupt log file %s\n",
			if_name);
		goto tracelog_out;
	}

	trc_ev_sz = trace_event_size(&th);
	start = th.trcbuf_offset;
	end = start + th.trcbuf_size - trc_ev_sz;
	head = start + th.trcbuf_head_offset;
	tail = start + th.trcbuf_tail_offset;

	printf("trace: start 0x%lx end 0x%lx head 0x%lx tail 0x%lx\n",
	       start, end, head, tail);

	/* set the start head sequence number */
	ev_seqnum = th.trcbuf_head_seqnum;
	while (1) {
		/* do not assume contigous read, seek to offset */
		head = lseek(infd, head, SEEK_SET);
		if (head < 0) {
			perror("file seek failed");
			break;
		}

		ret = read(infd, &ev, trc_ev_sz);
		if (ret < trc_ev_sz) {
			printf("failed to read event from file %s\n", if_name);
			goto tracelog_out;
		}

		prn_event_decoded(&th, &ev, ev_seqnum++);

		if (head == tail)
			/* get out */
			break;

		if (head == end)
			/* rollover */
			head = start;
		else
			/* go to next event */
			head += trc_ev_sz;
	}

tracelog_out:
	close(infd);

	return ret;
}

int main(int argc, char **argv)
{
	int sts = 0;
	static struct options opts;

	sts = parse_options(argc, argv, &opts);
	if (sts != 0)
		return sts;

	if (opts.of_name && opts.phys_addr && opts.buf_size) {
		char *file_buf;

		file_buf = trace_mmap_trcbuf(opts.phys_addr, opts.buf_size);
		if (file_buf) {
			sts = dump_to_file(opts.of_name, file_buf,
					   opts.buf_size);
			trace_munmap_trcbuf((void *) file_buf, opts.buf_size);
		}
	} else if (opts.if_name && opts.decode) {
		if (opts.clk_file)
			trace_decode_clk_name_init(opts.clk_file);
		if (sts == 0)
			sts = tracelog_from_file(opts.if_name);
	} else if (opts.phys_addr && opts.buf_size && opts.decode) {
		struct trace_header *th;
		struct trace_info *ti = get_trace_info();

		sts = trace_init(opts.phys_addr, opts.buf_size);
		if (sts == 0) {
			trace_decode_clk_name_init(dfs[CLK_SUM]);
			trace_log_from_mem();
			th = ti->trc_hdr;
			trace_munmap_trcbuf((void *)th, opts.buf_size);
		}
	}

	return sts;
}
