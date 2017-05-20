/*
 * Copyright (C) 2013-2016 Broadcom
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; version 2.1 of the License.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include <libgen.h>
#include <linux/brcmstb/cma_driver.h>

#define PAGE_SIZE 4096
#define TEST_BFR_LEN (64 * PAGE_SIZE)
#define MAX_CMD_NAME_LEN 16
#define MAX_DESC_LEN 64
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define BUG_PRINT(ret) fprintf(stderr, "bug @ line %d (%d)\n", __LINE__, (ret))

static const char *DEFAULT_CMA_FD = "/dev/brcm_cma0";

enum cmd_index {
	CMD_ALLOC = 0,
	CMD_FREE,
	CMD_LIST,
	CMD_LISTALL,
	CMD_GETPROT,
	CMD_SETPROT,
	CMD_UNITTEST,
	CMD_RESET,
	CMD_RESETALL,
	CMD_MMAP,
	CMD_VERSION,
	CMD_NOT_VALID,
};

struct cmd_arg_desc {
	enum cmd_index cmd_idx;
	char cmd_name[MAX_CMD_NAME_LEN];
	char desc[MAX_DESC_LEN];
	int num_opts;
};

static const struct cmd_arg_desc cmds[] = {
	{ CMD_ALLOC,
		"alloc",    "<cmadevidx> 0x<num_bytes> 0x<align_bytes>", 3 },
	{ CMD_FREE,
		"free",     "<cmadevidx> 0x<PA> 0x<num_bytes>",          3 },
	{ CMD_LIST,
		"list",     "<cmadevidx>",                               1 },
	{ CMD_LISTALL,
		"listall",  "",                                          0 },
	{ CMD_GETPROT,
		"getprot",  "",                                          0 },
	{ CMD_SETPROT,
		"setprot",  "<pg_prot val>",                             1 },
	{ CMD_UNITTEST,
		"unittest", "",                                          0 },
	{ CMD_RESET,
		"reset",    "<cmadevidx>",                               1 },
	{ CMD_RESETALL,
		"resetall", "",                                          0 },
	{ CMD_MMAP,
		"mmap",     "0x<addr> 0x<len> <test_file>",              3 },
	{ CMD_VERSION,
		"version",  "",                                          0 },
	{ CMD_NOT_VALID,
		"",         "",                                         -1 },
};

static int cma_get_mem(int fd, uint32_t dev_index, uint32_t num_bytes,
			uint32_t align_bytes, uint64_t *addr)
{
	int ret;
	struct ioc_params get_mem_p;

	memset(&get_mem_p, 0, sizeof(get_mem_p));

	get_mem_p.cma_dev_index = dev_index;
	get_mem_p.num_bytes = num_bytes;
	get_mem_p.align_bytes = align_bytes;

	ret = ioctl(fd, CMA_DEV_IOC_GETMEM, &get_mem_p);
	if (ret != 0) {
		printf("ioctl failed (%d)\n", ret);
		return ret;
	}

	*addr = get_mem_p.addr;

	ret = get_mem_p.status;
	if (ret == 0)
		printf("alloc PA=0x%"PRIx64" LEN=0x%x\n", *addr, num_bytes);
	else
		printf("alloc PA=0x%"PRIx64" LEN=0x%x failed (%d)\n", *addr,
			num_bytes, ret);

	return ret;
}

static int cma_put_mem(int fd, uint32_t dev_index, uint64_t addr,
	uint32_t num_bytes)
{
	int ret;
	struct ioc_params put_mem_p;

	memset(&put_mem_p, 0, sizeof(put_mem_p));

	put_mem_p.cma_dev_index = dev_index;
	put_mem_p.addr = addr;
	put_mem_p.num_bytes = num_bytes;

	ret = ioctl(fd, CMA_DEV_IOC_PUTMEM, &put_mem_p);
	if (ret)
		return ret;

	ret = put_mem_p.status;
	if (ret == 0)
		printf("freed PA=0x%"PRIx64" LEN=0x%x\n", addr, num_bytes);
	else
		printf("failed to free PA=0x%"PRIx64" LEN=0x%x (%d)\n", addr,
			num_bytes, ret);

	return ret;
}

static int cma_get_phys_info(int fd, uint32_t dev_index, uint64_t *addr,
	uint32_t *num_bytes, int32_t *memc)
{
	int ret;
	struct ioc_params physinfo_p;

	memset(&physinfo_p, 0, sizeof(physinfo_p));

	physinfo_p.cma_dev_index = dev_index;

	ret = ioctl(fd, CMA_DEV_IOC_GETPHYSINFO, &physinfo_p);
	if (ret)
		return ret;

	ret = physinfo_p.status;
	if (ret == 0) {
		*addr = physinfo_p.addr;
		*num_bytes = physinfo_p.num_bytes;
		*memc = physinfo_p.memc;
		printf("region %-2u   0x%016"PRIx64"-0x%016"PRIx64" %12u (MEMC%d)\n",
		       dev_index, *addr, *addr + *num_bytes, *num_bytes, *memc);
	} else
		printf("getphysinfo failed\n");

	return ret;
}

static int cma_get_num_regions(int fd, uint32_t dev_index, uint32_t *num)
{
	int ret;
	struct ioc_params getnumregs_p;
	memset(&getnumregs_p, 0, sizeof(getnumregs_p));
	getnumregs_p.cma_dev_index = dev_index;

	ret = ioctl(fd, CMA_DEV_IOC_GETNUMREGS, &getnumregs_p);
	if (ret)
		return ret;

	ret = getnumregs_p.status;
	*num = getnumregs_p.num_regions;

	return ret;
}

static int cma_get_region_info(int fd, uint32_t dev_index, uint32_t region_num,
				int32_t *memc, uint64_t *addr,
				uint32_t *num_bytes)
{
	int ret;
	struct ioc_params getreginfo_p;
	memset(&getreginfo_p, 0, sizeof(getreginfo_p));
	getreginfo_p.cma_dev_index = dev_index;
	getreginfo_p.region = region_num;

	ret = ioctl(fd, CMA_DEV_IOC_GETREGINFO, &getreginfo_p);
	if (ret)
		return ret;

	ret = getreginfo_p.status;

	*memc = getreginfo_p.memc;
	*addr = getreginfo_p.addr;
	*num_bytes = getreginfo_p.num_bytes;

	if (!ret) {
		printf("  alloc     0x%016"PRIx64"-0x%016"PRIx64" %12u\n", *addr, *addr+*num_bytes,
		       *num_bytes);
	} else
		printf("%s failed (%d)\n", __func__, ret);

	return ret;
}

static void reset_all(int fd, uint32_t cma_idx)
{
	uint32_t count;

	if (cma_get_num_regions(fd, cma_idx, &count) != 0)
		return;

	while (count != 0) {
		int ret;
		int32_t memc;
		uint64_t addr;
		uint32_t len;

		ret = cma_get_region_info(fd, cma_idx, 0, &memc, &addr, &len);
		if (ret == 0) {
			assert(cma_put_mem(fd, cma_idx, addr, len) == 0);
			assert(cma_get_num_regions(fd, cma_idx, &count) == 0);
		} else
			break;
	}
}

static int list_one(int fd, uint32_t cma_dev_index)
{
	int ret;
	unsigned int i;
	uint32_t num_regions;
	uint64_t addr;
	uint32_t len;
	int32_t memc;

	ret = cma_get_phys_info(fd, cma_dev_index, &addr, &len, &memc);
	if (ret)
		goto done;

	ret = cma_get_num_regions(fd, cma_dev_index, &num_regions);
	if (ret) {
		BUG_PRINT(ret);
		goto done;
	}

	for (i = 0; i < num_regions; i++) {
		int32_t reg_memc;
		uint64_t addr;
		uint32_t num_bytes;

		ret = cma_get_region_info(fd, cma_dev_index, i,
					  &reg_memc, &addr,
					  &num_bytes);
		if (ret) {
			BUG_PRINT(ret);
			goto done;
		}
	}

done:
	return ret;
}

static bool exists_region(int fd, uint32_t cma_idx)
{
	uint64_t addr;
	uint32_t len;
	int32_t memc;
	int ret;

	ret = cma_get_phys_info(fd, cma_idx, &addr, &len, &memc);
	return !ret;
}

static void __run_unit_tests(int fd, uint32_t cma_idx)
{
	int i;
	uint64_t addr[32], max_align = 16 * 1024 * 1024;
	uint32_t len[32];
	uint32_t x;
	uint64_t y;
	int32_t z;

	/* === TEST CASE 0 ===
	 * This isn't really a test per se, but we do need the region to
	 * exist and to be reset to be sure the unit tests will work.
	 */
	if (!exists_region(fd, cma_idx))
		return;
	printf("Resetting CMA region %u for tests.\n", cma_idx);
	reset_all(fd, cma_idx);  /* contains its own asserts */

	/* === TEST CASE 1 === */

	printf("t: alloc (1) region\n");
	len[0] = 0x1000;
	len[1] = 0x1000;
	assert(cma_get_mem(fd, cma_idx, len[0], 0, &addr[0]) == 0);
	printf("ok\n\n");

	printf("t: verify (1) region allocated\n");
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 1);
	printf("ok\n\n");

	printf("t: free (1) region\n");
	assert(cma_put_mem(fd, cma_idx, addr[0], len[0]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 2 === */

	printf("t: alloc (2) regions w/ 16MB alignment\n");
	for (i = 0; i < 2; i++) {
		len[i] = 0x1000;
		assert(cma_get_mem(fd, cma_idx, len[i], 16 * 1024 * 1024, &addr[i])
			== 0);
	}
	printf("ok\n\n");

	printf("t: verify (2) regions allocated\n");
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 2);
	printf("ok\n\n");

	printf("t: verify (2) regions have expected alignment and size\n");
	for (i = 0; i < 2; i++) {
		int32_t memc;
		uint64_t addr, align;
		uint32_t num_bytes;

		assert(cma_get_region_info(fd, cma_idx, i, &memc, &addr, &num_bytes)
			== 0);
		assert(num_bytes == len[i]);

		/* CONFIG_CMA_ALIGNMENT reduced to 2MB alignment */
		assert((addr % (2 * 1024 * 1024)) == 0);

		align = addr & -addr;
		if (align < max_align)
			max_align = align;
	}
	if (max_align != 16 * 1024 * 1024)
		printf("!!!WARNING: Max CMA alignment is %"PRId64
			" bytes!!!\n", max_align);
	printf("ok\n\n");

	printf("t: free (2) region\n");
	for (i = 0; i < 2; i++)
		assert(cma_put_mem(fd, cma_idx, addr[i], len[i]) == 0);
	printf("ok\n\n");

	if (max_align == 16 * 1024 * 1024)
		goto test_3;

	/* === TEST CASE 2B === */

	printf("t: alloc (2) regions w/ %"PRId64" byte alignment\n",
		max_align/2);
	for (i = 0; i < 2; i++) {
		len[i] = 0x1000;
		assert(cma_get_mem(fd, cma_idx, len[i], max_align/2, &addr[i])
			== 0);
	}
	printf("ok\n\n");

	printf("t: verify (2) regions allocated\n");
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 2);
	printf("ok\n\n");

	printf("t: verify (2) regions have expected alignment and size\n");
	for (i = 0; i < 2; i++) {
		int32_t memc;
		uint64_t addr;
		uint32_t num_bytes;

		assert(cma_get_region_info(fd, cma_idx, i, &memc, &addr, &num_bytes)
			== 0);
		assert(num_bytes == len[i]);

		assert((addr % (max_align/2)) == 0);
	}
	printf("ok\n\n");

	printf("t: free (2) region\n");
	for (i = 0; i < 2; i++)
		assert(cma_put_mem(fd, cma_idx, addr[i], len[i]) == 0);
	printf("ok\n\n");

test_3:
	/* === TEST CASE 3 === */

	printf("t: alloc and free (2) regions\n");
	len[0] = 0x1000;
	len[1] = 0x2000;
	for (i = 0; i < 2; i++)
		assert(cma_get_mem(fd, cma_idx, len[i], 0, &addr[i]) == 0);
	for (i = 0; i < 2; i++)
		assert(cma_put_mem(fd, cma_idx, addr[i], len[i]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 4 === */

	printf("t: alloc and attempt free with mismatched length\n");
	len[0] = 0x1000;
	assert(cma_get_mem(fd, cma_idx, len[0], 0, &addr[0]) == 0);
	assert(cma_put_mem(fd, cma_idx, addr[0], len[0] + 1) == -EINVAL);
	assert(cma_put_mem(fd, cma_idx, addr[0], len[0]) == 0);
	printf("ok\n\n");

	/* === TEST CASE 5 === */

	printf("t: alloc (5) and free in same order\n");
	for (i = 0; i < 5; i++) {
		len[i] = (i + 1) * 0x1000;
		assert(cma_get_mem(fd, cma_idx, len[i], 0, &addr[i]) == 0);
	}
	for (i = 0; i < 5; i++)
		assert(cma_put_mem(fd, cma_idx, addr[i], len[i]) == 0);
	printf("ok\n\n");

	printf("t: verify (0) regions allocated\n");
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 0);
	printf("ok\n\n");

	/* === TEST CASE 6 === */

	printf("t: alloc (5) and free in reverse order\n");
	for (i = 0; i < 5; i++) {
		len[i] = (i + 1) * 0x1000;
		assert(cma_get_mem(fd, cma_idx, len[i], 0, &addr[i]) == 0);
	}
	for (i = 4; i >= 0; i--)
		assert(cma_put_mem(fd, cma_idx, addr[i], len[i]) == 0);
	printf("ok\n\n");

	printf("t: verify (0) regions allocated\n");
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 0);
	printf("ok\n\n");

	/* === TEST CASE 7 === */

	printf("t: alloc (3), free middle\n");
	for (i = 0; i < 3; i++) {
		len[i] = (i + 1) * 0x1000;
		assert(cma_get_mem(fd, cma_idx, len[i], 0, &addr[i]) == 0);
	}
	assert(cma_put_mem(fd, cma_idx, addr[1], len[1]) == 0);
	printf("ok\n\n");

	printf("t: verify (2) regions allocated\n");
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 2);
	printf("ok\n\n");

	printf("t: free the remaining (2) regions, verify (0) regions\n");
	assert(cma_put_mem(fd, cma_idx, addr[0], len[0]) == 0);
	assert(cma_put_mem(fd, cma_idx, addr[2], len[2]) == 0);
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 0);
	printf("ok\n\n");

	/* === TEST CASE 8 === */

	printf("t: attempt retrieval of invalid region info\n");
	assert(cma_get_region_info(fd, cma_idx, 42, &z, &y, &x) == -EINVAL);
	printf("ok\n\n");

	/* === TEST CASE 9 === */
	printf("t: attempt retrieval of invalid phys info\n");
	assert(cma_get_phys_info(fd, 42, &y, &x, &z) == -1);
	printf("ok\n\n");

	/* === TEST CASE 10 === */
	printf("t: alloc (1) region and free a chunk from the middle\n");
	len[0] = 0x100000;
	len[1] = 0x1000;
	assert(cma_get_mem(fd, cma_idx, len[0], 0, &addr[0]) == 0);
	assert(cma_put_mem(fd, cma_idx, addr[0] + 0x1000, len[1]) == 0);
	assert(cma_get_num_regions(fd, cma_idx, &x) == 0);
	assert(x == 2);
	{
		int32_t  reg_memc;
		uint64_t reg_addr;
		uint32_t reg_numbytes;

		assert(cma_get_region_info(fd, cma_idx, 0, &reg_memc, &reg_addr,
					   &reg_numbytes) == 0);
		assert(reg_addr == addr[0]);
		assert(reg_numbytes == 0x1000);

		assert(cma_get_region_info(fd, cma_idx, 1, &reg_memc, &reg_addr,
					   &reg_numbytes) == 0);
		assert(reg_addr == (addr[0] + 0x2000));
		assert(reg_numbytes == (len[0] - 0x2000));

		assert(cma_put_mem(fd, cma_idx, addr[0], 0x1000) == 0);
		assert(cma_put_mem(fd, cma_idx, addr[0] + 0x2000,
				   len[0] - 0x2000) == 0);
	}
	printf("ok\n\n");

	/* Warn in case 16MB alignment is required */
	if (max_align < 16 * 1024 * 1024)
		printf("!!!WARNING: Max CMA alignment is no longer 16MB!!!\n");

	printf("UNIT TEST PASSED! (region %u)\n", cma_idx);
}

static void test_bfr_gen(uint32_t *bfr, uint32_t bfr_sz)
{
	unsigned int i;
	uint32_t *curr = bfr;

	for (i = 0; i < bfr_sz / sizeof(uint32_t); i++)
		curr[i] = i;
}

static void run_unit_tests(int fd)
{
	uint32_t i;
	for (i = 0; i < CMA_NUM_RANGES; i++)
		__run_unit_tests(fd, i);
	printf("\nAll unit tests passed.\n");
}

static int test_bfr_chk(uint32_t *bfr, uint32_t bfr_sz)
{
	int rc = 0;
	unsigned int i;
	uint32_t *curr = bfr;

	for (i = 0; i < bfr_sz / sizeof(uint32_t); i++) {
		if (curr[i] != i) {
			rc = -1;
			break;
		}
	}

	return rc;
}

static int test_file_gen(const char *path)
{
	/*
	 * Use stdio to create the test file. We'll use mmap() for
	 * reading it back.
	 */
	FILE *fp = fopen(path, "w");
	uint32_t *test_bfr = NULL;
	size_t count;
	int rc = 0;

	if (!fp) {
		rc = -1;
		goto done;
	}

	test_bfr = (uint32_t *)malloc(TEST_BFR_LEN);
	if (!test_bfr) {
		rc = -1;
		goto done;
	}

	test_bfr_gen(test_bfr, TEST_BFR_LEN);

	count = fwrite(test_bfr, 1, TEST_BFR_LEN, fp);
	if (count != TEST_BFR_LEN) {
		rc = -1;
		goto done;
	}

done:
	if (test_bfr)
		free(test_bfr);

	return rc;
}

static int test_file_chk(const char *path)
{
	/*
	 * Use stdio to create the test file. We'll use mmap() for
	 * reading it back.
	 */
	FILE *fp = fopen(path, "r");
	uint32_t *test_bfr = NULL;
	size_t count;
	int rc;

	if (!fp) {
		rc = -1;
		goto done;
	}

	test_bfr = (uint32_t *)malloc(TEST_BFR_LEN);
	if (!test_bfr) {
		rc = -1;
		goto done;
	}

	count = fread(test_bfr, 1, TEST_BFR_LEN, fp);
	if (count != TEST_BFR_LEN) {
		rc = -1;
		goto done;
	}

	rc = test_bfr_chk(test_bfr, TEST_BFR_LEN);
	if (rc)
		printf("file check failed (%d)\n", rc);

done:
	if (test_bfr)
		free(test_bfr);

	return rc;
}

static void cma_mmap(int cma_fd, uint32_t base, uint32_t len, char *path)
{
	int rc;
	int fd = -1;
	int cnt;
	uint32_t *mem = NULL;
	void *align_va = NULL;

	if ((base % PAGE_SIZE) != 0) {
		printf("error: base address is not page aligned!\n");
		return;
	}

	if ((len == 0) || ((len % PAGE_SIZE) != 0) || (len < TEST_BFR_LEN)) {
		printf("error: bad length! "
		       "(should be non-zero, page aligned, and at least "
		       "%d bytes)\n", TEST_BFR_LEN);
		return;
	}

	printf("generating test file at \"%s\"\n", path);
	rc = test_file_gen(path);
	if (rc) {
		printf("error: cannot generate test file (%d)\n", rc);
		return;
	}

	printf("verify file using stdio before using mmap()\n");
	rc = test_file_chk(path);
	if (rc) {
		printf("error: test file not valid (%d)\n", rc);
		return;
	}

	printf("allocating page aligned memory using posix_memalign()\n");
	rc = posix_memalign(&align_va, PAGE_SIZE, len);
	if (rc) {
		printf("posix_memalign failed (%d)\n", rc);
		return;
	}

	printf("mmap() base=0x%x len=0x%x at va=%ph\n", base, len, align_va);
	mem = mmap(align_va, len, PROT_READ | PROT_WRITE,
		   MAP_SHARED | MAP_FIXED, cma_fd, base);

	if (!mem) {
		printf("mmap() failed\n");
		goto cleanup;
	}

	if (mem != align_va) {
		printf("mmap() did not obey MAP_FIXED flag!\n");
		goto cleanup;
	}

	printf("trying to open \"%s\" with (O_SYNC | O_DIRECT) flags\n", path);
	fd = open(path, O_SYNC | O_DIRECT);
	printf("open fd: %d\n", fd);
	if (fd < 0) {
		printf("open failed, errno: %d\n", errno);
		goto cleanup;
	}

	printf("reading %d bytes from \"%s\"\n", TEST_BFR_LEN, path);
	cnt = read(fd, mem, TEST_BFR_LEN);
	printf("cnt: %d\n", cnt);
	if (cnt < 0) {
		printf("read failed, errno: %d\n", errno);
		goto cleanup;
	}

	/* TODO: cache coherency? */
	printf("verify file data read via mmap()\n");
	rc = test_bfr_chk(mem, TEST_BFR_LEN);
	if (rc)
		printf("file verification failed (%d)\n", rc);
	else
		printf("file OK!\n");

cleanup:
	if (fd >= 0)
		close(fd);

	if (mem)
		munmap(mem, len);

	if (align_va)
		free(align_va);
}

static void show_usage(char *argv)
{
	int i;
	char *execname = basename(argv);

	printf("Usage: %s [device] <command> <args...>\n\n", execname);
	printf("Commands:\n");

	for (i = 0; i < (int)ARRAY_SIZE(cmds); i++)
		printf("    %-9s %s\n", cmds[i].cmd_name, cmds[i].desc);

	printf("Examples:\n");
	printf("    %s alloc 2 0x1000 0x0\n", execname);
	printf("    %s unittest\n\n", execname);
	printf("    %s /dev/brcm_cma0 alloc 2 0x1000 0x0\n", execname);
	printf("    %s /dev/brcm_cma0 unittest\n\n", execname);

	printf("If device is not provided, it defaults to /dev/brcm_cma0.\n");
}

static enum cmd_index check_cmd(char *exec, const char *str, int num_opts)
{
	int i;
	enum cmd_index cmd_idx = CMD_NOT_VALID;

	for (i = 0; i < (int)ARRAY_SIZE(cmds); i++) {
		if (strncmp(str, cmds[i].cmd_name, MAX_CMD_NAME_LEN) == 0) {
			cmd_idx = cmds[i].cmd_idx;
			break;
		}
	}

	if ((cmd_idx == CMD_NOT_VALID) || (num_opts != cmds[i].num_opts)) {
		show_usage(exec);
		exit(1);
	}

	return cmd_idx;
}

/* read hex or dec depending on starting "0x" */
static int hex_or_dec_input_u32(char *str, uint32_t *bytes)
{
	if (strstr(str, "0x") == str)
		return sscanf(str, "%x", bytes);
	else
		return sscanf(str, "%u", bytes);
}

int main(int argc, char *argv[])
{
	int fd;
	int ret = 0;
	enum cmd_index cmd_idx;
	int cmd_argc;
	char **cmd_argv;
	const char *device_path;

	if (argc > 2 && strstr(argv[1], "/dev") == argv[1]) {
		/* We got the optional device arg */
		device_path = argv[1];
		cmd_argc = argc - 3; /* minus exec, cmd, path */
		cmd_idx = check_cmd(argv[0], argv[2], cmd_argc);
		cmd_argv = &argv[3];
	} else if (argc > 1) {
		device_path = DEFAULT_CMA_FD;
		cmd_argc = argc - 2; /* minus exec, cmd */
		cmd_idx = check_cmd(argv[0], argv[1], cmd_argc);
		cmd_argv = &argv[2];
	} else {
		show_usage(argv[0]);
		return -1;
	}

	fd = open(device_path, O_RDWR);
	if (fd < 0) {
		printf("couldn't open device %s\n", device_path);
		goto done;
	}

	switch (cmd_idx) {
	case CMD_ALLOC: {
		uint32_t cma_dev_index;
		uint32_t num_bytes;
		uint32_t align_bytes;
		uint64_t addr;

		sscanf(cmd_argv[0], "%u", &cma_dev_index);
		hex_or_dec_input_u32(cmd_argv[1], &num_bytes);
		hex_or_dec_input_u32(cmd_argv[2], &align_bytes);

		ret = cma_get_mem(fd, cma_dev_index, num_bytes, align_bytes,
			&addr);
		if (ret) {
			BUG_PRINT(ret);
			goto done;
		}

		printf("PA=0x%"PRIx64"\n", addr);
		break;
	}
	case CMD_FREE: {
		uint32_t cma_dev_index;
		uint64_t addr;
		uint32_t num_bytes;

		sscanf(cmd_argv[0], "%u", &cma_dev_index);
		sscanf(cmd_argv[1], "%"PRIx64, &addr);
		hex_or_dec_input_u32(cmd_argv[2], &num_bytes);

		ret = cma_put_mem(fd, cma_dev_index, addr, num_bytes);
		if (ret) {
			BUG_PRINT(ret);
			goto done;
		}
		break;
	}
	case CMD_LIST: {
		uint32_t cma_dev_index;

		sscanf(cmd_argv[0], "%x", &cma_dev_index);

		ret = list_one(fd, cma_dev_index);
		break;
	}
	case CMD_LISTALL: {
		uint32_t i;
		for (i = 0; i < CMA_NUM_RANGES; i++)
			list_one(fd, i);
		break;
	}
	case CMD_GETPROT: {
		uint32_t x;

		ret = ioctl(fd, CMA_DEV_IOC_GET_PG_PROT, &x);
		if (ret) {
			BUG_PRINT(ret);
			goto done;
		}

		printf("pg_prot is %d\n", x);
		break;
	}
	case CMD_SETPROT: {
		uint32_t x;

		sscanf(cmd_argv[0], "%x", &x);

		ret = ioctl(fd, CMA_DEV_IOC_SET_PG_PROT, &x);
		if (ret) {
			BUG_PRINT(ret);
			goto done;
		}
		break;
	}
	case CMD_UNITTEST: {
		run_unit_tests(fd);
		break;
	}
	case CMD_RESET: {
		uint32_t cma_idx;

		sscanf(cmd_argv[0], "%u", &cma_idx);
		reset_all(fd, cma_idx);
		break;
	}
	case CMD_RESETALL: {
		int i;
		for (i = 0; i < CMA_NUM_RANGES; i++)
			reset_all(fd, i);
		break;
	}
	case CMD_MMAP: {
		uint32_t base;
		uint32_t len;

		sscanf(cmd_argv[0], "%x", &base);
		sscanf(cmd_argv[1], "%x", &len);

		cma_mmap(fd, base, len, cmd_argv[2]);
		break;
	}
	case CMD_VERSION: {
		uint32_t version;
		ret = ioctl(fd, CMA_DEV_IOC_VERSION, &version);
		if (ret) {
			BUG_PRINT(ret);
			goto done;
		}
		printf("version is %u\n", version);
	}
	default:
		break;
	}

	close(fd);

done:
	return ret ? EXIT_FAILURE : 0;
}
