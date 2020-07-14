#!/bin/sh

set -u
set -e

arch_dir=`dirname "${TARGET_DIR}"`

# Remove kernel modules for other kernel versions to avoid artificially
# inflating the initramfs.
set +u
test -z ${BR_KEEP_KERNEL_MODULES} && BR_KEEP_KERNEL_MODULES=""
set -u
if [ "${BR_KEEP_KERNEL_MODULES}" != "" ]; then
	echo "Keeping previous kernel modules"
else
	echo "Removing previous kernel modules"
	rm -rf ${TARGET_DIR}/lib/modules/*
	# The kernel directory can be linux-custom or linux-$tag, only one of
	# which will exist. "[^t]" excludes linux-tools. That means we'll find
	# the kernel directory without having to hard-code its naming pattern.
	rm -f ${arch_dir}/build/linux-[^t]*/.stamp_target_installed
fi
