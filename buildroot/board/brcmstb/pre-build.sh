#!/bin/sh

#
# The purpose of this script at this time is to prepare a user's existing
# "target" directory for the transition from using /lib64 and /lib to using
# /lib and /lib32.
# The script detects whether /lib64 is an actual directory and not a symlink. If
# so, it'll remove the target directory and all "target_installed" stamp files.
# This will cause the build process to re-install everything into the target
# directory (without rebuilding everything). In the process of re-populating
# "target" the root file system will be transitioned to the new /lib and /lib32
# layout.
# Generally speaking, the partial wipe should be sufficient to transition to
# the new file system layout. However, it is not possible to completely rule
# out any and all build problems. A complete re-build may be recessary in some
# cases.
#

set -u
set -e

arch_dir=`dirname "${TARGET_DIR}"`
build_dir="${arch_dir}/build"
linux_tools="${build_dir}/linux-tools"
lib64="${TARGET_DIR}/lib64"

# Nothing to do if the target directory doesn't exist.
test -d "${TARGET_DIR}" || exit 0

# "test -d" implicitly dereferences symlinks, so it is not sufficient to
# test whether lib64 is a directory. We also have to ensure it's not a
# symlink.
if [ ! -h "${lib64}" -a -d "${lib64}" ]; then
	echo "====== CAUTION ==== CAUTION ====  CAUTION ==== CAUTION ====="
	echo "Found /lib64 directory. Performing a partial wipe to prepare"
	echo "for the new directory layout."
	echo "Should you experience inexplicable build errors, please wipe"
	echo "your ${arch_dir} directory"
	echo "and re-build from scratch."
	echo "====== CAUTION ==== CAUTION ====  CAUTION ==== CAUTION ====="
	echo "Wiping '${TARGET_DIR}' and '${linux_tools}'..."
	rm -rf "${TARGET_DIR}" "${linux_tools}"
	echo "Removing .stamp_target_installed files..."
	find "${build_dir}" -name '.stamp_target_installed' -exec rm -f {} \;
	echo "Done."
fi
