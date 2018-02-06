#!/bin/sh

set -u
set -e

prg=`basename $0`

# Temp directory for old skeleton (not currently used)
SKEL_DIR="${BUILD_DIR}/brcmstb_skel"
# Use newest rootfs tar-ball if there's more than one
UCLINUX_ROOTFS=`ls -1t dl/uclinux-rootfs*.tar.gz 2>/dev/null | head -1`

# BRCMSTB skeleton
if [ ! -r "${UCLINUX_ROOTFS}" ]; then
	echo "$prg: uclinux-rootfs tar-ball not found, not copying skel..." 1>&2
else
	# Extract old "skel" directory straight into our new rootfs. If we ever
	# need to be more selective, we'll extract it into a temporary location
	# first (${SKEL_DIR}) and pick what we need from there.
	tar -C "${TARGET_DIR}" -x -z -f "${UCLINUX_ROOTFS}" \
		--wildcards --strip-components=2 '*/skel'
fi

# Auto-login on serial console
if [ -e ${TARGET_DIR}/etc/inittab ]; then
	echo "Enabling auto-login on serial console..."
	sed -ie 's|.* # GENERIC_SERIAL$|::respawn:/bin/cttyhack /bin/sh -l|' \
		${TARGET_DIR}/etc/inittab
fi

if ls board/brcmstb/dropbear_*_host_key >/dev/null 2>&1; then
	echo "Installing pre-generated SSH keys..."
	rm -rf ${TARGET_DIR}/etc/dropbear
	mkdir -p ${TARGET_DIR}/etc/dropbear
	for key in board/brcmstb/dropbear_*_host_key; do
		b=`basename "${key}"`
		echo "    ${b}"
		install -D -p -m 0600 -t ${TARGET_DIR}/etc/dropbear ${key}
	done
fi
