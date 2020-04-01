#!/bin/sh

set -u
set -e

prg=`basename $0`
board_dir=`dirname $0`
custom_files="${BASE_DIR}/files"
custom_script="${BASE_DIR}/scripts/${prg}"

image_path="$1"
# The output directory is one level up from the image directory.
output_path=`dirname "$image_path"`
dot_config="${output_path}/.config"

sed -i 's|$(cat $DT_DIR|$(tr -d "\\0" <$DT_DIR|' \
	${TARGET_DIR}/etc/config/ifup.default

if [ ! -e ${TARGET_DIR}/bin/sh ]; then
	echo "Symlinking /bin/bash -> /bin/sh..."
	ln -s bash ${TARGET_DIR}/bin/sh
fi

if ! grep /bin/sh ${TARGET_DIR}/etc/shells >/dev/null; then
	echo "Adding /bin/sh to /etc/shells..."
	echo "/bin/sh" >>${TARGET_DIR}/etc/shells
fi

# Auto-login on serial console
if [ -e ${TARGET_DIR}/etc/inittab ]; then
	echo "Enabling auto-login on serial console..."
	sed -i 's|.* # GENERIC_SERIAL$|::respawn:/bin/cttyhack /bin/sh -l|' \
		${TARGET_DIR}/etc/inittab
fi

if ls ${board_dir}/dropbear_*_host_key >/dev/null 2>&1; then
	echo "Installing pre-generated SSH keys..."
	rm -rf ${TARGET_DIR}/etc/dropbear
	mkdir -p ${TARGET_DIR}/etc/dropbear
	for key in ${board_dir}/dropbear_*_host_key; do
		b=`basename "${key}"`
		echo "    ${b}"
		install -D -p -m 0600 -t ${TARGET_DIR}/etc/dropbear ${key}
	done
fi

# Enabling dropbear (rcS file inherited from the classic rootfs for now)
rcS="${TARGET_DIR}/etc/init.d/rcS"
if grep 'if [ -e /sbin/dropbear ]' "$rcS" >/dev/null; then
	echo "Enabling dropbear rcS..."
	sed -i 's| -e /sbin/dropbear | -e /usr/sbin/dropbear |' ${rcS}
fi

# Add SSH key for root
sshdir="${TARGET_DIR}/root/.ssh"
if [ -r ${board_dir}/brcmstb_root ]; then
	echo "Installing SSH key for root..."
	rm -rf "${sshdir}"
	mkdir "${sshdir}"
	chmod go= "${sshdir}"
	cp ${board_dir}/brcmstb_root.pub "${sshdir}/authorized_keys"
fi

# Create mount points
echo "Creating mount points..."
rm -r ${TARGET_DIR}/mnt
mkdir ${TARGET_DIR}/mnt
for d in flash hd nfs usb; do
	mkdir ${TARGET_DIR}/mnt/${d}
done

# Fixing symlinks in /var
echo "Turning some /var symlinks into directories..."
for d in `find ${TARGET_DIR}/var -type l`; do
	l=`readlink "$d"`
	# We don't want any symlinks into /tmp. They break the ability to
	# install into a FAT-formatted USB stick.
	if echo "$l" | grep '/tmp' >/dev/null; then
		rm -f "$d"
		mkdir "$d"
	fi
done

# Create /data
rm -rf ${TARGET_DIR}/data
mkdir ${TARGET_DIR}/data

# We don't want /etc/resolv.conf to be a symlink into /tmp, either
resolvconf="${TARGET_DIR}/etc/resolv.conf"
if [ -h "${resolvconf}" ]; then
	echo "Creating empty /etc/resolv.conf..."
	rm "${resolvconf}"
	touch "${resolvconf}"
fi

# Add ldd from the host's sysroot
echo "Copying ldd..."
cp -p ${HOST_DIR}/*gnu*/sysroot/usr/bin/ldd ${TARGET_DIR}/usr/bin
if grep '^RTLDLIST=/lib/ld-linux-aarch64' "${TARGET_DIR}/usr/bin/ldd" >/dev/null; then
	set +e
	ld32=`ls "${TARGET_DIR}/lib/"ld-linux-arm* 2>/dev/null`
	ld64=`ls "${TARGET_DIR}/lib/"ld-linux-aarch* 2>/dev/null`
	set -e
	if [ "${ld32}" = "" -o "${ld64}" = "" ]; then
		echo "Couldn't find shared library loader(s), not updating ldd!"
	else
		echo "Adding Aarch32 capabilities to ldd..."
		ld32="/lib/`basename ${ld32}`"
		ld64="/lib/`basename ${ld64}`"
		sed -i "s|^RTLDLIST=.*|RTLDLIST=\"${ld64} ${ld32}\"|" \
			"${TARGET_DIR}/usr/bin/ldd"
	fi
fi

# Generate brcmstb.conf
echo "Generating /etc/brcmstb.conf..."
arch=`basename ${BASE_DIR}`
# The Linux directory can be "linux-custom" or "linux-$tag". We also must ensure
# we don't pick up directories like "linux-tools" or "linux-firmware".
linux_dir=`ls -drt ${BUILD_DIR}/linux-* | egrep 'linux-(stb|custom)' | head -1`
linux_ver=`./bin/linuxver.sh $linux_dir`
cat >${TARGET_DIR}/etc/brcmstb.conf <<EOF
TFTPHOST=${TFTPHOST:=`hostname -f`}
TFTPPATH=${TFTPPATH:=$linux_ver}
PLAT=$arch
VERSION=$linux_ver
EOF

if grep 'BR2_NEED_LD_SO_CONF=y' "${dot_config}" >/dev/null; then
	echo "Copying ${board_dir}/ld.so.conf..."
	cp -p "${board_dir}/ld.so.conf" ${TARGET_DIR}/etc
	echo "Copying ldconfig..."
	cp -p ${HOST_DIR}/*gnu*/sysroot/sbin/ldconfig ${TARGET_DIR}/sbin
	echo "Running host-ldconfig..."
	./bin/ldconfig -r ${TARGET_DIR}
fi

# BR_SKIP_LEGAL_INFO permits a developer to skip the "make legal-info" stage to
# shorten build time. This option is for development only and must not be used
# for release builds.
set +u
test -z ${BR_SKIP_LEGAL_INFO} && BR_SKIP_LEGAL_INFO=""
set -u
if [ "${BR_SKIP_LEGAL_INFO}" != "" ]; then
	echo "Not generating GPL-3.0 packages list per user request."
	echo "See environment variable \$BR_SKIP_LEGAL_INFO."
else
	# Generate list of GPL-3.0 packages
	echo "Generating GPL-3.0 packages list"
	make -C ${BASE_DIR} legal-info
	rm -rf ${TARGET_DIR}/usr/share/legal-info/
	mkdir ${TARGET_DIR}/usr/share/legal-info/
	grep "GPL-3.0" ${BASE_DIR}/legal-info/manifest.csv | \
		cut -d, -f1 \
			> ${TARGET_DIR}/usr/share/legal-info/GPL-3.0-packages
	sed -i 's| -e /bin/gdbserver -o -e /bin/gdb | -s /usr/share/legal-info/GPL-3.0-packages |' ${rcS}
fi

# Copy directory structure from ${BASE_DIR}/files to the target
echo "Copying supplemental files..."
if [ -d "${custom_files}" ]; then
	rsync -a "${custom_files}/" "${TARGET_DIR}"
fi

# Checking for custom post-build script
if [ -x "${custom_script}" ]; then
	echo "Executing ${custom_script}..."
	"${custom_script}" "$@"
fi

# The diff command below may return an error code. We don't want to abort the
# script if that happens. We want to handle the error case ourselves.
set +e

# Check if start-up files somehow got overwritten. It should never happen, but
# we want to know at build time if it ever does.
SKEL_VERSION=`grep default package/brcm-pm/Config.in | awk '{print $2}'`
SKEL_PATH=`dirname ${TARGET_DIR}`/build/brcm-skel-${SKEL_VERSION}
echo "Performing consistency check..."
init_diff=`diff -u ${TARGET_DIR}/etc/inittab ${SKEL_PATH}/skel/etc/inittab`
if [ $? != 0 ]; then
	echo "Detected a potential problem with the start-up files."
	echo "It is recommended to remove the output/<arch> directory and"
	echo "rebuild from scratch."
	echo "$init_diff"
	exit 1
fi
