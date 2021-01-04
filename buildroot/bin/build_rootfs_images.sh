#!/bin/bash
prg=$(basename $0)
parent_dir=$(dirname $(dirname $0))

# UBIFS logical eraseblock limit - increase this number if mkfs.ubifs complains
max_leb_cnt=2047

# NAND ubifs images can be large files, especially for bigger eraseblock sizes.
# Building them is disabled by default. (Use option -u to enable.)
build_nand_ubifs=0

LINUXDIR=linux
DEVTABLE=misc/devtable.txt
WARN_FILE="THIS_IS_NOT_YOUR_ROOT_FILESYSTEM"

while getopts hu arg; do
	case $arg in
		h) hflag=1;;
		u) uflag=1;;
		?) exit 1;;
	esac
done
shift `expr $OPTIND - 1`

TARGET=$1
if [ -d "target" -a -d "images" ]; then
	OUTPUT_DIR="."
else
	OUTPUT_DIR="output/$1"
fi

hostpath="$parent_dir/output/$TARGET/host"

# For mkfs.jffs2 & co. Use the host-tools version if it exists. Try the system
# version otherwise.
if [ -d "$hostpath" ]; then
	PATH="$hostpath/bin:$hostpath/sbin:$PATH"
else
	PATH="$PATH:/sbin:/usr/sbin"
fi

export PATH

# Used by ubifs and jffs
fs="${OUTPUT_DIR}/target"

# Get the maximum Logical Erase Block size for UBIFS
function get_ubi_max_leb()
{
	pebk=$1
	page=$2

	peb=$(($1 * 1024))

	if [ $page -lt 64 ]; then
		leb=$(($peb - 64 * 2))
	else
		leb=$(($peb - $page * 2))
	fi

	# If it errors out, we use the LEB number mkfs.ubifs tells us. If
	# there's no error or it doesn't complain about the LEB count, we can
	# continue to use the default.
	msg=`mkfs.ubifs -U -D "$DEVTABLE" -r "$fs" -o tmp/tmp.img \
		-m $page -e $leb -c ${max_leb_cnt} 2>&1`
	rm -f tmp/tmp.img
	if echo "$msg" | grep 'max_leb_cnt too low' >/dev/null; then
		max_leb_cnt=`echo "$msg" | sed -e 's/.*(\([0-9]\+\).*/\1/'`
		# Add a little bit of a buffer
		max_leb_cnt=$(($max_leb_cnt + 20))
	fi
	echo "${max_leb_cnt}"
}

function make_ubi_img()
{
	pebk=$1
	page=$2

	peb=$(($1 * 1024))

	if [ $page -lt 64 ]; then
		leb=$(($peb - 64 * 2))
		minmsg="minimum write size $page (NOR)"
	else
		leb=$(($peb - $page * 2))
		minmsg="minimum write size $page (NAND)"
	fi

	out="${OUTPUT_DIR}/images/ubifs-${pebk}k-${page}-${TARGET}.img"

	echo "Writing UBIFS image for ${pebk}kB erase, ${minmsg}..."

	mkfs.ubifs -U -D "$DEVTABLE" -r "$fs" -o tmp/ubifs.img \
		-m $page -e $leb -c ${max_leb_cnt}

	vol_size=$(du -sm tmp/ubifs.img | cut -f1)

	cat > tmp/ubinize.cfg <<-EOF
	[ubifs]
	mode=ubi
	image=tmp/ubifs.img
	vol_id=0
	vol_size=${vol_size}MiB
	vol_type=dynamic
	vol_name=rootfs
	vol_flags=autoresize
	EOF

	ubinize -o tmp/ubi.img -m $page -p $peb tmp/ubinize.cfg

	mv tmp/ubi.img $out
	echo "  -> $out"
}

function make_jffs2_img()
{
	pebk=$1
	out="${OUTPUT_DIR}/images/jffs2-${pebk}k-${TARGET}.img"

	echo "Writing JFFS2 image for ${pebk}kB eraseblock size (NOR)..."
	mkfs.jffs2 -U -D "$DEVTABLE" -r "$fs" \
		-o tmp/jffs2.img -e ${pebk}KiB $JFFS2_ENDIAN
	sumtool -i tmp/jffs2.img -o $out -e ${pebk}KiB $JFFS2_ENDIAN
	echo "  -> $out"
}

#
# MAIN
#

if [ $# -lt 1 ] || [ ! -z $hflag ]; then
	echo "usage: $0 [-u] <arm|arm64|mips>" 1>&2
	echo "       -u....generate NAND UBI images (can be large)" 1>&2
	exit 1
fi

if [ ! -z $uflag ]; then
	build_nand_ubifs=1
fi

rm -rf tmp
mkdir -p tmp

if [[ "$TARGET" = "mips" ]]; then
	JFFS2_ENDIAN=-b
else
	JFFS2_ENDIAN=-l
fi

if [ ! -r "$DEVTABLE" ]; then
	if [ -r "../$DEVTABLE" ]; then
		DEVTABLE="../$DEVTABLE"
	elif [ -r "../../$DEVTABLE" ]; then
		DEVTABLE="../../$DEVTABLE"
	fi
fi

test -e "$OUTPUT_DIR/target/$WARN_FILE" && \
	mv "$OUTPUT_DIR/target/$WARN_FILE" tmp

test -e "$OUTPUT_DIR/target/dev/console" && \
	mv "$OUTPUT_DIR/target/dev/console" tmp

if which mksquashfs >/dev/null; then
	echo "Writing SQUASHFS image..."
	rm -f "${OUTPUT_DIR}/images/squashfs-${TARGET}.img"
	mksquashfs "${OUTPUT_DIR}/target" \
		"${OUTPUT_DIR}/images/squashfs-${TARGET}.img" \
		-root-owned -p "/dev/console c 0600 0 0 5 1"
	chmod 0644 "${OUTPUT_DIR}/images/squashfs-${TARGET}.img"
	echo "  -> ${OUTPUT_DIR}/images/squashfs-${TARGET}.img"
	echo ""
else
	echo "[WARNING] Couldn't find mksquashfs. Skipping squashfs images."
	echo "[WARNING] Please consider installing the respective package."
fi

set -e

if which mkfs.ubifs >/dev/null; then
	# 64k erase / 1B unit size - NOR
	make_ubi_img 64 1

	# 128k erase / 1B unit size - NOR
	make_ubi_img 128 1

	# 256k erase / 1B unit size - NOR
	make_ubi_img 256 1

	if [ "$build_nand_ubifs" = "1" ]; then
		echo "Calculating max. LEB count for UBI NAND..."
		max_leb_cnt=`get_ubi_max_leb 16 512`
		echo "  -> ${max_leb_cnt}"
		echo "Building NAND UBI images..."
		# 16k erase / 512B page - small NAND
		make_ubi_img 16 512

		# 128k erase / 2048B page - NAND
		make_ubi_img 128 2048

		# 256k erase / 4096B page - NAND
		make_ubi_img 256 4096

		# 512k erase / 4096B page - large NAND
		make_ubi_img 512 4096

		# 1MB erase / 4096B page - large NAND
		make_ubi_img 1024 4096

		# 1MB erase / 8192B page - large NAND
		make_ubi_img 1024 8192

		# 2MB erase / 4096B page - large NAND
		make_ubi_img 2048 4096

		# 2MB erase / 8192B page - large NAND
		make_ubi_img 2048 8192
	else
		echo "[INFO] Skipping NAND UBIFS images."
		echo "[INFO] Use option \"-u\" if they are needed."
	fi
else
	echo "[WARNING] Couldn't find mkfs.ubifs. Skipping ubifs images."
	echo "[WARNING] Please consider installing the respective package."
fi

if which mkfs.jffs2 >/dev/null; then
	# jffs2 NOR images for 64k, 128k, 256k erase sizes
	make_jffs2_img 64
	make_jffs2_img 128
	make_jffs2_img 256
else
	echo "[WARNING] Couldn't find mkfs.jffs2. Skipping jffs images."
	echo "[WARNING] Please consider installing the respective package."
fi

test -e "tmp/$WARN_FILE" && mv "tmp/$WARN_FILE" "$OUTPUT_DIR/target"
test -e "tmp/console" && mv "tmp/console" "$OUTPUT_DIR/target/dev"

rm -rf tmp

exit 0
