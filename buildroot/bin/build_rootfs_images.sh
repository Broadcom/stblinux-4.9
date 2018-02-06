#!/bin/bash

# Include /sbin and /usr/sbin in the PATH (for mkfs.jffs2, etc.)
export PATH=$PATH:/sbin:/usr/sbin

# UBIFS logical eraseblock limit - increase this number if mkfs.ubifs complains
max_leb_cnt=2047

# change to '1' to build NAND ubifs images (they might be large files,
# particularly for bigger eraseblock sizes)
build_nand_ubifs=0

LINUXDIR=linux
DEVTABLE=misc/devtable.txt
WARN_FILE="THIS_IS_NOT_YOUR_ROOT_FILESYSTEM"

TARGET=$1
if [ -d "target" -a -d "images" ]; then
	OUTPUT_DIR="."
else
	OUTPUT_DIR="output/$1"
fi

set -e

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
	fs="${OUTPUT_DIR}/target"

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

if [ $# -lt 1 ]; then
	echo "usage: $0 <arm|arm64|mips>" 1>&2
	exit 1
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

echo "Writing SQUASHFS image..."
rm -f "${OUTPUT_DIR}/images/squashfs-${TARGET}.img"
mksquashfs "${OUTPUT_DIR}/target" \
	"${OUTPUT_DIR}/images/squashfs-${TARGET}.img" \
	-processors 1 -root-owned -p "/dev/console c 0600 0 0 5 1"
chmod 0644 "${OUTPUT_DIR}/images/squashfs-${TARGET}.img"
echo "  -> ${OUTPUT_DIR}/images/squashfs-${TARGET}.img"
echo ""

# 64k erase / 1B unit size - NOR
make_ubi_img 64 1

# 128k erase / 1B unit size - NOR
make_ubi_img 128 1

# 256k erase / 1B unit size - NOR
make_ubi_img 256 1

if [ "$build_nand_ubifs" = "1" ]; then

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
	echo "Skipping NAND UBIFS images - set build_nand_ubifs=1 if they are needed"
	echo ""
fi

# jffs2 NOR images for 64k, 128k, 256k erase sizes
make_jffs2_img 64
make_jffs2_img 128
make_jffs2_img 256

#echo "Writing NFS rootfs tarball..."
#rm -f tmp/nfsroot.tar images/nfsroot-${TARGET}.tar.bz2
#rm -rf tmp/romfs

#mkdir -p tmp/romfs/boot
#cp $LINUXDIR/.config tmp/romfs/boot/config
#cp $LINUXDIR/Module.symvers tmp/romfs/boot/
#cp $LINUXDIR/System.map tmp/romfs/boot/
#cp $LINUXDIR/vmlinux tmp/romfs/boot/

#cp misc/devconsole.tar tmp/nfsroot.tar
#chmod u+w tmp/nfsroot.tar
#tar --exclude=".git*" --owner 0 --group 0 -rf tmp/nfsroot.tar romfs/
#tar --owner 0 --group 0 -rf tmp/nfsroot.tar -C tmp romfs/boot/
#bzip2 < tmp/nfsroot.tar > images/nfsroot-${TARGET}.tar.bz2
#echo "  -> images/nfsroot-${TARGET}.tar.bz2"

test -e "tmp/$WARN_FILE" && mv "tmp/$WARN_FILE" "$OUTPUT_DIR/target"
test -e "tmp/console" && mv "tmp/console" "$OUTPUT_DIR/target/dev"

rm -rf tmp

exit 0
