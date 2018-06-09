#!/bin/sh

# $1 is the images directory. This is automatically passed in by BR.
# $2 is the Linux version. This is explicitly passed in via the BR config
#    option BR2_ROOTFS_POST_SCRIPT_ARGS

prg=`basename $0`

LINUX_STAMPS=".stamp_kconfig_fixup_done .stamp_built .stamp_target_installed"
LINUX_STAMPS="$LINUX_STAMPS .stamp_images_installed .stamp_initramfs_rebuilt"

if [ $# -lt 2 ]; then
	echo "usage: $prg <image-path> <linux-version>"
	exit 1
fi

image_path="$1"
linux_ver="$2"
# The output directory is one level up from the image directory.
output_path=`dirname "$image_path"`
linux_dir="linux-$linux_ver"
# The architecture is the last component in the output path.
arch=`basename "$output_path"`
test "$arch" = "mips" && arch="bmips"

rootfs_cpio="$image_path/rootfs.cpio"
rootfs_tar="$image_path/rootfs.tar"
nfs_tar="nfsroot-$arch.tar"

for s in $LINUX_STAMPS; do
	stamp="$output_path/build/$linux_dir/$s"
	if [ -r "$stamp" ]; then
		echo "Removing $stamp..."
		rm "$stamp"
	fi
done

echo "Removing CONFIG_BLK_DEV_INITRD from kernel config..."
tmp="$output_path/build/$$"
kern_config="$output_path/build/$linux_dir/.config"
fgrep -v "CONFIG_BLK_DEV_INITRD=y" "$kern_config" >"$tmp"
mv "$tmp" "$kern_config"

if [ -r "$rootfs_cpio" ]; then
	echo "Removing rootfs_cpio..."
	rm "$rootfs_cpio"
fi

echo "Creating NFS tar-ball..."
rm -f "${rootfs_tar}.gz"
mkdir "$image_path/romfs"
# We need fakeroot, so mknod doesn't complain.
fakeroot tar -C "$image_path/romfs" -x -f "$rootfs_tar"
(cd "$image_path"; tar -c -f "$nfs_tar.bz2" -j --owner=root --group=root romfs)
rm -rf "$image_path/romfs"
rm -f "${rootfs_tar}"

if [ "$arch" = "bmips" ]; then
	linux_image="$image_path/vmlinux"
else
	linux_image=`ls "$image_path"/*Image 2>/dev/null`
fi

echo "Creating initrd image..."
if [ "$arch" = "arm64" -o "$arch" = "bmips" ]; then
	gzip -9 "$linux_image"
	mv "$linux_image.gz" "$image_path/vmlinuz-initrd-$arch"
else
	mv "$linux_image" "$image_path/vmlinuz-initrd-$arch"
fi
echo "Creating plain kernel image..."
if [ "$arch" = "arm64" -o "$arch" = "bmips" ]; then
	gzip -9 "$linux_image.norootfs"
	mv "$linux_image.norootfs.gz" "$image_path/vmlinuz-$arch"
else
	mv "$linux_image.norootfs" "$image_path/vmlinuz-$arch"
fi
