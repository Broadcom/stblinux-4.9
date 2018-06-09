#!/bin/sh

prg=`basename $0`
target_dir="$1"

first_hit=1

if [ $# != 1 ]; then
	echo "usage: $prg <target-dir>" 1>&2
	exit 1
fi

for lib in "$target_dir/lib" "$target_dir/usr/lib"; do
	for l in `find $lib -type f | egrep -v '(modules|plugins)'`; do
		file $l | grep 'ELF 64-bit' >/dev/null
		if [ $? != 0 ]; then
			continue
		fi

		if [ "$first_hit" = "1" ]; then
			first_hit=0
cat <<EOF
WARNING: Found 64-bit shared libs in 32-bit directory! I'll move them.
WARNING: Please fix the packages listed below to install their libraries
WARNING: into the proper location, most likely /usr/\$(BR2_ROOTFS_LIB_DIR).

EOF
		fi

		# Strip the version portion from the library name
		lib_base=`echo "$l" | sed -e 's/\.so.*/.so/'`
		library=`basename "$lib_base"`
		echo "Found $library, referenced by:"
		for b in bin usr/bin sbin usr/sbin; do
			grep -l "$library" "$target_dir/$b"/* 2>/dev/null | \
				sed -e "s|$target_dir|	|"
		done
		mv "$lib_base"* "${lib}64"
	done
done
