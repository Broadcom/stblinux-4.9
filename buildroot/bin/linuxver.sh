#!/bin/sh

dot=''
prefix=''

error_usage() {
	echo "Usage: $0 (<options>) <linux_kernel_directory>" 1>&2
	exit 1
}


while getopts 'dhpqs' opt; do
	case $opt in
		d) dot=1;;
		h) help=1;;
		p) prefix=1;;
		q) quiet=1;;
		s) short=1;;
		*) exit 1;;
	esac
done

shift $((OPTIND - 1))

[ $# -eq 0 -o "$help" = "1" ] && error_usage

LINUX_MAKEFILE="$1/Makefile"

if [ ! -r "$LINUX_MAKEFILE" ]; then
	if [ -z $quiet ]; then
		echo "Error: couldn't read $LINUX_MAKEFILE" 1>&2
	fi
	exit 1
fi

# Extract the first 10 lines of the Makefile, just in case there are additional
# variables in there
VAR="$(cat $LINUX_MAKEFILE | grep -v '^#' | head -10 | sed 's/ //g')"
for line in $(echo $VAR)
do
	[ -z $version ] && version=$(echo $line | grep VERSION | cut -d= -f2)
	[ -z $patch_lvl ] && patch_lvl=$(echo $line | grep PATCHLEVEL | cut -d= -f2)
	# remove the pre here, we do not want that
	[ -z $extra_ver ] && extra_ver=$(echo $line | grep EXTRAVERSION | cut -d= -f2 | sed s/pre//)
	if [ -n "$version" ] && [ -n "$patch_lvl" ] && [ -n "$etra_ver" ]; then
		break
	fi
done

[ "$dot" = "1" ] && dot='.'
[ "$prefix" = "1" ] && prefix='stb-'
fullversion="$prefix$version$dot$patch_lvl"
[ "$short" != "1" ] && fullversion="$fullversion$extra_ver"

if [ -z "$fullversion" ]; then
	echo "Error: version not found in $LINUX_MAKEFILE" 1>&2
	exit 1
fi

echo "$fullversion"
