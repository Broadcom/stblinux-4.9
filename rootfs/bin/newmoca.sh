#!/bin/bash

repos=/projects/stblinuxrel/RESTRICTED

usage="usage: $0 </path/to/moca/tarball> [ <jira_ticket> ]"

if [ -z "$1" ]; then
	echo $usage
	exit 1
fi

tarball="$1"
jid="$2"
base=$(basename $tarball)

if [[ "$tarball" = http* ]]; then
	local_tarball=/tmp/$base
	wget -O $local_tarball "$tarball" || exit 1
	tarball=$local_tarball
fi

if [[ "$base" = MoCADriver_* ]]; then
	mocadir=user/moca
	binaries="mocad,mocactl,soapserver"
	pkg=moca
else
	mocadir=user/moca2
	binaries="mocad,mocap"
	pkg=moca2
fi

if [ ! -d $mocadir ]; then
	echo "error: this program must be run from the uclinux-rootfs dir"
	exit 1
fi

oldver=$(cat $mocadir/version)

tc=$(cat toolchain)
if [ -d /opt/toolschains/$tc/bin ]; then
	export PATH=/opt/toolchains/$tc/bin:$PATH
elif [ -d /projects/stbtc/$tc/bin ]; then
	export PATH=/projects/stbtc/$tc/bin:$PATH
else
	if ! which mipsel-linux-gcc &> /dev/null; then
		echo "error: can't find toolchain $tc"
		exit 1
	else
		echo "warning: can't find toolchain $tc"
	fi
fi

set -ex

LIBCDIR="eglibc"

DIRARM=arm.$LIBCDIR
DIRMIPSEL=mipsel.$LIBCDIR
DIRMIPS=mips.$LIBCDIR

rm -rf $mocadir/{src,fw,$DIRMIPSEL,$DIRMIPS,$DIRARM}
mkdir $mocadir/{src,fw,$DIRMIPSEL,$DIRMIPS,$DIRARM}

cp $tarball $mocadir/src/TARBALL

pushd $mocadir/src
if [[ "$tarball" = *tar ]]; then
	tar xf TARBALL
	base="${base}.bz2"
	if [ -d $repos -a ! -e $repos/$base ]; then
		bzip2 < TARBALL > $repos/$base
	fi
elif [[ "$tarball" = *bz2 ]]; then
	tar jxf TARBALL
	if [ -d $repos -a ! -e $repos/$base ]; then
		cp TARBALL $repos/$base
	fi
else
	echo "error: don't know how to handle file type of $tarball"
	exit 1
fi

cp moca*-gen*.bin ../fw/

make clean
make CROSS=arm-linux-
eval cp bin/{$binaries} ../$DIRARM/
arm-linux-strip --strip-all ../$DIRARM/*
make clean
make CROSS=mipsel-linux-
eval cp bin/{$binaries} ../$DIRMIPSEL/
mipsel-linux-strip --strip-all ../$DIRMIPSEL/*
make clean
make CROSS=mips-linux-
eval cp bin/{$binaries} ../$DIRMIPS/
mips-linux-strip --strip-all ../$DIRMIPS/*

popd

ver=${base%.tar.bz2}
echo $ver > $mocadir/version

rm -rf $mocadir/src

if [ -n "$jid" ]; then
	git add $mocadir
	echo -e "$pkg: Update to $ver\n\nrefs #$jid" | git commit -sF -
fi

set +ex

echo ""
echo "Update complete"
echo ""
echo "Old version: $oldver"
echo "New version: $ver"
echo ""

exit 0
