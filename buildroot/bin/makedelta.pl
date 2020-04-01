#!/usr/bin/perl -w

#
# parse VERSION, PATCHLEVEL, SUBLEVEL, EXTRAVERSION, and BRCM EXTRAVERSION
# out of the kernel Makefile.  Then generate diffs based on the appropriate
# git tags.
#

use strict;
use POSIX;

sub run($)
{
	my ($cmdline) = @_;

	print "+ $cmdline\n";
	system($cmdline);
	my $ret = WEXITSTATUS($?);
	if ($ret != 0) {
		die "$0: command exited with status $ret";
	}
	return(0);
}


if ($#ARGV < 1) {
	print STDERR "usage: $0 <linux_dir> <src_dir>\n";
	exit 1;
}

my $linux_dir = $ARGV[0];
my $src_dir = $ARGV[1];

open(F, "$linux_dir/Makefile") or die "can't open Linux makefile in $linux_dir";

my ($VERSION, $PATCHLEVEL, $SUBLEVEL, $EXTRAVERSION, $BRCMVER) =
	("", "", "", "", "");

while (<F>) {
	if (m/^VERSION = (\S+)/) {
		$VERSION = $1;
	} elsif (m/^PATCHLEVEL = (\S+)/) {
		$PATCHLEVEL = $1;
	} elsif (m/^SUBLEVEL = (\S+)/) {
		$SUBLEVEL = $1;
	} elsif (m/^#\s*EXTRAVERSION = (\S+)/) {
		$EXTRAVERSION = $1;
	} elsif (m/^EXTRAVERSION = (\S+)/) {
		$BRCMVER = $1;
	}
}
close(F);

my $upstream = "${VERSION}.${PATCHLEVEL}";
$upstream .= ".$SUBLEVEL" if ($SUBLEVEL);
$upstream .= "$EXTRAVERSION" if ($EXTRAVERSION);
my $rel = "${VERSION}.${PATCHLEVEL}${BRCMVER}";

run("rm -f $src_dir/delta-*-brcm-*.patch*");
run("(cd $linux_dir && git diff --diff-filter=M v$upstream..HEAD) > ".
	"$src_dir/delta-$rel-brcm-changed.patch");
run("(cd $linux_dir && git diff --diff-filter=A v$upstream..HEAD) > ".
	"$src_dir/delta-$rel-brcm-new.patch");
run("bzip2 $src_dir/delta-*-brcm-*.patch");

exit 0;
