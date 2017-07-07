#!/usr/bin/perl -w

#
# Print /path/to/toolchain:$PATH if a suitable toolchain can be found:
# bin/checkpath.pl -a
#
# Print errors/warnings if a suitable toolchain is not currently in $PATH:
# bin/checkpath.pl
#

use strict;
use warnings;

my @searchpath = ( "/opt/toolchains", "/projects/stbtc",
	"/projects/stbopt_p/toolchains_303" );

my ($add, $kern_ver) = (0, 0);
while (@ARGV) {
	my $arg = shift @ARGV;
	if ($arg eq "-a") {
		$add = 1;
	} elsif ($arg =~ m/^([0-9].[0-9]+)(-\S+)?$/) {
		$kern_ver = $1;
	} else {
		print STDERR "Unknown parameter: $arg\n";
		exit 1;
	}
}

my $file = "<toolchain";
if ($kern_ver >= 4.9) {
	$file = "<toolchain_63";
} else {
	$file = "<toolchain";
}

open(F, $file) or die "can't open toolchain file";
my $toolchain = <F>;
close(F);

$toolchain =~ s/[\r\n]//g;
my $path = $ENV{'PATH'};

foreach my $x (split(/:/, $path)) {
	if(glob("$x/mipsel-linux-*gcc")
	   || glob("$x/mips-linux-*gcc")
	   || glob("$x/arm-linux-*gcc")) {
		if($x =~ m/$toolchain/) {
			# matches the recommended toolchain
			if($add) {
				print "$path\n";
			}
			exit 0;
		}
		if($add == 0) {
			if(($x !~ m/crosstools_hf-/) &&
			   ($x !~ m/stbgcc-/)) {
				   # user has renamed the toolchain - assume
				   # they know what they are doing
				   exit 0;
			}
			print "\n";
			print "WARNING: using toolchain binaries in:\n$x\n\n";
			print "The recommended toolchain for this release is:\n";
			print "$toolchain\n";
			print "\n";

			sleep(10);
			exit 0;
		}

		print "$path\n";
		exit 0;
	}
}

if($add == 1) {
	foreach my $x (@searchpath) {
		my $y = "$x/$toolchain/bin";
		if(   -e "$y/mipsel-linux-gcc"
		   || -e "$y/mips-linux-gcc"
		   || -e "$y/arm-linux-gcc") {
			print "$y:$path\n";
			exit 0;
		}
	}
	print "$path\n";
	exit 1;
} else {
	print STDERR "\n";
	print STDERR "ERROR: Toolchain is not in \$PATH\n";
	print STDERR "paths searched for are:\n";
	foreach my $path (@searchpath) {
		print STDERR "* $path\n";
	}
	print STDERR "\n";

	exit 1;
}
