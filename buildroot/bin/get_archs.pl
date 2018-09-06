#!/usr/bin/perl -w

# STB Linux buildroot build system v1.0
# Copyright (c) 2017 Broadcom
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

use strict;
use JSON;

use constant ARCH_FILES => ( qw(misc/release_builds.master
					misc/release_builds.json) );
use constant ARCHS_MASTER => 'misc/release_builds.master';
use constant ARCHS => 'misc/release_builds.json';

my $prg = $0;
$prg =~ s|.*/||;

sub get_archs($)
{
	my ($ver) = @_;
	my $archs;

	if (open(F, ARCHS_MASTER)) {
		my @archs = <F>;

		chomp(@archs);
		$archs = join(' ', @archs);
	} elsif (open(F, ARCHS)) {
		my @json = <F>;
		my $json = join('', @json);
		my $arch_hash = decode_json($json);

		if (!defined($arch_hash->{$ver})) {
			$archs = undef;
		} else {
			$archs = join(' ', @{$arch_hash->{$ver}});
		}
	} else {
		return undef;
	}
	close(F);

	return $archs;
}

if ($#ARGV < 0)  {
	print(STDERR "usage: $prg <linux-kernel-tag>\n");
	exit(1);
}

my $version = $ARGV[0];
my $archs = get_archs($version);

if (defined($archs)) {
	print("$archs\n");
} else {
	print("Couldn't find architectures\n");
}
