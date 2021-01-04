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
use warnings;
use Fcntl ':mode';
use File::Basename;
use File::Path qw(make_path remove_tree);
use Getopt::Std;
use JSON;
use LWP::UserAgent;
use POSIX;
use Socket;

use constant AUTO_MK => qw(brcmstb.mk);
use constant LOCAL_MK => qw(local.mk);
use constant BR_FRAG_FILE => qw(br_fragments.cfg);
use constant KERNEL_FRAG_FILE => qw(k_fragments.cfg);

use constant BR_MIRROR_PROTOCOL => qw(https://);
use constant BR_MIRROR_HOST => qw(stbgit.stb.broadcom.net);
use constant BR_MIRROR_PATH => qw(/mirror/buildroot);
use constant FORBIDDEN_PATHS => ( qw(. /tools/bin) );
# Trailing space after the user agent tells Perl to append "libwww-perl/x.y.z".
use constant HTTP_USER_AGENT => q(BRCMSTB/br_config.pl );
use constant MERGED_FRAGMENT => qw(merged_fragment);
use constant PRIVATE_CCACHE => qw($(HOME)/.buildroot-ccache);
use constant SHARED_CCACHE => qw(/local/users/stbdev/buildroot-ccache);
use constant SHARED_OSS_DIR => qw(/projects/stbdev/open-source);
use constant STB_CMA_DRIVER => qw(include/linux/brcmstb/cma_driver.h);
use constant TOOLCHAIN_DIR => qw(/opt/toolchains);
use constant TOOLCHAIN_FILE_CLASSIC => qw(misc/toolchain);
use constant TOOLCHAIN_FILE_JSON => qw(misc/toolchain.json);
use constant TOOLCHAIN_FILE_MASTER => qw(misc/toolchain.master);
use constant VERSION_FRAGMENT => qw(local_version.txt);
use constant VERSION_H => qw(/usr/include/linux/version.h);

use constant SHA_LEN => 12;
use constant SLEEP_TIME => 5;
use constant STALE_THRESHOLD => 7 * 24 * 60 * 60; 	# days in seconds
use constant WORLD_PERMS => (S_IRWXG | S_IRWXO);

my %compiler_map = (
	'arm64' => 'aarch64-linux-gcc',
	'arm' => 'arm-linux-gcc',
	'bmips' => 'mipsel-linux-gcc',
);

my %arch_config = (
	'arm64' => {
		'arch_name' => 'aarch64',
		'BR2_aarch64' => 'y',
		'BR2_cortex_a53' => 'y',
		'BR2_LINUX_KERNEL_DEFCONFIG' => 'brcmstb',
	},
	'arm' => {
		'arch_name' => 'arm',
		'BR2_arm' => 'y',
		'BR2_cortex_a15' => 'y',
		'BR2_LINUX_KERNEL_DEFCONFIG' => 'brcmstb',
	},
	'bmips' => {
		'arch_name' => 'mips',
		'BR2_mipsel' => 'y',
		'BR2_MIPS_SOFT_FLOAT' => '',
		'BR2_MIPS_FP32_MODE_32' => 'y',
		'BR2_LINUX_KERNEL_DEFCONFIG' => 'bmips_stb',
		'BR2_LINUX_KERNEL_VMLINUX' => 'y',
		'BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION' => 'stb-4.1',
	},
);

# It doesn't look like we need to set BR2_TOOLCHAIN_EXTERNAL_CUSTOM_PREFIX
# with stbgcc-6.3-x.y, since it has all the required symlinks.
my %toolchain_config = (
	'arm64' => {
#		'BR2_TOOLCHAIN_EXTERNAL_CUSTOM_PREFIX' => '$(ARCH)-linux-gnu'
	},
	'arm' => {
#		'BR2_TOOLCHAIN_EXTERNAL_CUSTOM_PREFIX' => '$(ARCH)-linux-gnueabihf'
	},
	'bmips' => {
#		'BR2_TOOLCHAIN_EXTERNAL_CUSTOM_PREFIX' => '$(ARCH)-linux-gnu'
	},
);

my %generic_config = (
	'BR2_LINUX_KERNEL_CUSTOM_REPO_URL' =>
				'git://stbgit.stb.broadcom.net/queue/linux.git',
	'BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION' => 'stb-4.9',
);

sub check_br()
{
	my $readme = 'README';

	# README file must exist
	return -1 if (! -r $readme);

	open(F, $readme);
	$_ = <F>;
	close(F);

	# First line must contain "Buildroot"
	return 0 if (/Buildroot/);

	return -1;
}

# Check if the shared open source directory exists
sub check_open_source_dir()
{
	return  (-d SHARED_OSS_DIR) ? 1 : 0;
}

# Without this explicit function prototype, Perl will complain that the
# recursive call inside fix_shared_permission() is happening too early
# to check the prototype.
sub fix_shared_permissions($);

sub fix_shared_permissions($)
{
	my ($dir) = @_;
	my @st = stat($dir);
	my $mode = $st[2] & 0777;
	my $fuid = $st[4];
	my $uid = $>;
	my $dh;

	# If directory doesn't have rwx permissions for group and other, we
	# add them. We silently fail if the attempt is denied, since there is
	# nothing more we can do.
	if ($uid == $fuid && (($mode & WORLD_PERMS) != WORLD_PERMS)) {
		chmod(0777, $dir) && print("Fixed permissions of $dir...\n");
		print("Setting ACL for $dir...\n");
		system("setfacl -m default:user::rwx $dir");
		system("setfacl -m default:group::rwx $dir");
		system("setfacl -m default:other::rwx $dir");
		system("setfacl -m default:mask::rwx $dir");
	}
	opendir($dh, $dir);
	while (my $entry = readdir($dh)) {
		# Skip "." and ".."
		next if ($entry =~ /^\.{1,2}$/);
		if (-d "$dir/$entry") {
			fix_shared_permissions("$dir/$entry");
		}
	}
	closedir($dh);
}

sub get_ccache_dir($)
{
	my ($shared_cache) = @_;
	my $base_dir = dirname($shared_cache);
	my $top_dir = dirname($base_dir);
	my $ret = $shared_cache;
	my $dh;

	# Can't have a shared cache if:
	#   * top dir doesn't exist or
	#   * top dir isn't writable and base dir doesn't exist
	if (! -d $top_dir || (! -w $top_dir && ! -d $base_dir)) {
		return PRIVATE_CCACHE;
	}

	if (! -d $base_dir) {
		mkdir($base_dir);
		chmod(0777, $base_dir);
	}

	# Can't have a shared cache if:
	#   * shared cache doesn't exist and base dir isn't writable
	return PRIVATE_CCACHE if (! -d $shared_cache && ! -w $base_dir);

	if (! -d $shared_cache) {
		mkdir($shared_cache);
		chmod(0777, $shared_cache);
	}

	# Lastly, the shared cache itself and its sub-directories must be
	# writable.
	if (! -w $shared_cache) {
		return PRIVATE_CCACHE;
	}

	opendir($dh, $shared_cache);
	while (my $entry = readdir($dh)) {
		# We only care about directories named 0-9 and a-f, as well as
		# tmp.
		if ($entry =~ /^[0-9a-f]$/ || $entry eq 'tmp') {
			if (! -w "$shared_cache/$entry") {
				$ret = PRIVATE_CCACHE;
				last;
			}
		}
	}
	closedir($dh);

	fix_shared_permissions($shared_cache);

	return $ret;
}

# Find downloaded Linux tar-balls
sub find_tarball($)
{
	my ($dir) = @_;
	my @found = ();
	my $dh;

	opendir($dh, $dir);
	while (readdir($dh)) {
		if (/linux-stb.*\.tar/) {
			push(@found, $_);
		}
	}
	closedir($dh);

	return @found;
}

# Search for outdated sources in the download directory and delete them.
sub check_oss_stale_sources($$)
{
	my ($dir, $output_dir) = @_;
	my $linux = "$dir/linux";
	my @tar_balls;

	print("Checking $dir for stale sources...\n");
	if (-d $linux) {
		my $now = time();
		my $mtime;

		if (-d "$linux/git") {
			$mtime = (stat("$linux/git"))[9];
		} else {
			$mtime = (stat($linux))[9];
		}

		@tar_balls = find_tarball($linux);

		if ($now - $mtime > STALE_THRESHOLD) {
			print("$linux is stale, removing it...\n");
			remove_tree($linux, {keep_root => 1});

			# Use the tar-balls we found to remove Linux build
			# directories if they exist.
			foreach my $v (@tar_balls) {
				my $linux_build;
				$v =~ s/\.tar.*//;
				$linux_build = "$output_dir/build/$v";
				if (-d $linux_build) {
					print("Removing $linux_build...\n");
					remove_tree($linux_build);
				}
			}
		}
	}
}

sub find_stb_toolchain_match($$)
{
	my ($tc_data, $ver) = @_;

	foreach my $entry (@$tc_data) {
		my ($cur_ver, $cur_tc) = @$entry;

		# Ensure it's a regex before we attempt any matching. If we
		# we tried matching against all version entries, it would
		# lead to unexpected results.
		# E.g. using stb-4.1 as regex would match stb-4.16.
		if ($cur_ver !~ /^stb-\d+\.\d+$/ && $ver =~ /$cur_ver/) {
			return $cur_tc;
		}
	}

	return undef;
}

sub find_stb_toolchain_entry($$)
{
	my ($tc_data, $ver) = @_;

	foreach my $entry (@$tc_data) {
		if ($entry->[0] eq $ver) {
			return $entry->[1];
		}
	}

	return undef;
}

sub get_json_toolchain($$)
{
	my ($file, $local_linux) = @_;
	my ($version, $patch, $extra) = get_stbrelease($local_linux);
	my @json;
	my $json;
	my $kernel_version;
	my $tc_data;
	my $tc;

	return undef if (!open(F, $file));

	@json = <F>;
	$json = join('', @json);
	close(F);

	# Escape backslashes. decode_json() expects it.
	$json =~ s/\\/\\\\/g;
	$tc_data = decode_json($json);

	if (defined($version)) {
		$kernel_version = "stb-$version.$patch";
	} else {
		$kernel_version =
			$generic_config{'BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION'};
	}

	# There's an exact match. Return it.
	$tc = find_stb_toolchain_entry($tc_data, $kernel_version);
	return $tc if (defined($tc));

	# Try pattern matching to determine the recommended toolchain.
	return find_stb_toolchain_match($tc_data, $kernel_version);
}

sub get_plaintext_toolchain($)
{
	my ($file) = @_;
	my $recommended;

	return undef if (!open(F, $file));

	$recommended = <F>;
	chomp($recommended);
	close(F);

	return $recommended;
}

sub get_recommended_toolchain($)
{
	my ($local_linux) = @_;
	my $recommended;

	# Try it the master repo first.
	$recommended = get_plaintext_toolchain(TOOLCHAIN_FILE_MASTER);
	if (defined($recommended)) {
		return $recommended;
	}

	# Next, let's look for the JSON file with toolchain information.
	$recommended = get_json_toolchain(TOOLCHAIN_FILE_JSON, $local_linux);
	if (defined($recommended)) {
		return $recommended;
	}

	# Lastly, let's try the classic plain-text file.
	$recommended = get_plaintext_toolchain(TOOLCHAIN_FILE_CLASSIC) || '';

	return $recommended;
}

# Check if the specified toolchain is the recommended one.
sub check_toolchain($$)
{
	my ($toolchain, $local_linux) = @_;
	my $recommended;

	$toolchain =~ s|.*/||;
	$recommended = get_recommended_toolchain($local_linux);

	# If we don't know what the recommended toolchain is, we accept the
	# one that was specified.
	return ($recommended ne $toolchain) ? $recommended : '';
}

my @linux_build_artefacts = (
	".config",
	"vmlinux",
	"*.o",
	"*.s",
	"generated/",
	"vmlinuz",
	"System.map",
);

# Check the host environment for troublesome settings.
sub sanity_check($)
{
	my ($prg) = @_;
	my @path = split(/:/, $ENV{'PATH'});

	foreach my $p (@path) {
		foreach my $f (FORBIDDEN_PATHS) {
			if ($f eq $p) {
				print(STDERR "$prg: \"$f\" must not be in ".
					"your PATH\n");
				print(STDERR $ENV{'PATH'}."\n");
				return 0;
			}
		}
	}

	return 1;
}

# Check for some obvious build artifacts that show us the local Linux source
# tree is not clean.
sub check_linux($)
{
	my ($local_linux) = @_;

	foreach (@linux_build_artefacts) {
		return 0 if (-e "$local_linux/$_");
	}

	return 1;
}

sub check_cma_driver($)
{
	my ($local_linux) = @_;

	return (-r "$local_linux/".STB_CMA_DRIVER);
}

sub get_cores()
{
	my $num_cores;

	$num_cores = `getconf _NPROCESSORS_ONLN 2>/dev/null`;
	# Maybe somebody wants to run this on BSD? :-)
	if ($num_cores eq '') {
		$num_cores = `getconf NPROCESSORS_ONLN 2>/dev/null`;
	}
	# Still no luck, try /proc.
	if ($num_cores eq '') {
		$num_cores = `grep -c -P '^processor\\s+:' /proc/cpuinfo 2>/dev/null`;
	}
	# Can't figure out the number of cores. Assume just 1 core.
	if ($num_cores eq '') {
		$num_cores = 1;
	}
	chomp($num_cores);

	return $num_cores;
}

sub get_stbrelease($)
{
	my ($linux_dir) = @_;
	my ($version, $patch_level, $extra_version);

	if (!defined($linux_dir)) {
		return undef;
	}
	if (!open(F, "$linux_dir/Makefile")) {
		return undef;
	}

	while (my $line = <F>) {
		chomp($line);
		if ($line =~ /^VERSION\s+=\s+(.*)/) {
			$version = $1;
		}
		if ($line =~ /^PATCHLEVEL\s*=\s*(.*)/) {
			$patch_level = $1;
		}
		if ($line =~ /^EXTRAVERSION\s*=\s*(.*)/) {
			$extra_version = $1;
			# No need to keep parsing the Makefile.
			last;
		}
	}
	close(F);

	return ($version, $patch_level, $extra_version);
}

sub get_stbrelease_string($)
{
	my ($linux_dir) = @_;
	my $ret;
	my ($version, $patch_level, $extra_version) =
		get_stbrelease($linux_dir);

	if (!defined($version)) {
		return '<unknown>';
	}

	$ret = $version;
	if (defined($patch_level)) {
		$ret .= ".$patch_level";
	}
	if (defined($extra_version)) {
		$ret .= $extra_version;
	}

	return $ret;
}

sub get_libc($$)
{
	my ($toolchain, $arch) = @_;
	my $full_path = "$toolchain/bin/".$compiler_map{$arch};
	my $compiler = readlink($full_path);

	if (!defined($compiler)) {
		return undef;
	}
	if ($compiler =~ /(musl|uclibc)/) {
		return $1;
	}

	return 'glibc';
}

sub find_toolchain($)
{
	my ($toolchain_ver) = @_;
	my @path = split(/:/, $ENV{'PATH'});
	my @toolchains;
	my $dh;

	foreach my $dir (@path) {
		# We don't support anything before stbgcc-6.x at this point.
		if ($dir =~ /stbgcc-[6-9]/ && $dir =~ $toolchain_ver) {
			$dir =~ s|/bin/?$||;
			# Only use the directory if it actually exists.
			if (-d $dir) {
				return $dir;
			}
		}
	}

	# If we didn't find a toolchain in the $PATH, we look in the standard
	# location.
	return undef unless (opendir($dh, TOOLCHAIN_DIR));

	# Sort in reverse order, so newer toolchains appear first. Also, make
	# sure we only match stbgcc version 6 and newer. Lastly, the toolchain
	# directory must end with a digit (e.g. stbgcc-6.3-1.7). This excludes
	# development toolchains that may have a suffix after the version number
	# from being searched automatically.
	@toolchains = sort { $b cmp $a }
		grep { /stbgcc-[6-9].*\d$/ } readdir($dh);
	closedir($dh);

	foreach my $dir (@toolchains) {
		my $d = TOOLCHAIN_DIR."/$dir";

		# If the toolchain version matches or if the version is empty,
		# return the toolchain, provided the "bin" directory exists.
		if (-d "$d/bin" && ($dir =~ $toolchain_ver ||
		    $toolchain_ver eq '')) {
			return $d;
		}
	}

	return undef;
}

sub set_target_toolchain($$)
{
	my ($toolchain, $arch) = @_;
	my $stbgcc = $toolchain."/bin/".$compiler_map{$arch};
	my $version = `$stbgcc -v 2>&1 | grep 'gcc version'`;
	my $libc = get_libc($toolchain, $arch) || '';
	my $libc_sel = 'BR2_TOOLCHAIN_EXTERNAL_CUSTOM_'.uc($libc);

	if (!-e $stbgcc) {
		return -1;
	}
	if ($libc eq '') {
		return -2;
	}

	if ($version =~ /\s+(\d+)\.(\d+)\.(\d+)/) {
		my ($major, $minor, $patch) = ($1, $2, $3);
		my $config_str = "BR2_TOOLCHAIN_EXTERNAL_GCC_$major";

		print("Detected GCC $major ($major.$minor)...\n");
		print("C library is $libc...\n");
		$toolchain_config{$arch}{$config_str} = 'y';
	} else {
		print("WARNING! Couldn't determine GCC version number. ".
			"Build may fail.\n");
		print("Toolchain: $toolchain\n");
	}

	$toolchain_config{$arch}{$libc_sel} = 'y';
	$toolchain_config{$arch}{'BR2_TOOLCHAIN_EXTERNAL_PATH'} = $toolchain;

	return 0;
}

sub get_linux_remote()
{
	return $generic_config{'BR2_LINUX_KERNEL_CUSTOM_REPO_URL'};
}

sub verify_mirror_host()
{
	my $addr = gethostbyname(BR_MIRROR_HOST);
	my ($a, $b, $c, $d);

	# If we can't resolve it, we can't use it.
	if (!defined($addr)) {
		return 0;
	}

	($a, $b, $c, $d) = unpack('W4', $addr);

	# If it's a private IP on the 10.x.x.x, 172.14.x.x-172.31.x.x or
	# 192.168.x.x networks, we are good to go.
	if ($a == 10 ||
	    ($a == 172 && $b >= 14 && $b <= 31) ||
	    ($a == 192 && $b == 168)) {
		return 1;
	}

	# If it's a public IP, it won't be the server we are looking for.
	return 0;
}

sub get_br_mirror_host()
{
	my $br_mirror = BR_MIRROR_PROTOCOL.BR_MIRROR_HOST.BR_MIRROR_PATH;
	my $ua = LWP::UserAgent->new('agent' => HTTP_USER_AGENT);
	my $res;

	# Only use the Broadcom mirror if we can resolve the name *AND* it is
	# a private IP address. Otherwise, we'll either run into a DNS timeout
	# for every package we need to download or we'll try to download the
	# packages from a server that isn't actually BR_MIRROR_HOST.
	if (!verify_mirror_host()) {
		return undef;
	}

	# Check if BR_MIRROR_PATH exists on BR_MIRROR_HOST
	$res = $ua->head($br_mirror);
	if (!$res->is_success) {
		return undef;
	}

	return $br_mirror;
}

sub resolve_remote($)
{
	my ($remote) = @_;

	# If it's a URL, extract the host portion.
	if ($remote =~ /^\w+:\/\/([^\/:]+)/) {
		return defined(gethostbyname($1));
	}

	# Not a URL. Try to resolve it as-is.
	return defined(gethostbyname($remote));
}

sub resolve_linux_remote()
{
	return resolve_remote(get_linux_remote());
}

sub trigger_toolchain_sync($$)
{
	my ($output_dir, $arch) = @_;
	my $path;
	my @files;

	# First, we delete all toolchain symlinks
	$path = "$output_dir/host/bin";
	if (!opendir(D, $path)) {
		return;
	}
	@files = grep { /^$arch/ } readdir(D);
	closedir(D);
	foreach my $f (@files) {
		unlink("$path/$f") if (-l "$path/$f");
	}

	# Secondly, we delete the stamp files, so BR knows to re-sync the
	# toolchain.
	$path = "$output_dir/build/toolchain-external-custom";
	if (!opendir(D, $path)) {
		return;
	}
	@files = grep { /^(\.stamp|.applied)/ } readdir(D);
	closedir(D);
	foreach my $f (@files) {
		unlink("$path/$f");
	}
}

sub get_kconfig_var($$)
{
	my ($fname, $key) = @_;
	my $val;

	open(F, $fname) || return undef;
	while (<F>) {
		if (/^$key\s*=\s*(.*)/) {
			$val = $1;
			$val =~ s/["']//g;
			last;
		}
	}
	close(F);

	return $val;
}

sub get_kernel_header_version($$)
{
	my ($toolchain, $arch) = @_;
	my ($compiler_arch, $sys_root, $version_path);
	my $version_code;

	$compiler_arch = $arch_config{$arch}->{'arch_name'};
	# The MIPS compiler may be called "mipsel-*" not just "mips-*".
	if (defined($arch_config{$arch}->{'BR2_mipsel'})) {
		$compiler_arch .= "el";
	}
	$sys_root = `ls -d "$toolchain/$compiler_arch"*/sys*root 2>/dev/null`;
	chomp($sys_root);
	if ($sys_root eq '') {
		return undef;
	}
	$version_path = $sys_root.VERSION_H;

	open(F, $version_path) || return undef;
	while (<F>) {
		chomp;
		if (/LINUX_VERSION_CODE\s+(\d+)/) {
			$version_code = $1;
			last;
		}
	}
	close(F);

	return undef if (!defined($version_code));

	return [($version_code >> 16) & 0xff, ($version_code >> 8) & 0xff];
}

sub get_linux_sha($$$)
{
	my ($fragments, $fragment_dir, $cmd) = @_;

	my $version_fragment = "$fragment_dir/".VERSION_FRAGMENT;
	my $git_sha = `$cmd`;

	chomp($git_sha);
	if ($git_sha eq '') {
		print("Couldn't determine SHA for kernel.\n".
			"Was using \"$cmd\".\n");
		unlink($version_fragment) if (-e $version_fragment);
		return $fragments;
	}
	print("Local Linux kernel is version $git_sha...\n");
	open(F, ">$version_fragment");
	print(F "CONFIG_LOCALVERSION=\"-g$git_sha\"\n");
	close(F);

	if (!defined($fragments)) {
		return '';
	}

	$version_fragment .= "," if ($fragments ne '');

	return $version_fragment.$fragments;
}

sub get_linux_sha_local($$$)
{
	my ($fragments, $fragment_dir, $linux_dir) = @_;
	my $git_dir = "$linux_dir/.git";

	# Let the caller know that there's no GIT SHA
	if (!-e $git_dir) {
		return undef;
	}

	# If the .git entry is a file rather than a directory, it means
	# we are dealing with a submodule. We handle that case first.
	if (-f $git_dir) {
		open(F, $git_dir);
		while (<F>) {
			chomp;
			if (m|^gitdir:\s+(.*/linux$)|) {
				$git_dir = $1;
				last;
			}
		}
		close(F);
	}
	if (-d $git_dir) {
		my $git_cmd = "git --git-dir=\"$git_dir\" rev-parse ".
			"--short=".SHA_LEN." HEAD";
		$fragments = get_linux_sha($fragments, $fragment_dir, $git_cmd);
	}

	return $fragments;
}

sub get_linux_sha_remote($$)
{
	my ($fragments, $fragment_dir) = @_;

	my $git_remote = get_linux_remote();
	my $git_branch =
		$generic_config{'BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION'};
	my $git_cmd = "git ls-remote \"$git_remote\" | ".
			"grep \"refs/heads/$git_branch\$\" | ".
			"awk '{ print \$1 }' | ".
			"cut -c1-".SHA_LEN;

	return get_linux_sha($fragments, $fragment_dir, $git_cmd);
}

sub parse_cmdline_fragments($$)
{
	my ($out_file, $frag_str) = @_;
	my @frags = split(/[;,]/, $frag_str);

	return '' if ($frag_str eq '');

	printf("Generating temporary fragment file $out_file...\n");

	# This function strips white space and quotes around fragments, as it is
	# fairly easy to end up with extra quotes or spaces when passing config
	# fragments on the command line. (They may have to be escaped from the
	# shell, after all.)

	open(F, ">$out_file");
	foreach my $frag (@frags) {
		# Strip leading and trailing whitespace.
		$frag =~ s/^\s+//;
		$frag =~ s/\s+$//;
		if ($frag =~ /^["']/) {
			# Strip quotes around the entire fragment. Make sure the
			# quote at the end is the same as at the beginning.
			$frag =~ s/^(["'])(.*)\1$/$2/;
		}
		print(F "$frag\n");
	}
	close(F);

	return $out_file;
}

sub merge_br_fragments($$$)
{
	my ($prg, $output_dir, $fragments) = @_;
	my $out_frag = "$output_dir/".MERGED_FRAGMENT;
	my $ret = $out_frag;

	open(D, ">$out_frag");

	foreach my $frag (split(/,/, $fragments)) {
		print("Processing BR fragment $frag...\n");
		if (!open(S, $frag)) {
			print(STDERR "$prg: couldn't open $frag\n");
			$ret = undef;
			last;
		}
		while (my $line = <S>) {
			chomp($line);
			print(D "$line\n");
		}
		close(S);
	}

	close(D);

	unlink($out_frag) if (!defined($ret));

	return $ret;
}

sub move_merged_config($$$$)
{
	my ($prg, $arch, $sname, $dname) = @_;
	my $line;

	open(S, $sname) || die("couldn't open $sname");
	open(D, ">$dname") || die("couldn't create $dname");
	print(D "#" x 78, "\n".
		"# This file was automatically generated by $prg.\n".
		"#\n".
		"# Target architecture: ".uc($arch)."\n".
		"#\n".
		"# ".("-- DO NOT EDIT!!! " x 3)."--\n".
		"#\n".
		"# ".strftime("%a %b %e %T %Z %Y", localtime())."\n".
		"#" x 78, "\n\n");
	while ($line = <S>) {
		chomp($line);
		print(D "$line\n");
	}
	close(D);
	close(S);
	unlink($sname);
}

sub write_localmk($$)
{
	my ($prg, $output_dir) = @_;
	my $local_dest = "$output_dir/".LOCAL_MK;
	my @buf;


	if (open(F, $local_dest)) {
		my $auto_mk = AUTO_MK;

		@buf = <F>;
		close(F);
		# Check if we are already including our auto-generated makefile
		# snippet. Bail if we do.
		foreach my $line (@buf) {
			return if ($line =~ /include .*$auto_mk/);
		}
	}

	# Add header and include directive for our auto-generated makefile.
	open(F, ">$local_dest");
	print(F "#" x 78, "\n".
		"# The following include was added automatically by $prg.\n".
		"# Please do not remove it. Delete ".AUTO_MK." instead, ".
		"if necessary.\n".
		"# You may also add your own make directives underneath.\n".
		"#" x 78, "\n".
		"#\n".
		"-include $output_dir/".AUTO_MK."\n".
		"#\n".
		"# Custom settings start below.\n".
		"#" x 78, "\n\n");

	# Preserve the contents local.mk had before we started modifying it.
	foreach my $line (@buf) {
		chomp($line);
		print(F $line."\n");
	}

	close(F);
}

sub write_brcmstbmk($$$)
{
	my ($prg, $output_dir, $linux_dir) = @_;
	my $auto_dest = "$output_dir/".AUTO_MK;

	open(F, ">$auto_dest");
	print(F "#" x 78, "\n".
		"# Do not edit. Automatically generated by $prg. It may also ".
		"be deleted\n".
		"# without warning by $prg.\n".
		"#" x 78, "\n".
		"#\n".
		"# You may delete this file manually to remove the settings ".
		"below.\n".
		"#\n".
		"#" x 78, "\n\n".
		"LINUX_OVERRIDE_SRCDIR = $linux_dir\n" .
		"LINUX_OVERRIDE_SRCDIR_RSYNC_EXCLUSIONS = \\\n" .
		"\t--exclude-from=\"$linux_dir/.gitignore\"\n");
	print(F "\n");
	close(F);
}

sub write_config($$$)
{
	my ($config, $fname, $truncate) = @_;

	unlink($fname) if ($truncate);

	open(F, ">>$fname");
	foreach my $key (keys(%$config)) {
		my $val = $config->{$key};

		# Only write keys that start with BR2_ to config file.
		next if ($key !~ /^BR2_/);

		if ($val eq '') {
			print(F "# $key is not set\n");
			next;
		}

		# Numbers and 'y' don't require quotes. Strings do.
		if ($val ne 'y' && $val !~ /^\d+$/) {
			$val = "\"$val\"";
		}

		print(F "$key=$val\n");
	}
	close(F);
}

sub print_host_info($$)
{
	my ($orig_cmdline, $local_linux) = @_;
	my $host_gcc_ver = `gcc -v 2>&1 | grep '^gcc'`;
	my $host_kernel_ver = `uname -r`;
	my $host_name = `hostname -f 2>/dev/null`;
	my $host_os_ver = `lsb_release -d 2>/dev/null`;
	my $host_perl_ver = `perl -v | grep '^This is'`;
	my $stb_release = get_stbrelease_string($local_linux);
	my @br_vars = grep { /^BR_/ } keys(%ENV);
	my $host_addr;

	chomp($host_name);
	chomp($host_kernel_ver);
	$host_gcc_ver =~ s/(.*\S)\s*\n/$1/s;
	$host_perl_ver =~ s/.*\(([^)]+)\).*/$1/s;
	$host_os_ver =~ s/.*:\s+(.*)\n$/$1/s;
	$host_addr = inet_ntoa(inet_aton($host_name)) || '';

	print("Host is running $host_os_ver...\n");
	print("Host kernel is $host_kernel_ver...\n");
	print("Host name is $host_name ($host_addr)...\n");
	print("Host GCC is $host_gcc_ver...\n");
	print("Host perl is $host_perl_ver...\n");

	print("Host environment:\n") if ($#br_vars >= 0);
	foreach my $key (@br_vars) {
		print("\t$key = ".$ENV{$key}."\n");
	}

	print("Command line is \"@$orig_cmdline\"...\n");
	if (defined($stb_release) && defined($local_linux)) {
		print("STB version is $stb_release ($local_linux)...\n");
	}
}

sub get_32bit_runtime($$$)
{
	my ($arch, $runtime_base, $rt_path) = @_;

	if (defined($rt_path)) {
		if (! -d $rt_path && $rt_path ne '-') {
			print(STDERR "WARNING: 32-bit directory $rt_path does ".
				"not exist!\n");
			$rt_path = '';
		}
	} else {
		my $arch32 = $arch;

		$arch32 =~ s|64||;
		# "sysroot" and "sys-root" are being used as directory names
		$rt_path = `ls -d "$runtime_base/$arch32"*/sys*root 2>/dev/null`;
		chomp($rt_path);
	}

	if ($rt_path eq '') {
		print("32-bit libraries not found, disabling 32-bit ".
			"support...\n".
			"Use command line option -3 <path> to specify your ".
			"32-bit sysroot.\n");
	} elsif ($rt_path eq '-') {
		printf("Disabling 32-bit support by user request\n");
	} else {
		my $arch64 = $arch_config{$arch}{'arch_name'};
		my $rt64_path =
			`ls -d "$runtime_base/$arch64"*/sys*root 2>/dev/null`;
		chomp($rt64_path);

		print("Using $rt_path for 32-bit environment\n");
		$arch_config{$arch}{'BR2_ROOTFS_RUNTIME32'} = 'y';
		$arch_config{$arch}{'BR2_ROOTFS_RUNTIME32_PATH'} = $rt_path;

		# Additional KConfig variables are derived from the value of
		# BR2_ROOTFS_LIB_DIR in system/Config.in.
		if (-l "$rt64_path/lib64") {
			print("Found new toolchain using /lib and /lib32...\n");

		} else {
			print("Found traditional toolchain using /lib64 and ".
				"/lib...\n");
			$arch_config{$arch}{'BR2_NEED_LD_SO_CONF'} = 'y';
		}
		print("Root file system will use /lib and /lib32...\n");
	}
}

sub run_clean_mode($$)
{
	my ($prg, $br_outputdir) = @_;
	my $err;

	print("Cleaning $br_outputdir...\n");
	remove_tree($br_outputdir, { error => \$err });

	# No error, let's exit.
	exit(0) if ($#$err < 0);

	# See https://perldoc.perl.org/File/Path.html#ERROR-HANDLING
	for my $diag (@$err) {
		my ($file, $message) = %$diag;
		my $errmsg;

		if ($file eq '') {
			$errmsg = $message;
		} else {
			$errmsg = "error deleting $file -- $message";
		}
		print(STDERR "$prg: $errmsg\n");
	}

	exit(1);
}

sub run_hash_mode($$$)
{
	my ($prg, $arch, $br_outputdir) = @_;
	my $version_frag = VERSION_FRAGMENT;
	my $auto_mk =  "$br_outputdir/".AUTO_MK;
	my $k_frags = get_kconfig_var("$br_outputdir/.config",
		'BR2_LINUX_KERNEL_CONFIG_FRAGMENT_FILES');
	my $local_linux;

	print("Running in hash mode for $arch...\n");

	# Sanity checks that updating the hash makes sense. We don't do it if
	#     1. The version fragment file hasn't been configured.
	#     2. A non-custom kernel is being used (e.g. a cloned default
	#        kernel).
	#     3. SHA versioning has been disabled.
	# Setting the hash would have no effect in the first case, since the
	# kernel fragment would never be included. In the case of a non-custom
	# kernel, updating the hash would more likely be misleading than
	# helpful. Such a kernel should also not be modified locally. And if SHA
	# versioning has been turned off, it would make no sense to update it.
	if ($k_frags !~ /$version_frag/) {
		print(STDERR "$prg: $version_frag isn't being used; ".
			"won't update hash\n");
		exit(0);
	}
	if (!-e $auto_mk) {
		print(STDERR
			"$prg: $auto_mk doesn't exist; won't update hash\n");
		exit(0);
	}

	$local_linux = get_kconfig_var($auto_mk, 'LINUX_OVERRIDE_SRCDIR');
	get_linux_sha_local(undef, $br_outputdir, $local_linux);
	exit(0);
}

sub run_tc_info_mode($)
{
	my ($local_linux) = @_;

	print(get_recommended_toolchain($local_linux), "\n");
	exit(0);
}

sub option_cannot_be_combined($$$$)
{
	my ($prg, $flag, $option, $options) = @_;

	if ($flag && $options =~ /[^$option]/) {
		print(STDERR "$prg: option -$option can't be combined with ".
			"another option\n");
		exit(1);
	}
}

sub print_usage($)
{
	my ($prg) = @_;

	print(STDERR "usage: $prg [argument(s)] arm|arm64|bmips\n".
		"          -3 <path>....path to 32-bit run-time ('-' to ".
			"disable)\n".
		"          -b...........launch build after configuring\n".
		"          -C...........display compiler information\n".
		"          -c...........clean (remove output/\$platform)\n".
		"          -D...........use platform's default kernel config\n".
		"          -d <fname>...use <fname> as kernel defconfig\n".
		"          -F <fname>...use <fname> as kernel fragment file\n".
		"          -f <fname>...use <fname> as BR fragment file\n".
		"          -H...........obtain Linux GIT SHA only\n".
		"          -h...........show this help text\n".
		"          -i...........like -b, but also build FS images\n".
		"          -j <jobs>....run <jobs> parallel build jobs\n".
		"          -L <path>....use local <path> as Linux kernel\n".
		"          -l <url>.....use <url> as the Linux kernel repo\n".
		"          -M <url>.....use <url> as BR mirror ('-' for none)\n".
		"          -n...........do not use shared download cache\n".
		"          -o <path>....use <path> as the BR output directory\n".
		"          -R <str>.....use <str> as kernel fragment(s)\n".
		"          -r <str>.....use <str> as BR fragments\n".
		"          -S...........suppress using SHA in Linux version\n".
		"          -T <verstr>..use this toolchain version\n".
		"          -t <path>....use <path> as toolchain directory\n".
		"          -v <tag>.....use <tag> as Linux version tag\n".
		"          -X <path>....use <path> as CCACHE ('-' for none)\n");
}

########################################
# MAIN
########################################
my $prg = basename($0);

my @orig_cmdline = @ARGV;
my $merged_config = 'brcmstb_merged_defconfig';
my $br_output_default = 'output';
my $temp_config = 'temp_config';
my $clean_mode = 0;
my $hash_mode = 0;
my $tc_info_mode = 0;
my $ret = 0;
my $is_64bit = 0;
my $relative_outputdir;
my $br_outputdir;
my $br_mirror;
my $br_ccache;
my $inline_kernel_frag_file;
my $kernel_frag_files;
my $local_linux;
my $toolchain;
my $toolchain_ver;
my $recommended_toolchain;
my $kernel_header_version;
my $arch;
my $opt_keys;
my %opts;

getopts('3:bCcDd:F:f:Hhij:L:l:M:no:R:r:ST:t:v:X:', \%opts);
$opt_keys = join('', keys(%opts));
$arch = $ARGV[0];

if ($#ARGV < 0 || $opts{'h'}) {
	print_usage($prg);
	exit(1);
}

if (check_br() < 0) {
	print(STDERR
		"$prg: must be called from buildroot top level directory\n");
	exit(1);
}

$clean_mode = 1 if ($opts{'c'});
$hash_mode = 1 if ($opts{'H'});
$tc_info_mode = 1 if ($opts{'C'});

option_cannot_be_combined($prg, $clean_mode, 'c', $opt_keys);
option_cannot_be_combined($prg, $hash_mode, 'H', $opt_keys);

# Treat mips as an alias for bmips.
$arch = 'bmips' if ($arch eq 'mips');
# Are we building for a 64-bit platform?
$is_64bit = ($arch =~ /64/);

if (!defined($arch_config{$arch})) {
	print(STDERR "$prg: unknown architecture $arch\n");
	exit(1);
}

if (defined($opts{'L'}) && defined($opts{'l'})) {
	print(STDERR "$prg: options -L and -l cannot be specified together\n");
	exit(1);
}

if (defined($opts{'T'}) && defined($opts{'t'})) {
	print(STDERR "$prg: leave out option -T if you are using -t\n");
	exit(1);
}

if (!sanity_check($prg)) {
	exit(1);
}

if (defined($opts{'3'}) && !$is_64bit) {
	print(STDERR "$prg: WARNING! Option \"-3\" is a no-op for 32-bit ".
		"platforms.\n");
}

# Set local Linux directory from environment, if configured.
if (defined($ENV{'BR_LINUX_OVERRIDE'})) {
	$local_linux = $ENV{'BR_LINUX_OVERRIDE'};
}

# Command line option -L supersedes environment to specify local Linux directory
if (defined($opts{'L'})) {
	# Option "-L -" clears the local Linux directory. This can be used to
	# pretend environment variable BR_LINUX_OVERRIDE is not set, without
	# having to clear it.
	if ($opts{'L'} eq '-') {
		undef($local_linux);
	} else {
		$local_linux = $opts{'L'};
	}
}

if (defined($local_linux) && $local_linux eq '') {
	print(STDERR "$prg: The path to the Linux directory can't be empty.\n");
	exit(1);
}

if (defined($opts{'v'})) {
	$generic_config{'BR2_LINUX_KERNEL_CUSTOM_REPO_VERSION'} = $opts{'v'};
}

# Display information about the toolchain
run_tc_info_mode($local_linux) if ($tc_info_mode);

if (defined($opts{'o'})) {
	$br_outputdir = $opts{'o'};
	$relative_outputdir = $br_outputdir;
} else {
	# Output goes under ./output/ by default. We use an absolute path.
	$br_outputdir = getcwd()."/$br_output_default";
	$relative_outputdir = $br_output_default;
}
# Always add arch-specific sub-directory to output directory.
$br_outputdir .= "/$arch";
$relative_outputdir .= "/$arch";

# Create output directory. "make defconfig" needs it to store $temp_config
# before it would create it itself.
if (! -d $br_outputdir) {
	make_path($br_outputdir);
}

# Clean up output directory
run_clean_mode($prg, $br_outputdir) if ($clean_mode);

# In hash mode, we only update the kernel hash and nothing else.
run_hash_mode($prg, $arch, $br_outputdir) if ($hash_mode);

# This information may help troubleshoot build problems.
print_host_info(\@orig_cmdline, $local_linux);

if (defined($opts{'o'})) {
	print("Using ".$opts{'o'}." as output directory...\n");
}

# Our temporary defconfig goes in the output directory.
$temp_config = "$br_outputdir/$temp_config";

if (defined($ENV{'BR_CCACHE'})) {
	$br_ccache = $ENV{'BR_CCACHE'};
}
if (defined($opts{'X'})) {
	$br_ccache = $opts{'X'};
}
if (defined($br_ccache)) {
	if ($br_ccache eq '-') {
		print("Not using CCACHE to build...\n");
		$generic_config{'BR2_CCACHE'} = '';
	} else {
		print("Using custom CCACHE cache $br_ccache...\n");
		$generic_config{'BR2_CCACHE_DIR'} = $br_ccache;
	}
} else {
	$br_ccache = get_ccache_dir(SHARED_CCACHE);
	print("Using CCACHE with cache $br_ccache...\n");
	$generic_config{'BR2_CCACHE_DIR'} = $br_ccache;
	if ($br_ccache eq SHARED_CCACHE) {
		$generic_config{'BR2_CCACHE_INITIAL_SETUP'} =
			"-M 10G -o 'umask=0'";
	}
}

$kernel_frag_files = $opts{'F'} || '';

$toolchain_ver = $opts{'T'} || '';
if ($toolchain_ver eq '' && !defined($opts{'t'})) {
	my $tc_ver = get_recommended_toolchain($local_linux);
	if ($tc_ver ne '') {
		print("Trying to find recommended toolchain $tc_ver...\n");
		$toolchain = find_toolchain($tc_ver);
	}
	if (!defined($toolchain)) {
		print("Trying to find any toolchain...\n");
	}
}
$toolchain = find_toolchain($toolchain_ver) if (!defined($toolchain));

if (!defined($toolchain) && !defined($opts{'t'})) {
	print(STDERR
		"$prg: couldn't find toolchain in your path, use option -t\n");
	exit(1);
}

if (check_open_source_dir() && !defined($opts{'n'})) {
	my $br_oss_cache = SHARED_OSS_DIR.'/buildroot';

	if (! -d $br_oss_cache) {
		print("Creating shared open source directory ".
			"$br_oss_cache...\n");
		if (!mkdir($br_oss_cache)) {
			print(STDERR
				"$prg: couldn't create $br_oss_cache -- $!\n");
		} else {
			chmod(0777, $br_oss_cache);
			# Setting the default UMASK to world-writable.
			system("setfacl -m default:user::rwx $br_oss_cache");
			system("setfacl -m default:group::rwx $br_oss_cache");
			system("setfacl -m default:other::rwx $br_oss_cache");
			system("setfacl -m default:mask::rwx $br_oss_cache");
		}
	}

	# This is a best-effort attempt to fix up directory permissions in the
	# shared download cache. It will only work if the directories with the
	# wrong permissions are owned by the user running br_config.pl
	fix_shared_permissions($br_oss_cache) if (-d $br_oss_cache);

	# Make sure the cache directory is writable. Don't use it if we can't
	# write to it.
	if (-w $br_oss_cache) {
		print("Using $br_oss_cache as download cache...\n");
		$generic_config{'BR2_DL_DIR'} = $br_oss_cache;
		$generic_config{'BR2_DL_DIR_OPTS'} = '-m 777';
		check_oss_stale_sources($br_oss_cache, $br_outputdir);
	} else {
		print("Ignoring non-writable download cache ".
			"$br_oss_cache...\n");
	}
}
if (!defined($generic_config{'BR2_DL_DIR'})) {
	check_oss_stale_sources('dl', $br_outputdir);
}

if (defined($opts{'d'})) {
	my $cfg = $opts{'d'};

	# "defconfig" is a special case. It represents the default config for
	# many architectures.
	if ($cfg eq 'defconfig') {
		$opts{'D'} = 1;
		undef($opts{'d'});
	} else {
		# Buildroot expects the trailing "_defconfig" to be stripped.
		$cfg =~ s/_?defconfig$//;
		print("Using $cfg as Linux kernel configuration...\n");
		$arch_config{$arch}{'BR2_LINUX_KERNEL_DEFCONFIG'} = $cfg;
	}
}

if (defined($opts{'D'})) {
	print("Using default Linux kernel configuration...\n");
	$arch_config{$arch}{'BR2_LINUX_KERNEL_USE_ARCH_DEFAULT_CONFIG'} = 'y';
	delete($arch_config{$arch}{'BR2_LINUX_KERNEL_DEFCONFIG'});
}

if (defined($opts{'j'})) {
	my $jval = $opts{'j'};

	if ($jval !~ /^\d+$/) {
		print(STDERR "$prg: option -j requires an interger argument\n");
		exit(1);
	}
	if ($jval < 1) {
		print(STDERR "$prg: the argument to -j must be 1 or larger\n");
		exit(1);
	}

	if ($jval == 1) {
		print("Disabling parallel builds...\n");
	} else {
		print("Configuring for $jval parallel build jobs...\n");
	}
	$generic_config{'BR2_JLEVEL'} = $jval;
} else {
	$generic_config{'BR2_JLEVEL'} = get_cores() + 1;
}

if (defined($opts{'l'})) {
	print("Using ".$opts{'l'}." as Linux kernel repo...\n");
	$generic_config{'BR2_LINUX_KERNEL_CUSTOM_REPO_URL'} = $opts{'l'};
}

if (defined($opts{'v'})) {
	print("Using ".$opts{'v'}." as Linux kernel version...\n");
}

if (defined($local_linux)) {
	print("Using $local_linux as Linux kernel directory...\n");
	if (!-d $local_linux) {
		print(STDERR "$prg: Linux directory $local_linux doesn't exist\n");
		exit(1);
	}
	if (!check_linux($local_linux)) {
		print(STDERR "$prg: your local Linux directory must be ".
			"pristine; pre-existing\n".
			"configuration files or build artifacts can interfere ".
			"with the build.\n");
		exit(1);
	}
	if (!check_cma_driver($local_linux)) {
		print("Disabling CMATOOL since kernel doesn't support it...\n");
		$generic_config{'BR2_PACKAGE_CMATOOL'} = '';
	}

	write_brcmstbmk($prg, $relative_outputdir, $local_linux);
	write_localmk($prg, $relative_outputdir);
	# Get the kernel GIT SHA locally if it's a GIT tree.
	if (!defined($opts{'S'})) {
		my $kff = get_linux_sha_local($kernel_frag_files,
				$relative_outputdir, $local_linux);
		if (!defined($kff)) {
			print("No GIT hash available for Linux kernel, ".
				"not setting local version...\n");
		} else {
			$kernel_frag_files = $kff;
		}
	}
} else {
	my $linux_remote_resolves = resolve_linux_remote();

	# Delete our custom makefile, so we don't override the Linux directory.
	if (-e "$br_outputdir/".AUTO_MK) {
		unlink("$br_outputdir/".AUTO_MK);
	}

	if (!$linux_remote_resolves) {
		my $linux_remote = get_linux_remote();
		print("WARNING! Couldn't resolve $linux_remote!\n".
			"Build may fail.\n");
	}

	# Determine the kernel GIT SHA remotely. The tree hasn't been cloned
	# yet. We can't do anything if we can't resolve the remote host.
	if ($linux_remote_resolves && !defined($opts{'S'})) {
		$kernel_frag_files = get_linux_sha_remote($kernel_frag_files,
			$relative_outputdir);
	}
}

$inline_kernel_frag_file = $relative_outputdir."/".KERNEL_FRAG_FILE;
if (defined($opts{'R'})) {
	my $frag_file = parse_cmdline_fragments($inline_kernel_frag_file,
		$opts{'R'});

	if ($frag_file ne '') {
		$kernel_frag_files .= ' ' if ($kernel_frag_files ne '');
		$kernel_frag_files .= $frag_file;
	}
} else {
	# Keep things clean. Remove the frag file if we don't need it.
	unlink($inline_kernel_frag_file) if (-e $inline_kernel_frag_file);
}

# If requested, don't append GIT SHA to kernel version string. Primarily used
# for release builds.
if (defined($opts{'S'})) {
	# Delete our Linux custom version fragment.
	if (-e "$br_outputdir/".VERSION_FRAGMENT) {
		unlink("$br_outputdir/".VERSION_FRAGMENT);
	}
}

if ($kernel_frag_files ne '') {
	# BR wants fragment files to be separated by spaces
	$kernel_frag_files =~ s/,/ /g;
	print("Linux config fragment file(s): $kernel_frag_files\n");
	$generic_config{'BR2_LINUX_KERNEL_CONFIG_FRAGMENT_FILES'} =
		$kernel_frag_files;
}

if (defined($opts{'t'})) {
	$toolchain = $opts{'t'};
	# Remove trailing slashes if there are any.
	$toolchain =~ s|/+$||;
}

$recommended_toolchain = check_toolchain($toolchain, $local_linux);
if ($recommended_toolchain ne '') {
	my $t = $toolchain;

	$t =~ s|.*/||;
	print(STDERR "WARNING: you are using toolchain $t. Recommended is ".
		"$recommended_toolchain.\n");
	print(STDERR "Hit Ctrl-C now or wait ".SLEEP_TIME." seconds...\n");
	sleep(SLEEP_TIME);
}
$ret = set_target_toolchain($toolchain, $arch);
if ($ret == 0) {
	print("Using $toolchain as toolchain...\n");
} else {
	if ($ret == -1) {
		print(STDERR "$prg: $toolchain doesn't exist for $arch\n");
	} elsif ($ret == -2) {
		print(STDERR "$prg: couldn't determine libc for $toolchain\n");
	} else {
		print(STDERR "$prg: unknown toolchain error\n");
	}
	exit(1);
}

# The toolchain may have changed since we last configured Buildroot. We need to
# force it to create the symlinks again, so we are sure to use the toolchain
# specified now.
trigger_toolchain_sync($relative_outputdir, $arch);

$kernel_header_version = get_kernel_header_version($toolchain, $arch);
if (defined($kernel_header_version)) {
	my ($major, $minor) = @$kernel_header_version;
	my $ext_headers = "BR2_TOOLCHAIN_EXTERNAL_HEADERS_${major}_${minor}";
	print("Found kernel header version ${major}.${minor}...\n");
	$toolchain_config{$arch}{$ext_headers} = 'y';
} else {
	print("WARNING: couldn't detect kernel header version; build may ".
		"fail\n");
}

# Set custom Buildroot mirror
if (defined($ENV{'BR_MIRROR'})) {
	$br_mirror = $ENV{'BR_MIRROR'};
}

# Command line option -M supersedes environment to specify mirror
if (defined($opts{'M'})) {
	# Option "-M -" disables using a mirror. This overrides the environment
	# variable BR_MIRROR and the built-in default.
	$br_mirror = $opts{'M'};
}

if (!defined($br_mirror)) {
	$br_mirror = get_br_mirror_host();
}
if (defined($br_mirror) && $br_mirror ne '-') {
	print("Using $br_mirror as Buildroot mirror...\n");
	$generic_config{'BR2_PRIMARY_SITE'} = $br_mirror;

} else {
	print("Not using a Buildroot mirror...\n");
}

get_32bit_runtime($arch, $toolchain, $opts{'3'}) if ($is_64bit);

write_config(\%generic_config, $temp_config, 1);
write_config($arch_config{$arch}, $temp_config, 0);
write_config($toolchain_config{$arch}, $temp_config, 0);

system("support/kconfig/merge_config.sh -m configs/brcmstb_defconfig ".
	"\"$temp_config\"");
if (defined($opts{'f'})) {
	my $fragment_file = merge_br_fragments($prg, $br_outputdir, $opts{'f'});

	exit(1) if (!defined($fragment_file));

	# Preserve the merged configuration from above and use it as the
	# starting point.
	rename('.config', $temp_config);
	system("support/kconfig/merge_config.sh -m $temp_config ".
		"\"$fragment_file\"");
	unlink($fragment_file);
}
if (defined($opts{'r'})) {
	my $f = $relative_outputdir."/".BR_FRAG_FILE;
	my $fragment_file = parse_cmdline_fragments($f, $opts{'r'});

	if ($fragment_file ne '') {
		# Preserve the merged configuration from above and use it as the
		# starting point.
		rename('.config', $temp_config);
		system("support/kconfig/merge_config.sh -m $temp_config ".
			"\"$fragment_file\"");
		unlink($fragment_file);
	}
}
unlink($temp_config);
move_merged_config($prg, $arch, ".config", "configs/$merged_config");

# Finalize the configuration by running make ..._defconfig.
system("make O=\"$br_outputdir\" \"$merged_config\"");

print("Buildroot has been configured for ".uc($arch).".\n");
if (defined($opts{'i'})) {
	print("Launching build, including file system images...\n");
	# The "images" target only exists in the generated Makefile in
	# $br_outputdir, so using "make O=..." does not work here.
	$ret = system("make -C \"$br_outputdir\" images");
	$ret >>= 8;
} elsif (defined($opts{'b'})) {
	print("Launching build...\n");
	$ret = system("make O=\"$br_outputdir\"");
	$ret >>= 8;
} else {
	print("To build it, run the following commands:\n".
	"\tcd $br_outputdir; make\n");
}

print(STDERR "$prg: exiting with code $ret\n") if ($ret > 0);
exit($ret);
