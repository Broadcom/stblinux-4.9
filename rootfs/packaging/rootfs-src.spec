Name:       rootfs-src
Summary:    STB rootfs source
Version:    0
Release:    0
Group:      Base/File Systems
License:    see LICENSE in source
Source0:    %{name}-%{version}.tar.gz
Source100:  %{name}-rpmlintrc

AutoReqProv: no
BuildArch:   noarch

%description
This is the rootfs source for STB.

%prep
%setup -q

%build
echo "... nothing to build ..."

%install
install -d %{buildroot}/opt/stb-src/rootfs
cp -pr * %{buildroot}/opt/stb-src/rootfs
rm -rf %{buildroot}/opt/stb-src/rootfs/packaging

%files
%defattr(-,root,root,-)
/opt/stb-src/rootfs
