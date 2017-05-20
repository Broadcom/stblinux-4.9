KERNEL_INCLUDE=$(ROOTDIR)/$(LINUXDIR)/include
CC	:= $(CROSS_COMPILE)gcc

export KERNEL_INCLUDE CC PKG_CONFIG_PATH

.PHONY: all

all:
	echo "PKG_CONFIG:=/bin/true" > Config
	echo "AR:=$(CROSS_COMPILE)ar" >> Config
	echo "CC:=$(CC)" >> Config
	echo "TC_CONFIG_IPSET:=n" >> Config
	echo "IPT_LIB_DIR:=/lib/xtables" >> Config
	echo "IP_CONFIG_SETNS:=y" >> Config
	echo "HAVE_SELINUX:=n" >> Config
	echo "HAVE_LATEX:=n" >> Config
	echo "HAVE_PDFLATEX:=n" >> Config
	$(MAKE) -f Makefile all
	
romfs:
	$(ROMFSINST) etc/etc
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_IP ip/ip /sbin/ip
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_RTMON ip/rtmon /sbin/rtmon
	
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_IFCFG ip/ifcfg /sbin/ifcfg
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_RTPR  ip/rtpr /sbin/rtpr
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_ROUTEL ip/routel /sbin/routel
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_ROUTEF ip/routef /sbin/routef
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_TC_TC tc/tc /sbin/tc
	$(ROMFSINST) -e CONFIG_USER_IPROUT2_MISC_SS misc/ss /sbin/ss
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_MISC_IFSTAT misc/ifstat /sbin/ifstat 
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_IP_RTACCT misc/rtacct /sbin/rtacct
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_MISC_LNSTAT misc/lnstat /sbin/lnstat
	if [ -n $CONFIG_USER_IPROUTE2_MISC_LNSTAT ]; then\
		$(ROMFSINST) -s lnstat /sbin/rtstat; \
		$(ROMFSINST) -s lnstat /sbin/ctstat; \
	fi
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_GENL genl/genl /sbin/genl
	$(ROMFSINST) -e CONFIG_USER_IPROUTE2_BRIDGE bridge/bridge /sbin/bridge

.PHONY: clean distclean
clean distclean::
	$(MAKE) -f Makefile clean
	
%:: FORCE
	$(MAKE) -f Makefile $@
