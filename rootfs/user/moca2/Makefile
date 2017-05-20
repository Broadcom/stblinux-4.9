#
# Makefile for binary-only MoCA 2.0 support
#

include $(ROOTDIR)/config.arch

.PHONY: all clean distclean
all clean distclean:

.PHONY: romfs
romfs:
	mkdir -p $(ROMFSDIR)/etc/moca
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		$(MACHINE).$(LIBCDIR)/mocad /bin/mocad
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		$(MACHINE).$(LIBCDIR)/mocap /bin/mocap
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		$(MACHINE).$(LIBCDIR)/mocap /bin/mocactl
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		fw/moca20core-gen21.bin /etc/moca/moca20core-gen21.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		fw/moca20core-gen22.bin /etc/moca/moca20core-gen22.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		fw/moca20core-gen23.bin /etc/moca/moca20core-gen23.bin
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
		fw/moca20core-gen24.bin /etc/moca/moca20core-gen24.bin
