#
# Makefile for binary-only MoCA 2.0 support
#
# This makefile is shared between uclinux-rootfs and buildroot!
#

# "include" needs to be conditional, so buildroot doesn't fail.
ifeq ($(MACHINE),)
include $(ROOTDIR)/config.arch
endif

MOCA_BINS=	mocad mocap
MOCA_FW=	moca20core-gen21.bin moca20core-gen22.bin \
		moca20core-gen23.bin moca20core-gen24.bin

# We don't have AArch64 binaries of MoCA.
ifeq ($(MACHINE),aarch64)
MACHINE=arm
endif

MOCA_DIR?=	$(MACHINE).$(LIBCDIR)

.PHONY: all clean distclean
all clean distclean:

.PHONY: install romfs

# buildroot install target
install: all
	@for f in $(MOCA_BINS); do \
		echo "Installing $(MOCA_DIR)/$$f..."; \
		install -D -p -m 0755 -t $(DESTDIR)/bin $(MOCA_DIR)/$$f; \
	done

	@rm -f $(DESTDIR)/bin/mocactl
	@ln -s mocap $(DESTDIR)/bin/mocactl

	# Older versions of "install" don't create the directory, but error
	# out instead.
	@test -d $(DESTDIR)/etc/moca || mkdir $(DESTDIR)/etc/moca
	@for f in $(MOCA_FW); do \
		echo "Installing fw/$$f..."; \
		install -D -p -m 0644 -t $(DESTDIR)/etc/moca fw/$$f; \
	done

# uclinux-rootfs install target
romfs:
	mkdir -p $(ROMFSDIR)/etc/moca

	@for f in $(MOCA_BINS); do \
		$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
			$(MOCA_DIR)/$$f /bin/$$f; \
	done

	rm -f $(ROMFSDIR)/bin/mocactl
	$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 -s mocap /bin/mocactl

	@for f in $(MOCA_FW); do \
		$(ROMFSINST) -e CONFIG_USER_MOCA_MOCA2 \
			fw/$$f /etc/moca/$$f; \
	done
