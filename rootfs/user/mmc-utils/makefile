include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os -DDANGEROUS_COMMANDS_ENABLED

export CC CFLAGS

.PHONY: all
all::
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) mmc
	mkdir -p $(ROMFSDIR)/bin/
	$(ROMFSINST) -e CONFIG_USER_MMCUTILS mmc /bin/mmc

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
