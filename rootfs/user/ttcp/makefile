include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) ttcp
	$(ROMFSINST) -e CONFIG_USER_TTCP_TTCP ttcp /bin/ttcp

# ttcp does not have distclean target
.PHONY: distclean
distclean::
	$(MAKE) -f Makefile clean

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
