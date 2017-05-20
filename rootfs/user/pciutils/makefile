include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os -Wno-error

MAKE_FLAGS		+= \
			   CFLAGS="$(CFLAGS) -I$(ROOTDIR)/lib/zlib" \
			   LDFLAGS="-L$(ROOTDIR)/lib/zlib" \
			   PREFIX="/usr" \
			   HOST="Linux" \
			   CROSS_COMPILE="$(CROSS_COMPILE)" \
			   ZLIB="yes" \
			   HWDB="no" \
			   LIBKMOD="no"

export CC CFLAGS

.PHONY: all
all::
	$(MAKE) $(MAKE_FLAGS) -f Makefile

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

.PHONY: romfs
romfs::
	$(ROMFSINST) -e CONFIG_USER_PCIUTILS_SETPCI setpci \
		/bin/setpci
	$(ROMFSINST) -e CONFIG_USER_PCIUTILS_LSPCI lspci \
		/bin/lspci

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
