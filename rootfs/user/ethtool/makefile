include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

ifeq ($(CONFIG_USER_ETHTOOL_PRETTY_DUMP),y)
CONFIGURE_ARGS:=--enable-pretty-dump
else
CONFIGURE_ARGS:=--disable-pretty-dump
endif

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux \
			$(CONFIGURE_ARGS) \
			ac_cv_func_malloc_0_nonnull=yes || exit 1 ; \
	fi
	$(MAKE) -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) ethtool 
	$(ROMFSINST) -e CONFIG_USER_ETHTOOL ethtool /bin/ethtool

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
