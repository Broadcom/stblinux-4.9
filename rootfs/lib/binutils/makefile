include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		MAKEINFO=missing \
		./configure --disable-shared --host=$(MACHINE)-linux \
			--disable-nls --disable-etc \
			--disable-gas || exit 1 ; \
	fi
	$(MAKE) -f Makefile configure-host
	find . -name Makefile | xargs sed -i 's/\(SUBDIRS.*\) doc/\1/g'
	find . -name Makefile | xargs sed -i '/INFO_DEPS/d'
	$(MAKE) -f Makefile

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -j1 -f Makefile $@

.PHONY: romfs
romfs:
	$(STRIP) binutils/objdump
	$(ROMFSINST) -e CONFIG_LIB_BINUTILS_OBJDUMP binutils/objdump \
		/bin/objdump

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
