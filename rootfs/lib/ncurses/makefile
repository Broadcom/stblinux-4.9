include $(ARCH_CONFIG)

BOGUS_TARGETS		:= FORCE makefile $(ARCH_CONFIG)

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACHINE)-linux \
			--without-shared --enable-termcap || exit 1 ; \
	fi
	$(MAKE) -j1 -C ncurses
	$(MAKE) -j1 -C panel
	ln -sf curses.h include/ncurses.h

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

.PHONY: romfs
romfs::

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
