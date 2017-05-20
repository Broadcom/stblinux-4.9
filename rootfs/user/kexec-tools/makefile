include $(ROOTDIR)/config.arch

CFLAGS       := -Os
CFLAGS       += -I$(ROOTDIR)/lib/zlib
LDFLAGS      := -L$(ROOTDIR)/lib/zlib

export CFLAGS LDFLAGS

all:
	if [ ! -f Makefile ]; then \
		./configure --host=$(MACHINE)-linux --prefix=/; \
	fi
	$(MAKE) -f Makefile

clean:
	[ ! -f Makefile ] || make -f Makefile $@

distclean:
	[ ! -f Makefile ] || make -f Makefile clean; rm Makefile

romfs:
	$(eval BINARIES=$(wildcard build/sbin/*))
	$(MAKE) install DESTDIR=$(ROMFSDIR) TARGETS="$(BINARIES)"

%:
	$(MAKE) -f Makefile $@

.PHONY: all clean distclean romfs
