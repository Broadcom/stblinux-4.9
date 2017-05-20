include $(ARCH_CONFIG)

CFG = "--host=$(MACHINE)-linux --prefix=$(ROMFSDIR) \
 --libdir=$(ROMFSDIR)/lib/$(OSLIB)"

default:
	make LDFLAGS='$(filter-out -Wl, $(LDFLAGS))' \
		CFLAGS="-fPIC" \
		CFG=$(CFG) -f Makefile
	make -f Makefile
	make -f Makefile install

distclean clean:
	[ ! -f Makefile ] || make -f Makefile distclean

romfs:
	rm -f $(ROMFSDIR)/bin/freetype-config
	rm -f $(ROMFSDIR)/lib/$(OSLIB)/libfreetype.*a
	rm -f $(ROMFSDIR)/share/aclocal/freetype.m4
