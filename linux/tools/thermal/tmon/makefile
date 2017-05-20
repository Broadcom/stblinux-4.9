include $(ROOTDIR)/config.arch

LDFLAGS += -L$(ROOTDIR)/lib/ncurses/lib/ -lncurses -lpanel
CFLAGS  += -I$(ROOTDIR)/lib/ncurses/include/

all:
	$(MAKE) -f Makefile \
		CC="$(CROSS_COMPILE)gcc" \
		CFLAGS="$(CFLAGS) -DVERSION=\"1.0\"" \
		LDFLAGS="$(LDFLAGS)" \
		PKG_CONFIG="/bin/false"

all-% install-% clean-% distclean-%:
	$(MAKE) -f Makefile $@

clean:
	$(MAKE) -f Makefile $(COMMON_MAKEARGS) clean

distclean: clean
	-$(MAKE) -f Makefile $(COMMON_MAKEARGS) distclean

romfs:
	$(STRIP) tmon
	$(ROMFSINST) tmon /bin/tmon
