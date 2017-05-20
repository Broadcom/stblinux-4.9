include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

export CC CFLAGS

.PHONY: all
all::
	$(MAKE) -C src \
		CC="$(CC)" \
		CFLAGS="$(CFLAGS)" \
		DEBUG="" \
		INCLUDEDIR="-I../support -I$(ROOTDIR)/lib/ncurses/include/ \
			-I$(ROOTDIR)/lib/ncurses//panel" \
		LDOPTS="-L$(ROOTDIR)/lib/ncurses/lib \
			-L$(ROOTDIR)/lib/ncurses/panel " \
		PLATFORM="-DPLATFORM=\\\"Linux/$(MACHINE)\\\"" \
		TARGET="/bin" \
		WORKDIR="/tmp" \
		all

.PHONY: romfs
romfs::
	$(STRIP) src/iptraf
	$(ROMFSINST) -e CONFIG_USER_IPTRAF_IPTRAF src/iptraf /bin/iptraf

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -C src $@
