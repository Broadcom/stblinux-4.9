include $(ARCH_CONFIG)

BOGUS_TARGETS		:= FORCE makefile $(ARCH_CONFIG)

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

ifeq ($(LIBCDIR),uClibc)
LDFLAGS += -L$(ROOTDIR)/lib/gettext/intl/.libs -lintl
CFLAGS  += -I$(ROOTDIR)/lib/libuargp -I$(ROOTDIR)/lib/gettext/intl
endif

ifeq ($(ARCH),mips)
MACH_LIB = $(ARCH)
else
MACH_LIB = $(MACHINE)
endif

export CC CFLAGS

INSTALL_TMP_DIR = $(shell pwd)/install-tmp

.PHONY: all
all:
	if [ ! -e Makefile ]; then \
		./configure --host=$(MACH_LIB)-linux \
		--prefix=$(INSTALL_TMP_DIR) \
		CFLAGS="-U_FORTIFY_SOURCE -DDEBUG -g" \
		--enable-debug --enable-debug-frame \
		--disable-documentation;\
	fi
	sed -i 's/\(SUBDIRS.*\) doc/\1/g' Makefile
	make -f Makefile install

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@
	rm -fr $(INSTALL_TMP_DIR)

.PHONY: romfs
romfs:
	-$(RM) -f $(ROMFSDIR)/lib/$(OSLIB)/libunwind.so*
	cp -af src/.libs/libunwind.so src/.libs/libunwind.so*[0-9] \
		$(ROMFSDIR)/lib/$(OSLIB)/
	$(STRIP) $(ROMFSDIR)/lib/$(OSLIB)/libunwind.so
	-$(RM) -f $(ROMFSDIR)/lib/$(OSLIB)/libunwind-$(MACH_LIB).so*
	cp -af src/.libs/libunwind-$(MACH_LIB).so src/.libs/libunwind-$(MACH_LIB).so*[0-9] \
		$(ROMFSDIR)/lib/$(OSLIB)/
	$(STRIP) $(ROMFSDIR)/lib/$(OSLIB)/libunwind-$(MACH_LIB).so

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@

