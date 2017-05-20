include $(ROOTDIR)/config.arch

LDFLAGS += -L$(ROOTDIR)/lib/elfutils/libelf -lelf
EXTRA_CFLAGS  += -I$(ROOTDIR)/lib/elfutils/libelf

ifeq ($(LIBCDIR),uClibc)
LDFLAGS += -L$(ROOTDIR)/lib/gettext/intl/.libs -lintl
EXTRA_CFLAGS  += -I$(ROOTDIR)/lib/gettext/intl
endif

OPTION_MAKEARGS = WERROR=0 NO_LIBPYTHON=1 \
		  NO_GTK2=1 NO_LIBAUDIT=1 NO_NEWT=1 NO_LIBPERL=1 \
		  NO_LIBPYTHON=1 NO_DEMANGLE=1

LIBDW_DIR = $(ROOTDIR)/lib/elfutils/install-tmp
LIBUNWIND_DIR = $(ROOTDIR)/lib/libunwind/install-tmp

all :
	$(MAKE) -f Makefile prefix=$(ROOTDIR)/romfs $(OPTION_MAKEARGS)

all-% install-% clean-% distclean-%:
	$(MAKE) -f Makefile $@

clean:
	$(MAKE) -f Makefile $(COMMON_MAKEARGS) clean

distclean: clean
	-$(MAKE) -f Makefile $(COMMON_MAKEARGS) distclean

romfs:
	$(STRIP) perf
	$(ROMFSINST) perf /bin/perf
