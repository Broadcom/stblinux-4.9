BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

ifdef CONFIG_USER_GDB

.PHONY: all
all::
	$(MAKE) -f Makefile

else

-include $(ROOTDIR)/config.arch

TARGET := $(shell $(CC) -dumpmachine)

.PHONY: all
all::
	if [ ! -e Makefile ]; then \
	( \
	AR=$(CROSS)ar						\
	AS=$(CROSS)as						\
	LD=$(CROSS)ld						\
	NM=$(CROSS)nm						\
	CC=$(CROSS)gcc						\
	GCC=$(CROSS)gcc						\
	CXX=$(CROSS)g++						\
	RANLIB=$(CROSS)ranlib					\
	gdb_cv_func_sigsetjmp=yes					\
	ac_cv_type_uintptr_t=yes					\
	gt_cv_func_gettext_libintl=yes				\
	ac_cv_func_dcgettext=yes					\
	bash_cv_func_strcoll_broken=no				\
	bash_cv_must_reinstall_sighandlers=no				\
	bash_cv_func_sigsetjmp=present				\
	bash_cv_have_mbstate_t=yes					\
	srv_linux_thread_db=yes					\
	export STD_INC="$(GCCTOOL_DIR_PREFIX)/$(TARGET)/include"; \
	export GCC_INC="$(GCCTOOL_DIR_PREFIX)/$(TARGET)/include"; \
	test -f config.cache && rm -f config.cache; \
	./configure			\
		--prefix=/	\
		--target=$(TARGET)						\
		--build=i686-pc-linux-gnu						\
		--host=$(TARGET)						\
		--disable-nls						\
		--without-uiout --disable-gdbmi				\
		--disable-tui --disable-gdbtk --without-x			\
		--without-included-gettext	\
	) ; \
	fi
	$(MAKE) -f Makefile  CFLAGS+=-L../../../../romfs/lib/

endif  # CONFIG_USER_GDB

.PHONY: clean distclean
clean distclean:
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

.PHONY: romfs
romfs::
	$(ROMFSINST) -e CONFIG_USER_GDBSERVER_GDBSERVER /bin/gdbserver
	$(ROMFSINST) -e CONFIG_USER_GDBSERVER_GDBREPLAY /bin/gdbreplay


# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
# # Since this file doesn't contain dependency information, don't parallelize
# # here.
.NOTPARALLEL: %
%:: FORCE
	$(MAKE) -f Makefile $@
