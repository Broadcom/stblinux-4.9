include $(ARCH_CONFIG)

unexport MAKEFILE_LIST

BOGUS_TARGETS		:= FORCE makefile $(ARCH_CONFIG)

SHELL := /bin/bash

# This guard is necessary because the generated Makefile in the build folder
# does make -C on this directory causing an infinite loop.
ifneq ($(EGLIBC_MAKEFILE_GUARD),1)
EGLIBC_MAKEFILE_GUARD = 1

# Only build locally if necessary
ifeq ($(CONFIG_LIB_LIBC_FROM_ROOTFS),y)
libc-source := rootfs
else
libc-source := toolchain
endif

.PHONY: all
all:: all-$(libc-source)

.PHONY: all-rootfs
all-rootfs:
	if [ ! -e build/lib/$(OSLIB)/Makefile ]; then \
		mkdir -p build/lib/$(OSLIB); \
		cd build/lib/$(OSLIB); \
		../../configure --host=$(shell $(CC) -dumpmachine) \
			--prefix=/usr \
			--with-sysroot=$(shell $(CC) -print-sysroot) \
			--enable-obsolete-rpc \
			--disable-nscd; \
	fi
	$(MAKE) -C build/lib/$(OSLIB)

.PHONY: all-toolchain
all-toolchain:

.PHONY: clean
clean::
	[ ! -f build/lib/$(OSLIB)/Makefile ] || $(MAKE) -C build/lib/$(OSLIB) $@

.PHONY: distclean
distclean::
	rm -rf build temp

# We install to a temporary folder first and generate the list of
# files we actually want from that folder, then copy those to the romfs.
.PHONY: romfs romfs-rootfs romfs-toolchain
romfs:: romfs-$(libc-source)
	-$(STRIP) temp/lib/*.so* temp/sbin/ldconfig
	mkdir -p $(ROMFSDIR)/lib
	cp -df temp/lib/*.so* $(ROMFSDIR)/lib
ifneq ($(OSLIB),.)
	-$(STRIP) temp/lib/$(OSLIB)/*.so*
	mkdir -p $(ROMFSDIR)/lib/$(OSLIB)
	-cp -df temp/lib/$(OSLIB)/*.so* $(ROMFSDIR)/lib/$(OSLIB)
endif
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDD temp/bin/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDCONFIG temp/sbin/ldconfig /sbin/ldconfig

romfs-rootfs::
	rm -fr temp
	mkdir -p temp
	make -C build/lib/$(OSLIB) install DESTDIR=$(CURDIR)/temp

romfs-toolchain::
	rm -fr temp
	mkdir -p temp/{bin,lib,sbin}
	cp -df $(GCCSYSROOT)/lib/*.so* temp/lib
	if [ -d "$(GCCSYSROOT)/lib64" ]; then \
		mkdir -p temp/lib64; \
		cp -df $(GCCSYSROOT)/lib64/*.so* temp/lib64; \
	fi
	# FIXME: libgcc was removed from sysroot in stbgcc-4.8-1.1, eventually
	# delete the next line
	rm -f temp/lib/libgcc*.so*
	cp $(GCCSYSROOT)/usr/bin/ldd temp/bin/ldd
	cp $(GCCSYSROOT)/sbin/ldconfig temp/sbin/ldconfig

endif # EGLIBC_MAKEFILE_GUARD

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
.NOTPARALLEL: %
%:: FORCE
	$(MAKE) -f Makefile "$@"
