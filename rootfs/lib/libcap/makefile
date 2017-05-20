#
# General settings
#

# for $(CROSS), $(CC), $(STRIPTOOL)
include $(ARCH_CONFIG)

BOGUS_TARGETS		:= FORCE makefile $(ARCH_CONFIG)

CFLAGS			:= -Os

export CC CFLAGS

#
# Settings that are specific to this package
#


#
# Build targets
#

.PHONY: all
all:
	cp -f _makenames.sh libcap/_makenames
	$(MAKE) -f Makefile CC=$(CC) AR=$(AR) RANLIB=$(RANLIB) BUILD_CC=true \
		DYNAMIC="yes" LDFLAGS="-shared" LIBATTR=no \
		BUILD_GPERF="false" \
		-C libcap

.PHONY: romfs
romfs:
	-$(RM) -f $(ROMFSDIR)/lib/$(OSLIB)/libcap.so*
	cp -af libcap/libcap.so* $(ROMFSDIR)/lib/$(OSLIB)/
	$(STRIP) $(ROMFSDIR)/lib/$(OSLIB)/libcap.so*

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
