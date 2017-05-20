include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CROSS			:= $(CROSS_COMPILE)

export CROSS

.PHONY: all
all::
	$(MAKE) -f Makefile

.PHONY: clean distclean
clean distclean::
	[ ! -f Makefile ] || $(MAKE) -f Makefile $@

.PHONY: romfs
romfs::
	$(ROMFSINST) /bin/cpufreq-aperf
	$(ROMFSINST) /bin/cpufreq-info
	$(ROMFSINST) /bin/cpufreq-set
	$(ROMFSINST) /lib/$(OSLIB)/libcpufreq.so
	$(ROMFSINST) /lib/$(OSLIB)/libcpufreq.so.0
	$(ROMFSINST) /lib/$(OSLIB)/libcpufreq.so.0.0.0

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
