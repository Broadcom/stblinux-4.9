include $(ROOTDIR)/config.arch

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

CC			:= $(CROSS_COMPILE)gcc
STRIP			:= $(CROSS_COMPILE)strip
CFLAGS			:= -Os

BASE_PLUGINS		= blk function hrtimer sched_switch \
			  kmem

export CC CFLAGS

.PHONY: all
all::
	$(MAKE) CONFIG_FLAGS="--std=gnu99" NO_PYTHON=1 prefix=/ -f Makefile

.PHONY: romfs
romfs::
	$(STRIP) trace-cmd
	mkdir -p $(ROMFSDIR)/bin/
	$(ROMFSINST) -e CONFIG_USER_TRACECMD trace-cmd /bin/trace-cmd
	mkdir -p $(ROMFSDIR)/lib/trace-cmd/plugins
	$(foreach p,$(BASE_PLUGINS),$(ROMFSINST) -e CONFIG_USER_TRACECMD \
		plugin_$(p).so /lib/trace-cmd/plugins;)

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
%:: FORCE
	$(MAKE) -f Makefile $@
