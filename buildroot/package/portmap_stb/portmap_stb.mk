################################################################################
#
# portmap
#
################################################################################

PORTMAP_STB_VERSION = 6.0
PORTMAP_STB_SOURCE = portmap-$(PORTMAP_STB_VERSION).tgz
PORTMAP_STB_SITE = https://fossies.org/linux/misc/old

# So the Makefile doesn't try to link in libwrap
PORTMAP_STB_MAKE_ENV = $(TARGET_MAKE_ENV) NO_TCP_WRAPPER=1
# So the compiler doesn't try to include tcpd.h
PORTMAP_STB_CONFIGURE_OPTS = $(TARGET_CONFIGURE_OPTS) CPPFLAGS+=-DNO_TCP_WRAPPER

define PORTMAP_STB_BUILD_CMDS
	$(PORTMAP_STB_MAKE_ENV) $(MAKE) $(PORTMAP_STB_CONFIGURE_OPTS) -C $(@D)
endef

define PORTMAP_STB_INSTALL_TARGET_CMDS
	$(INSTALL) -m 0755 -D $(@D)/portmap $(TARGET_DIR)/bin/portmap
endef

$(eval $(generic-package))
