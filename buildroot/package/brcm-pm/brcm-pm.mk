################################################################################
#
# brcm-pm
#
################################################################################

BRCMROOT_VERSION = $(call qstrip,$(BR2_BRCM_STB_TOOLS_VERSION))

BRCM_PM_VERSION = $(BRCMROOT_VERSION)
BRCM_PM_SITE = git://stbgit.stb.broadcom.net/queue/stbtools.git
BRCM_PM_SOURCE = stbtools-$(BRCMROOT_VERSION).tar.gz
BRCM_PM_DIR = stbtools
BRCM_PM_LICENSE = BSD-3-Clause

# Extract only what we need to save space.
define BRCM_PM_EXTRACT_CMDS
	$(call suitable-extractor,$(BRCM_PM_SOURCE)) \
		$(BRCM_PM_DL_DIR)/$(BRCM_PM_SOURCE) | \
		$(TAR) --strip-components=1 -C $(BRCM_PM_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/brcm-pm'
endef

define BRCM_PM_BUILD_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/brcm-pm $(TARGET_CONFIGURE_OPTS)
endef

define BRCM_PM_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/brcm-pm install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
