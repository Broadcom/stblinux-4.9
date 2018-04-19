################################################################################
#
# brcm-pm
#
################################################################################

BRCMROOT_VERSION = c38822d042bd729481f2cea6b2fd138b5e190fa8

BRCM_PM_VERSION = master
BRCM_PM_SITE = git://stbgit.broadcom.com/mm940762/uclinux-rootfs.git
BRCM_PM_SOURCE = uclinux-rootfs-$(BRCMROOT_VERSION).tar.gz
BRCM_PM_DIR = uclinux-rootfs
BRCM_PM_LICENSE = BSD-3-Clause

# Extract only what we need to save space.
define BRCM_PM_EXTRACT_CMDS
	$(call suitable-extractor,$(BRCM_PM_SOURCE)) \
		$(DL_DIR)/$(BRCM_PM_SOURCE) | \
		$(TAR) --strip-components=1 -C $(BRCM_PM_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/user/brcm-pm'
endef

define BRCM_PM_BUILD_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/user/brcm-pm $(TARGET_CONFIGURE_OPTS)
endef

define BRCM_PM_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/user/brcm-pm install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
