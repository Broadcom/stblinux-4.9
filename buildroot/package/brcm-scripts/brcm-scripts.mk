################################################################################
#
# brcm-scripts
#
################################################################################

BRCMROOT_VERSION = c38822d042bd729481f2cea6b2fd138b5e190fa8

BRCM_SCRIPTS_VERSION = master
BRCM_SCRIPTS_SITE = git://stbgit.broadcom.com/mm940762/uclinux-rootfs.git
BRCM_SCRIPTS_SOURCE = uclinux-rootfs-$(BRCMROOT_VERSION).tar.gz
BRCM_SCRIPTS_LICENSE = GPL-2.0

# Extract only what we need to save space.
define BRCM_SCRIPTS_EXTRACT_CMDS
	$(call suitable-extractor,$(BRCM_SCRIPTS_SOURCE)) \
		$(DL_DIR)/$(BRCM_SCRIPTS_SOURCE) | \
		$(TAR) --strip-components=1 -C $(BRCM_SCRIPTS_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/skel/bin'
endef

define BRCM_SCRIPTS_INSTALL_TARGET_CMDS
	cp -p $(@D)/skel/bin/* $(TARGET_DIR)/bin
endef

$(eval $(generic-package))
