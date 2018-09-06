################################################################################
#
# brcm-scripts
#
################################################################################

BRCMROOT_VERSION = $(call qstrip,$(BR2_BRCM_STB_TOOLS_VERSION))

BRCM_SCRIPTS_VERSION = $(BRCMROOT_VERSION)
BRCM_SCRIPTS_SITE = git://stbgit.broadcom.com/mm940762/stbtools.git
BRCM_SCRIPTS_SOURCE = stbtools-$(BRCMROOT_VERSION).tar.gz
BRCM_SCRIPTS_DL_SUBDIR = brcm-pm
BRCM_SCRIPTS_LICENSE = GPL-2.0

# Extract only what we need to save space.
define BRCM_SCRIPTS_EXTRACT_CMDS
	$(call suitable-extractor,$(BRCM_SCRIPTS_SOURCE)) \
		$(BRCM_SCRIPTS_DL_DIR)/$(BRCM_SCRIPTS_SOURCE) | \
		$(TAR) --strip-components=1 -C $(BRCM_SCRIPTS_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/skel/bin'
endef

define BRCM_SCRIPTS_INSTALL_TARGET_CMDS
	cp -p $(@D)/skel/bin/* $(TARGET_DIR)/bin
endef

$(eval $(generic-package))
