################################################################################
#
# brcm-skel
#
################################################################################

BRCMROOT_VERSION = $(call qstrip,$(BR2_BRCM_STB_TOOLS_VERSION))

BRCM_SKEL_VERSION = $(BRCMROOT_VERSION)
BRCM_SKEL_SITE = git://stbgit.broadcom.com/queue/stbtools.git
BRCM_SKEL_SOURCE = stbtools-$(BRCMROOT_VERSION).tar.gz
BRCM_SKEL_DL_SUBDIR = brcm-pm
BRCM_SKEL_LICENSE = GPL-2.0

ifeq ($(BR2_INIT_BUSYBOX),y)
# Ensure that packages initscripts & busybox don't overwrite any of our files
BRCM_SKEL_DEPENDENCIES = initscripts busybox
else
# Don't install rcS if we have neither INIT_BUSYBOX nor INIT_SYSV
ifeq ($(BR2_INIT_SYSV),)
INSTALL_EXCLUDE = --exclude init.d
endif
endif

# Extract only what we need to save space.
define BRCM_SKEL_EXTRACT_CMDS
	$(call suitable-extractor,$(BRCM_SKEL_SOURCE)) \
		$(BRCM_SKEL_DL_DIR)/$(BRCM_SKEL_SOURCE) | \
		$(TAR) --strip-components=1 -C $(BRCM_SKEL_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/skel'
endef

define BRCM_SKEL_INSTALL_TARGET_CMDS
	rsync -a --exclude .gitignore \
		$(INSTALL_EXCLUDE) \
		$(@D)/skel/ $(TARGET_DIR)
endef

$(eval $(generic-package))
