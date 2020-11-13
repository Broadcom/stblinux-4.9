################################################################################
#
# moca2
#
################################################################################

BRCMROOT_VERSION = $(call qstrip,$(BR2_BRCM_STB_TOOLS_VERSION))

MOCA2_VERSION = $(BRCMROOT_VERSION)
MOCA2_SITE = git://stbgit.stb.broadcom.net/queue/stbtools.git
MOCA2_SOURCE = stbtools-$(BRCMROOT_VERSION).tar.gz
MOCA2_DL_SUBDIR = brcm-pm
MOCA2_LICENSE = Proprietary

# Extract only what we need to save space.
define MOCA2_EXTRACT_CMDS
	$(call suitable-extractor,$(MOCA2_SOURCE)) \
		$(MOCA2_DL_DIR)/$(MOCA2_SOURCE) | \
		$(TAR) --strip-components=1 -C $(MOCA2_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/moca2'
endef

define MOCA2_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) MACHINE=$(BR2_ARCH) LIBCDIR=eglibc \
		$(MAKE) -C $(@D)/moca2 install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
