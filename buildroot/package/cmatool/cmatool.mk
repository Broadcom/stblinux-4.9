################################################################################
#
# cmatool
#
################################################################################

BRCMROOT_VERSION = $(call qstrip,$(BR2_BRCM_STB_TOOLS_VERSION))

CMATOOL_VERSION = $(BRCMROOT_VERSION)
CMATOOL_SITE = git://stbgit.stb.broadcom.net/queue/stbtools.git
CMATOOL_SOURCE = stbtools-$(BRCMROOT_VERSION).tar.gz
CMATOOL_DL_SUBDIR = brcm-pm
CMATOOL_LICENSE = LGPL-2.1

# We only need the kernel to be extracted, not actually built
CMATOOL_PATCH_DEPENDENCIES = linux

# The cmatool Makefile needs to know where to find Linux
TARGET_CONFIGURE_OPTS += ROOTDIR=../..
TARGET_CONFIGURE_OPTS += LINUXDIR=linux-$(LINUX_VERSION)

# Extract only what we need to save space.
define CMATOOL_EXTRACT_CMDS
	$(call suitable-extractor,$(CMATOOL_SOURCE)) \
		$(CMATOOL_DL_DIR)/$(CMATOOL_SOURCE) | \
		$(TAR) --strip-components=1 -C $(CMATOOL_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/cmatool'
endef

define CMATOOL_BUILD_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/cmatool $(TARGET_CONFIGURE_OPTS)
endef

define CMATOOL_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/cmatool install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
