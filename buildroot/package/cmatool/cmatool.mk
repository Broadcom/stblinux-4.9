################################################################################
#
# cmatool
#
################################################################################

BRCMROOT_VERSION = e301a2d729cbc11f7052fdbf2b4d3297950eb16c

CMATOOL_VERSION = master
CMATOOL_SITE = git://stbgit.broadcom.com/mm940762/uclinux-rootfs.git
CMATOOL_SOURCE = uclinux-rootfs-$(BRCMROOT_VERSION).tar.gz
CMATOOL_LICENSE = GPL-2.0
CMATOOL_LICENSE_FILES = COPYING

# We only need the kernel to be extracted, not actually built
CMATOOL_PATCH_DEPENDENCIES = linux

# The cmatool Makefile needs to know where to find Linux
TARGET_CONFIGURE_OPTS += ROOTDIR=../../..
TARGET_CONFIGURE_OPTS += LINUXDIR=linux-$(LINUX_VERSION)

# Extract only what we need to save space.
define CMATOOL_EXTRACT_CMDS
	$(call suitable-extractor,$(CMATOOL_SOURCE)) \
		$(DL_DIR)/$(CMATOOL_SOURCE) | \
		$(TAR) --strip-components=1 -C $(CMATOOL_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/user/cmatool'
endef

define CMATOOL_BUILD_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/user/cmatool $(TARGET_CONFIGURE_OPTS)
endef

define CMATOOL_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) \
		$(MAKE) -C $(@D)/user/cmatool install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
