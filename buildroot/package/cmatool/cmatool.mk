################################################################################
#
# cmatool
#
################################################################################

BRCMROOT_VERSION = c38822d042bd729481f2cea6b2fd138b5e190fa8

CMATOOL_VERSION = master
CMATOOL_SITE = git://stbgit.broadcom.com/mm940762/uclinux-rootfs.git
CMATOOL_SOURCE = uclinux-rootfs-$(BRCMROOT_VERSION).tar.gz
CMATOOL_LICENSE = LGPL-2.1

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
