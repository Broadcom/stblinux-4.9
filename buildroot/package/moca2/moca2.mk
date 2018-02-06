################################################################################
#
# moca2
#
################################################################################

BRCMROOT_VERSION = e301a2d729cbc11f7052fdbf2b4d3297950eb16c

MOCA2_VERSION = master
MOCA2_SITE = git://stbgit.broadcom.com/mm940762/uclinux-rootfs.git
MOCA2_SOURCE = uclinux-rootfs-$(BRCMROOT_VERSION).tar.gz
MOCA2_LICENSE = GPL-2.0
MOCA2_LICENSE_FILES = COPYING

# Extract only what we need to save space.
define MOCA2_EXTRACT_CMDS
	$(call suitable-extractor,$(MOCA2_SOURCE)) \
		$(DL_DIR)/$(MOCA2_SOURCE) | \
		$(TAR) --strip-components=1 -C $(MOCA2_DIR) \
			--wildcards $(TAR_OPTIONS) - '*/user/moca2'
endef

define MOCA2_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) MACHINE=$(BR2_ARCH) LIBCDIR=eglibc \
		$(MAKE) -C $(@D)/user/moca2 install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
