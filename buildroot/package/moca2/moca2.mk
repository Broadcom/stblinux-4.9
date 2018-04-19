################################################################################
#
# moca2
#
################################################################################

BRCMROOT_VERSION = c38822d042bd729481f2cea6b2fd138b5e190fa8

MOCA2_VERSION = master
MOCA2_SITE = git://stbgit.broadcom.com/mm940762/uclinux-rootfs.git
MOCA2_SOURCE = uclinux-rootfs-$(BRCMROOT_VERSION).tar.gz
MOCA2_LICENSE = Proprietary

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
