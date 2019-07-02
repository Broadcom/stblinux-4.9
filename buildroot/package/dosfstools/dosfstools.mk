################################################################################
#
# dosfstools
#
################################################################################

DOSFSTOOLS_VERSION = 4.1
DOSFSTOOLS_SOURCE = dosfstools-$(DOSFSTOOLS_VERSION).tar.xz
DOSFSTOOLS_SITE = https://github.com/dosfstools/dosfstools/releases/download/v$(DOSFSTOOLS_VERSION)
DOSFSTOOLS_LICENSE = GPL-3.0+
DOSFSTOOLS_LICENSE_FILES = COPYING
DOSFSTOOLS_CONF_OPTS = --enable-compat-symlinks --exec-prefix=/
DOSFSTOOLS_TEMP=$(BASE_DIR)/build/dosfstools-install
HOST_DOSFSTOOLS_CONF_OPTS = --enable-compat-symlinks

ifeq ($(BR2_PACKAGE_HAS_UDEV),y)
DOSFSTOOLS_CONF_OPTS += --with-udev
DOSFSTOOLS_DEPENDENCIES += udev
else
DOSFSTOOLS_CONF_OPTS += --without-udev
endif

ifneq ($(BR2_ENABLE_LOCALE),y)
DOSFSTOOLS_CONF_OPTS += LIBS="-liconv"
DOSFSTOOLS_DEPENDENCIES += libiconv
endif

# Install into temporary location
DOSFSTOOLS_INSTALL_TARGET_OPTS = \
	DESTDIR=$(DOSFSTOOLS_TEMP) \
	install

# Clean out our temporary install location before installation
define DOSFSTOOLS_CLEAR_TEMP
	rm -rf $(DOSFSTOOLS_TEMP)
endef
DOSFSTOOLS_PRE_INSTALL_TARGET_HOOKS += DOSFSTOOLS_CLEAR_TEMP

# Sanitize temporary install location after installation
ifeq ($(BR2_PACKAGE_DOSFSTOOLS_FATLABEL),)
define DOSFSTOOLS_REMOVE_FATLABEL
	rm -f $(addprefix $(DOSFSTOOLS_TEMP)/sbin/,dosfslabel fatlabel)
endef
DOSFSTOOLS_POST_INSTALL_TARGET_HOOKS += DOSFSTOOLS_REMOVE_FATLABEL
endif

ifeq ($(BR2_PACKAGE_DOSFSTOOLS_FSCK_FAT),)
define DOSFSTOOLS_REMOVE_FSCK_FAT
	rm -f $(addprefix $(DOSFSTOOLS_TEMP)/sbin/,fsck.fat dosfsck fsck.msdos fsck.vfat)
endef
DOSFSTOOLS_POST_INSTALL_TARGET_HOOKS += DOSFSTOOLS_REMOVE_FSCK_FAT
endif

ifeq ($(BR2_PACKAGE_DOSFSTOOLS_MKFS_FAT),)
define DOSFSTOOLS_REMOVE_MKFS_FAT
	rm -f $(addprefix $(DOSFSTOOLS_TEMP)/sbin/,mkfs.fat mkdosfs mkfs.msdos mkfs.vfat)
endef
DOSFSTOOLS_POST_INSTALL_TARGET_HOOKS += DOSFSTOOLS_REMOVE_MKFS_FAT
endif

# Install what's left into the actual target directory
define DOSFSTOOLS_TARGET_INSTALL
	cp -a $(DOSFSTOOLS_TEMP)/sbin/* $(TARGET_DIR)/sbin
	rm -rf $(DOSFSTOOLS_TEMP)
endef
DOSFSTOOLS_POST_INSTALL_TARGET_HOOKS += DOSFSTOOLS_TARGET_INSTALL

$(eval $(autotools-package))
$(eval $(host-autotools-package))
