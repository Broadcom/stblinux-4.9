################################################################################
#
# cpufrequtils
#
################################################################################

CPUFREQUTILS_VERSION = 008
CPUFREQUTILS_SITE = http://http.debian.net/debian/pool/main/c/cpufrequtils
CPUFREQUTILS_SOURCE = cpufrequtils_$(CPUFREQUTILS_VERSION).orig.tar.bz2
CPUFREQUTILS_PATCH = cpufrequtils_$(CPUFREQUTILS_VERSION)-1.debian.tar.gz
CPUFREQUTILS_BR2_PATCHES = 0000-CPPFLAGS-override.patch
CPUFREQUTILS_LICENSE = GPL-2.0
CPUFREQUTILS_LICENSE_FILES = COPYING
CPUFREQUTILS_MAKE_ENV = $(TARGET_MAKE_ENV) NLS=false

# Apply patches that are required to make cpufrequtils build with buildroot
define CPUFREQUTILS_APPLY_BR2_PATCHES
	for p in $(@D)/$(CPUFREQUTILS_BR2_PATCHES); do \
		$(APPLY_PATCHES) $(@D) $(@D) $$p; \
	done
endef

CPUFREQUTILS_POST_PATCH_HOOKS += CPUFREQUTILS_APPLY_BR2_PATCHES

define CPUFREQUTILS_BUILD_CMDS
	$(CPUFREQUTILS_MAKE_ENV) $(MAKE) -C $(@D) $(TARGET_CONFIGURE_OPTS)
endef

define CPUFREQUTILS_INSTALL_TARGET_CMDS
	$(CPUFREQUTILS_MAKE_ENV) libdir=/usr/$(BR2_ROOTFS_LIB_DIR) \
		$(MAKE) -C $(@D) install DESTDIR=$(TARGET_DIR)
endef

$(eval $(generic-package))
