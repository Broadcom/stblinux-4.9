BRCM_TEST_MODULES_SITE_METHOD = local
BRCM_TEST_MODULES_SITE = $(LINUX_DIR)/tools/testing/brcmstb/

BRCM_TEST_MODULES = dvfs-api gpio-api irq-api memory-api reg-api

BRCM_TEST_MODULES_MODULE_SUBDIRS += $(foreach test-module,$(BRCM_TEST_MODULES),\
	$(if $(BR2_PACKAGE_BRCM_TEST_MODULES_$(call UPPERCASE,$(test-module))),\
		$(test-module)))

$(eval $(kernel-module))
$(eval $(generic-package))
