obj-y	:=	irq.o \
		reg.o
obj-$(CONFIG_GPIOLIB) += gpio.o

ifdef CONFIG_BRCMSTB_NEXUS_SPI_API
obj-y 		+= spi.o
endif
ifdef CONFIG_BRCMSTB_NEXUS_CLK_API
obj-y 		+= clk.o
endif
ifdef CONFIG_BRCMSTB_NEXUS_DVFS_API
obj-y		+= dvfs.o
endif
ifdef CONFIG_BRCMSTB_SCMI_VPUCOM
obj-y		+= vpucom.o
endif
