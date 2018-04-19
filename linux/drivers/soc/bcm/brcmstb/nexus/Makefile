obj-y	:=	irq.o \
		reg.o
obj-$(CONFIG_GPIOLIB) += gpio.o

# Might be built as a module
ifdef CONFIG_SPI_BRCMSTB_MSPI
obj-y 		+= spi.o
endif
ifdef CONFIG_ARM_SCMI_PROTOCOL
obj-y		+= dvfs.o
endif
ifdef CONFIG_ARM_BRCMSTB_AVS_CPUFREQ
obj-y		+= dvfs.o
endif
