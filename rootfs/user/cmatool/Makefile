include $(ROOTDIR)/config.arch

LINUXDIR := linux

CFLAGS += -idirafter $(ROOTDIR)/$(LINUXDIR)/include/uapi -Wall -Wextra -Werror

.PHONY: all
all: cmatool

cmatool: cmatool.c
	$(CC) $(CFLAGS) $< -o $@

.PHONY: clean distclean romfs
clean:
	rm -f cmatool

distclean: clean

romfs:
	$(ROMFSINST) -e CONFIG_USER_CMATOOL cmatool /bin/cmatool
