#
# This makefile is shared between uclinux-rootfs and buildroot!
#

# "include" needs to be conditional, so buildroot doesn't fail.
-include $(ROOTDIR)/config.arch

# "?=" required by buildroot, since it sets its own path to Linux
LINUXDIR?=	linux

# "override" is required by buildroot.
override CFLAGS += -idirafter $(ROOTDIR)/$(LINUXDIR)/include/uapi -Wall -Wextra -Werror

.PHONY: all
all: cmatool

cmatool: cmatool.c
	$(CC) $(CFLAGS) $< -o $@

.PHONY: clean distclean install romfs
clean:
	rm -f cmatool

distclean: clean

# buildroot install target
install: all
	install -D -p -m 0755 cmatool $(DESTDIR)/bin

# uclinux-rootfs install target
romfs:
	$(ROMFSINST) -e CONFIG_USER_CMATOOL cmatool /bin/cmatool
