#
# This makefile is shared between uclinux-rootfs and buildroot!
#

# "include" needs to be conditional, so buildroot doesn't fail.
-include $(ROOTDIR)/config.arch

# Nexus is using -W -Wall -Wpedantic to build pmlib.c, so we use it as well.
# This has the following implications:
#   * We need at least C99 standard so struct initializations are possible
#     without warning. (-Wpedantic warns about them otherwise.)
#   * We need GNU99 rather than C99, because we want function prototypes for
#     strnlen() and usleep() as well as the type definition for caddr_t.
#     Apparently, those don't get defined when -std=c99 is being used.
# "override" is required by buildroot.
override CFLAGS += -I. -L. -W -Wall -Wpedantic -Werror -std=gnu99

ifeq ($(ARCH)$(MAJOR_VERSION),mips314)
SUFFIX:=-legacy
endif

.PHONY: all
all: libbrcm-pm.a pmtest

libbrcm-pm.a: pmlib$(SUFFIX).o
	$(AR) r $@ $<
	$(RANLIB) $@

pmlib.o: pmlib$(SUFFIX).c pmlib$(SUFFIX).h
	$(CC) $(CFLAGS) -c $< -o $@

pmtest: pmtest$(SUFFIX).c libbrcm-pm.a
	$(CC) $(CFLAGS) $< -o $@ -lbrcm-pm

.PHONY: clean distclean install romfs
clean:
	rm -f libbrcm-pm.a pmtest pmlib.o

distclean: clean

# buildroot install target
install: all
	install -D -p -m 0755 pmtest $(DESTDIR)/bin

# uclinux-rootfs install target
romfs:
	$(ROMFSINST) -e CONFIG_USER_BRCM_PM pmtest /bin/pmtest
