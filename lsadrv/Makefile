CC 	?= gcc
LD 	?= ld
RM	= /bin/rm -f

# currently running kernel release
KERNELRELEASE := $(shell uname -r)
ARCH := $(shell uname -m | sed 's/i.86/i386/')
CARCH := $(shell uname -m)

MODLIB := /lib/modules/$(KERNELRELEASE)

# parse KERNELRELEASE to override version.h
VERSION := $(shell echo $(KERNELRELEASE) | sed -e 's/\([0-9]*\)[.]\([0-9]*\)[.]\([0-9]*\)\(.*\)/\1/')
PATCHLEVEL := $(shell echo $(KERNELRELEASE) | sed -e 's/\([0-9]*\)[.]\([0-9]*\)[.]\([0-9]*\)\(.*\)/\2/')
SUBLEVEL := $(shell echo $(KERNELRELEASE) | sed -e 's/\([0-9]*\)[.]\([0-9]*\)[.]\([0-9]*\)\(.*\)/\3/')
EXTRAVERSION := $(shell echo $(KERNELRELEASE) | sed -e 's/\([0-9]*\)[.]\([0-9]*\)[.]\([0-9]*\)\(.*\)/\4/')

KERNEL_SRC := $(shell for i in $(MODLIB)/build /usr/src/linux-headers-$(KERNELRELEASE) /usr/src/kernels/$(KERNELRELEASE)-$(CARCH) /usr/src/linux; \
			do \
			if [ -d $$i/include/linux ]; then \
				echo $$i; \
				break; \
			fi; \
		done)
KERNELINC := $(KERNEL_SRC)/include

TARGETS := $(KERNELRELEASE)/lsadrv.ko
comma = ,
HEADERS = lsadrv.h lsadrv-ioctl.h lsadrv-vkey.h fakemouse.h
SOURCES = lsadrv-main.c lsadrv-sub.c fakemouse.c
SOURCESX = lsadrv-ioctl.c lsadrv-isoc.c lsadrv-vkey.c
OBJS	= $(patsubst %.c,%.o,$(SOURCES))
OBJSX	= $(patsubst %.c,%.o,$(SOURCESX))

obj-m := lsadrv.o
lsadrv-objs := $(OBJS) $(OBJSX)
clean-files := *.o *.ko *.mod.[co] $(TARGETS) *~

$(TARGETS): $(SOURCES) $(SOURCESX)
	mkdir -p $(KERNELRELEASE)
	@echo "make -C $(KERNEL_SRC) M=`pwd` V=1 modules"
	make -C $(KERNEL_SRC) M=`pwd` V=1 modules
	mv -f lsadrv.ko $(KERNELRELEASE)

.PHONY: dummy
linux/version.h: dummy
	@mkdir -p linux
	@echo \#define UTS_RELEASE \"$(KERNELRELEASE)\" > .ver
	@echo \#define LINUX_VERSION_CODE `expr $(VERSION) \\* 65536 + $(PATCHLEVEL) \\* 256 + $(SUBLEVEL)` >> .ver
	@echo '#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))' >>.ver
	@mv -f .ver $@

.PHONY: clean
clean:
	make -C $(KERNEL_SRC) M=`pwd` clean
	$(RM) *.o $(KERNELRELEASE)/*.o tags linux/version.h
	@if [ -e linux ]; then rmdir linux; fi
	@if [ -e $(KERNELRELEASE) ]; then rmdir $(KERNELRELEASE); fi

.PHONY: install
install: $(TARGETS)
	mkdir -p $(MODLIB)/kernel/drivers/usb/input
	cp -f $(TARGETS) $(MODLIB)/kernel/drivers/usb/input/
	/sbin/depmod -a $(KERNELRELEASE)
	if [ -f /etc/modules ]; then \
		grep lsadrv /etc/modules >/dev/null || echo "lsadrv" >>/etc/modules; \
	elif [ -f /etc/rc.modules ]; then \
		grep lsadrv /etc/rc.modules >/dev/null || echo "modprobe lsadrv" >>/etc/rc.modules; \
		chmod a+x /etc/rc.modules; \
	else \
		echo "#!/bin/sh" >/etc/rc.modules; \
		echo "modprobe lsadrv" >>/etc/rc.modules; \
		chmod a+x /etc/rc.modules; \
	fi

.PHONY: uninstall
uninstall: 
	cd $(MODLIB)/kernel/drivers/usb/input && $(RM) lsadrv.ko
	/sbin/depmod -a $(KERNELRELEASE)
	@cat $(MODLIB)/modules.usbmap | sed -e /^lsadrv/d > .usbmap && \
	mv -f .usbmap $(MODLIB)/modules.usbmap
	@if [ -f /etc/rc.modules ] && grep lsadrv /etc/rc.modules >/dev/null; then \
		cp -p /etc/rc.modules .rc.modules && \
		cat .rc.modules | sed -e '/lsadrv/d' >/etc/rc.modules && \
		$(RM) .rc.modules; \
	fi

