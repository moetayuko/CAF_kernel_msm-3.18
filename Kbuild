
obj-m += maxim_sti.o
CFLAGS_MODULE +=-fno-pic

all:
	$(Q)$(MAKE) -C kernel_$(PLATFORM) M=`pwd` modules
clean:
	$(Q)$(MAKE) -C kernel_$(PLATFORM) M=`pwd` clean

