Please use the following steps to integrate vl6180 driver into your kernel:

- copy the include and drivers folders into the root of kernel source:
	cp -r drivers <KERNEL_SRC_ROOT>/

- edit input Kconfig (drivers/input/misc/Kconfig) to include STMVL6180 config
  adding the following line:
	source "drivers/input/misc/vl6180/Kconfig"

- edit input Makefile (drivers/input/misc/Makefile) adding the following line:
	obj-y += vl6180/


To enable this driver you have to make the needed changes in the board file or
into platform device tree.
	 

