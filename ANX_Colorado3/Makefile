#ccflags-y += -D QUALCOMM_SMB1357
ccflags-y += -D DONGLE_CABLE_INSERT=1

obj-$(CONFIG_SLIMPORT_COLORADO3)	+= co3.o
co3-objs := slimport.o \
	    slimport_tx_drv.o \
	    slimport_tx_cec.o \
	    quick_charge.o \
	    anx_slimport.o
