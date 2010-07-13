   zreladdr-y		:= 0x02008000
params_phys-y		:= 0x02000100
initrd_phys-y		:= 0x02800000

# for now, override for QSD8x50
  zreladdr-$(CONFIG_ARCH_QSD8X50)		:= 0x20008000
params_phys-$(CONFIG_ARCH_QSD8X50)		:= 0x20000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)		:= 0x21000000
