1. anx_slimport.h is under kernel/include/video/anx_slimport.h

2. ANX_Colorado3 is under: kernel/drivers/video/msm/ANX_Colorado3

3. slimport_Colorado3.dtsi is under: kernel/arch/arm/boot/dts/slimport_Colorado3.dtsi, and this file should be included in kernel/arch/arm/boot/dts/msm8974.dtsi

4. Add source "drivers/video/msm/ANX_Colorado3/Kconfig" to file kernel/drivers/video/msm/Kconfig

5. Add obj-$(CONFIG_SLIMPORT_COLORADO3) += ANX_Colorado3/ to file kernel/drivers/video/msm/Makefile

6. Add CONFIG_SLIMPORT_COLORADO3=y to file kernel/arch/arm/configs/msm8974_defconfig

7. Build the project.
8. Our cable detect signal is high (1) when the dongle cable is plugged, and low (0) when dongle cable is unplugged. But in system msm8974, the value we get using gpio_get_value from GPIO is opposite with the signal.So if you want to modify the value of GPIO, you can modify the makefile drivers/video/msm/ANX_Colorado3/Makefile:
ccflags- += -D DONGLE_CABLE_INSERT=1 to ccflags- += -D DONGLE_CABLE_INSERT=0
9. If the PMIC chip is SMB1357,should open ccflags-y += -D QUALCOMM_SMB1357 in 
Makefile:drivers/video/msm/ANX_Colorado3/Makefile